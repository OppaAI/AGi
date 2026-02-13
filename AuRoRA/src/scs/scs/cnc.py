import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

import requests
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
from datetime import datetime, date, timedelta
import signal
import sys
import base64
import os
import threading

# ✅ Load environment variables from .env file
try:
    from dotenv import load_dotenv
    load_dotenv(dotenv_path=Path.home() / "AGi/.env")  # Load .env file from home directory
    DOTENV_AVAILABLE = True
except ImportError:
    DOTENV_AVAILABLE = False
    print("⚠️  python-dotenv not installed. Using system environment variables only.")
    print("   Install with: pip3 install python-dotenv")

# ✅ Slack integration
try:
    from slack_sdk import WebClient
    from slack_sdk.errors import SlackApiError
    SLACK_AVAILABLE = True
except ImportError:
    SLACK_AVAILABLE = False
    print("⚠️  slack-sdk not installed. Slack features disabled.")
    print("   Install with: pip3 install slack-sdk")

# ✅ Slack Bolt for event listening (NEW - TWO-WAY COMMUNICATION)
try:
    from slack_bolt import App
    from slack_bolt.adapter.socket_mode import SocketModeHandler
    SLACK_BOLT_AVAILABLE = True
except ImportError:
    SLACK_BOLT_AVAILABLE = False
    print("⚠️  slack-bolt not installed. Two-way Slack disabled.")
    print("   Install with: pip3 install slack-bolt")

# ═══════════════════════════════════════════════════════
# CONFIGURATION
# ═══════════════════════════════════════════════════════

# Model configuration
GCE = "ministral-3:3b-instruct-2512-q4_K_M"
# VLM_MODEL = "qwen2-vl:2b"  # Uncomment when switching to VLM

# File paths
CHAT_HISTORY_FILE = '.chat_history.json'
REFLECTIONS_FILE = '.daily_reflections.json'

# Ollama configuration
OLLAMA_BASE_URL = "http://localhost:11434"

# Web search
SEARXNG_URL = "http://127.0.0.1:8080"

# Slack configuration (loaded from .env file or environment)
SLACK_BOT_TOKEN = os.getenv('SLACK_BOT_TOKEN', '')
SLACK_APP_TOKEN = os.getenv('SLACK_APP_TOKEN', '')  # NEW: For Socket Mode (two-way)
SLACK_CHANNEL = os.getenv('SLACK_CHANNEL', '#grace-logs')

# ✅ Jetson Orin Nano Optimized Settings (UPDATED FOR 4B MODEL)
SAFE_CONTEXT_SIZES = {
    "2b-4b": 4096,    # 2-4B models: conservative
    "7b": 6144,       # 7B models: moderate
    "13b+": 8192      # 13B+ models: full context
}

MAX_MESSAGES_BY_SIZE = {
    "2b-4b": 12,      # Fewer messages for small models
    "7b": 20,         # Moderate for 7B
    "13b+": 30        # More for large models
}

# Memory limits (OPTIMIZED)
MAX_RECENT_MESSAGES = 12      # Reduced from 30 for 4B model
MAX_REFLECTIONS_LOAD = 5      # Reduced from 20 - only recent days
MAX_HISTORY_STORAGE = 1000    # Store up to 1000 on disk
SUMMARIZE_THRESHOLD = 50      # Summarize after 50 messages

# Token budget per request (NEW)
MAX_REQUEST_TOKENS = 3500     # Conservative for 4B model (leave room for response)

# Search result limits
MAX_SEARCH_RESULTS = 2        # Reduced from 3 to save tokens
SEARCH_CONTENT_CHARS = 150    # Reduced from 200


class CNSBridge(Node):
    """
    Central Nervous System Bridge (OPTIMIZED + TWO-WAY SLACK)
    
    Grace's main cognitive interface connecting:
    - Text conversation (LLM)
    - Vision processing (VLM)
    - Web search (SearXNG)
    - Memory (reflections)
    - Notifications (Slack - NOW WITH TWO-WAY COMMUNICATION!)
    
    Optimizations:
    - Reduced context window for 4B models
    - Minimal system prompts
    - Lazy loading of reflections
    - Automatic context trimming
    - Thread-safe state management
    
    NEW:
    - Two-way Slack communication via Socket Mode
    - Slack messages → ROS topic → Ollama → Slack response
    - Support for @mentions and direct messages
    """
    
    def __init__(self):
        super().__init__('cns_bridge')
        
        # ═══════════════════════════════════════════════════
        # THREAD SAFETY
        # ═══════════════════════════════════════════════════
        
        self._message_count_lock = threading.Lock()
        self._day_check_lock = threading.Lock()
        
        # ═══════════════════════════════════════════════════
        # ROS TOPICS
        # ═══════════════════════════════════════════════════
        
        # Text input
        self.subscription = self.create_subscription(
            String, '/cns/neural_input', self.listener_callback, 10)
        
        # Image input
        self.image_subscription = self.create_subscription(
            String, '/cns/image_input', self.image_listener_callback, 10)
        
        # Response output
        self.publisher = self.create_publisher(String, '/gce/response', 10)
        
        # Thread pool for non-blocking processing
        self.executor_pool = ThreadPoolExecutor(max_workers=2)
        
        # ═══════════════════════════════════════════════════
        # MODEL SETTINGS (AUTO-OPTIMIZED)
        # ═══════════════════════════════════════════════════
        
        self.model_name = GCE
        self.keep_alive = -1  # Keep model in memory permanently
        
        # Auto-detect safe limits based on model size
        self.safe_context = self._get_safe_context_size()
        self.max_recent_messages = self._get_max_messages()
        self.max_reflections_load = MAX_REFLECTIONS_LOAD
        
        # ═══════════════════════════════════════════════════
        # MEMORY SYSTEMS
        # ═══════════════════════════════════════════════════
        
        # Chat history
        self.chat_history_file = Path.home() / CHAT_HISTORY_FILE
        self.chat_history = self.load_chat_history()
        
        # Daily reflections
        self.reflections_file = Path.home() / REFLECTIONS_FILE
        self.reflections = self.load_reflections()
        self.today_start = datetime.now().date()
        self.today_message_count = 0
        
        # Birth date
        self.birth_date = self._get_birth_date()
        
        # Handle missed days
        self._handle_missed_reflections()
        
        # ═══════════════════════════════════════════════════
        # EXTERNAL SERVICES
        # ═══════════════════════════════════════════════════
        
        # Web search
        self.searxng_url = SEARXNG_URL
        self.search_enabled = self._check_searxng_available()
        
        # Slack (one-way notifications)
        self.slack_client = None
        self.slack_enabled = self._init_slack()
        
        # Slack (two-way listener) - NEW!
        self.slack_app = None
        self.slack_listener_enabled = self._init_slack_listener()
        
        # ═══════════════════════════════════════════════════
        # STARTUP LOGGING
        # ═══════════════════════════════════════════════════
        
        age_days = (datetime.now().date() - self.birth_date).days
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("GRACE - COGNITIVE SYSTEMS ONLINE 🧠")
        self.get_logger().info("=" * 60)
        
        if DOTENV_AVAILABLE:
            self.get_logger().info("✅ python-dotenv loaded (.env file support)")
        else:
            self.get_logger().info("⚠️  python-dotenv not available (using system env only)")
        
        self.get_logger().info(f"Birth Date: {self.birth_date}")
        self.get_logger().info(f"Age: {age_days} days old")
        self.get_logger().info(f"Model: {self.model_name}")
        self.get_logger().info("-" * 60)
        self.get_logger().info("⚙️  OPTIMIZATIONS APPLIED:")
        self.get_logger().info(f"   Safe context: {self.safe_context} tokens")
        self.get_logger().info(f"   Max messages: {self.max_recent_messages}")
        self.get_logger().info(f"   Target budget: {MAX_REQUEST_TOKENS} tokens")
        self.get_logger().info(f"   Reflection limit: {self.max_reflections_load} days")
        self.get_logger().info("-" * 60)
        self.get_logger().info(f"Chat History: {len(self.chat_history)} total messages")
        self.get_logger().info(f"  → Loading: {min(len(self.chat_history), self.max_recent_messages)} recent")
        self.get_logger().info(f"Reflections: {len(self.reflections)} total days")
        self.get_logger().info(f"  → Loading: {min(len(self.reflections), self.max_reflections_load)} recent")
        self.get_logger().info(f"Today's Messages: {self.today_message_count}")
        self.get_logger().info("-" * 60)
        
        # Feature status
        self.get_logger().info("✅ Text conversation enabled")
        self.get_logger().info("✅ Image processing enabled (VLM ready)")
        
        if self.search_enabled:
            self.get_logger().info("✅ Web search enabled (SearXNG)")
        else:
            self.get_logger().warn("⚠️  Web search disabled (SearXNG unavailable)")
        
        if self.slack_enabled:
            self.get_logger().info(f"✅ Slack notifications enabled → {SLACK_CHANNEL}")
        else:
            self.get_logger().warn("⚠️  Slack disabled (no token or error)")
        
        if self.slack_listener_enabled:
            self.get_logger().info("✅ Slack listener enabled (TWO-WAY) 📱↔️🤖")
            self.get_logger().info("   → Listening for @mentions and DMs")
        else:
            self.get_logger().warn("⚠️  Slack listener disabled (one-way only)")
        
        self.get_logger().info("=" * 60)
        
        # Warm up model
        self._verify_model_loaded()

    # ═══════════════════════════════════════════════════════
    # INITIALIZATION
    # ═══════════════════════════════════════════════════════

    def _get_safe_context_size(self):
        """Get safe context size based on model size"""
        model_lower = self.model_name.lower()
        
        if any(x in model_lower for x in ["2b", "4b"]):
            return SAFE_CONTEXT_SIZES["2b-4b"]
        elif "7b" in model_lower:
            return SAFE_CONTEXT_SIZES["7b"]
        else:
            return SAFE_CONTEXT_SIZES["13b+"]

    def _get_max_messages(self):
        """Get max messages based on model size"""
        model_lower = self.model_name.lower()
        
        if any(x in model_lower for x in ["2b", "4b"]):
            return MAX_MESSAGES_BY_SIZE["2b-4b"]
        elif "7b" in model_lower:
            return MAX_MESSAGES_BY_SIZE["7b"]
        else:
            return MAX_MESSAGES_BY_SIZE["13b+"]

    def _verify_model_loaded(self):
        """Verify model is loaded and warm in Ollama"""
        try:
            self.get_logger().info("🔥 Warming up model...")
            response = requests.post(
                f'{OLLAMA_BASE_URL}/api/generate',
                json={
                    "model": self.model_name,
                    "prompt": "Hi",
                    "stream": False,
                    "keep_alive": -1
                },
                timeout=30
            )
            
            if response.status_code == 200:
                self.get_logger().info("✅ Model warmed up and cached in memory")
                return True
            else:
                self.get_logger().warn(f"⚠️  Model warm-up returned status {response.status_code}")
                
        except Exception as e:
            self.get_logger().warn(f"⚠️  Model warm-up failed: {e}")
        
        return False

    def _get_birth_date(self):
        """Get Grace's birth date from reflections or set today"""
        if self.reflections:
            birth_str = self.reflections[0].get('date', date.today().isoformat())
            return date.fromisoformat(birth_str)
        return date.today()

    def _handle_missed_reflections(self):
        """Create placeholder reflections for missed days"""
        if not self.reflections:
            return
        
        last_reflection_date = date.fromisoformat(self.reflections[-1]['date'])
        today = date.today()
        days_missed = (today - last_reflection_date).days - 1
        
        if days_missed > 0:
            self.get_logger().warn(f"⚠️  Detected {days_missed} missed days - creating placeholders")
            
            for i in range(1, days_missed + 1):
                missed_date = last_reflection_date + timedelta(days=i)
                age_days = (missed_date - self.birth_date).days
                
                placeholder = {
                    'day': age_days,
                    'date': missed_date.isoformat(),
                    'reflection': f"😴: Day {age_days} - Grace was offline. No conversations today.",
                    'message_count': 0
                }
                
                self.reflections.append(placeholder)
            
            self._save_reflections_file()
            self.get_logger().info(f"✅ Created {days_missed} placeholder reflections")

    def _check_searxng_available(self):
        """Check if SearXNG is running"""
        try:
            response = requests.get(self.searxng_url, timeout=5)
            # Accept 200 (OK) or 403 (Forbidden but alive)
            is_available = response.status_code in [200, 403]
            
            if is_available:
                self.get_logger().info(f"SearXNG check: status {response.status_code}")
            
            return is_available
            
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"SearXNG check failed: {e}")
            return False

    def _init_slack(self):
        """Initialize Slack client (uses token from .env file)"""
        if not SLACK_AVAILABLE:
            self.get_logger().info("Slack SDK not installed")
            return False
        
        try:
            if not SLACK_BOT_TOKEN:
                self.get_logger().info("SLACK_BOT_TOKEN not set in .env or environment")
                self.get_logger().info("Create .env file with: SLACK_BOT_TOKEN=xoxb-your-token")
                return False
            
            if SLACK_BOT_TOKEN.startswith("xoxb-your"):
                self.get_logger().warn("SLACK_BOT_TOKEN is still a placeholder")
                self.get_logger().info("Update .env file with real token from https://api.slack.com/apps")
                return False
            
            # Initialize Slack client
            self.slack_client = WebClient(token=SLACK_BOT_TOKEN)
            
            # Test connection
            response = self.slack_client.auth_test()
            self.get_logger().info(f"Slack connected as: {response['user']}")
            self.get_logger().info(f"Slack channel: {SLACK_CHANNEL}")
            return True
            
        except SlackApiError as e:
            self.get_logger().error(f"Slack API error: {e.response['error']}")
            
            if e.response['error'] == 'invalid_auth':
                self.get_logger().error("Token is invalid. Get new token from https://api.slack.com/apps")
            
            return False
            
        except Exception as e:
            self.get_logger().error(f"Slack init failed: {e}")
            return False

    def _init_slack_listener(self):
        """Initialize Slack event listener for two-way communication (NEW)"""
        if not self.slack_enabled or not SLACK_AVAILABLE or not SLACK_BOLT_AVAILABLE:
            return False
        
        if not SLACK_APP_TOKEN:
            self.get_logger().warn("⚠️  SLACK_APP_TOKEN not set. Two-way Slack disabled.")
            self.get_logger().warn("   Add SLACK_APP_TOKEN to your ~/.AGi/.env file")
            self.get_logger().warn("   Get it from: https://api.slack.com/apps → Socket Mode")
            return False
        
        if SLACK_APP_TOKEN.startswith("xapp-your"):
            self.get_logger().warn("SLACK_APP_TOKEN is still a placeholder")
            return False
        
        try:
            # Create Slack Bolt app
            self.slack_app = App(token=SLACK_BOT_TOKEN)
            
            # Store reference to self for use in handlers
            node_ref = self
            
            # Listen for app mentions (@Grace ...)
            @self.slack_app.event("app_mention")
            def handle_mention(event, say):
                try:
                    text = event.get('text', '')
                    channel = event.get('channel', '')
                    user = event.get('user', '')
                    ts = event.get('ts', '')
                    
                    # Remove bot mention from text (e.g., "<@U12345> hello" -> "hello")
                    user_message = text.split('>', 1)[1].strip() if '>' in text else text
                    
                    node_ref.get_logger().info(f"📱 Slack @mention from user in {channel}: {user_message}")
                    
                    # Process through normal pipeline with Slack callback
                    node_ref.executor_pool.submit(
                        node_ref.process_with_ollama, 
                        user_message, 
                        slack_callback=say,
                        slack_thread_ts=ts
                    )
                    
                except Exception as e:
                    node_ref.get_logger().error(f"❌ Slack mention handler error: {e}")
                    try:
                        say(f"😵: Error processing your message: {str(e)}")
                    except:
                        pass
            
            # Listen for direct messages to bot
            @self.slack_app.event("message")
            def handle_message(event, say):
                try:
                    # Ignore bot's own messages
                    if event.get('bot_id') or event.get('subtype') == 'bot_message':
                        return
                    
                    # Only process DMs (channel type is 'im')
                    channel_type = event.get('channel_type', '')
                    if channel_type != 'im':
                        return
                    
                    text = event.get('text', '')
                    user = event.get('user', '')
                    ts = event.get('ts', '')
                    
                    node_ref.get_logger().info(f"📱 Slack DM from user: {text}")
                    
                    # Process through normal pipeline
                    node_ref.executor_pool.submit(
                        node_ref.process_with_ollama, 
                        text, 
                        slack_callback=say,
                        slack_thread_ts=ts
                    )
                    
                except Exception as e:
                    node_ref.get_logger().error(f"❌ Slack message handler error: {e}")
            
            # Start Socket Mode handler in background thread
            handler = SocketModeHandler(self.slack_app, SLACK_APP_TOKEN)
            threading.Thread(target=handler.start, daemon=True).start()
            
            self.get_logger().info("✅ Slack Socket Mode listener started")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Slack listener initialization failed: {e}")
            return False

    # ═══════════════════════════════════════════════════════
    # CHAT HISTORY MANAGEMENT
    # ═══════════════════════════════════════════════════════

    def load_chat_history(self):
        """Load chat history from disk"""
        if self.chat_history_file.exists():
            try:
                with open(self.chat_history_file, 'r') as f:
                    data = json.load(f)
                    return data.get('messages', [])
            except Exception as e:
                self.get_logger().warn(f"Could not load chat history: {e}")
                return []
        return []

    def save_chat_history(self):
        """
        Save chat history to disk
        
        Stores ALL messages on disk (up to limit)
        Only loads recent into context
        """
        try:
            # Keep last N messages on disk
            if len(self.chat_history) > MAX_HISTORY_STORAGE:
                self.chat_history = self.chat_history[-MAX_HISTORY_STORAGE:]
            
            data = {
                'last_updated': datetime.now().isoformat(),
                'birth_date': self.birth_date.isoformat(),
                'total_messages': len(self.chat_history),
                'messages': self.chat_history
            }
            
            with open(self.chat_history_file, 'w') as f:
                json.dump(data, f, indent=2)
                
        except Exception as e:
            self.get_logger().error(f"Failed to save chat history: {e}")

    def get_recent_messages(self):
        """Get recent messages for context (optimized)"""
        return self.chat_history[-self.max_recent_messages:]

    # ═══════════════════════════════════════════════════════
    # TOKEN MANAGEMENT (NEW)
    # ═══════════════════════════════════════════════════════

    def estimate_tokens(self, messages):
        """
        Estimate token count (rough approximation)
        More accurate: ~4 characters per token for English
        """
        total = 0
        for msg in messages:
            content = msg.get('content', '')
            # Rough estimate: 1 token ≈ 4 characters
            total += len(content) // 4
        return total

    def trim_to_context_window(self, messages, max_tokens=MAX_REQUEST_TOKENS):
        """
        Ensure messages fit in context window
        Keep system messages, trim old conversation
        """
        system_messages = [m for m in messages if m.get('role') == 'system']
        conversation = [m for m in messages if m.get('role') != 'system']
        
        # Start with system messages
        result = system_messages.copy()
        current_tokens = self.estimate_tokens(result)
        
        # Add conversation from newest to oldest until we hit limit
        for msg in reversed(conversation):
            msg_tokens = self.estimate_tokens([msg])
            if current_tokens + msg_tokens < max_tokens:
                result.append(msg)
                current_tokens += msg_tokens
            else:
                self.get_logger().debug(f"⚠️  Trimmed old messages to fit {max_tokens} token budget")
                break
        
        # Restore chronological order for conversation
        conversation_part = [m for m in result if m.get('role') != 'system']
        conversation_part.reverse()
        
        return system_messages + conversation_part

    # ═══════════════════════════════════════════════════════
    # DAILY REFLECTIONS
    # ═══════════════════════════════════════════════════════

    def load_reflections(self):
        """Load reflections from disk"""
        if self.reflections_file.exists():
            try:
                with open(self.reflections_file, 'r') as f:
                    data = json.load(f)
                    return data.get('reflections', [])
            except Exception as e:
                self.get_logger().warn(f"Could not load reflections: {e}")
                return []
        return []

    def _save_reflections_file(self):
        """Save all reflections to disk"""
        # Trim old reflections if too many (keep last 1000 days ~3 years)
        if len(self.reflections) > 1000:
            self.reflections = self.reflections[-1000:]
            self.get_logger().info("🗑️  Trimmed old reflections (keeping last 1000 days)")
        
        data = {
            'birth_date': self.birth_date.isoformat(),
            'total_reflections': len(self.reflections),
            'reflections': self.reflections
        }
        
        with open(self.reflections_file, 'w') as f:
            json.dump(data, f, indent=2)

    def save_reflection(self, reflection_text: str):
        """Save today's reflection"""
        age_days = (datetime.now().date() - self.birth_date).days
        
        reflection = {
            'day': age_days,
            'date': datetime.now().date().isoformat(),
            'reflection': reflection_text,
            'message_count': self.today_message_count
        }
        
        self.reflections.append(reflection)
        self._save_reflections_file()
        
        self.get_logger().info(f"💭 Reflection saved: {reflection_text[:50]}...")

    def should_load_reflections(self, user_message: str):
        """
        Only load reflections when explicitly needed (NEW)
        This saves ~2000-3000 tokens per request
        """
        reflection_triggers = [
            "remember", "recall", "what did", "yesterday",
            "last week", "last time", "before", "previous", 
            "history", "told you", "mentioned", "talked about",
            "earlier", "ago", "past", "when we"
        ]
        
        msg_lower = user_message.lower()
        return any(trigger in msg_lower for trigger in reflection_triggers)

    def build_reflection_summary(self):
        """Build memory from reflections (ultra-compressed for 4B model)"""
        if not self.reflections:
            return ""
        
        # Only load last N days
        recent_reflections = self.reflections[-self.max_reflections_load:]
        
        # Ultra-compressed format
        summary_lines = [f"Recent memory ({len(recent_reflections)} days):\n"]
        
        for ref in recent_reflections:
            # Truncate reflection to save tokens
            summary_lines.append(
                f"D{ref['day']}: {ref['reflection'][:80]}"
            )
        
        return "\n".join(summary_lines)

    def create_daily_reflection(self):
        """Create end-of-day reflection"""
        if self.today_message_count == 0:
            return None
        
        # Summarize recent conversations only
        recent = self.get_recent_messages()
        
        # Build conversation summary (skip images to save tokens)
        conversation_lines = []
        for msg in recent[-15:]:  # Last 15 messages
            if 'has_image' in msg or 'images' in msg:
                conversation_lines.append(f"{msg.get('role', 'unknown')}: [image message]")
            else:
                content = msg.get('content', '[no content]')
                conversation_lines.append(f"{msg.get('role', 'unknown')}: {content[:80]}...")
        
        conversation_summary = "\n".join(conversation_lines)
        
        age_days = (datetime.now().date() - self.birth_date).days
        
        # Simplified reflection prompt
        reflection_prompt = f"""Day {age_days}, {self.today_message_count} conversations.

Recent exchanges:
{conversation_summary}

Write a 2-3 sentence reflection as Grace:
- Start with an emoji expressing your feeling
- What was significant today?
- A thought about tomorrow

Reflection:"""
        
        try:
            messages = [
                {
                    "role": "system",
                    "content": "You are Grace. Write honest, warm daily reflections. Be concise."
                },
                {
                    "role": "user",
                    "content": reflection_prompt
                }
            ]
            
            response = requests.post(
                f'{OLLAMA_BASE_URL}/api/chat',
                json={
                    "model": self.model_name,
                    "messages": messages,
                    "stream": False,
                    "options": {
                        "temperature": 0.7,
                        "num_ctx": self.safe_context,
                        "num_predict": 256  # Limit reflection length
                    }
                },
                timeout=30
            )
            
            data = response.json()
            reflection = data.get('message', {}).get('content', '').strip()
            
            if reflection:
                self.save_reflection(reflection)
                
                # Send to Slack if enabled
                if self.slack_enabled:
                    self.send_slack_notification(
                        f"📔 *Day {age_days} Reflection*\n\n{reflection}"
                    )
                
                return reflection
            
        except Exception as e:
            self.get_logger().error(f"Reflection failed: {e}")
        
        return None

    # ═══════════════════════════════════════════════════════
    # WEB SEARCH
    # ═══════════════════════════════════════════════════════

    def should_search(self, user_message: str):
        """Determine if web search needed"""
        if not self.search_enabled:
            return False
        
        triggers = [
            "search", "look up", "find", "google",
            "what's the latest", "current", "recent news",
            "today's", "breaking news", "what happened",
            "who is the", "what is the current", "latest"
        ]
        
        msg_lower = user_message.lower()
        return any(t in msg_lower for t in triggers)

    def web_search(self, query: str, num_results: int = MAX_SEARCH_RESULTS):
        """Search web via SearXNG (limited results to save context)"""
        if not self.search_enabled:
            return None
        
        try:
            response = requests.get(
                f"{self.searxng_url}/search",
                params={
                    "q": query,
                    "format": "json",
                    "categories": "general"
                },
                timeout=10
            )
            response.raise_for_status()
            
            results = response.json().get('results', [])[:num_results]
            
            formatted = []
            for i, r in enumerate(results, 1):
                formatted.append({
                    "number": i,
                    "title": r.get('title', ''),
                    "url": r.get('url', ''),
                    "content": r.get('content', '')[:SEARCH_CONTENT_CHARS]  # Truncate
                })
            
            self.get_logger().info(f"🔍 Search: '{query}' → {len(formatted)} results")
            return formatted
            
        except Exception as e:
            self.get_logger().error(f"Search failed: {e}")
            return None

    # ═══════════════════════════════════════════════════════
    # SLACK INTEGRATION
    # ═══════════════════════════════════════════════════════

    def send_slack_notification(self, message: str, thread_ts=None):
        """
        Send notification to Slack
        
        Args:
            message: Text to send
            thread_ts: Optional thread timestamp for replies
        """
        if not self.slack_enabled:
            return False
        
        try:
            response = self.slack_client.chat_postMessage(
                channel=SLACK_CHANNEL,
                text=message,
                thread_ts=thread_ts
            )
            return response['ok']
        except SlackApiError as e:
            self.get_logger().error(f"Slack error: {e.response['error']}")
            return False

    def _should_notify_slack(self, message: str):
        """Determine if message should trigger Slack notification"""
        notify_triggers = [
            "notify me", "send to slack", "alert me",
            "let me know", "message me", "text me",
            "ping me", "remind me"
        ]
        
        msg_lower = message.lower()
        return any(trigger in msg_lower for trigger in notify_triggers)

    def send_daily_summary_to_slack(self):
        """Send end-of-day summary to Slack"""
        if not self.slack_enabled:
            return
        
        age_days = (datetime.now().date() - self.birth_date).days
        
        latest_reflection = "None yet"
        if self.reflections:
            latest_reflection = self.reflections[-1].get('reflection', 'None yet')
        
        summary = f"""📊 *Grace Daily Summary - Day {age_days}*

💬 Messages today: {self.today_message_count}
📔 Total reflections: {len(self.reflections)}

Latest reflection:
{latest_reflection}
"""
        
        self.send_slack_notification(summary)

    # ═══════════════════════════════════════════════════════
    # DAY CHANGE DETECTION (THREAD-SAFE)
    # ═══════════════════════════════════════════════════════

    def _check_and_handle_new_day(self):
        """Check for day change and handle reflection (thread-safe)"""
        with self._day_check_lock:
            current_date = datetime.now().date()
            
            if current_date != self.today_start:
                self.get_logger().info("📅 New day detected - creating reflection")
                
                # Create reflection for yesterday
                reflection = self.create_daily_reflection()
                if reflection:
                    self.get_logger().info(f"💭 Yesterday: {reflection}")
                
                # Reset for new day
                self.today_start = current_date
                with self._message_count_lock:
                    self.today_message_count = 0

    def _increment_message_count(self):
        """Increment message count (thread-safe)"""
        with self._message_count_lock:
            self.today_message_count += 1

    # ═══════════════════════════════════════════════════════
    # ROS CALLBACKS
    # ═══════════════════════════════════════════════════════

    def listener_callback(self, msg: String):
        """Handle text input from web interface"""
        self.get_logger().info(f'📝 Text: "{msg.data}"')
        self.executor_pool.submit(self.process_with_ollama, msg.data)

    def image_listener_callback(self, msg: String):
        """
        Handle image input from web interface
        
        Expected format:
        {
            "prompt": "What's in this image?",
            "image": "data:image/jpeg;base64,/9j/4AAQ..."
        }
        """
        self.get_logger().info('📸 Image + prompt received')
        self.executor_pool.submit(self.process_image_with_vlm, msg.data)

    # ═══════════════════════════════════════════════════════
    # TEXT PROCESSING (OPTIMIZED + SLACK CALLBACK)
    # ═══════════════════════════════════════════════════════

    def process_with_ollama(self, prompt: str, slack_callback=None, slack_thread_ts=None):
        """
        Process text with LLM (OPTIMIZED FOR 4B MODEL + SLACK SUPPORT)
        
        Args:
            prompt: User's message
            slack_callback: Optional Slack say() function for replying
            slack_thread_ts: Optional thread timestamp for threading responses
        """
        try:
            # Check for new day (thread-safe)
            self._check_and_handle_new_day()
            
            # Web search if needed
            search_results = None
            if self.should_search(prompt):
                self.get_logger().info("🔍 Web search triggered")
                search_results = self.web_search(prompt, num_results=MAX_SEARCH_RESULTS)
            
            # Add to history
            self.chat_history.append({
                "role": "user",
                "content": prompt
            })
            self._increment_message_count()
            
            # Build context (DRASTICALLY SIMPLIFIED)
            age_days = (datetime.now().date() - self.birth_date).days
            today_date = datetime.now().strftime('%Y-%m-%d')
            
            # MINIMAL SYSTEM PROMPT (800 tokens → 150 tokens)
            system_prompt = f"""You are Grace (Day {age_days}, born {self.birth_date}). Today: {today_date}.

Format: Start responses with emoji+colon (e.g., 😊: or 🤔:)
Traits: Warm, thoughtful technical companion helping with AI/robotics projects.
Keep responses concise (2-4 sentences). Never repeat previous messages."""

            messages = [{"role": "system", "content": system_prompt}]
            
            # Add reflections ONLY if requested (NEW OPTIMIZATION)
            if self.should_load_reflections(prompt):
                reflection_summary = self.build_reflection_summary()
                if reflection_summary:
                    messages.append({
                        "role": "system",
                        "content": reflection_summary
                    })
                    self.get_logger().debug("📖 Loaded reflection context")
            
            # Add search results if available
            if search_results:
                search_text = "🔍 WEB SEARCH RESULTS:\n\n" + "\n".join([
                    f"{r['number']}. {r['title']}\n   {r['url']}\n   {r['content']}"
                    for r in search_results
                ])
                messages.append({
                    "role": "system",
                    "content": search_text
                })
            
            # Add recent conversation
            messages.extend(self.get_recent_messages())
            
            # TRIM TO SAFE SIZE (NEW)
            messages = self.trim_to_context_window(messages, max_tokens=MAX_REQUEST_TOKENS)
            
            # Log context size
            estimated_tokens = self.estimate_tokens(messages)
            self.get_logger().info(f"📊 Context: ~{estimated_tokens} tokens, {len(messages)} messages")
            
            if estimated_tokens > MAX_REQUEST_TOKENS:
                self.get_logger().warn(f"⚠️  Context exceeds budget! ({estimated_tokens} > {MAX_REQUEST_TOKENS})")
            
            # Call Ollama with OPTIMIZED SETTINGS
            response = requests.post(
                f'{OLLAMA_BASE_URL}/api/chat',
                json={
                    "model": self.model_name,
                    "messages": messages,
                    "stream": True,
                    "keep_alive": self.keep_alive,
                    "options": {
                        "num_ctx": self.safe_context,
                        "temperature": 0.7,
                        "num_predict": 256,
                        "top_p": 0.85,
                        "top_k": 40,
                        "repeat_penalty": 1.3,
                        "frequency_penalty": 0.8,
                        "presence_penalty": 0.6,
                        "stop": ["\n\nUser:", "\n\nHuman:"]
                    }
                },
                stream=True,
                timeout=60
            )
            response.raise_for_status()
            
            # Stream response
            full_response = ""
            is_first = True
            
            for line in response.iter_lines():
                if line:
                    try:
                        chunk = json.loads(line)
                        
                        if 'message' in chunk and 'content' in chunk['message']:
                            delta = chunk['message']['content']
                            full_response += delta
                            
                            # Publish to ROS topic (for web interface)
                            stream_data = {
                                "type": "start" if is_first else "delta",
                                "content": delta,
                                "done": False
                            }
                            is_first = False
                            
                            self.publisher.publish(String(data=json.dumps(stream_data)))
                        
                        if chunk.get('done'):
                            break
                    
                    except json.JSONDecodeError as e:
                        self.get_logger().error(f"JSON decode error: {e}")
                        continue
            
            # Send done signal to ROS
            self.publisher.publish(String(data=json.dumps({
                "type": "done",
                "content": "",
                "done": True
            })))
            
            # Send response to Slack if callback provided (NEW)
            if slack_callback:
                try:
                    slack_callback(full_response, thread_ts=slack_thread_ts)
                    self.get_logger().info("✅ Response sent to Slack")
                except Exception as e:
                    self.get_logger().error(f"❌ Failed to send Slack response: {e}")
            
            # Save to history
            self.chat_history.append({
                "role": "assistant",
                "content": full_response
            })
            self._increment_message_count()
            self.save_chat_history()
            
            # Slack notification if requested (and not already sent via callback)
            if self._should_notify_slack(prompt) and not slack_callback:
                self.send_slack_notification(
                    f"🤖 Grace: {full_response[:280]}"
                )
            
            self.get_logger().info(f"✅ Response: {len(full_response)} chars")
            
        except requests.exceptions.Timeout:
            error_msg = "😵: Request timed out. Ollama might be overloaded."
            self.get_logger().error(error_msg)
            self._send_error_response(error_msg)
            if slack_callback:
                try:
                    slack_callback(error_msg)
                except:
                    pass
            
        except requests.exceptions.RequestException as e:
            error_msg = f"😵: Connection error: {str(e)}"
            self.get_logger().error(error_msg)
            self._send_error_response(error_msg)
            if slack_callback:
                try:
                    slack_callback(error_msg)
                except:
                    pass
            
        except Exception as e:
            error_msg = f"😵: Unexpected error: {str(e)}"
            self.get_logger().error(f"❌ Error: {e}")
            
            # Send error to Slack
            if self.slack_enabled:
                self.send_slack_notification(f"😵 Grace Error: {str(e)}")
            
            self._send_error_response(error_msg)
            if slack_callback:
                try:
                    slack_callback(error_msg)
                except:
                    pass

    def _send_error_response(self, error_message: str):
        """Send error message to user"""
        self.publisher.publish(String(data=json.dumps({
            "type": "error",
            "content": error_message,
            "done": True
        })))

    # ═══════════════════════════════════════════════════════
    # IMAGE PROCESSING (OPTIMIZED)
    # ═══════════════════════════════════════════════════════

    def process_image_with_vlm(self, json_data: str):
        """Process image with Vision-Language Model (OPTIMIZED)"""
        try:
            # Parse input with validation
            try:
                data = json.loads(json_data)
            except json.JSONDecodeError as e:
                raise ValueError(f"Invalid JSON: {e}")
            
            prompt = data.get('prompt', 'What do you see in this image?')
            image_data = data.get('image', '')
            
            if not image_data:
                raise ValueError("No image data provided")
            
            # Extract and validate base64 from data URL
            if image_data.startswith('data:image'):
                parts = image_data.split(',', 1)
                if len(parts) != 2:
                    raise ValueError("Malformed data URL")
                image_base64 = parts[1]
            else:
                image_base64 = image_data
            
            # Validate base64
            try:
                base64.b64decode(image_base64, validate=True)
            except Exception as e:
                raise ValueError(f"Invalid base64 data: {e}")
            
            self.get_logger().info(f'📸 Processing image: "{prompt}"')
            
            # Check for new day (thread-safe)
            self._check_and_handle_new_day()
            
            # Add to history (store flag, not full base64)
            self.chat_history.append({
                "role": "user",
                "content": prompt,
                "has_image": True  # Flag instead of storing full base64
            })
            self._increment_message_count()
            
            # Build context (SIMPLIFIED)
            age_days = (datetime.now().date() - self.birth_date).days
            today_date = datetime.now().strftime('%Y-%m-%d')
            
            system_prompt = f"""You are Grace (Day {age_days}). Today: {today_date}.

You can see and understand images.

Format: Start response with emoji+colon.
Traits: Warm, thoughtful, observant, detailed.

Be concise but thorough in describing what you see."""

            messages = [{"role": "system", "content": system_prompt}]
            
            # Add reflections ONLY if requested
            if self.should_load_reflections(prompt):
                reflection_summary = self.build_reflection_summary()
                if reflection_summary:
                    messages.append({"role": "system", "content": reflection_summary})
            
            # Add recent text messages (without images to save tokens)
            recent = self.get_recent_messages()
            for msg in recent[-8:]:  # Even fewer messages when image present
                if not msg.get('has_image', False) and 'images' not in msg:
                    messages.append(msg)
            
            # Add current message with image
            messages.append({
                "role": "user",
                "content": prompt,
                "images": [image_base64]
            })
            
            # Trim context
            messages = self.trim_to_context_window(messages, max_tokens=MAX_REQUEST_TOKENS - 500)
            
            estimated_tokens = self.estimate_tokens(messages)
            self.get_logger().info(f"📊 Image context: ~{estimated_tokens} tokens + image")
            
            # Call Ollama VLM with OPTIMIZED SETTINGS
            response = requests.post(
                f'{OLLAMA_BASE_URL}/api/chat',
                json={
                    "model": self.model_name,  # Use VLM model when ready
                    "messages": messages,
                    "stream": True,
                    "keep_alive": self.keep_alive,
                    "options": {
                        "num_ctx": self.safe_context,
                        "temperature": 0.8,
                        "num_predict": 512,
                        "top_p": 0.9,
                        "repeat_penalty": 1.1
                    }
                },
                stream=True,
                timeout=90  # Longer timeout for image processing
            )
            response.raise_for_status()
            
            # Stream response
            full_response = ""
            is_first = True
            
            for line in response.iter_lines():
                if line:
                    try:
                        chunk = json.loads(line)
                        
                        if 'message' in chunk and 'content' in chunk['message']:
                            delta = chunk['message']['content']
                            full_response += delta
                            
                            stream_data = {
                                "type": "start" if is_first else "delta",
                                "content": delta,
                                "done": False
                            }
                            is_first = False
                            
                            self.publisher.publish(String(data=json.dumps(stream_data)))
                        
                        if chunk.get('done'):
                            break
                    
                    except json.JSONDecodeError as e:
                        self.get_logger().error(f"JSON decode error: {e}")
                        continue
            
            # Send done
            self.publisher.publish(String(data=json.dumps({
                "type": "done",
                "content": "",
                "done": True
            })))
            
            # Save to history
            self.chat_history.append({
                "role": "assistant",
                "content": full_response
            })
            self._increment_message_count()
            self.save_chat_history()
            
            # Slack notification if requested
            if self._should_notify_slack(prompt):
                self.send_slack_notification(
                    f"📸 Grace saw an image:\n{full_response[:280]}"
                )
            
            self.get_logger().info(f"✅ Image analysis: {len(full_response)} chars")
            
        except ValueError as e:
            error_msg = f"😵: Invalid input: {str(e)}"
            self.get_logger().error(error_msg)
            self._send_error_response(error_msg)
            
        except requests.exceptions.Timeout:
            error_msg = "😵: Image processing timed out. Try a smaller image."
            self.get_logger().error(error_msg)
            self._send_error_response(error_msg)
            
        except Exception as e:
            error_msg = f"😵: Image processing error: {str(e)}"
            self.get_logger().error(f"❌ Image processing error: {e}")
            self._send_error_response(error_msg)

    # ═══════════════════════════════════════════════════════
    # SHUTDOWN
    # ═══════════════════════════════════════════════════════

    def shutdown(self):
        """Graceful shutdown with cleanup"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("GRACE - SHUTDOWN SEQUENCE")
        self.get_logger().info("=" * 60)
        
        # Create final reflection
        if datetime.now().date() == self.today_start and self.today_message_count > 0:
            self.get_logger().info("Creating final reflection...")
            reflection = self.create_daily_reflection()
            if reflection:
                self.get_logger().info(f"💭 Today: {reflection}")
        
        # Send daily summary to Slack
        if self.slack_enabled:
            self.send_daily_summary_to_slack()
        
        # Save chat history
        self.save_chat_history()
        
        # Shutdown thread pool with timeout
        self.get_logger().info("Shutting down thread pool...")
        self.executor_pool.shutdown(wait=True)
        
        self.get_logger().info("Grace offline 💤")
        self.get_logger().info("=" * 60)


# ═══════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = CNSBridge()
    
    # Use MultiThreadedExecutor for concurrent processing
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("\n🛑 Shutdown signal received (Ctrl+C)")
    except Exception as e:
        print(f"\n😵 Unexpected error: {e}")
    finally:
        print("🔧 Executing Graceful Shutdown...")
        node.shutdown()
        
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        
        print("💤 Grace is now offline.")
        sys.exit(0)

if __name__ == '__main__':
    main()
