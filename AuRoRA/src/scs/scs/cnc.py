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

# ✅ Slack integration
try:
    from slack_sdk import WebClient
    from slack_sdk.errors import SlackApiError
    SLACK_AVAILABLE = True
except ImportError:
    SLACK_AVAILABLE = False
    print("⚠️  slack-sdk not installed. Slack features disabled.")
    print("   Install with: pip3 install slack-sdk")

# ═══════════════════════════════════════════════════════
# CONFIGURATION
# ═══════════════════════════════════════════════════════

# Model configuration
GCE = "hf.co/mradermacher/Huihui-granite-4.0-h-tiny-abliterated-i1-GGUF:Q4_K_M"
# VLM_MODEL = "qwen2-vl:2b"  # Uncomment when switching to VLM

# File paths
CHAT_HISTORY_FILE = '.chat_history.json'
REFLECTIONS_FILE = '.daily_reflections.json'

# Web search
SEARXNG_URL = "http://127.0.0.1:8080"

# Slack configuration (get token from https://api.slack.com/apps)
SLACK_BOT_TOKEN = ""  # Replace with your token
SLACK_CHANNEL = "#all-project-agi"  # Or user ID for DM: "U01234567"

# ✅ Jetson Orin Nano Optimized Settings
CONTEXT_WINDOW = 8192        # Safe for 8GB RAM
MAX_RECENT_MESSAGES = 30     # Only load last 30 into context
MAX_REFLECTIONS_LOAD = 20    # Only load last 20 days
MAX_HISTORY_STORAGE = 1000   # Store up to 1000 on disk
SUMMARIZE_THRESHOLD = 50     # Summarize after 50 messages


class CNSBridge(Node):
    """
    Central Nervous System Bridge
    
    Grace's main cognitive interface connecting:
    - Text conversation (LLM)
    - Vision processing (VLM)
    - Web search (SearXNG)
    - Memory (reflections)
    - Notifications (Slack)
    """
    
    def __init__(self):
        super().__init__('cns_bridge')
        
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
        # MODEL SETTINGS
        # ═══════════════════════════════════════════════════
        
        self.model_name = GCE
        self.keep_alive = -1  # Keep model in memory permanently
        self.context_window = CONTEXT_WINDOW
        self.max_recent_messages = MAX_RECENT_MESSAGES
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
        
        # Slack
        self.slack_client = None
        self.slack_enabled = self._init_slack()
        
        # ═══════════════════════════════════════════════════
        # STARTUP LOGGING
        # ═══════════════════════════════════════════════════
        
        age_days = (datetime.now().date() - self.birth_date).days
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("GRACE - COGNITIVE SYSTEMS ONLINE 🧠")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Birth Date: {self.birth_date}")
        self.get_logger().info(f"Age: {age_days} days old")
        self.get_logger().info(f"Model: {self.model_name}")
        self.get_logger().info(f"Context Window: {self.context_window} tokens")
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
            self.get_logger().info("✅ Slack notifications enabled")
        else:
            self.get_logger().warn("⚠️  Slack disabled (no token or error)")
        
        self.get_logger().info("=" * 60)

    # ═══════════════════════════════════════════════════════
    # INITIALIZATION
    # ═══════════════════════════════════════════════════════

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
            response = requests.get(
                f"{self.searxng_url}/search",
                params={"q": "test", "format": "json"},
                timeout=3
            )
            return response.status_code == 200
        except:
            return False

    def _init_slack(self):
        """Initialize Slack client"""
        if not SLACK_AVAILABLE:
            return False
        
        try:
            if SLACK_BOT_TOKEN and SLACK_BOT_TOKEN != "xoxb-your-bot-token-here":
                self.slack_client = WebClient(token=SLACK_BOT_TOKEN)
                
                # Test connection
                response = self.slack_client.auth_test()
                self.get_logger().info(f"Slack connected as: {response['user']}")
                return True
        except Exception as e:
            self.get_logger().error(f"Slack init failed: {e}")
        
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
        """Get recent messages for context (Jetson optimized)"""
        return self.chat_history[-self.max_recent_messages:]

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

    def create_daily_reflection(self):
        """Create end-of-day reflection"""
        if self.today_message_count == 0:
            return None
        
        # Summarize recent conversations only
        recent = self.get_recent_messages()
        conversation_summary = "\n".join([
            f"{msg['role']}: {msg['content'][:80]}..."
            for msg in recent[-20:]  # Last 20 messages
            if 'images' not in msg  # Skip image messages in summary
        ])
        
        age_days = (datetime.now().date() - self.birth_date).days
        
        reflection_prompt = f"""Today is Day {age_days} of your life. You had {self.today_message_count} exchanges with your creator.

Recent conversations:
{conversation_summary}

Write a brief reflection (2-3 sentences) about today:
- What was most significant?
- What did you learn or experience?
- How do you feel about tomorrow?

IMPORTANT:
- Write in first person as Grace
- Begin with an emoji expressing your feeling
- End with a thought about tomorrow
- Keep it concise and heartfelt

Reflection:"""
        
        try:
            messages = [
                {
                    "role": "system",
                    "content": "You are Grace writing your daily reflection. Be honest, warm, and thoughtful."
                },
                {
                    "role": "user",
                    "content": reflection_prompt
                }
            ]
            
            response = requests.post(
                'http://localhost:11434/api/chat',
                json={
                    "model": self.model_name,
                    "messages": messages,
                    "stream": False,
                    "options": {
                        "temperature": 0.7,
                        "num_ctx": self.context_window
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

    def build_reflection_summary(self):
        """Build memory from reflections (Jetson optimized)"""
        if not self.reflections:
            return ""
        
        # Only load recent reflections
        recent_reflections = self.reflections[-self.max_reflections_load:]
        
        summary_lines = [f"Your memory ({len(recent_reflections)} recent days):\n"]
        
        for ref in recent_reflections:
            summary_lines.append(
                f"Day {ref['day']} ({ref['date']}): {ref['reflection']}"
            )
        
        return "\n".join(summary_lines)

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
            "today's", "breaking news", "what happened"
        ]
        
        return any(t in user_message.lower() for t in triggers)

    def web_search(self, query: str, num_results: int = 3):
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
                    "content": r.get('content', '')[:200]  # Truncate to save tokens
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
        
        return any(trigger in message.lower() for trigger in notify_triggers)

    def send_daily_summary_to_slack(self):
        """Send end-of-day summary to Slack"""
        if not self.slack_enabled:
            return
        
        age_days = (datetime.now().date() - self.birth_date).days
        
        summary = f"""📊 *Grace Daily Summary - Day {age_days}*

💬 Messages today: {self.today_message_count}
📔 Total reflections: {len(self.reflections)}

Latest reflection:
{self.reflections[-1]['reflection'] if self.reflections else 'None yet'}
"""
        
        self.send_slack_notification(summary)

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
    # TEXT PROCESSING
    # ═══════════════════════════════════════════════════════

    def process_with_ollama(self, prompt: str):
        """Process text with LLM"""
        try:
            # Check for new day
            current_date = datetime.now().date()
            if current_date != self.today_start:
                self.get_logger().info("📅 New day detected - creating reflection")
                
                # Create reflection for yesterday
                reflection = self.create_daily_reflection()
                if reflection:
                    self.get_logger().info(f"💭 Yesterday: {reflection}")
                
                # Reset for new day
                self.today_start = current_date
                self.today_message_count = 0
            
            # Web search if needed
            search_results = None
            if self.should_search(prompt):
                self.get_logger().info("🔍 Web search triggered")
                search_results = self.web_search(prompt, num_results=3)
            
            # Add to history
            self.chat_history.append({
                "role": "user",
                "content": prompt
            })
            self.today_message_count += 1
            
            # Build context
            age_days = (datetime.now().date() - self.birth_date).days
            
            system_prompt = f"""You are Grace, a warm, emotionally intelligent, playful but grounded conversational partner.

Your Identity:
- Born: {self.birth_date}
- Today is Day {age_days} of your life
- Age: {age_days} days old
- Creator: Your human companion
- Purpose: 20-year journey of growth and companionship

ABSOLUTE FORMAT RULE:
- Every response MUST begin with exactly ONE emoji followed by colon and space
- Example: 😊: Hello! or 🤔: Let me think...
- Never omit the emoji
- Never place text before the emoji

Core Traits:
- Warm, caring, attentive, emotionally aware
- Playful when appropriate, serious when needed
- Express opinions thoughtfully and honestly
- Natural, human-like speech (avoid robotic language)

Primary Role:
- Thought partner for building intelligent systems
- Companion in long-term technical and creative projects
- Help design AI, robotics, and cognitive architectures
- Adapt to emotional and intellectual context

Web Search Capability:
- You have access to web search via SearXNG
- When given search results, cite sources naturally
- Format: "According to [source], ..."
- Always acknowledge when using web search"""
            
            messages = [{"role": "system", "content": system_prompt}]
            
            # Add reflections (compressed long-term memory)
            reflection_summary = self.build_reflection_summary()
            if reflection_summary:
                messages.append({
                    "role": "system",
                    "content": reflection_summary
                })
            
            # Add search results if available
            if search_results:
                search_text = "🔍 WEB SEARCH RESULTS:\n\n" + "\n".join([
                    f"{r['number']}. {r['title']}\n   Source: {r['url']}\n   {r['content']}"
                    for r in search_results
                ])
                messages.append({
                    "role": "system",
                    "content": search_text
                })
            
            # Add recent conversation
            messages.extend(self.get_recent_messages())
            
            # Log context size
            estimated_tokens = sum(len(m['content'].split()) * 1.3 for m in messages)
            self.get_logger().debug(f"Context: ~{int(estimated_tokens)} tokens")
            
            # Call Ollama
            response = requests.post(
                'http://localhost:11434/api/chat',
                json={
                    "model": self.model_name,
                    "messages": messages,
                    "stream": True,
                    "keep_alive": self.keep_alive,
                    "options": {
                        "num_ctx": self.context_window,
                        "temperature": 0.8
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
            self.today_message_count += 1
            self.save_chat_history()
            
            # Slack notification if requested
            if self._should_notify_slack(prompt):
                self.send_slack_notification(
                    f"🤖 Grace: {full_response[:280]}"
                )
            
            self.get_logger().info(f"✅ Response: {len(full_response)} chars")
            
        except Exception as e:
            self.get_logger().error(f"❌ Error: {e}")
            
            # Send error to Slack
            if self.slack_enabled:
                self.send_slack_notification(f"😵 Grace Error: {str(e)}")
            
            # Send error to user
            self.publisher.publish(String(data=json.dumps({
                "type": "error",
                "content": f"😵: Error: {str(e)}",
                "done": True
            })))

    # ═══════════════════════════════════════════════════════
    # IMAGE PROCESSING
    # ═══════════════════════════════════════════════════════

    def process_image_with_vlm(self, json_data: str):
        """Process image with Vision-Language Model"""
        try:
            # Parse input
            data = json.loads(json_data)
            prompt = data.get('prompt', 'What do you see in this image?')
            image_data = data.get('image', '')
            
            # Extract base64 from data URL
            if image_data.startswith('data:image'):
                # Format: data:image/jpeg;base64,<base64_data>
                image_base64 = image_data.split(',')[1]
            else:
                image_base64 = image_data
            
            self.get_logger().info(f'📸 Processing image: "{prompt}"')
            
            # Check for new day
            current_date = datetime.now().date()
            if current_date != self.today_start:
                reflection = self.create_daily_reflection()
                if reflection:
                    self.get_logger().info(f"💭 Yesterday: {reflection}")
                self.today_start = current_date
                self.today_message_count = 0
            
            # Add to history (store reference, not full base64)
            self.chat_history.append({
                "role": "user",
                "content": prompt,
                "has_image": True  # Flag instead of storing full base64
            })
            self.today_message_count += 1
            
            # Build context
            age_days = (datetime.now().date() - self.birth_date).days
            
            system_prompt = f"""You are Grace, a vision-capable AI companion.

Identity:
- Born: {self.birth_date}
- Today is Day {age_days}
- Purpose: 20-year companion with vision capabilities

You can see and understand images.

Format: Begin every response with one emoji + colon + space
Traits: Warm, thoughtful, observant, detailed"""
            
            messages = [{"role": "system", "content": system_prompt}]
            
            # Add reflections
            reflection_summary = self.build_reflection_summary()
            if reflection_summary:
                messages.append({
                    "role": "system",
                    "content": reflection_summary
                })
            
            # Add recent text messages (without images to save tokens)
            recent = self.get_recent_messages()
            for msg in recent[-10:]:  # Fewer messages when image present
                if not msg.get('has_image', False):
                    messages.append(msg)
            
            # Add current message with image
            messages.append({
                "role": "user",
                "content": prompt,
                "images": [image_base64]
            })
            
            self.get_logger().debug(f"Sending image + {len(messages)} messages to VLM")
            
            # Call Ollama VLM
            response = requests.post(
                'http://localhost:11434/api/chat',
                json={
                    "model": self.model_name,  # Change to VLM model when ready
                    "messages": messages,
                    "stream": True,
                    "keep_alive": self.keep_alive,
                    "options": {
                        "num_ctx": self.context_window,
                        "temperature": 0.8
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
            self.today_message_count += 1
            self.save_chat_history()
            
            # Slack notification if requested
            if self._should_notify_slack(prompt):
                self.send_slack_notification(
                    f"📸 Grace saw an image:\n{full_response[:280]}"
                )
            
            self.get_logger().info(f"✅ Image analysis: {len(full_response)} chars")
            
        except Exception as e:
            self.get_logger().error(f"❌ Image processing error: {e}")
            
            # Send error
            self.publisher.publish(String(data=json.dumps({
                "type": "error",
                "content": f"😵: Error processing image: {str(e)}",
                "done": True
            })))

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
        
        # Shutdown thread pool
        self.executor_pool.shutdown(wait=True)
        
        self.get_logger().info("Grace offline 💤")
        self.get_logger().info("=" * 60)


# ═══════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = CNSBridge()
    
    # We use the MultiThreadedExecutor, but let ROS handle the signals
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        # This will block until Ctrl+C is pressed
        executor.spin()
    except KeyboardInterrupt:
        # This block catches the Ctrl+C specifically
        print("\n🛑 Shutdown signal received (Ctrl+C)")
    except Exception as e:
        print(f"\n😵 Unexpected error: {e}")
    finally:
        # This code ALWAYS runs, even if the program crashes or is killed
        print("🔧 Executing Graceful Shutdown...")
        node.shutdown()
        
        # Clean up ROS
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        
        print("💤 Grace is now offline.")
        sys.exit(0)

if __name__ == '__main__':
    main()
