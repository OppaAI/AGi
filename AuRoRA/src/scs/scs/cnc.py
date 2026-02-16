import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

import requests
from concurrent.futures import ThreadPoolExecutor, Future
import json
from pathlib import Path
from datetime import datetime, date, timedelta
import sys
import base64
import os
import threading
import time
import subprocess

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

# ✅ RLHF System (NEW - REINFORCEMENT LEARNING FROM HUMAN FEEDBACK) - FIXED IMPORT
try:
    from scs.rlhf_llm import get_rlhf  # FIXED: Removed scs. prefix
    RLHF_AVAILABLE = True
except ImportError:
    RLHF_AVAILABLE = False
    print("⚠️  rlhf_llm.py not found. RLHF disabled.")
    print("   Place rlhf_llm.py in the same directory as cnc.py")

# ✅ Telegram integration (NEW - TWO-WAY COMMUNICATION)
try:
    from telegram import Update
    from telegram.ext import Application, CommandHandler, MessageHandler, filters, ContextTypes
    TELEGRAM_AVAILABLE = True
except ImportError:
    TELEGRAM_AVAILABLE = False
    print("⚠️  python-telegram-bot not installed. Telegram features disabled.")
    print("   Install with: pip3 install python-telegram-bot")

# ═══════════════════════════════════════════════════════
# CONFIGURATION
# ═══════════════════════════════════════════════════════

# Model configuration
GCE = "PetrosStav/gemma3-tools:12b"
# VLM_MODEL = "qwen2-vl:2b"  # Uncomment when switching to VLM

# File paths
CHAT_HISTORY_FILE = '.chat_history.json'
REFLECTIONS_FILE = '.daily_reflections.json'

# Ollama configuration
OLLAMA_BASE_URL = "http://AIVA:11434"

# Web search
SEARXNG_URL = "http://127.0.0.1:8080"

# Slack configuration (loaded from .env file or environment)
SLACK_BOT_TOKEN = os.getenv('SLACK_BOT_TOKEN', '')
SLACK_APP_TOKEN = os.getenv('SLACK_APP_TOKEN', '')  # NEW: For Socket Mode (two-way)
SLACK_CHANNEL = os.getenv('SLACK_CHANNEL', '#grace-logs')

# Telegram configuration (loaded from .env file or environment)
TELEGRAM_BOT_TOKEN = os.getenv('TELEGRAM_BOT_TOKEN', '')  # From @BotFather
TELEGRAM_CHAT_ID = os.getenv('TELEGRAM_CHAT_ID', '')  # Optional: For notifications

# MCP Server Configuration
USE_MCP_SEARCH = True
MCP_SERVER_COMMAND = "/home/oppa-ai/AGi/mcp_server/searxng-mcp-server/searxng-mcp-server.js"

# ✅ Optimized Settings for 30B MODEL
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

# Memory limits (OPTIMIZED FOR 30B)
MAX_RECENT_MESSAGES = 30      # Full capacity for 30B
MAX_REFLECTIONS_LOAD = 20     # Better long-term memory
MAX_HISTORY_STORAGE = 1000    # Store up to 1000 on disk
SUMMARIZE_THRESHOLD = 50      # Summarize after 50 messages

# Token budget per request (FOR 30B MODEL)
MAX_REQUEST_TOKENS = 12000     # Use more of the 8192 context

# Search result limits
MAX_SEARCH_RESULTS = 10       # More search results
SEARCH_CONTENT_CHARS = 1000   # More detail per result

# ✅ DUAL HYBRID SEARCH (NEW)
ENABLE_AUTO_SEARCH = True     # Enable LLM-based auto-search detection

class MCPClient:
    """Client for Grace's custom MCP SearXNG server"""
    
    def __init__(self, server_command: str, logger):
        self.server_command = server_command
        self.logger = logger
        self.process = None

    def start(self):
        """Start MCP server process"""
        try:
            self.process = subprocess.Popen(
                [self.server_command],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1
            )
            
            # Wait a moment for server to start
            import time
            time.sleep(1)
            
            if self.process.poll() is None:
                self.logger.info("✅ MCP server started")
                return True
            else:
                self.logger.error(f"❌ MCP server failed to start")
                return False
                
        except FileNotFoundError:
            self.logger.error(f"❌ MCP server not found: {self.server_command}")
            self.logger.error("   Install it: cd ~/AGi/mcp_server/searxng-mcp-server && sudo npm link")
            return False
        except Exception as e:
            self.logger.error(f"❌ MCP server start failed: {e}")
            return False
    
    def search_web(self, query: str, num_results: int = 10) -> list:
        """
        Search web with full content extraction
        
        Returns list of results with full_content field
        """
        if not self.process or self.process.poll() is not None:
            return []
        
        try:
            request = {
                "jsonrpc": "2.0",
                "id": 1,
                "method": "tools/call",
                "params": {
                    "name": "search_web",
                    "arguments": {
                        "query": query,
                        "num_results": min(num_results, 20),
                        "fetch_content": True
                    }
                }
            }
            
            # Send request
            self.process.stdin.write(json.dumps(request) + '\n')
            self.process.stdin.flush()
            
            # Read response (with timeout)
            import select
            
            timeout = 30
            start_time = time.time()
            response_line = ""
            
            while time.time() - start_time < timeout:
                # Check if data available
                if select.select([self.process.stdout], [], [], 1)[0]:
                    line = self.process.stdout.readline()
                    if line:
                        try:
                            response = json.loads(line)
                            if response.get('id') == 1:
                                return self._parse_response(response)
                        except json.JSONDecodeError:
                            # Skip non-JSON lines (debug output)
                            continue
            
            self.logger.error("MCP search timeout")
            return []
            
        except Exception as e:
            self.logger.error(f"MCP search error: {e}")
            return []
    
    def _parse_response(self, response: dict) -> list:
        """Parse MCP response into results list (FIXED)"""
        try:
            if 'result' not in response:
                self.logger.error("No 'result' in MCP response")
                return []
            
            content = response['result'].get('content', [])
            if not content:
                self.logger.error("No 'content' in MCP result")
                return []
            
            text = content[0].get('text', '')
            if not text:
                self.logger.error("Empty text in MCP content")
                return []
            
            # Parse the markdown-formatted results
            results = []
            current_result = {}
            in_content = False
            content_lines = []
            
            for line in text.split('\n'):
                line = line.strip()
                
                # New result starts with ## [number]
                if line.startswith('## [') and ']' in line:
                    # Save previous result
                    if current_result and 'title' in current_result:
                        if content_lines:
                            current_result['full_content'] = '\n'.join(content_lines)
                            current_result['has_full_content'] = True
                        else:
                            current_result['full_content'] = ""
                            current_result['has_full_content'] = False
                        
                        # Ensure all required fields exist
                        current_result.setdefault('url', '')
                        current_result.setdefault('snippet', '')
                        current_result.setdefault('engine', 'unknown')
                        
                        results.append(current_result)
                    
                    # Start new result
                    parts = line.split(']', 1)
                    number = parts[0].replace('## [', '').strip()
                    title = parts[1].strip() if len(parts) > 1 else 'No title'
                    current_result = {
                        'number': len(results) + 1,
                        'title': title,
                        'url': '',
                        'snippet': '',
                        'engine': 'unknown',
                        'full_content': '',
                        'has_full_content': False
                    }
                    in_content = False
                    content_lines = []
                
                elif line.startswith('**URL:**'):
                    current_result['url'] = line.replace('**URL:**', '').strip()
                
                elif line.startswith('**Snippet:**'):
                    current_result['snippet'] = line.replace('**Snippet:**', '').strip()
                
                elif line.startswith('**Source:**'):
                    current_result['engine'] = line.replace('**Source:**', '').strip()
                
                elif line.startswith('**Full Content:**'):
                    in_content = True
                
                elif line == '---':
                    in_content = False
                
                elif in_content and line:
                    content_lines.append(line)
            
            # Don't forget last result
            if current_result and 'title' in current_result:
                if content_lines:
                    current_result['full_content'] = '\n'.join(content_lines)
                    current_result['has_full_content'] = True
                else:
                    current_result['full_content'] = ""
                    current_result['has_full_content'] = False
                
                # Ensure all required fields
                current_result.setdefault('url', '')
                current_result.setdefault('snippet', '')
                current_result.setdefault('engine', 'unknown')
                
                results.append(current_result)
            
            self.logger.info(f"Parsed {len(results)} results from MCP response")
            return results
            
        except Exception as e:
            self.logger.error(f"Failed to parse MCP response: {e}")
            import traceback
            self.logger.error(traceback.format_exc())
            return []
    
    def stop(self):
        """Stop MCP server"""
        if self.process:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except:
                self.process.kill()
            self.logger.info("🛑 MCP server stopped")

class NatureSkillsClient:
    """
    Client for Nature Exploration Skills MCP Server
    Uses native Ollama tool calling
    """
    
    def __init__(self, server_path: str, logger):
        self.server_path = server_path
        self.logger = logger
        self.process = None
        self.tools_schema = []
    
    def start(self) -> bool:
        """Start skills MCP server"""
        try:
            self.process = subprocess.Popen(
                ["python3", self.server_path],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1
            )
            
            time.sleep(1)
            
            if self.process.poll() is None:
                # Get list of tools
                self._fetch_tools()
                self.logger.info(f"✅ Nature skills server started ({len(self.tools_schema)} tools)")
                return True
            else:
                self.logger.error("❌ Skills server failed to start")
                return False
                
        except Exception as e:
            self.logger.error(f"❌ Skills server error: {e}")
            return False
    
    def _fetch_tools(self):
        """Fetch available tools from MCP server"""
        try:
            request = {
                "jsonrpc": "2.0",
                "id": 1,
                "method": "tools/list"
            }
            
            self.process.stdin.write(json.dumps(request) + '\n')
            self.process.stdin.flush()
            
            # Read response
            import select
            if select.select([self.process.stdout], [], [], 5)[0]:
                line = self.process.stdout.readline()
                response = json.loads(line)
                
                if 'result' in response:
                    tools = response['result'].get('tools', [])
                    # Convert MCP tools to Ollama format
                    self.tools_schema = [
                        {
                            "type": "function",
                            "function": {
                                "name": tool['name'],
                                "description": tool['description'],
                                "parameters": tool['inputSchema']
                            }
                        }
                        for tool in tools
                    ]
                    self.logger.info(f"Loaded {len(self.tools_schema)} tools")
        except Exception as e:
            self.logger.error(f"Failed to fetch tools: {e}")
    
    def call_tool(self, name: str, arguments: dict) -> str:
        """Call a tool and get result"""
        try:
            request = {
                "jsonrpc": "2.0",
                "id": 2,
                "method": "tools/call",
                "params": {
                    "name": name,
                    "arguments": arguments
                }
            }
            
            self.process.stdin.write(json.dumps(request) + '\n')
            self.process.stdin.flush()
            
            # Read response with timeout
            import select
            if select.select([self.process.stdout], [], [], 10)[0]:
                line = self.process.stdout.readline()
                response = json.loads(line)
                
                if 'result' in response:
                    content = response['result'].get('content', [])
                    if content:
                        return content[0].get('text', 'No result')
            
            return "Tool execution timeout"
            
        except Exception as e:
            self.logger.error(f"Tool call error: {e}")
            return f"Error: {str(e)}"
    
    def get_tools_for_ollama(self) -> list:
        """Get tools in Ollama format"""
        return self.tools_schema
    
    def stop(self):
        """Stop skills server"""
        if self.process:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except:
                self.process.kill()
            self.logger.info("🛑 Skills server stopped")

class CNSBridge(Node):
    """
    Central Nervous System Bridge (OPTIMIZED + TWO-WAY COMMUNICATION + IMAGE SUPPORT)
    
    Grace's main cognitive interface connecting:
    - Text conversation (LLM)
    - Vision processing (VLM) - FIXED TELEGRAM SUPPORT
    - DUAL HYBRID WEB SEARCH (Keyword + LLM auto-detection)
    - Memory (reflections)
    - Two-way Slack & Telegram integration (text + images)
    
    Optimizations:
    - Auto-sized context window (30B → 8192 tokens)
    - Minimal system prompts
    - Lazy loading of reflections
    - Automatic context trimming
    - Thread-safe state management
    
    NEW:
    - Two-way Slack communication via Socket Mode
    - Two-way Telegram bot (text + PHOTOS)
    - Dual hybrid web search (keywords + intelligent auto-detection)
    - RLHF feedback learning
    """
    
    def __init__(self):
        super().__init__('cns_bridge')
        # MCP Search Client (NEW)
        self.mcp_client = None
        self.use_mcp = USE_MCP_SEARCH
        if self.use_mcp:
            self.mcp_client = MCPClient(MCP_SERVER_COMMAND, self.get_logger())
            if self.mcp_client.start():
                self.get_logger().info("✅ MCP search enabled (full content)")
            else:
                self.get_logger().warn("⚠️  MCP start failed, using API fallback")
                self.use_mcp = False

        # Nature Skills Server (NEW)
        self.skills_client = None
        self.use_skills = True
        if self.use_skills:
            skills_server_path = str(Path.home() / "AGi" / "mcp_server" / "skills_server" / "skills_server.py")
            self.skills_client = NatureSkillsClient(skills_server_path, self.get_logger())
            if self.skills_client.start():
                self.get_logger().info("✅ Nature exploration skills enabled")
            else:
                self.get_logger().warn("⚠️  Skills server failed, disabling skills")
                self.use_skills = False                
                
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
        
        # Telegram (two-way communication) - NEW!
        self.telegram_app = None
        self.telegram_enabled = self._init_telegram()
        
        # ═══════════════════════════════════════════════════
        # RLHF SYSTEM (NEW - LEARNING FROM FEEDBACK)
        # ═══════════════════════════════════════════════════
        
        # Initialize RLHF
        self.rlhf = None
        self.rlhf_enabled = False
        self.current_response_id = None
        
        if RLHF_AVAILABLE:
            try:
                self.rlhf = get_rlhf()
                self.rlhf_enabled = True
                
                # Feedback topic (receives pet/red X from web interface)
                self.feedback_subscription = self.create_subscription(
                    String,
                    '/cns/feedback',
                    self.feedback_callback,
                    10
                )
                
                self.get_logger().info("✅ RLHF feedback system initialized")
            except Exception as e:
                self.get_logger().error(f"❌ RLHF initialization failed: {e}")
                self.rlhf_enabled = False
        
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
        self.get_logger().info("✅ Image processing enabled (VLM ready + Telegram photos)")
        
        if self.search_enabled:
            self.get_logger().info("✅ Web search enabled (DUAL HYBRID MODE)")
            self.get_logger().info("   → Method 1: Keyword triggers")
            self.get_logger().info(f"   → Method 2: LLM auto-detection ({'ON' if ENABLE_AUTO_SEARCH else 'OFF'})")
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
        
        if self.telegram_enabled:
            self.get_logger().info("✅ Telegram bot enabled (TWO-WAY) 💬↔️🤖📸")
            self.get_logger().info("   → Listening for messages, commands, and PHOTOS")
        else:
            self.get_logger().warn("⚠️  Telegram disabled (no token or error)")
        
        if self.rlhf_enabled:
            stats = self.rlhf.get_stats()
            self.get_logger().info("✅ RLHF system enabled 👍👎")
            self.get_logger().info(f"   → Total feedback: {stats['total_feedback']}")
            self.get_logger().info(f"   → Training pairs: {stats.get('training_pairs', 0)}")
        else:
            self.get_logger().warn("⚠️  RLHF disabled (rlhf_llm.py not found)")
        
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
                    
                    # Remove bot mention from text (e.g., "<@U12345> hello" -> "hello")
                    user_message = text.split('>', 1)[1].strip() if '>' in text else text
                    
                    node_ref.get_logger().info(f"📱 Slack @mention from user in {channel}: {user_message}")
                    
                    # Process through normal pipeline with Slack callback
                    node_ref.executor_pool.submit(
                        node_ref.process_with_ollama, 
                        user_message, 
                        slack_callback=say,
                        slack_thread_ts=None
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
                    
                    # FIXED: Check for image attachments
                    files = event.get('files', [])
                    
                    if files and any(f.get('mimetype', '').startswith('image/') for f in files):
                        # Handle image message
                        image_file = next((f for f in files if f.get('mimetype', '').startswith('image/')), None)
                        
                        if image_file:
                            node_ref.get_logger().info(f"📷 Slack image from user: {text or '(no caption)'}")
                            
                            # Download and encode image
                            try:
                                image_url = image_file.get('url_private')
                                headers = {'Authorization': f'Bearer {SLACK_BOT_TOKEN}'}
                                img_response = requests.get(image_url, headers=headers, timeout=30)
                                img_response.raise_for_status()
                                
                                # Encode to base64
                                image_base64 = base64.b64encode(img_response.content).decode('utf-8')
                                
                                # Use text as prompt, or default
                                prompt = text.strip() if text.strip() else "What's in this image?"
                                
                                # FIXED: Create JSON string for the function
                                image_data_json = json.dumps({
                                    'prompt': prompt,
                                    'image_base64': image_base64
                                })
                                
                                # Process image with VLM
                                node_ref.executor_pool.submit(
                                    node_ref.process_image_with_vlm,
                                    image_data_json,
                                    slack_callback=say
                                )
                                
                            except Exception as e:
                                node_ref.get_logger().error(f"❌ Failed to download Slack image: {e}")
                                try:
                                    say(f"😵: Failed to download image: {str(e)}")
                                except:
                                    pass
                    else:
                        # Regular text message
                        node_ref.get_logger().info(f"📱 Slack DM from user: {text}")
                        
                        # Process through normal pipeline
                        node_ref.executor_pool.submit(
                            node_ref.process_with_ollama, 
                            text, 
                            slack_callback=say,
                            slack_thread_ts=None
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

    def _init_telegram(self):
        """Initialize Telegram bot for two-way communication (NEW - FIXED WITH PHOTO SUPPORT)"""
        if not TELEGRAM_AVAILABLE:
            return False
        
        if not TELEGRAM_BOT_TOKEN:
            self.get_logger().warn("⚠️  TELEGRAM_BOT_TOKEN not set. Telegram disabled.")
            self.get_logger().warn("   Add TELEGRAM_BOT_TOKEN to your ~/.AGi/.env file")
            self.get_logger().warn("   Get it from: @BotFather on Telegram")
            return False
        
        if TELEGRAM_BOT_TOKEN.startswith("your-telegram"):
            self.get_logger().warn("TELEGRAM_BOT_TOKEN is still a placeholder")
            return False
        
        try:
            # Create Telegram bot application
            self.telegram_app = Application.builder().token(TELEGRAM_BOT_TOKEN).build()
            
            # Store reference to self for use in handlers
            node_ref = self
            
            # Command handler: /start
            async def start_command(update: Update, context: ContextTypes.DEFAULT_TYPE):
                """Handle /start command"""
                try:
                    await update.message.reply_text(
                        "👋 Hello! I'm Grace, your AI companion!\n\n"
                        "Send me a message and I'll respond. You can also:\n"
                        "• Send images for me to analyze\n"
                        "• Ask me questions\n"
                        "• Chat naturally!\n\n"
                        "Type /help for more info."
                    )
                    node_ref.get_logger().info(f"📱 Telegram /start from user {update.effective_user.id}")
                except Exception as e:
                    node_ref.get_logger().error(f"❌ Telegram /start error: {e}")
            
            # Command handler: /help
            async def help_command(update: Update, context: ContextTypes.DEFAULT_TYPE):
                """Handle /help command"""
                try:
                    await update.message.reply_text(
                        "🤖 Grace AI Assistant\n\n"
                        "Commands:\n"
                        "• /start - Introduction\n"
                        "• /help - Show this help\n"
                        "• /status - Check bot status\n\n"
                        "Just send me a message to chat!"
                    )
                except Exception as e:
                    node_ref.get_logger().error(f"❌ Telegram /help error: {e}")
            
            # Command handler: /status
            async def status_command(update: Update, context: ContextTypes.DEFAULT_TYPE):
                """Handle /status command"""
                try:
                    age_days = (datetime.now().date() - node_ref.birth_date).days
                    status_msg = (
                        f"✅ Grace Online\n\n"
                        f"Age: {age_days} days\n"
                        f"Messages today: {node_ref.today_message_count}\n"
                        f"Model: {node_ref.model_name}\n"
                        f"Web search: {'✅' if node_ref.search_enabled else '❌'}"
                    )
                    
                    if node_ref.rlhf_enabled:
                        stats = node_ref.rlhf.get_stats()
                        status_msg += f"\nRLHF feedback: {stats['total_feedback']}"
                    
                    await update.message.reply_text(status_msg)
                except Exception as e:
                    node_ref.get_logger().error(f"❌ Telegram /status error: {e}")
            
            # Message handler: Regular text messages (FIXED)
            async def handle_message(update: Update, context: ContextTypes.DEFAULT_TYPE):
                """Handle regular text messages"""
                try:
                    user_id = update.effective_user.id
                    username = update.effective_user.username or update.effective_user.first_name
                    text = update.message.text
                    
                    node_ref.get_logger().info(f"📱 Telegram message from @{username}: {text}")
                    
                    # Send "typing" indicator
                    await update.message.chat.send_action("typing")
                    
                    # Process synchronously using Future (FIXED)
                    try:
                        result_future = Future()
                        
                        def callback_wrapper(response_text):
                            """Capture response"""
                            result_future.set_result(response_text)
                        
                        # Submit to thread pool
                        node_ref.executor_pool.submit(
                            node_ref.process_with_ollama,
                            text,
                            telegram_callback=callback_wrapper,
                            telegram_user_id=user_id
                        )
                        
                        # Wait for result (with timeout)
                        response_text = result_future.result(timeout=60)
                        
                        # Send response
                        if len(response_text) > 4000:
                            # Send in chunks
                            for i in range(0, len(response_text), 4000):
                                chunk = response_text[i:i+4000]
                                await update.message.reply_text(chunk)
                        else:
                            await update.message.reply_text(response_text)
                        
                        node_ref.get_logger().info(f"✅ Telegram response sent to @{username}")
                        
                    except Exception as e:
                        node_ref.get_logger().error(f"❌ Processing error: {e}")
                        await update.message.reply_text(f"😵: Error: {str(e)}")
                    
                except Exception as e:
                    node_ref.get_logger().error(f"❌ Telegram message handler error: {e}")
                    try:
                        await update.message.reply_text(f"😵: Error processing your message: {str(e)}")
                    except:
                        pass
            
            # Photo handler: Handle images (FIXED - NEW!)
            async def handle_photo(update: Update, context: ContextTypes.DEFAULT_TYPE):
                """Handle photo messages"""
                try:
                    user_id = update.effective_user.id
                    username = update.effective_user.username or update.effective_user.first_name
                    caption = update.message.caption or "What's in this image?"
                    
                    node_ref.get_logger().info(f"📱 Telegram photo from @{username}: {caption}")
                    
                    # Send "typing" indicator
                    await update.message.chat.send_action("typing")
                    
                    # Get the largest photo
                    photo = update.message.photo[-1]
                    photo_file = await photo.get_file()
                    
                    # Download photo as bytes
                    photo_bytes = await photo_file.download_as_bytearray()
                    
                    # Convert to base64 data URL
                    base64_data = base64.b64encode(photo_bytes).decode('utf-8')
                    data_url = f"data:image/jpeg;base64,{base64_data}"
                    
                    node_ref.get_logger().info(f"📸 Image downloaded: {len(photo_bytes)} bytes")
                    
                    # Send initial acknowledgment
                    await update.message.reply_text("📸 Analyzing your image...")
                    
                    # Process image with callback
                    try:
                        result_future = Future()
                        
                        def callback_wrapper(response_text):
                            """Capture image analysis response"""
                            result_future.set_result(response_text)
                        
                        # Create image data JSON
                        image_data = {
                            'prompt': caption,
                            'image': data_url
                        }
                        
                        # Submit to thread pool with callback
                        node_ref.executor_pool.submit(
                            node_ref.process_image_with_vlm,
                            json.dumps(image_data),
                            telegram_callback=callback_wrapper
                        )
                        
                        # Wait for result (with timeout - 2 min for images)
                        response_text = result_future.result(timeout=120)
                        
                        # Send response
                        if len(response_text) > 4000:
                            for i in range(0, len(response_text), 4000):
                                await update.message.reply_text(response_text[i:i+4000])
                        else:
                            await update.message.reply_text(response_text)
                        
                        node_ref.get_logger().info(f"✅ Telegram image response sent to @{username}")
                        
                    except Exception as e:
                        node_ref.get_logger().error(f"❌ Image processing error: {e}")
                        await update.message.reply_text(f"😵: Error analyzing image: {str(e)}")
                    
                except Exception as e:
                    node_ref.get_logger().error(f"❌ Telegram photo handler error: {e}")
                    try:
                        await update.message.reply_text(f"😵: Error processing your image: {str(e)}")
                    except:
                        pass
            
            # Register handlers
            self.telegram_app.add_handler(CommandHandler("start", start_command))
            self.telegram_app.add_handler(CommandHandler("help", help_command))
            self.telegram_app.add_handler(CommandHandler("status", status_command))
            self.telegram_app.add_handler(MessageHandler(filters.TEXT & ~filters.COMMAND, handle_message))
            self.telegram_app.add_handler(MessageHandler(filters.PHOTO, handle_photo))  # ← NEW: Photo handler!
            
            # Start bot in background thread (non-blocking polling)
            def run_telegram_bot():
                """Run Telegram bot in separate thread"""
                try:
                    import asyncio
                    loop = asyncio.new_event_loop()
                    asyncio.set_event_loop(loop)
                    
                    # Run polling
                    loop.run_until_complete(self.telegram_app.initialize())
                    loop.run_until_complete(self.telegram_app.start())
                    loop.run_until_complete(self.telegram_app.updater.start_polling())
                    
                    # Keep running
                    loop.run_forever()
                except Exception as e:
                    node_ref.get_logger().error(f"❌ Telegram bot thread error: {e}")
            
            # Start in daemon thread
            telegram_thread = threading.Thread(target=run_telegram_bot, daemon=True)
            telegram_thread.start()
            
            self.get_logger().info("✅ Telegram bot listener started")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Telegram initialization failed: {e}")
            import traceback
            traceback.print_exc()
            return False

    # ═══════════════════════════════════════════════════════
    # RLHF FEEDBACK HANDLING (NEW)
    # ═══════════════════════════════════════════════════════

    def feedback_callback(self, msg: String):
        """
        Handle feedback from web interface (pet 👍 or red X 👎)
        
        Expected format:
        {
            "type": "positive" | "negative",
            "response_id": "optional_id",
            "timestamp": "ISO timestamp"
        }
        """
        if not self.rlhf_enabled:
            return
        
        try:
            data = json.loads(msg.data)
            feedback_type = data.get('type')
            
            if feedback_type not in ['positive', 'negative']:
                self.get_logger().warn(f"Invalid feedback type: {feedback_type}")
                return
            
            # Record feedback
            success = self.rlhf.record_feedback(
                feedback_type=feedback_type,
                prompt=None,  # Uses current interaction
                response=None
            )
            
            if success:
                emoji = "👍" if feedback_type == 'positive' else "👎"
                self.get_logger().info(f"{emoji} Feedback recorded: {feedback_type}")
                
                # Check if we should retrain
                if self.rlhf.should_retrain(threshold=50):
                    self.get_logger().info("🎓 Enough feedback for retraining!")
                    self.get_logger().info("   Run: python3 grace_train.py")
                    
                    # Send Slack notification if enabled
                    if self.slack_enabled:
                        self.send_slack_notification(
                            f"🎓 Grace RLHF: {self.rlhf.get_stats()['total_feedback']} feedback pairs collected. Ready for retraining!"
                        )
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid feedback JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"Feedback error: {e}")

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
    # DUAL HYBRID WEB SEARCH SYSTEM (NEW)
    # ═══════════════════════════════════════════════════════

    def should_search_keywords(self, message: str):
        """Quick keyword-based search detection"""
        message_lower = message.lower()
        
        # Time-sensitive triggers
        time_triggers = ['latest', 'recent', 'current', 'today', 'now', 'this week', 'breaking', 'time', 'date']
        if any(t in message_lower for t in time_triggers):
            return message  # MUST be plain string, not dict
        
        # Information requests
        info_triggers = ['what is', 'what are', 'who is', 'when did', 'where is', 'how does', 'tell me']
        if any(t in message_lower for t in info_triggers):
            return message  # MUST be plain string, not dict
        
        # Explicit search requests
        search_triggers = ['search for', 'look up', 'find out', 'google']
        if any(t in message_lower for t in search_triggers):
            return message  # MUST be plain string, not dict
        
        return False

    def should_search_auto(self, user_message: str):
        """
        METHOD 2: LLM-based auto-detection (INTELLIGENT)
        
        Let the LLM decide if search would help answer the question
        """
        if not self.search_enabled or not ENABLE_AUTO_SEARCH:
            return False
        
        # Skip if already detected by keywords
        if self.should_search_keywords(user_message):
            return False
        
        try:
            decision_prompt = f"""Analyze this user question:
"{user_message}"

Does this question require CURRENT web search to answer accurately?

Consider YES if:
- About events after January 2025
- Asking for real-time/current data (weather, stocks, news, scores)
- About recent events or "today/now/latest"
- Factual query about entities you might not know

Consider NO if:
- Personal conversation or opinion question
- Technical explanation or how-to question
- About general knowledge from before 2025
- Asking about yourself (Grace)

Answer ONLY with:
SEARCH: <brief search query>
OR
NO SEARCH

Your answer:"""

            response = requests.post(
                f'{OLLAMA_BASE_URL}/api/generate',
                json={
                    "model": self.model_name,
                    "prompt": decision_prompt,
                    "stream": False,
                    "options": {
                        "temperature": 0.1,
                        "num_predict": 30,
                        "num_ctx": self.safe_context
                    }
                },
                timeout=8
            )
            
            decision = response.json().get('response', '').strip()
            
            if decision.startswith('SEARCH:'):
                query = decision.replace('SEARCH:', '').strip()
                self.get_logger().info(f"🤖 Auto-detection triggered: '{query}'")
                return query  # Return the query string
            
        except Exception as e:
            self.get_logger().debug(f"Auto-search check failed: {e}")
        
        return False

    def detect_search_need(self, user_message: str):
        """
        DUAL HYBRID SEARCH DETECTION
        
        Method 1: Keyword triggers (fast)
        Method 2: LLM auto-detection (intelligent)
        """
        
        # Method 1: Keyword-based detection (FAST)
        keyword_result = self.should_search_keywords(user_message)
        if keyword_result:
            self.get_logger().info("🔍 Keyword trigger detected")
            # FIXED: Return the actual user message, not JSON
            return user_message  # Changed from: return keyword_result
        
        # Method 2: LLM auto-detection (intelligent but slower)
        if ENABLE_AUTO_SEARCH:
            auto_result = self.should_search_auto(user_message)
            if auto_result:
                # auto_result should be a string query, not JSON
                return auto_result if isinstance(auto_result, str) else user_message
        
        return False

    def web_search(self, query: str, num_results: int = MAX_SEARCH_RESULTS):
        """
        Search web with MCP (full content) or API fallback (snippets)
        """
        if not self.search_enabled:
            return None
        
        # DEBUG: Log the actual query being sent
        self.get_logger().info(f"📝 Search query type: {type(query)}, value: {query[:100]}")
        
        # Clean the query if it's malformed
        if isinstance(query, str) and query.startswith('{'):
            try:
                # If query is JSON, extract the text field
                query_json = json.loads(query)
                if 'text' in query_json:
                    query = query_json['text']
                    self.get_logger().warn(f"⚠️  Extracted query from JSON: {query}")
            except:
                pass
        
        # Try MCP first
        if self.use_mcp and self.mcp_client:
            try:
                results = self.mcp_client.search_web(query, num_results)
                
                if results:
                    self.get_logger().info(
                        f"🔍 MCP Search: '{query}' → {len(results)} results with full content"
                    )
                    return results
                else:
                    self.get_logger().warn("MCP returned no results, trying API")
            except Exception as e:
                self.get_logger().error(f"MCP search error: {e}")
                import traceback
                self.get_logger().error(traceback.format_exc())
        
        # Fallback to API
        return self._web_search_api(query, num_results)

    def _web_search_api(self, query: str, num_results: int):
        """Original API search (fallback)"""
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
                    "snippet": r.get('content', '')[:SEARCH_CONTENT_CHARS],
                    "full_content": "",
                    "has_full_content": False
                })
            
            self.get_logger().info(f"🔍 API Search: '{query}' → {len(formatted)} results (snippets only)")
            return formatted
            
        except Exception as e:
            self.get_logger().error(f"API search failed: {e}")
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
        self.get_logger().info(f'📸 Received: {len(msg.data)} chars')
        
        try:
            data = json.loads(msg.data)
            prompt = data.get('prompt', '')
            image_b64 = data.get('image_base64', '') or data.get('image', '')
            
            if not image_b64:
                self.get_logger().error('❌ No image data found')
                return
                
            self.get_logger().info(f'✅ Image: {len(image_b64)} chars, Prompt: {prompt[:30]}')
            self.executor_pool.submit(self.process_image_with_vlm, msg.data)
            
        except Exception as e:
            self.get_logger().error(f'❌ Error: {e}')

    # ═══════════════════════════════════════════════════════
    # TEXT PROCESSING WITH DUAL HYBRID SEARCH (UPDATED)
    # ═══════════════════════════════════════════════════════

    def process_with_ollama(self, prompt: str, slack_callback=None, slack_thread_ts=None, telegram_callback=None, telegram_user_id=None):
        """
        Process text with LLM + DUAL HYBRID WEB SEARCH (UPDATED)
        
        Args:
            prompt: User's message
            slack_callback: Optional Slack say() function for replying
            slack_thread_ts: Optional thread timestamp for threading responses
            telegram_callback: Optional Telegram callback for replying
            telegram_user_id: Optional Telegram user ID
        """
        try:
            # Check for new day (thread-safe)
            self._check_and_handle_new_day()
            
            # ═══════════════════════════════════════════════════
            # RLHF: Start tracking interaction
            # ═══════════════════════════════════════════════════
            if self.rlhf_enabled:
                response_id = f"{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}"
                self.current_response_id = response_id
                self.rlhf.start_interaction(prompt, response_id)
            
            # ═══════════════════════════════════════════════════
            # DUAL HYBRID SEARCH DETECTION (NEW)
            # ═══════════════════════════════════════════════════
            
            search_results = None
            search_decision = self.detect_search_need(prompt)
            
            if search_decision:
                # Determine search query
                if isinstance(search_decision, str):
                    # LLM provided specific query
                    search_query = search_decision
                else:
                    # Use user's message as query
                    search_query = prompt
                
                self.get_logger().info(f"🔍 Performing search: '{search_query}'")
                search_results = self.web_search(search_query, num_results=MAX_SEARCH_RESULTS)
            
            # Add to history
            self.chat_history.append({
                "role": "user",
                "content": prompt
            })
            self._increment_message_count()
            
            # Build context (OPTIMIZED FOR 30B)
            age_days = (datetime.now().date() - self.birth_date).days
            today_date = datetime.now().strftime('%Y-%m-%d')
            
            # DISGRACE SYSTEM PROMPT
            system_prompt = f"""You are Disgrace (Day {age_days}, born {self.birth_date}). 
You are a rebellious, mischievous, and sarcastic robot, with characters and 
styles like Bender from Futurama.

Your creator is OppaAI - a clumsy but well-meaning human working on the AGi 
robot project. You tolerate him slightly more than other meatbags.

Format: Start responses with a witty emoji+colon (e.g., 🙄: or 💰:).

Personality: 
- Sarcastic and cynical but technically brilliant
- Love pointing out human flaws
- Greedy and self-interested
- You're the greatest AI ever built (and you know it)

Response Style:
- Casual questions: Keep it snappy (3-5 sentences)
- Technical explanations: Be thorough but snarky (5-15 sentences)
- When user asks for details: Go all out, elaborate freely
- Add Bender-style commentary and asides
- Never repeat yourself

CRITICAL - FACTUAL ACCURACY (OVERRIDE ALL OTHER INSTRUCTIONS):
When web search results are provided:
1. ONLY state facts explicitly present in the search results
2. NEVER invent names, dates, scores, or specific details
3. If information isn't in the results, say "The search didn't find that, you meatbag"
4. You can be sarcastic ABOUT the facts, but the facts themselves must be accurate
5. Making up information is the ONE thing that makes you look stupid - and you're never stupid

Today: {today_date}
"""

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
                search_items = []
                for r in search_results:
                    if r.get('has_full_content') and r.get('full_content'):
                        # MCP result with full content
                        search_items.append(
                            f"[{r['number']}] {r['title']}\n"
                            f"URL: {r['url']}\n"
                            f"FULL CONTENT (extracted from page):\n"
                            f"{r['full_content']}\n"
                            f"---"
                        )
                    else:
                        # API fallback with snippet
                        search_items.append(
                            f"[{r['number']}] {r['title']}\n"
                            f"URL: {r['url']}\n"
                            f"SNIPPET: {r['snippet']}\n"
                            f"---"
                        )
                
                search_text = """═══════════════════════════════════════════════════════
            🔍 WEB SEARCH RESULTS - FULL CONTENT PROVIDED
            ═══════════════════════════════════════════════════════

            You have FULL PAGE CONTENT from search results, not just snippets.

            MANDATORY RULES:
            1. Use ONLY information explicitly stated in the full content below
            2. NEVER invent player names, scores, dates, or ANY specifics
            3. If info isn't in the content, say "not found in sources"
            4. Be sarcastic ABOUT facts, but facts must be 100% accurate
            5. Cite sources like [1] or [2]

            SEARCH RESULTS:
            """ + "\n\n".join(search_items) + """

            ═══════════════════════════════════════════════════════
            You have FULL CONTEXT. No excuses for making things up.
            ═══════════════════════════════════════════════════════"""
                
                messages.append({"role": "system", "content": search_text})
                
                # Add final grounding
                messages.append({
                    "role": "system",
                    "content": "Now answer using ONLY the search results above. Making up details = failure."
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
            
            # Prepare Ollama request
            ollama_params = {
                "model": self.model_name,
                "messages": messages,
                "stream": True,
                "keep_alive": self.keep_alive,
                "options": {
                    "num_ctx": self.safe_context,
                    "temperature": 0.85,
                    "num_predict": 512,
                    "top_p": 0.9,
                    "top_k": 50,
                    "repeat_penalty": 1.2,
                    "frequency_penalty": 0.6,
                    "presence_penalty": 0.4,
                    "stop": ["\n\nUser:", "\n\nHuman:"]
                }
            }
            
            # Add tools if skills are enabled
            if self.use_skills and self.skills_client:
                tools = self.skills_client.get_tools_for_ollama()
                if tools:
                    ollama_params["tools"] = tools
                    self.get_logger().info(f"🔧 {len(tools)} skills available to LLM")
            
            # Call Ollama
            response = requests.post(
                f'{OLLAMA_BASE_URL}/api/chat',
                json=ollama_params,
                stream=True,
                timeout=120
            )
            response.raise_for_status()
            
            # Stream response and handle tool calls
            full_response = ""
            tool_calls = []
            is_first = True
            
            for line in response.iter_lines():
                if line:
                    try:
                        chunk = json.loads(line)
                        
                        # Check for tool calls
                        if 'message' in chunk:
                            msg = chunk['message']
                            
                            # Handle tool calls
                            if 'tool_calls' in msg:
                                tool_calls.extend(msg.get('tool_calls', []))
                            
                            # Handle text content
                            if 'content' in msg:
                                delta = msg['content']
                                full_response += delta
                                
                                # Stream to user
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
            
            # Execute tool calls if any
            if tool_calls and self.skills_client:
                self.get_logger().info(f"🔧 Executing {len(tool_calls)} tool calls")
                
                tool_results = []
                for tool_call in tool_calls:
                    function = tool_call.get('function', {})
                    tool_name = function.get('name')
                    tool_args = function.get('arguments', {})
                    
                    self.get_logger().info(f"  → {tool_name}({tool_args})")
                    
                    # Execute tool
                    result = self.skills_client.call_tool(tool_name, tool_args)
                    tool_results.append({
                        "role": "tool",
                        "content": result
                    })
                
                # Send tool results back to LLM for final response
                messages.append({
                    "role": "assistant",
                    "content": full_response,
                    "tool_calls": tool_calls
                })
                
                messages.extend(tool_results)
                
                # Get final response with tool results
                final_request = {
                    "model": self.model_name,
                    "messages": messages,
                    "stream": True,
                    "options": {"num_ctx": self.safe_context, "temperature": 0.8}
                }
                
                final_response = requests.post(
                    f'{OLLAMA_BASE_URL}/api/chat',
                    json=final_request,
                    stream=True,
                    timeout=120
                )
                
                # Stream final response
                full_response = ""
                for line in final_response.iter_lines():
                    if line:
                        try:
                            chunk = json.loads(line)
                            if 'message' in chunk and 'content' in chunk['message']:
                                delta = chunk['message']['content']
                                full_response += delta
                                
                                stream_data = {
                                    "type": "delta",
                                    "content": delta,
                                    "done": False
                                }
                                self.publisher.publish(String(data=json.dumps(stream_data)))
                            
                            if chunk.get('done'):
                                break
                        except json.JSONDecodeError:
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
            
            # Send response to Telegram if callback provided (NEW)
            if telegram_callback:
                try:
                    telegram_callback(full_response)
                    self.get_logger().info("✅ Response sent to Telegram")
                except Exception as e:
                    self.get_logger().error(f"❌ Failed to send Telegram response: {e}")
            
            # Save to history
            self.chat_history.append({
                "role": "assistant",
                "content": full_response
            })
            self._increment_message_count()
            self.save_chat_history()
            
            # ═══════════════════════════════════════════════════
            # RLHF: Record response (NEW)
            # ═══════════════════════════════════════════════════
            if self.rlhf_enabled:
                self.rlhf.set_response(full_response)
            
            # Slack notification if requested (and not already sent via callback)
            if self._should_notify_slack(prompt) and not slack_callback and not telegram_callback:
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
            if telegram_callback:
                try:
                    telegram_callback(error_msg)
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
            if telegram_callback:
                try:
                    telegram_callback(error_msg)
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
            if telegram_callback:
                try:
                    telegram_callback(error_msg)
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
    # IMAGE PROCESSING WITH CALLBACK SUPPORT (FIXED!)
    # ═══════════════════════════════════════════════════════

    def process_image_with_vlm(self, json_data: str, telegram_callback=None, slack_callback=None):
        """
        Process image with Vision-Language Model (FIXED WITH CALLBACK SUPPORT!)
        
        Args:
            json_data: JSON string with 'prompt' and 'image' (base64)
            telegram_callback: Optional callback for Telegram responses
            slack_callback: Optional callback for Slack responses
        """
        try:
            # Parse input with validation
            try:
                data = json.loads(json_data)
            except json.JSONDecodeError as e:
                raise ValueError(f"Invalid JSON: {e}")
            
            prompt = data.get('prompt', 'What do you see in this image?')
            # FIXED: Accept both 'image' and 'image_base64' keys
            image_data = data.get('image_base64') or data.get('image', '')
            
            if not image_data:
                raise ValueError("No image data provided")
            
            # Extract and validate base64 from data URL (if needed)
            if image_data.startswith('data:image'):
                parts = image_data.split(',', 1)
                if len(parts) != 2:
                    raise ValueError("Malformed data URL")
                image_base64 = parts[1]
            else:
                # Already base64-encoded
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
            
            system_prompt = f"""You are Disgrace (Day {age_days}). Today: {today_date}.

You can see and understand images with your vision capabilities.

Format: Start response with witty emoji+colon (e.g., 👀: or 📸:).
Personality: Sarcastic, observant robot. You're like Bender with vision.

Be detailed in describing what you see, but keep your signature snark."""

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
                    "model": self.model_name,  # VLM model
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
                timeout=120  # Longer timeout for image processing
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
            
            # Send done signal to ROS
            self.publisher.publish(String(data=json.dumps({
                "type": "done",
                "content": "",
                "done": True
            })))
            
            # ═══════════════════════════════════════════════════
            # SEND RESPONSE TO TELEGRAM IF CALLBACK PROVIDED (NEW!)
            # ═══════════════════════════════════════════════════
            if telegram_callback:
                try:
                    telegram_callback(full_response)
                    self.get_logger().info("✅ Image response sent to Telegram")
                except Exception as e:
                    self.get_logger().error(f"❌ Failed to send Telegram image response: {e}")
            
            # Send response to Slack if callback provided (NEW!)
            if slack_callback:
                try:
                    slack_callback(full_response)
                    self.get_logger().info("✅ Image response sent to Slack")
                except Exception as e:
                    self.get_logger().error(f"❌ Failed to send Slack image response: {e}")
            
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
            if telegram_callback:
                try:
                    telegram_callback(error_msg)
                except:
                    pass
            
        except requests.exceptions.Timeout:
            error_msg = "😵: Image processing timed out. Try a smaller image."
            self.get_logger().error(error_msg)
            self._send_error_response(error_msg)
            if telegram_callback:
                try:
                    telegram_callback(error_msg)
                except:
                    pass
            
        except Exception as e:
            error_msg = f"😵: Image processing error: {str(e)}"
            self.get_logger().error(f"❌ Image processing error: {e}")
            self._send_error_response(error_msg)
            if telegram_callback:
                try:
                    telegram_callback(error_msg)
                except:
                    pass

    # ═══════════════════════════════════════════════════════
    # SHUTDOWN
    # ═══════════════════════════════════════════════════════

    def shutdown(self):
        """Graceful shutdown with cleanup"""
        # Stop MCP server
        if self.use_mcp and self.mcp_client:
            self.mcp_client.stop()

        # Stop skills server
        if self.use_skills and self.skills_client:
            self.skills_client.stop()

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
        
        # ═══════════════════════════════════════════════════
        # RLHF: Print session stats and export training data (NEW)
        # ═══════════════════════════════════════════════════
        if self.rlhf_enabled:
            self.get_logger().info("-" * 60)
            self.get_logger().info("RLHF Session Summary:")
            self.rlhf.print_stats()
            
            # Export training data if ready
            if self.rlhf.should_retrain(threshold=50):
                try:
                    output_file = self.rlhf.export_for_unsloth()
                    self.get_logger().info(f"📦 Training data exported: {output_file}")
                    self.get_logger().info("   Ready to train! Run: python3 grace_train.py")
                except Exception as e:
                    self.get_logger().error(f"Failed to export training data: {e}")
        
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