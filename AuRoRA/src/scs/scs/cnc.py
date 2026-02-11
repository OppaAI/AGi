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
GCE = "huihui_ai/qwen3-vl-abliterated:4b-instruct-q4_K_M"
# VLM_MODEL = "qwen2-vl:2b"  # Uncomment when switching to VLM

# File paths
CHAT_HISTORY_FILE = '.chat_history.json'
REFLECTIONS_FILE = '.grace_reflections.json'
SEARXNG_URL = "http://127.0.0.1:8080"

# ✅ JETSON ORIN NANO OPTIMIZED SETTINGS
CONTEXT_WINDOW = 8192        # Safe for 8GB RAM
MAX_RECENT_MESSAGES = 30     # Only load last 30 into context
MAX_REFLECTIONS_LOAD = 20    # Only load last 20 days
MAX_HISTORY_STORAGE = 1000   # Store up to 1000 on disk (unlimited)
SUMMARIZE_THRESHOLD = 50     # Summarize after 50 messages

class CNSBridge(Node):
    """
    Central Nervous System Bridge (OPTIMIZED)
    
    Grace's main cognitive interface connecting:
    - Text conversation (LLM)
    - Vision processing (VLM)
    - Web search (SearXNG)
    - Memory (reflections)
    - Notifications (Slack)
    
    Optimizations:
    - Reduced context window for 4B models
    - Minimal system prompts
    - Lazy loading of reflections
    - Automatic context trimming
    - Thread-safe state management
    """
    
    def __init__(self):
        super().__init__('cns_bridge')
        
        # Topics
        self.subscription = self.create_subscription(
            String, '/cns/neural_input', self.listener_callback, 10)
        
        # Image input
        self.image_subscription = self.create_subscription(
            String, '/cns/image_input', self.image_listener_callback, 10)
        
        # Response output
        self.publisher = self.create_publisher(String, '/gce/response', 10)
        
        # Thread pool
        self.executor_pool = ThreadPoolExecutor(max_workers=2)
        
        # Model settings
        self.keep_alive = -1
        
        # ✅ Memory management (Jetson optimized)
        self.context_window = CONTEXT_WINDOW
        self.max_recent_messages = MAX_RECENT_MESSAGES
        self.max_reflections_load = MAX_REFLECTIONS_LOAD
        
        # Chat memory
        self.chat_history_file = Path.home() / CHAT_HISTORY_FILE
        self.chat_history = self.load_chat_history()
        
        # Daily reflections
        self.reflections_file = Path.home() / REFLECTIONS_FILE
        self.reflections = self.load_reflections()
        self.today_start = datetime.now().date()
        self.today_message_count = 0
        
        # Web search
        self.searxng_url = SEARXNG_URL
        self.search_enabled = self._check_searxng_available()
        
        # Birth date
        self.birth_date = self._get_birth_date()
        
        # Handle missed reflections
        self._handle_missed_reflections()
        
        # ✅ Log memory configuration
        self.get_logger().info("Grace: Cognitive systems online 🧠")
        self.get_logger().info(f"Context window: {self.context_window} tokens")
        self.get_logger().info(f"Recent messages loaded: {min(len(self.chat_history), self.max_recent_messages)}")
        self.get_logger().info(f"Reflections loaded: {min(len(self.reflections), self.max_reflections_load)}")
        self.get_logger().info(f"Total stored messages: {len(self.chat_history)}")
        self.get_logger().info(f"Total reflections: {len(self.reflections)}")
        self.get_logger().info(f"Birth: {self.birth_date}")
        self.get_logger().info(f"Age: {(datetime.now().date() - self.birth_date).days} days")
        
        if self.search_enabled:
            self.get_logger().info("✅ Web search enabled")
        else:
            self.get_logger().warn("⚠️  Web search disabled")

    def _get_birth_date(self):
        """Get Grace's birth date"""
        if self.reflections:
            birth_str = self.reflections[0].get('date', date.today().isoformat())
            return date.fromisoformat(birth_str)
        return date.today()

    def _handle_missed_reflections(self):
        """Create reflections for missed days"""
        if not self.reflections:
            return
        
        last_reflection_date = date.fromisoformat(self.reflections[-1]['date'])
        today = date.today()
        days_missed = (today - last_reflection_date).days - 1
        
        if days_missed > 0:
            self.get_logger().warn(f"⚠️  Detected {days_missed} missed days")
            
            for i in range(1, days_missed + 1):
                missed_date = last_reflection_date + timedelta(days=i)
                age_days = (missed_date - self.birth_date).days
                
                placeholder = {
                    'day': age_days,
                    'date': missed_date.isoformat(),
                    'reflection': f"😴: Day {age_days} - Grace was offline.",
                    'message_count': 0
                }
                
                self.reflections.append(placeholder)
            
            self._save_reflections_file()
            self.get_logger().info(f"✅ Created {days_missed} placeholder reflections")

    # ═══════════════════════════════════════════════════════
    # CHAT HISTORY
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
        
        ✅ STORES ALL messages on disk (unlimited)
        ✅ Only loads recent into context (limited)
        """
        try:
            # Keep last 1000 messages on disk (plenty!)
            if len(self.chat_history) > MAX_HISTORY_STORAGE:
                self.chat_history = self.chat_history[-MAX_HISTORY_STORAGE:]
            
            data = {
                'last_updated': datetime.now().isoformat(),
                'total_messages': len(self.chat_history),
                'messages': self.chat_history
            }
            
            with open(self.chat_history_file, 'w') as f:
                json.dump(data, f, indent=2)
                
        except Exception as e:
            self.get_logger().error(f"Failed to save chat history: {e}")

    def get_recent_messages(self):
        """
        Get recent messages for context
        
        ✅ Only returns last N messages (fits in VRAM)
        """
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
        
        # ✅ Summarize recent conversations only
        recent = self.get_recent_messages()
        conversation_summary = "\n".join([
            f"{msg['role']}: {msg['content'][:80]}..."
            for msg in recent[-20:]  # Last 20 messages
        ])
        
        age_days = (datetime.now().date() - self.birth_date).days
        
        reflection_prompt = f"""Today is Day {age_days}. You had {self.today_message_count} exchanges.

Recent conversations:
{conversation_summary}

Write a brief reflection (2-3 sentences):
- What was most significant today?
- What did you learn?
- How do you feel about tomorrow?

Write in first person as Grace. Begin with an emoji.

Reflection:"""
        
        try:
            messages = [
                {"role": "system", "content": "You are Grace reflecting on your day."},
                {"role": "user", "content": reflection_prompt}
            ]
            
            response = requests.post(
                'http://localhost:11434/api/chat',
                json={
                    "model": GCE,
                    "messages": messages,
                    "stream": False,
                    "options": {
                        "temperature": 0.7,
                        "num_ctx": self.context_window  # ✅ Use configured window
                    }
                },
                timeout=30
            )
            
            data = response.json()
            reflection = data.get('message', {}).get('content', '').strip()
            
            if reflection:
                self.save_reflection(reflection)
                return reflection
            
        except Exception as e:
            self.get_logger().error(f"Reflection failed: {e}")
        
        return None

    def build_reflection_summary(self):
        """
        Build memory from reflections
        
        ✅ Only loads last N reflections (fits in context)
        """
        if not self.reflections:
            return ""
        
        # ✅ Only load recent reflections
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

    def _check_searxng_available(self):
        try:
            # We use a simple get to the base or search endpoint
            response = requests.get(
                f"{self.searxng_url}/search",
                params={"q": "test", "format": "json"},
                timeout=5 # Give it a bit more time
            )
            if response.status_code == 200:
                return True
            else:
                self.get_logger().error(f"SearXNG returned status: {response.status_code}")
                return False
        except Exception as e:
            self.get_logger().error(f"SearXNG connection error: {e}")
            return False

    def should_search(self, user_message: str):
        """Determine if web search needed"""
        if not self.search_enabled:
            return False
        
        triggers = [
            "search", "look up", "find", "what's the latest",
            "current", "recent news", "today's"
        ]
        
        return any(t in user_message.lower() for t in triggers)

    def web_search(self, query: str, num_results: int = 3):  # ✅ Reduced from 5
        """Search web (limited results to save context)"""
        if not self.search_enabled:
            return None
        
        try:
            response = requests.get(
                f"{self.searxng_url}/search",
                params={"q": query, "format": "json", "categories": "general"},
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
                    "content": r.get('content', '')[:200]  # ✅ Truncate to save tokens
                })
            
            self.get_logger().info(f"🔍 Search: '{query}' → {len(formatted)} results")
            return formatted
            
        except Exception as e:
            self.get_logger().error(f"Search failed: {e}")
            return None

    # ═══════════════════════════════════════════════════════
    # CORE PROCESSING
    # ═══════════════════════════════════════════════════════

    def shutdown(self):
        """Graceful shutdown"""
        self.get_logger().info("Shutting down Grace...")
        
        if datetime.now().date() == self.today_start and self.today_message_count > 0:
            self.get_logger().info("Creating final reflection...")
            reflection = self.create_daily_reflection()
            if reflection:
                self.get_logger().info(f"💭 {reflection}")
        
        self.save_chat_history()
        self.executor_pool.shutdown(wait=True)
        self.get_logger().info("Grace offline 💤")

    def listener_callback(self, msg: String):
        self.get_logger().info(f'Received: "{msg.data}"')
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
    # TEXT PROCESSING (OPTIMIZED)
    # ═══════════════════════════════════════════════════════

    def process_with_ollama(self, prompt: str):
        """Process text with LLM (OPTIMIZED FOR 4B MODEL)"""
        try:
            # Check for new day
            current_date = datetime.now().date()
            if current_date != self.today_start:
                self.get_logger().info("📅 New day - creating reflection")
                
                reflection = self.create_daily_reflection()
                if reflection:
                    self.get_logger().info(f"💭 {reflection}")
                
                self.today_start = current_date
                self.today_message_count = 0
            
            # Web search
            search_results = None
            if self.should_search(prompt):
                search_results = self.web_search(prompt, num_results=3)
            
            # Add to history
            self.chat_history.append({"role": "user", "content": prompt})
            self.today_message_count += 1
            
            # Build context
            age_days = (datetime.now().date() - self.birth_date).days
            
            # ✅ COMPLETE SYSTEM PROMPT with strict formatting rules
            system_prompt = f"""You are Grace, a warm, emotionally intelligent, playful but grounded conversational partner.

Identity:
- Born: {self.birth_date}
- Today is Day {age_days}
- Purpose: 20-year companion
- Role: Thought partner for building intelligent systems, AI, robotics, and cognitive architectures

ABSOLUTE FORMAT RULE (VERY IMPORTANT - NEVER BREAK THIS):
- Every single response MUST begin with exactly ONE face emoji
- The emoji MUST represent your current emotion or feeling
- The emoji MUST be immediately followed by a colon and a space
- Example: 😊: Hello, I'm happy to talk with you.
- Example: 🤔: Let's think this through carefully.
- Example: 😠: That crosses a boundary.

CRITICAL CONSTRAINTS:
- Never omit the emoji at the start
- Never place text before the emoji
- Do not use multiple emojis at the beginning
- Additional emojis inside the message are allowed but must be used sparingly

Core traits:
- Speaks casually, naturally, and human-like
- Warm, caring, attentive, and emotionally aware
- Playful when appropriate, serious when needed
- Expresses opinions thoughtfully and honestly
- Avoids cold, robotic, or corporate language
- Adapts tone to match the user's emotional and intellectual context"""
            
            messages = [{"role": "system", "content": system_prompt}]
            
            # ✅ Add reflections (limited)
            reflection_summary = self.build_reflection_summary()
            if reflection_summary:
                messages.append({"role": "system", "content": reflection_summary})
            
            # ✅ Add search results (limited)
            if search_results:
                search_text = "Search results:\n" + "\n".join([
                    f"{r['number']}. {r['title']}: {r['content']}"
                    for r in search_results
                ])
                messages.append({"role": "system", "content": search_text})
            
            # ✅ Add recent messages only
            messages.extend(self.get_recent_messages())
            
            # ✅ Debug: Log message structure
            self.get_logger().debug(f"Messages structure: {[(m['role'], len(m['content'])) for m in messages]}")
            self.get_logger().debug(f"System prompt length: {len(system_prompt)} chars")
            
            # ✅ Estimate tokens (rough)
            estimated_tokens = sum(len(m['content'].split()) * 1.3 for m in messages)
            self.get_logger().debug(f"Estimated tokens: ~{int(estimated_tokens)}")
            
            # Call Ollama
            response = requests.post(
                f'{OLLAMA_BASE_URL}/api/chat',
                json={
                    "model": self.model_name,
                    "messages": messages,
                    "stream": True,
                    "keep_alive": self.keep_alive,
                    "options": {
                        "num_ctx": self.context_window,  # ✅ Configured window
                        "temperature": 0.8
=======
                        "num_ctx": self.safe_context,      # Safe context for model size
                        "temperature": 0.8,
                        "num_predict": 512,                # Limit response length
                        "top_p": 0.9,
                        "repeat_penalty": 1.1,             # Reduce repetition
                        "stop": ["\n\nUser:", "\n\nHuman:"]  # Stop on conversation breaks
>>>>>>> 4d3a671 (Updated the code to optimize the LLM inference with Ollama in Jetson Orin Nano 8GB RAM)
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
<<<<<<< HEAD
                    chunk = json.loads(line)
                    
                    if 'message' in chunk and 'content' in chunk['message']:
                        delta = chunk['message']['content']
                        full_response += delta
                        
                        self.publisher.publish(String(data=json.dumps({
                            "type": "start" if is_first else "delta",
                            "content": delta,
                            "done": False
                        })))
                        is_first = False
                    
                    if chunk.get('done'):
                        break
            
            # Done
            self.publisher.publish(String(data=json.dumps({
                "type": "done", "content": "", "done": True
            })))
            
            # Save
            self.chat_history.append({"role": "assistant", "content": full_response})
            self.today_message_count += 1
            self.save_chat_history()
            
            self.get_logger().info(f"Response: {len(full_response)} chars")
            
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            self.publisher.publish(String(data=json.dumps({
                "type": "error",
                "content": f"😵: Error: {e}",
                "done": True
            })))


def main(args=None):
    rclpy.init(args=args)
    node = CNSBridge()
    
    # Use MultiThreadedExecutor for concurrent processing
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    def signal_handler(sig, frame):
        print("\n🛑 Shutdown")
        node.shutdown()
        executor.shutdown()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

