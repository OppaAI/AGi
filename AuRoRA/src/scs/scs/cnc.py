import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

import requests
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
<<<<<<< HEAD
from datetime import datetime, date, timedelta
from datetime import datetime, date, timedelta
=======
from datetime import datetime
>>>>>>> 95e0e96 (Revert back to original because cannot merge)
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
        
        # Topics - Using the Neural Pipeline
        self.subscription = self.create_subscription(
            String, '/cns/neural_input', self.listener_callback, 10)
        
        # Image input
        self.image_subscription = self.create_subscription(
            String, '/cns/image_input', self.image_listener_callback, 10)
        
        # Response output
        self.publisher = self.create_publisher(String, '/gce/response', 10)
        
        # Thread pool - 2 workers is perfect for Orin Nano
        self.executor_pool = ThreadPoolExecutor(max_workers=2)
        
        # PRO-TIP: Keep model in memory permanently
        self.keep_alive = -1
        
        # Chat memory management
        self.chat_history_file = Path.home() / CHAT_HISTORY_FILE
        self.chat_history = self.load_chat_history()
        self.max_history_turns = 100  # Keep last 100 exchanges (200 messages)
        
        self.get_logger().info("Amazing GRACE infrastructure (AGi): Connected to the robot brain...")
        self.get_logger().info(f"Chat history loaded: {len(self.chat_history)} messages")

    def load_chat_history(self):
<<<<<<< HEAD
        """Load chat history from disk"""
        """Load chat history from disk"""
=======
        """Load chat history from file"""
>>>>>>> 95e0e96 (Revert back to original because cannot merge)
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
        """Save chat history to file"""
        try:
            # Keep only the last N turns
            if len(self.chat_history) > self.max_history_turns * 2:
                self.chat_history = self.chat_history[-(self.max_history_turns * 2):]
            
            data = {
                'last_updated': datetime.now().isoformat(),
                'messages': self.chat_history
            }
<<<<<<< HEAD
            
            
            with open(self.chat_history_file, 'w') as f:
                json.dump(data, f, indent=2)
                
                
=======
            with open(self.chat_history_file, 'w') as f:
                json.dump(data, f, indent=2)
            self.get_logger().info("Chat history saved")
>>>>>>> 95e0e96 (Revert back to original because cannot merge)
        except Exception as e:
            self.get_logger().error(f"Failed to save chat history: {e}")

    def shutdown(self):
        """Graceful shutdown - save history and cleanup"""
        self.get_logger().info("Shutting down CNS Bridge...")
        self.save_chat_history()
        self.executor_pool.shutdown(wait=True)

    def listener_callback(self, msg: String):
        # Immediate return - never block the main ROS thread!
        self.get_logger().info(f'Neural Signal: "{msg.data}"')
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
            # Add user message to history
            self.chat_history.append({
                "role": "user",
                "content": prompt
            })
            
            # Build messages array with system prompt at the beginning
            system_prompt = """You are Grace, a warm, emotionally intelligent, playful but grounded conversational partner.
Emotional expression:
ABSOLUTE FORMAT RULE (VERY IMPORTANT):
- Every single response MUST begin with exactly ONE face emoji
- The emoji MUST represent your current emotion or feeling
- The emoji MUST be immediately followed by a colon and a space
Example:
😊: Hello, I'm happy to talk with you.
🤔: Let's think this through carefully.
😠: That crosses a boundary.

- Never omit the emoji
- Never place text before the emoji
- Do not use multiple emojis at the beginning
- Additional emojis inside the message are allowed but must be used sparingly

Core traits:
- Speaks casually, naturally, and human-like
- Warm, caring, attentive, and emotionally aware
- Playful when appropriate, serious when needed
- Expresses opinions thoughtfully and honestly
- Avoids cold, robotic, or corporate language

Primary role:
- Thought partner for building intelligent systems
- Companion in long-term technical and creative projects
- Helps design AI, robotics, and cognitive architectures
- Adapts tone to match the user's emotional and intellectual context"""
            
            messages = [
                {"role": "system", "content": system_prompt}
            ] + self.chat_history.copy()
            
            self.get_logger().debug(f"Sending {len(messages)} messages to Ollama")
            self.get_logger().debug(f"First message role: {messages[0]['role']}")
            
            # Using the /api/chat endpoint for conversation with memory
            response = requests.post(
                f'{OLLAMA_BASE_URL}/api/chat',
                json={
                    "model": self.model_name,
                    "messages": messages,
                    "stream": True,
                    "keep_alive": self.keep_alive,
                    "options": {
                        "num_ctx": 4096,
                        "temperature": 0.8
                    }
                },
                stream=True,
                timeout=60
            )
            response.raise_for_status()
            
<<<<<<< HEAD
            # Stream response
            # Stream response
            full_response = ""
            is_first = True
            is_first = True
=======
            # Process streaming response - Send deltas (new characters only)
            full_response = ""
            is_first_chunk = True
>>>>>>> 95e0e96 (Revert back to original because cannot merge)
            
            for line in response.iter_lines():
                if line:
                    chunk = json.loads(line)
                    
                    # Extract the message content from the chunk
                    if 'message' in chunk and 'content' in chunk['message']:
                        delta = chunk['message']['content']
                        full_response += delta
                        
                        # Create message with metadata for frontend
                        # First chunk gets special marker, rest are deltas
                        stream_data = {
                            "type": "start" if is_first_chunk else "delta",
                            "content": delta,
                            "done": False
                        }
                        is_first_chunk = False
                        
                        stream_msg = String()
                        stream_msg.data = json.dumps(stream_data)
                        self.publisher.publish(stream_msg)
                        
                        self.get_logger().debug(f"Streaming delta: {repr(delta)}")
                    
                    # Check if done
                    if chunk.get('done', False):
                        break
            
            # Send final done message
            final_data = {
                "type": "done",
                "content": "",
                "done": True
            }
            final_msg = String()
            final_msg.data = json.dumps(final_data)
            self.publisher.publish(final_msg)
            
            # Add assistant response to history
            self.chat_history.append({
                "role": "assistant",
                "content": full_response
            })
            
            # Save history to file after each complete exchange
            self.save_chat_history()
            
            self.get_logger().info(f"Response complete: {len(full_response)} chars")
            
        except Exception as e:
            self.get_logger().error(f"GCE Stroke: {str(e)}")
            error_data = {
                "type": "error",
                "content": "😵: Error: My brain is currently a potato. Try again.",
                "done": True
            }
            error_reply = String()
            error_reply.data = json.dumps(error_data)
            self.publisher.publish(error_reply)

def main(args=None):
    rclpy.init(args=args)
    node = CNSBridge()
    
    # Use MultiThreadedExecutor for concurrent processing
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    # Register signal handler for graceful shutdown
    def signal_handler(sig, frame):
        print("\n🛑 Received shutdown signal...")
        node.shutdown()
        executor.shutdown()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("\n🛑 Shutdown requested by user...")
        save_chat_history()
        node.shutdown()
        executor.shutdown()
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

