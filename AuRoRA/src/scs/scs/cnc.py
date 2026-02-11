import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

import requests
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
from datetime import datetime
import signal
import sys

GCE = "hf.co/mradermacher/Huihui-granite-4.0-h-tiny-abliterated-i1-GGUF:Q4_K_M"
CHAT_HISTORY_FILE = '.chat_history.json'

class CNSBridge(Node):
    def __init__(self):
        super().__init__('cns_bridge')
        
        # Topics - Using the Neural Pipeline
        self.subscription = self.create_subscription(
            String, '/cns/neural_input', self.listener_callback, 10)
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
        """Load chat history from file"""
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
            with open(self.chat_history_file, 'w') as f:
                json.dump(data, f, indent=2)
            self.get_logger().info("Chat history saved")
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

    def process_with_ollama(self, prompt: str):
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
                'http://localhost:11434/api/chat',
                json={
                    "model": GCE,
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
            
            # Process streaming response - Send deltas (new characters only)
            full_response = ""
            is_first_chunk = True
            
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
