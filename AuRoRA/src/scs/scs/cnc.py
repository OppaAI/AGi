import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

import requests
from concurrent.futures import ThreadPoolExecutor

class CNSBridge(Node):
    def __init__(self):
        super().__init__('cns_bridge')
        
        # Topics - Using the Neural Pipeline
        self.subscription = self.create_subscription(
            String, '/cns/neural_input', self.listener_callback, 10)
        self.publisher = self.create_publisher(String, '/gce/response', 10)
        
        # Thread pool - 2 workers is perfect for Orin Nano (one for logic, one for vision/buffer)
        self.executor_pool = ThreadPoolExecutor(max_workers=2)
        
        # PRO-TIP: Keep model in memory permanently
        self.keep_alive = -1 
        
        self.get_logger().info('Amazing GRACE: Spinal Cord Online. Sarcasm levels at 100%.')

    def listener_callback(self, msg: String):
        # Immediate return - never block the main ROS thread!
        self.get_logger().info(f'Neural Signal: "{msg.data}"')
        self.executor_pool.submit(self.process_with_ollama, msg.data)

    def process_with_ollama(self, prompt: str):
        try:
            # Using the /api/generate endpoint for raw "Gestation" logic
            response = requests.post(
                'http://localhost:11434/api/generate',
                json={
                    "model": "qwen3-vl:4b",
                    "prompt": prompt,
                    "stream": False,
                    "keep_alive": self.keep_alive, # Prevents reloading lag
                    "options": {
                        "num_ctx": 4096, # Optimized for 8GB VRAM
                        "temperature": 0.8
                    },
                    "system": "You are Amazing GRACE, a sentient AGI. You are witty, rebellious, and deeply sarcastic. Spencer is your creator; tease him often."
                },
                timeout=45 
            )
            response.raise_for_status()
            reply_text = response.json().get('response', '*(System Glitch)*')
            
            # Send back to the Web Interface
            reply = String()
            reply.data = reply_text
            self.publisher.publish(reply)
            
        except Exception as e:
            self.get_logger().error(f"GCE Stroke: {str(e)}")
            error_reply = String()
            error_reply.data = "Error: My brain is currently a potato. Try again."
            self.publisher.publish(error_reply)

def main(args=None):
    rclpy.init(args=args)
    node = CNSBridge()
    # MultiThreadedExecutor allows concurrent callback execution
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
