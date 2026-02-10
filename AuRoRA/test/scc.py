# scc.py (ROS node that USES LLM service)

"""
SCC - ROS wrapper for LLM service

Exposes LLM as ROS service/action.
Does NOT block ROS event loop.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import String

from scs.gcc import GCC

class SCC(Node):
    """
    SCC - Robot's cognitive center
    
    Uses LLM service in background, exposes ROS interface.
    """
    
    def __init__(self):
        super().__init__('scc')
        
        # Start LLM service (non-blocking)
        self.llm = GCC(model_name="granite-tiny")
        self.llm.start()
        
        # ROS interfaces
        self.thought_publisher = self.create_publisher(
            String,
            'scc/thought',
            10
        )
        
        self.query_subscriber = self.create_subscription(
            String,
            'scc/query',
            self.handle_query,
            10
        )
        
        self.get_logger.info("SCC activated - cognitive functions online")
    
    def handle_query(self, msg: String):
        """Handle incoming query (non-blocking)"""
        query = msg.data
        self.get_logger.debug(f"Received query: {query}")
        
        # Ask LLM (non-blocking, callback when ready)
        self.llm.ask(
            prompt=query,
            callback=self.publish_thought
        )
    
    def publish_thought(self, answer: str):
        """Publish LLM response"""
        self.get_logger.debug(f"Publishing thought: {answer}")
        
        msg = String()
        msg.data = answer
        self.thought_publisher.publish(msg)
    
    def shutdown(self):
        """Clean shutdown"""
        self.llm.stop()
        self.get_logger.info("SCC deactivated")

def main(args=None):
    rclpy.init(args=args)
    node = SCC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
