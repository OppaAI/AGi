"""
Igniter (Temporary) Module

Welcome to my Igniter Module! 
Currently, this temporary module is designed to simulate the ignition sequence (bootstrap) of the robot's system.
It serves as a placeholder until the full Igniter Module is developed.

This module includes the following features that load at startup:
    - TallEEE: Centralized Triple-Aggregation Logger & Ledger Emergency and Exception Entry (TALLEEE) for logging and handling critical events.
"""

# System modules
from enum import Enum

# ROS2 modules
import rclpy
from rclpy.node import Node

# AGi modules
from scs.eee import get_logger, EEEAggregator
from scs.eee_plugins import ReflexPlugin, AwarenessPlugin  # Optional!

# State of all modules for the robot
class StateOfModule(str, Enum):
    INIT = "INIT"
    RUN = "RUN"
    DEGRADED = "DEGRADED"
    OFF = "OFF"


class Igniter(Node):
    """
    This is the ROS 2 Wrapper. It acts as a bridge between ROS and your TALLEEE system.
    """
    super().__init__('igniter')
        
        # Load EEE ROS plugins (optional - comment out to disable)
        self.plugins = []
        
        try:
            self.plugins.append(ReflexPlugin(self))
            self.plugins.append(AwarenessPlugin(self))
            self.get_logger().info("EEE ROS plugins loaded")
        except Exception as e:
            self.get_logger().warn(f"EEE plugins failed: {e}")
            self.get_logger().warn("Falling back to disk-only logging")
        
        self.get_logger().info("Igniter ready")
    
    def shutdown(self):
        """Cleanup plugins."""
        for plugin in self.plugins:
            plugin.shutdown()
        EEEAggregator.shutdown()

# --- THE BOOTSTRAP (IGNITER) ---
def main(args=None):
    rclpy.init(args=args)
    node = Igniter()
    
    try:
        rclpy.spin(node)
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
