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
from scs.eee import get_logger

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
    def __init__(self):
        # We name the ROS node 'igniter'
        super().__init__('igniter')
        
        # Inject the logger into the global namespace
        get_logger()
        
        self.get_logger().info("Igniter Node is spinning. Ready to judge all subsystems.")
        
    def log_report(self, msg):
        """Example method if you want to expose a service later"""
        get_logger().info(f"ROS Node Report: {msg}")

# --- THE BOOTSTRAP (IGNITER) ---
def main(args=None):
    rclpy.init(args=args)
    
    # Initialize the aggregator node
    node = Igniter()
    
    try:
        # Now every other module loaded in this process has access to the logger
        get_logger().info("Ignition sequence complete. Let's break something.")
        rclpy.spin(node)
    except KeyboardInterrupt:
        get_logger().warning("Shutdown signal received. Boring.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()