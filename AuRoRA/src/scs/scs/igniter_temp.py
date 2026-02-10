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
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import Log
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# AGi modules
from scs.eee import get_logger, EEEAggregator
from scs.eee_ros_bridge import EEEROSBridge, ReflexPlugin, AwarenessPlugin

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
    def init(self):
        super().init('igniter')
        self.eee = get_logger("SCS.IGNITER")
        
    def shutdown(self):
        """Cleanup plugins."""
        EEEAggregator.shutdown()

#--- THE BOOTSTRAP (IGNITER) ---

def main(args=None):
    """
    This is the Ignition Sequence.
    """
    rclpy.init(args=args)

    # 1. Instantiate both nodes
    # The Bridge handles the ROS network plumbing
    bridge_node = EEEROSBridge()

    # The Igniter handles the robot's logic and plugin management
    igniter_node = Igniter()
    
    # 2. Use the Executor to run them together
    # This is the "Party Bus" that carries both nodes
    executor = MultiThreadedExecutor()
    executor.add_node(bridge_node)
    executor.add_node(igniter_node)
    
    logger = get_logger("SCS.BOOTSTRAP")
    logger.info("Ignition sequence started. 🍺")
    
    try:
        #This spins BOTH nodes simultaneously
        executor.spin()

    except KeyboardInterrupt:
        logger.warning("Shutdown requested by human meatbag.")
    finally:
        # Cleanup everything
        executor.shutdown()
        igniter_node.shutdown()
        bridge_node.destroy_node()
        igniter_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()