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
import ollama

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

GCE = "hf.co/mradermacher/Huihui-granite-4.0-h-tiny-abliterated-i1-GGUF:Q4_K_M"

class Igniter(Node):
    """
    This is the ROS 2 Wrapper. It acts as a bridge between ROS and your TALLEEE system.
    """
    def __init__(self):
        super().__init__('igniter')
        #self.eee = get_logger("SCS.IGNITER")
        self.model_name = GCE  # Replace with your desired model
        self.messages = []

        # Initialize the model (replace 'granite4-tiny-h' with your desired model)
        try:
            response = ollama.chat(model=self.model_name, messages=self.messages)
            print("Bot:", response.message.content)
        except Exception as e:
            #self.eee.error(f"Error initializing Ollama model: {e}")
            print("Error initializing Ollama model. Check Ollama service.")


    #def shutdown(self):
    #    """Cleanup plugins."""
    #    EEEAggregator.shutdown()

    def run_chat_loop(self):
        while True:
            user_input = input("You: ")
            if not user_input:
                break
            self.messages.append({"role": "user", "content": user_input})
            try:
                response = ollama.chat(model=self.model_name, messages=self.messages)
                answer = response.message.content
                print("Bot:", answer)
            except Exception as e:
                #self.eee.error(f"Error during Ollama chat: {e}")
                print(f"Error during Ollama chat: {e}")

# --- THE BOOTSTRAP (IGNITER) ---
def main(args=None):
    rclpy.init(args=args)
    # 1. Instantiate both nodes
    # The Bridge handles the ROS network plumbing
    #bridge_node = EEEROSBridge()
    # The Igniter handles the robot's logic and plugin management
    igniter_node = Igniter()

    # 2. Use the Executor to run them together
    executor = MultiThreadedExecutor()
    #executor.add_node(bridge_node)
    executor.add_node(igniter_node)

    logger = get_logger("SCS.BOOTSTRAP")
    logger.info("Ignition sequence started. 🍺")

    try:
        executor.spin()
    except KeyboardInterrupt:
        logger.warning("Shutdown requested by human meatbag.")
    finally:
        executor.shutdown()
        igniter_node.shutdown()
        #bridge_node.destroy_node()
        igniter_node.destroy_node()
  
if __name__ == '__main__':
    main()
