#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

RED = "\033[91m"
RESET = "\033[0m"

class Heartbeat(Node):
    def __init__(self):
        super().__init__('heartbeat')
        # Publisher for robot's own heartbeat
        self.publisher_ = self.create_publisher(String, 'heartbeat', 10)
        # Publisher for feedback to core
        self.feedback_pub = self.create_publisher(String, 'heartbeat_feedback', 10)

        self.show_heart = True
        self.interval = 0.8
        self.bpm = 60 / self.interval

        self.last_core_heartbeat = None
        self.start_time = time.time()
        self.grace_period = self.interval * 3

        # Subscribe to core heartbeat
        self.subscription = self.create_subscription(
            String,
            'core_heartbeat',
            self.core_heartbeat_callback,
            10
        )

        # Timer for robot heartbeat
        self.timer = self.create_timer(self.interval, self.timer_callback)

    def core_heartbeat_callback(self, msg):
        # Update last received core heartbeat
        self.last_core_heartbeat = time.time()
        # Send feedback to core
        feedback_msg = String()
        feedback_msg.data = "Core heartbeat detected by robot"
        self.feedback_pub.publish(feedback_msg)

    def timer_callback(self):
        # Check if core heartbeat is alive
        core_alive = False
        if self.last_core_heartbeat is not None:
            core_alive = (time.time() - self.last_core_heartbeat) < (self.interval * 2)

        node_age = time.time() - self.start_time
        if core_alive:
            display_text = "Core Heartbeat detected"
        elif node_age >= self.grace_period:
            # Publish own heartbeat if core offline
            msg = String()
            msg.data = "heartbeat"
            self.publisher_.publish(msg)
            display_text = f"Heartbeat at {self.bpm:.0f} BPM"
        else:
            display_text = "Detecting for heartbeat..."

        heart = "❤️ " if self.show_heart else "  "
        print(f"\r{heart:<2} {RED}{display_text}{RESET}", end="", flush=True)
        self.show_heart = not self.show_heart

def main(args=None):
    rclpy.init(args=args)
    node = Heartbeat()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nHeartbeat stopped.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
