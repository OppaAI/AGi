#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

RED = "\033[91m"
RESET = "\033[0m"

class CoreHeartbeat(Node):
    def __init__(self):
        super().__init__('core_heartbeat')
        self.publisher_ = self.create_publisher(String, 'core_heartbeat', 10)
        self.show_heart = True
        self.interval = 0.8  # seconds per beat
        self.bpm = 60 / self.interval
        self.timer = self.create_timer(self.interval, self.timer_callback)

        # Subscribe to feedback from robots
        self.feedback_sub = self.create_subscription(
            String,
            'heartbeat_feedback',
            self.feedback_callback,
            10
        )

    def timer_callback(self):
        if self.show_heart:
            print(f"\r{'❤️':<3} {RED}Heartbeat at Rate: {self.bpm:.0f} BPM{RESET}", end="", flush=True)
            print("\a", end="", flush=True)  # ASCII bell
        else:
            print(f"\r{'':<2} {RED}Heartbeat at Rate: {self.bpm:.0f} BPM{RESET}", end="", flush=True)
        self.show_heart = not self.show_heart

        # Publish core heartbeat
        msg = String()
        msg.data = "core_heartbeat"
        self.publisher_.publish(msg)

    def feedback_callback(self, msg):
        now = time.strftime("%H:%M:%S", time.localtime())
        print(f"\n{RED}Feedback received at {now}: {msg.data}{RESET}")

def main(args=None):
    rclpy.init(args=args)
    node = CoreHeartbeat()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nCore Heartbeat stopped.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
