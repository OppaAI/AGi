#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

RED = "\033[91m"
RESET = "\033[0m"

class Heartbeat(Node):
    def __init__(self):
        super().__init__('heartbeat')
        self.publisher_ = self.create_publisher(String, 'heartbeat', 10)
        self.show_heart = True
        self.interval = 0.8  # seconds per beat
        self.bpm = 60 / self.interval  # calculate beats per minute
        self.timer = self.create_timer(self.interval, self.timer_callback)

    def timer_callback(self):
        if self.show_heart:
            # Show heart and red text with bpm
            print(f"\r{'❤️':<3} {RED}Heartbeat at Rate: {self.bpm:.0f} BPM{RESET}", end="", flush=True)
            # Beep sound
            print("\a", end="", flush=True)  # ASCII bell
        else:
            # Heart disappears, text remains
            print(f"\r{'':<2} {RED}Heartbeat at Rate: {self.bpm:.0f} BPM{RESET}", end="", flush=True)

        self.show_heart = not self.show_heart  # toggle for next beat

        # Publish heartbeat message
        msg = String()
        msg.data = "heartbeat"
        self.publisher_.publish(msg)

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
