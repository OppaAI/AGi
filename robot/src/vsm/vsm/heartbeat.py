#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

RED = "\033[91m"
RESET = "\033[0m"

class RobotHeartbeat(Node):
    def __init__(self):
        super().__init__('heartbeat')
        self.publisher_ = self.create_publisher(String, 'heartbeat', 10)
        self.show_heart = True
        self.interval = 0.8  # seconds per beat
        self.bpm = 60 / self.interval
        self.timer = self.create_timer(self.interval, self.timer_callback)

    def timer_callback(self):
        if self.show_heart:
            print(f"\r{'❤️':<3} {RED}Heartbeat at {self.bpm:.0f} BPM{RESET}", end="", flush=True)
        else:
            print(f"\r{'':<2} {RED}Heartbeat at {self.bpm:.0f} BPM{RESET}", end="", flush=True)
        self.show_heart = not self.show_heart

        msg = String()
        msg.data = "heartbeat"
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotHeartbeat()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nHeartbeat stopped.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
