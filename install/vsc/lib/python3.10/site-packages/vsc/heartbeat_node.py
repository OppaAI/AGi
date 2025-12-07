#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

RED = "\033[91m"
RESET = "\033[0m"

class HeartbeatNode(Node):
    def __init__(self):
        super().__init__('heartbeat_node')
        self.publisher_ = self.create_publisher(String, 'heartbeat', 10)
        self.show_heart = True
        self.timer = self.create_timer(0.5, self.timer_callback)  # 2 Hz

    def timer_callback(self):
        if self.show_heart:
            # Show heart and red text
            print(f"\r{'❤️':<3} {RED}Heartbeat{RESET}", end="", flush=True)
        else:
            # Heart disappears, clear line (keep text blank)
            print(f"\r{'':<2} {RED}Heartbeat{RESET}", end="", flush=True)  # 2 Hz

        self.show_heart = not self.show_heart  # toggle for next beat

        # Publish heartbeat message
        msg = String()
        msg.data = "heartbeat"
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nHeartbeat node stopped.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
