#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

RED = "\033[91m"
RESET = "\033[0m"

class VCCOscillator(Node):
    def __init__(self):
        super().__init__('vcc_oscillator')
        self.publisher_ = self.create_publisher(String, 'vcc_oscillator', 10)
        
        self.show_heart = True
        self.interval = 0.8  # seconds per beat (initial)
        self.bpm = 60 / self.interval

        # last time receiving robots vcc feedback
        self.last_feedback_time = None

        self.timer = self.create_timer(self.interval, self.timer_callback)

        # Subscribe to feedback from robots
        self.feedback_sub = self.create_subscription(
            String,
            'vcc_feedback',
            self.feedback_callback,
            10
        )

    def timer_callback(self):
        # Show vcc vital pulse animation with updated bpm
        if self.show_heart:
            print(f"\r{'❤️':<3} {RED}VCC Pulse Rate: {self.bpm:.1f} BPM{RESET}", end="", flush=True)
        else:
            print(f"\r{'':<2} {RED}VCC Pulse Rate: {self.bpm:.1f} BPM{RESET}", end="", flush=True)

        self.show_heart = not self.show_heart

        # Publish vcc vital pulse
        msg = String()
        msg.data = "vcc_oscillator"
        self.publisher_.publish(msg)

    def feedback_callback(self, msg):
        now = time.time()

        if self.last_feedback_time is not None:
            interval = now - self.last_feedback_time

            if interval > 0:
                hz = 1.0 / interval
                self.bpm = hz * 60

        self.last_feedback_time = now

def main(args=None):
    rclpy.init(args=args)
    node = VCCOscillator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nVCC Pulse stopped.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
