#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import json

class VTCOscillator(Node):
    def __init__(self):
        super().__init__('vtc_oscillator')

        self.publisher_ = self.create_publisher(String, 'vtc_oscillator', 10)
        self.feedback_sub = self.create_subscription(
            String,
            'vcc_feedback',
            self.feedback_callback,
            10
        )

        self.interval = 0.2  # faster for smoother wave
        self.timer = self.create_timer(self.interval, self.send_pulse)

        # wave pattern (can be repeated to create longer wave)
        self.wave_pattern = list("ـــــﮩ٨ـ")
        self.repeat_count = 3  # number of waves
        self.step = 0

    def send_pulse(self):
        now = time.time()
        msg = String()
        msg.data = json.dumps({"robot_time": now})
        self.publisher_.publish(msg)

        # create repeated wave pattern
        full_wave = self.wave_pattern * self.repeat_count

        # rotate the wave for animation
        display = ''.join(full_wave[self.step:] + full_wave[:self.step])
        print(f"\r{display}", end='', flush=True)

        self.step = (self.step + 1) % len(self.wave_pattern)

    def feedback_callback(self, msg):
        data = json.loads(msg.data)
        bpm = data.get('bpm', 0)
        print(f"  | Server BPM: {bpm:.1f}", end='', flush=True)

def main(args=None):
    rclpy.init(args=args)
    node = VTCOscillator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nVTC Vital Pulse stopped.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
