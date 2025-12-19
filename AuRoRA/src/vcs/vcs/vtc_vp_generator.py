#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import json

# ---------------- CONSTANTS ----------------
ROBOT_ID = "AuRoRA_Zero_Prototype"  # safe for ROS
PUBLISHER_TOPIC = "vtc_vp_generator"
FEEDBACK_TOPIC = "vcc_vital_feedback"
NODE_NAME = "vtc_vp_generator_node"
INTERVAL = 0.8  # seconds per pulse (~75 BPM resting heart rate)

ASCII_HEART = "❤"  # heart symbol

# -------------------------------------------

class VTCPulseGenerator(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        self.publisher_ = self.create_publisher(String, PUBLISHER_TOPIC, 10)
        self.feedback_sub = self.create_subscription(
            String,
            FEEDBACK_TOPIC,
            self.feedback_callback,
            10
        )

        self.timer = self.create_timer(INTERVAL, self.send_pulse)
        self.show_heart = True
        self.bpm = 0.0  # calculated from interval
        self.step = 0   # for simple "pulse" animation

    def send_pulse(self):
        now = time.time()
        # Calculate pulses per minute from interval
        self.bpm = (1 / INTERVAL) * 60  # oscillations per minute (opm)

        # Publish JSON message with robot time and pulse rate
        msg = String()
        msg.data = json.dumps({
            "robot_id": ROBOT_ID,
            "robot_time": now,
            "vital_pulse_opm": self.bpm
        })
        self.publisher_.publish(msg)

        # Simple pulse effect: alternate showing heart bigger/smaller
        heart_display = ASCII_HEART * (1 + self.step % 2)

        # Print pulse in terminal
        print(f"\r{heart_display} Vital Pulse oscillation rate: {self.bpm:.1f} opm", end="", flush=True)

        self.step += 1

    def feedback_callback(self, msg):
        data = json.loads(msg.data)
        bpm = data.get('bpm', 0)
        print(f"  | Server BPM: {bpm:.1f}", flush=True)

def main(args=None):
    rclpy.init(args=args)
    node = VTCPulseGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nVTC Vital Pulse stopped.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
