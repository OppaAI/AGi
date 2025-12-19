#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import json

# ---------------- CONSTANTS ----------------
ROBOT_ID = "AuRoRA_Zero_Prototype"  # used as namespace
USER_ID = "OppaAI"
PUBLISHER_TOPIC = "vital_pulse"     # topic inside namespace
FEEDBACK_TOPIC = "vital_feedback"   # topic inside namespace
NODE_NAME = "vtc_vp_generator"      # node name only, no slashes

INTERVAL = 1.0  # seconds per pulse (~60 BPM resting heart rate)
ASCII_HEART = "❤"  # heart symbol

# -------------------------------------------
# Add color codes
DARK_ORANGE = "\033[38;5;166m"
BRIGHT_ORANGE = "\033[38;5;214m"
RESET = "\033[0m"

class VitalPulseGenerator(Node):
    def __init__(self):
        super().__init__(NODE_NAME, namespace=ROBOT_ID)

        self.publisher_ = self.create_publisher(String, PUBLISHER_TOPIC, 10)
        self.feedback_sub = self.create_subscription(
            String,
            FEEDBACK_TOPIC,
            self.feedback_callback,
            10
        )

        self.timer = self.create_timer(INTERVAL, self.send_pulse)
        self.show_heart = True
        self.bpm = 0.0
        self.step = 0
        self.server_online = False  # Track server feedback

    def send_pulse(self):
        now = time.time()
        self.bpm = (1 / INTERVAL) * 60

        msg = String()
        msg.data = json.dumps({
            "robot_id": ROBOT_ID,
            "robot_time": now,
            "vital_pulse_opm": self.bpm
        })
        self.publisher_.publish(msg)

        # Blink heart effect
        heart_display = ASCII_HEART if self.step % 2 == 0 else " "

        # Choose color based on server status
        color = BRIGHT_ORANGE if self.server_online else DARK_ORANGE

        # Print pulse in terminal with color
        print(f"\r{color}{heart_display} Vital Pulse oscillation rate: {self.bpm:.1f} opm{RESET}", end="", flush=True)

        self.step += 1

    def feedback_callback(self, msg):
        data = json.loads(msg.data)
        bpm = data.get('bpm', 0)
        self.server_online = True  # Server is online if feedback received
        print(f"\n  | Server BPM: {bpm:.1f}", flush=True)

def main(args=None):
    rclpy.init(args=args)
    node = VitalPulseGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nVTC Vital Pulse stopped.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
