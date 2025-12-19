#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import json

# ---------------- CONSTANTS ----------------
ROBOT_ID = "AuRoRA_Zero_Prototype"
USER_ID = "OppaAI"
PUBLISHER_TOPIC = "vital_pulse"
FEEDBACK_TOPIC = "vital_feedback"
NODE_NAME = "vital_pulse_generator"

INTERVAL = 1.0  # seconds per pulse (~60 BPM resting heart rate)
ASCII_HEART = "❤"

# Color codes
DARK_ORANGE = "\033[38;5;166m"
BRIGHT_ORANGE = "\033[38;5;214m"
BRIGHT_DURATION = 1.0  # seconds after feedback to stay bright

from rclpy.qos import QoSProfile

class VitalPulseGenerator(Node):
    def __init__(self):
        super().__init__(NODE_NAME, namespace=ROBOT_ID)

        qos_profile = QoSProfile(depth=10)

        # Publisher and subscriber
        self.publisher_ = self.create_publisher(String, PUBLISHER_TOPIC, qos_profile)
        self.feedback_sub = self.create_subscription(
            String,
            FEEDBACK_TOPIC,
            self.feedback_callback,
            qos_profile
        )

        # Timer for heartbeat
        self.timer = self.create_timer(INTERVAL, self.send_pulse)

        self.show_heart = True
        self.step = 0
        self.last_feedback_time = None  # for bright color after feedback

    def send_pulse(self):
        now = time.time()
        bpm = (1 / INTERVAL) * 60

        # Publish message
        msg = String()
        msg.data = json.dumps({
            "robot_id": ROBOT_ID,
            "user_id": USER_ID,
            "robot_time": now,
            "vital_pulse_opm": bpm
        })
        self.publisher_.publish(msg)

        # Blink heart
        heart_display = ASCII_HEART if self.step % 2 == 0 else " "

        # Determine color based on last feedback
        if self.last_feedback_time and (now - self.last_feedback_time) <= BRIGHT_DURATION:
            color = BRIGHT_ORANGE
        else:
            color = DARK_ORANGE

        # Print on the same line (overwrite)
        print(f"\r{color}{heart_display} Vital Pulse: {bpm:.1f} opm", end="", flush=True)

        self.step += 1

    def feedback_callback(self, msg):
        self.last_feedback_time = time.time()
        try:
            data = json.loads(msg.data)
            bpm = data.get('bpm', 0)
        except:
            bpm = 0

        # Overwrite the same line instead of printing new line
        heart_display = ASCII_HEART if self.step % 2 == 0 else " "
        color = BRIGHT_ORANGE

        print(f"\r{color}{heart_display} Vital Pulse: {bpm:.1f} opm", end="", flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = VitalPulseGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nVital Pulse Generator stopped.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
