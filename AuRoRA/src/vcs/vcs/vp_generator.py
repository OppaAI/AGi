#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import json
from rclpy.qos import QoSProfile

# ---------------- CONSTANTS ----------------
ROBOT_ID = "AuRoRA_Zero_Prototype"
USER_ID = "OppaAI"
PUBLISHER_TOPIC = "vital_pulse"
FEEDBACK_TOPIC = "vital_feedback"
NODE_NAME = "vital_pulse_generator"

INTERVAL = 1.0  # seconds per pulse (~60 opm resting heart rate)
ASCII_HEART = "❤"

# Cyan color codes
DARK_CYAN = "\033[38;5;30m"
BRIGHT_CYAN = "\033[38;5;51m"
BRIGHT_DURATION = 1.0  # seconds after feedback to stay bright

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
        opm = (1 / INTERVAL) * 60

        # Publish message
        msg = String()
        msg.data = json.dumps({
            "robot_id": ROBOT_ID,
            "user_id": USER_ID,
            "robot_time": now,
            "vital_pulse_opm": opm
        })
        self.publisher_.publish(msg)

        # Blink heart
        heart_display = ASCII_HEART if self.step % 2 == 0 else " "

        # Determine color based on last feedback
        color = BRIGHT_CYAN if self.last_feedback_time and (now - self.last_feedback_time) <= BRIGHT_DURATION else DARK_CYAN

        # Print on the same line (overwrite)
        print(f"\r{color}{heart_display} Vital Pulse: {opm:.1f} OPM", end="", flush=True)

        self.step += 1

    def feedback_callback(self, msg):
        self.last_feedback_time = time.time()
        try:
            data = json.loads(msg.data)
            opm = data.get('opm', 0)
        except:
            opm = 0

        # Overwrite the same line instead of printing new line
        heart_display = ASCII_HEART if self.step % 2 == 0 else " "
        color = BRIGHT_CYAN

        print(f"\r{color}{heart_display} Vital Pulse: {opm:.1f} OPM", end="", flush=True)


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
