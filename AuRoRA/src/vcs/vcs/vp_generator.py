#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
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

# ANSI color codes
DARK_CYAN = "\033[38;5;30m"
BRIGHT_CYAN = "\033[38;5;51m"
RESET_COLOR = "\033[0m"
BRIGHT_DURATION = 1.0  # seconds after feedback to stay bright

class VitalPulseGenerator(Node):
    def __init__(self):
        super().__init__(NODE_NAME, namespace=ROBOT_ID)

        qos_profile = QoSProfile(depth=10)
        self.publisher_ = self.create_publisher(String, PUBLISHER_TOPIC, qos_profile)
        self.feedback_sub = self.create_subscription(String, FEEDBACK_TOPIC, self.feedback_callback, qos_profile)

        self.timer = self.create_timer(INTERVAL, self.send_pulse)
        self.step = 0
        self.last_feedback_time = None
        self.fire_timestamp = None

    def get_now_sec(self):
        now = self.get_clock().now()
        return now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] / 1e9

    def send_pulse(self):
        now = self.get_now_sec()
        self.fire_timestamp = now
        opm = (1 / INTERVAL) * 60

        # Publish pulse message
        msg = String()
        msg.data = json.dumps({
            "robot_id": ROBOT_ID,
            "user_id": USER_ID,
            "timestamp": now,
            "vital_pulse_opm": opm
        })
        self.publisher_.publish(msg)

        # Blink heart logic
        heart_display = ASCII_HEART if self.step % 2 == 0 else " "
        color = DARK_CYAN
        if self.last_feedback_time:
            elapsed = now - self.last_feedback_time
            if 0 <= elapsed <= BRIGHT_DURATION:
                color = BRIGHT_CYAN

        print(f"\r{color}{heart_display} Vital Pulse: {opm:.1f} OPM | Sent at: {now:.2f}s {RESET_COLOR}", end="", flush=True)
        self.step += 1

    def feedback_callback(self, msg):
        self.last_feedback_time = self.get_now_sec()
        try:
            data = json.loads(msg.data)
            opm = data.get('opm', 0)
            rtt = (self.last_feedback_time - self.fire_timestamp) * 1000 if self.fire_timestamp else 0
        except:
            opm = 0
            rtt = 0

        heart_display = ASCII_HEART if self.step % 2 == 0 else " "
        color = BRIGHT_CYAN
        print(f"\r{color}{heart_display} Vital Pulse: {opm:.1f} OPM | RTT: {rtt:.0f}ms {RESET_COLOR}".ljust(100), end="", flush=True)

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
