#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from vital_core import VitalCore

RESET_COLOR = "\033[0m"
ASCII_HEART = "❤"
SUBSCRIBE_TOPIC = "vital_pulse"
FEEDBACK_TOPIC = "vital_feedback"
NODE_NAME = "vital_pulse_analyzer"

class VitalPulseAnalyzer(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.vc = VitalCore(blink_duration=1.0)
        qos_profile = 10

        self.subscriber_ = self.create_subscription(String, SUBSCRIBE_TOPIC, self.pulse_callback, qos_profile)
        self.publisher_ = self.create_publisher(String, FEEDBACK_TOPIC, qos_profile)

    def pulse_callback(self, msg):
        now = self.vc.get_now_sec(self)
        try:
            data = json.loads(msg.data)
            robot_id = data.get("robot_id", "Unknown")
            user_id = data.get("user_id", "Unknown")
            pulse = data.get("vital_pulse_opm", 0.0)
            timestamp = data.get("timestamp", None)
        except json.JSONDecodeError:
            self.get_logger().error("Failed to decode JSON")
            return

        self.vc.last_pulse_time = now
        self.vc.current_opm = pulse
        self.vc.compute_rtt(timestamp, now)

        # Send feedback
        feedback_msg = String()
        feedback_msg.data = json.dumps({"opm": pulse, "timestamp": now})
        self.publisher_.publish(feedback_msg)

        # Display terminal
        heart_display = ASCII_HEART if int(now*2) % 2 == 0 else " "
        color = self.vc.blink_color(now)
        print(f"\r{color}{heart_display} Robot [{robot_id}] | Vital Pulse: {pulse:.1f} OPM | RTT: {self.vc.current_rtt:.0f}ms {RESET_COLOR}".ljust(100), end="", flush=True)

def main(args=None):
    rclpy.init(args=args)
    node = VitalPulseAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nVital Pulse Analyzer stopped.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
