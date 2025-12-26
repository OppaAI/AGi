#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from rclpy.qos import QoSProfile

# ANSI color codes
DARK_CYAN = "\033[38;5;30m"
BRIGHT_CYAN = "\033[38;5;51m"
RESET_COLOR = "\033[0m"

ROBOT_NAMESPACE = "AuRoRA_Zero_Prototype"
NODE_NAME = "vital_pulse_analyzer"
SUBSCRIBE_TOPIC = "vital_pulse"
FEEDBACK_TOPIC = "vital_feedback"
BLINK_DURATION = 1.0

class VitalPulseAnalyzer(Node):
    def __init__(self):
        super().__init__(NODE_NAME, namespace=ROBOT_NAMESPACE)
        qos_profile = QoSProfile(depth=10)

        self.subscriber_ = self.create_subscription(String, SUBSCRIBE_TOPIC, self.pulse_callback, qos_profile)
        self.publisher_ = self.create_publisher(String, FEEDBACK_TOPIC, qos_profile)

        self.last_blink_start_time = None

    def get_now_sec(self):
        now = self.get_clock().now()
        return now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] / 1e9

    def pulse_callback(self, msg):
        now = self.get_now_sec()
        try:
            data = json.loads(msg.data)
            robot_id = data.get("robot_id", "Unknown")
            user_id = data.get("user_id", "Unknown")
            pulse = data.get("vital_pulse_opm", 0.0)
        except:
            self.get_logger().error("Failed to decode JSON from pulse message")
            return

        self.last_blink_start_time = now

        # Send feedback
        feedback_msg = String()
        feedback_msg.data = json.dumps({"opm": pulse, "timestamp": now})
        self.publisher_.publish(feedback_msg)

        # Blink color logic
        elapsed = now - self.last_blink_start_time
        color = BRIGHT_CYAN if elapsed <= BLINK_DURATION else DARK_CYAN

        print(f"\r{color}Robot ID: [{robot_id}] | User: {user_id} | Vital Pulse: {pulse:.1f} OPM | Received at: {now:.2f}s {RESET_COLOR}".ljust(100), flush=True)

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
