#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
from rclpy.qos import QoSProfile

# ANSI color codes
DARK_CYAN = "\033[38;5;30m"    # darker cyan
BRIGHT_CYAN = "\033[38;5;51m"  # bright cyan

# Constants
ROBOT_NAMESPACE = "AuRoRA_Zero_Prototype"
NODE_NAME = "vital_pulse_analyzer"
SUBSCRIBE_TOPIC = "vital_pulse"
FEEDBACK_TOPIC = "vital_feedback"
BLINK_DURATION = 1.0

class VitalPulseAnalyzer(Node):
    def __init__(self):
        super().__init__(NODE_NAME, namespace=ROBOT_NAMESPACE)

        qos_profile = QoSProfile(depth=10)

        # Subscriber for robot vital pulse
        self.subscriber_ = self.create_subscription(
            String,
            SUBSCRIBE_TOPIC,
            self.pulse_callback,
            qos_profile
        )

        # Publisher to send feedback
        self.publisher_ = self.create_publisher(String, FEEDBACK_TOPIC, qos_profile)

        self.last_pulse_time = None

    def get_now_sec(self):
        """
        Return current ROS time in seconds as float.
        """
        now = self.get_clock().now()
        return now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] / 1e9

    def pulse_callback(self, msg):
        now = self.get_now_sec()
        try:
            data = json.loads(msg.data)
            robot_id = data.get("robot_id", "Unknown")
            user_id = data.get("user_id", "Unknown")
            pulse = data.get("vital_pulse_opm", 0.0)
        except json.JSONDecodeError:
            self.get_logger().error("Failed to decode JSON from robot pulse message")
            return

        self.last_pulse_time = now

        # Send feedback to robot
        feedback_msg = String()
        feedback_msg.data = json.dumps({"opm": pulse, "timestamp": now})
        self.publisher_.publish(feedback_msg)

        # Determine color based on recent pulse
        if self.last_pulse_time and (now - self.last_pulse_time) <= BLINK_DURATION:
            color = BRIGHT_CYAN
        else:
            color = DARK_CYAN

        # Print terminal display
        print(f"\r{color}Robot ID: [{robot_id}] | User: {user_id} | Vital Pulse: {pulse:.1f} OPM | Received at: {now:.2f}s", flush=True)

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
