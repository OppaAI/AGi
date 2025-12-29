#!/usr/bin/env python3

"""
Vital Pulse Analyzer:
- Receives VitalPulse msgs from robots
- Tracks connected robots/users
- Detects timeouts/disconnections
- Displays vital signs in the terminal
"""

import rclpy
from rclpy.node import Node
import threading
import time

from vp.msg import VitalPulse  # <-- 使用自訂訊息
from vcs.vcc import VitalCentralCore

RESET_COLOR = "\033[0m"

ROBOT_ID = "AuRoRA_Zero_Prototype"
SERVER_ID = "AIVA"
SYSTEM_NAME = "VCS"
NODE_NAME = "vital_pulse_analyzer"
SUBSCRIBE_TOPIC = "vital_pulse"
FEEDBACK_TOPIC = "vital_feedback"
TIMEOUT_SEC = 1.0  # timeout for robot disconnect
BASELINE_OPM = 60.0

class VitalPulseAnalyzer(Node):
    def __init__(self):
        super().__init__(NODE_NAME, namespace=ROBOT_ID + "/" + SYSTEM_NAME)
        self.vc = VitalCentralCore(blink_duration=60/BASELINE_OPM)

        qos_profile = 5
        self.subscriber_ = self.create_subscription(
            VitalPulse,
            SUBSCRIBE_TOPIC,
            self.pulse_callback,
            qos_profile
        )
        self.publisher_ = self.create_publisher(VitalPulse, FEEDBACK_TOPIC, qos_profile)

        # 顯示 thread
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()

    def pulse_callback(self, msg: VitalPulse):
        robot_id = msg.robot_id
        user_id = msg.user_id
        pulse = float(msg.vital_pulse_opm)
        timestamp = msg.timestamp

        # ⭐ store identity + pulse
        self.vc.record_identity(robot_id, user_id)
        self.vc.record_remote_pulse(pulse, timestamp)

        # ⭐ 發 feedback
        feedback = VitalPulse()
        feedback.robot_id = ROBOT_ID
        feedback.user_id = robot_id
        feedback.timestamp = self.get_clock().now().nanoseconds / 1e9
        feedback.vital_pulse_opm = pulse
        self.publisher_.publish(feedback)

    def display_loop(self):
        while rclpy.ok():
            now = self.vc.get_now_sec()

            # ⭐ timeout check
            if self.vc.last_pulse_time is None or (now - self.vc.last_pulse_time) > TIMEOUT_SEC:
                self.vc.clear_identity_and_pulse()

            status = self.vc.get_status()
            pulse = status["opm"]
            color = status["color"]
            robot_id = self.vc.robot_id
            user_id = self.vc.user_id

            if robot_id == "Unknown":
                line = f"\r{color}Waiting for vital pulse...{RESET_COLOR}".ljust(100)
            else:
                line = f"\r{color}Robot [{robot_id}] | User [{user_id}] | Vital Pulse: {pulse:.1f} OPM{RESET_COLOR}".ljust(100)

            print(line, end="", flush=True)
            time.sleep(0.05)  # 20Hz refresh

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
