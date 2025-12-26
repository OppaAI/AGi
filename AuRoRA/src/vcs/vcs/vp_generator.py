#!/usr/bin/env python3

"""
Vital Pulse Generator
- Periodically publishes lightweight pulses containing telemetry data and timestamp.
- Receives feedback from the server to verify connection.
- Detects timeouts and disconnections.
- Displays vital signs in the terminal.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import threading
import time
from vcs.vtc import VitalTerminalCore

RESET_COLOR = "\033[0m"
ASCII_HEART = "❤"

ROBOT_ID = "AuRoRA_Zero_Prototype"
USER_ID = "OppaAI"
PUBLISHER_TOPIC = "vital_pulse"
FEEDBACK_TOPIC = "vital_feedback"
TIMEOUT_SEC = 1.0  # 斷線重置脈搏和 RTT

class VitalPulseGenerator(Node):
    def __init__(self):
        super().__init__("vital_pulse_generator", namespace=ROBOT_ID)
        self.vc = VitalTerminalCore(interval=1.0, blink_duration=1.0)

        qos_profile = 10
        self.publisher_ = self.create_publisher(String, PUBLISHER_TOPIC, qos_profile)
        self.feedback_sub = self.create_subscription(String, FEEDBACK_TOPIC, self.feedback_callback, qos_profile)
        self.timer = self.create_timer(self.vc.interval, self.send_pulse)

        # ⭐ 初始化心跳步進
        self.heart_step = 0

        # ⭐ Display thread
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()

    def send_pulse(self):
        now = self.vc.get_now_sec(self)
        self.vc.fire_timestamp = now
        self.vc.compute_opm()

        msg = String()
        msg.data = json.dumps({
            "robot_id": ROBOT_ID,
            "user_id": USER_ID,
            "timestamp": now,
            "vital_pulse_opm": self.vc.current_opm
        })
        self.publisher_.publish(msg)

    def feedback_callback(self, msg):
        now = self.vc.get_now_sec(self)
        self.vc.last_feedback_time = now
        try:
            data = json.loads(msg.data)
            rtt = (self.vc.last_feedback_time - self.vc.fire_timestamp) * 1000 if self.vc.fire_timestamp else 0
        except Exception:
            rtt = 0
        self.vc.current_rtt = rtt

    def display_loop(self):
        while rclpy.ok():
            now = self.vc.get_now_sec(self)

            # ⭐ 超過 TIMEOUT_SEC 沒收到 feedback，重置脈搏和 RTT
            if self.vc.last_feedback_time is None or (now - self.vc.last_feedback_time) > TIMEOUT_SEC:
                self.vc.current_opm = 60.0
                self.vc.current_rtt = 0.0

            # ⭐ 心跳動畫每次迴圈都更新
            self.heart_step += 1
            heart_display = ASCII_HEART if self.heart_step % 2 == 0 else " "
            color = self.vc.blink_color(now)
            opm = self.vc.current_opm
            rtt = self.vc.current_rtt
            print(f"\r{color}{heart_display} Vital Pulse: {opm:.2f} OPM | RTT: {rtt:.2f}ms {RESET_COLOR}".ljust(100), end="", flush=True)

            # ⭐ 控制心跳速度，0.5秒切換一次
            time.sleep(0.5)

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
