#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import threading
import time
from vtc import VitalTerminalCore

RESET_COLOR = "\033[0m"
ASCII_HEART = "❤"

ROBOT_ID = "AuRoRA_Zero_Prototype"
USER_ID = "OppaAI"
PUBLISHER_TOPIC = "vital_pulse"
FEEDBACK_TOPIC = "vital_feedback"

class VitalPulseGenerator(Node):
    def __init__(self):
        super().__init__("vital_pulse_generator", namespace=ROBOT_ID)
        self.vc = VitalTerminalCore(interval=1.0, blink_duration=1.0)

        qos_profile = 10
        self.publisher_ = self.create_publisher(String, PUBLISHER_TOPIC, qos_profile)
        self.feedback_sub = self.create_subscription(String, FEEDBACK_TOPIC, self.feedback_callback, qos_profile)

        self.timer = self.create_timer(self.vc.interval, self.send_pulse)

        # Display thread
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
        self.vc.step += 1

    def feedback_callback(self, msg):
        self.vc.last_feedback_time = self.vc.get_now_sec(self)
        try:
            data = json.loads(msg.data)
            rtt = (self.vc.last_feedback_time - self.vc.fire_timestamp) * 1000 if self.vc.fire_timestamp else 0
        except:
            rtt = 0
        self.vc.current_rtt = rtt

    def display_loop(self):
        while rclpy.ok():
            now = self.vc.get_now_sec(self)
            heart_display = ASCII_HEART if self.vc.step % 2 == 0 else " "
            color = self.vc.blink_color(now)
            opm = self.vc.current_opm
            rtt = self.vc.current_rtt
            print(f"\r{color}{heart_display} Vital Pulse: {opm:.1f} OPM | RTT: {rtt:.0f}ms {RESET_COLOR}".ljust(100), end="", flush=True)
            time.sleep(0.05)

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
