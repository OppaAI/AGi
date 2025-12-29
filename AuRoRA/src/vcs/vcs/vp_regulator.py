#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
import time

# AGi modules
from vp.msg import VitalPulse           # Vital Pulse
from vcs.vtc import VitalTerminalCore   # Vital Terminal Core

RESET_COLOR = "\033[0m"
ASCII_HEART = "❤"

ROBOT_ID = "AuRoRA_Zero_Prototype"
SYSTEM_ID = "VCS"
USER_ID = "OppaAI"
PUBLISHER_TOPIC = "vital_pulse"
FEEDBACK_TOPIC = "vital_feedback"
TIMEOUT_SEC = 1.0  # 超過時間沒回應就 reset

class VitalPulseGenerator(Node):
    def __init__(self):
        super().__init__("vital_pulse_generator", namespace=ROBOT_ID + "/" + SYSTEM_ID)
        self.vc = VitalTerminalCore(interval=1.0, blink_duration=1.0)

        qos_profile = 5
        # Publisher
        self.publisher_ = self.create_publisher(VitalPulse, PUBLISHER_TOPIC, qos_profile)
        # Subscriber for feedback
        self.feedback_sub = self.create_subscription(
            VitalPulse,
            FEEDBACK_TOPIC,
            self.feedback_callback,
            qos_profile
        )

        self.timer = self.create_timer(self.vc.interval, self.send_pulse)

        self.heart_step = 0
        self.fire_timestamp = None
        self.current_opm = 60.0
        self.current_rtt = 0.0
        self.vc.last_feedback_time = None

        # Display thread
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()

    def send_pulse(self):
        now = self.vc.get_now_sec(self)

        self.fire_timestamp = now

        self.vc.compute_opm()
        self.current_opm = self.vc.current_opm

        msg = VitalPulse()
        msg.robot_id = ROBOT_ID
        msg.user_id = USER_ID
        msg.timestamp = now
        msg.vital_pulse_opm = self.current_opm
        self.publisher_.publish(msg)


    def feedback_callback(self, msg: VitalPulse):
        """Handle feedback from server and update RTT"""
        if msg.robot_id != ROBOT_ID:
            return  # ignore feedback meant for other robots

        now = self.vc.get_now_sec(self)
        self.vc.last_feedback_time = now

        if self.fire_timestamp is not None:
            self.current_rtt = (now - self.fire_timestamp) * 1000.0  # RTT in ms

    def display_loop(self):
        while rclpy.ok():
            now = self.vc.get_now_sec(self)

            # Reset if feedback timed out
            if self.vc.last_feedback_time is None or (now - self.vc.last_feedback_time) > TIMEOUT_SEC:
                self.current_opm = 60.0
                self.current_rtt = 0.0

            # Heartbeat animation
            self.heart_step += 1
            heart_display = ASCII_HEART if self.heart_step % 2 == 0 else " "
            color = self.vc.blink_color(now)
            opm = self.current_opm
            rtt = self.current_rtt
            print(f"\r{color}{heart_display} Vital Pulse: {opm:.2f} OPM | RTT: {rtt:.2f}ms {RESET_COLOR}".ljust(100), end="", flush=True)

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
