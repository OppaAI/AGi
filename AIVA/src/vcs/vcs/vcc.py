#!/usr/bin/env python3

"""
Vital Pulse Analyzer (Self-contained):
- Receives VitalPulse msgs from robots
- Tracks connected robots/users
- Detects timeouts/disconnections
- Displays vital signs in the terminal
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, LivelinessPolicy
from rclpy.duration import Duration

from vp.msg import VitalPulse

# ANSI colors
RESET_COLOR = "\033[0m"
DARK_CYAN = "\033[38;5;30m"
BRIGHT_CYAN = "\033[38;5;51m"

ROBOT_ID = "AuRoRA_Zero_Prototype"
SERVER_ID = "AIVA"
SYSTEM_NAME = "VCS"
NODE_NAME = "vital_pulse_analyzer"
VITAL_PULSE_SIGNAL = "vital_pulse_signal"
VITAL_PULSE_RESPONSE = "vital_pulse_response"
VITAL_PULSE_TIMEOUT = 1.5  # Timeout in seconds
BASELINE_OPM = 60.0

# ------------------ Vital Central Core ------------------
class VitalCentralCore:
    def __init__(self, blink_duration=1.0):
        self.blink_duration = blink_duration

        self.last_pulse_time = None
        self.current_opm = 0.0
        self.current_rtt = 0.0

        # Identity
        self.robot_id = "Unknown"
        self.user_id = "Unknown"
        self.cpu_temp = 0.0
        self.gpu_temp = 0.0

        self._blink_step = 0

    def get_now_sec(self):
        """Return current ROS time in seconds as float"""
        try:
            from rclpy.clock import Clock
            return Clock().now().nanoseconds / 1e9
        except Exception:
            return None

    def record_identity(self, robot_id, user_id):
        self.robot_id = robot_id
        self.user_id = user_id

    def record_remote_pulse(self, opm, remote_ts):
        now = self.get_now_sec()
        self.last_pulse_time = now
        self.current_opm = float(opm)

        if remote_ts is not None:
            try:
                self.current_rtt = max(0.0, (now - float(remote_ts)) * 1000.0)
            except Exception:
                self.current_rtt = 0.0

    def blink_color(self, now):
        if self.last_pulse_time is None:
            return DARK_CYAN
        if (now - self.last_pulse_time) <= self.blink_duration:
            return BRIGHT_CYAN
        return DARK_CYAN

    def get_status(self):
        self._blink_step += 1
        return {
            "opm": self.current_opm,
            "rtt": self.current_rtt,
            "blink": self._blink_step % 2,
            "color": self.blink_color(self.get_now_sec())
        }

    def clear_identity_and_pulse(self):
        self.robot_id = "Unknown"
        self.user_id = "Unknown"
        self.current_opm = 0.0
        self.current_rtt = 0.0
        self.last_pulse_time = None
        self.cpu_temp = 0.0  # Reset on disconnect
        self.gpu_temp = 0.0

# ------------------ Vital Pulse Analyzer ------------------
class VitalPulseAnalyzer(Node):
    def __init__(self):
        super().__init__(NODE_NAME, namespace=ROBOT_ID + "/" + SYSTEM_NAME)
        self.vc = VitalCentralCore(blink_duration=60/BASELINE_OPM)

        # QoS profile
        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            liveliness=LivelinessPolicy.AUTOMATIC,
            liveliness_lease_duration=Duration(seconds=VITAL_PULSE_TIMEOUT * 1.5),
            lifespan=Duration(seconds=VITAL_PULSE_TIMEOUT),
            deadline=Duration(seconds=VITAL_PULSE_TIMEOUT * 1.1)
        )

        # Subscriber & Publisher
        self.subscriber_ = self.create_subscription(
            VitalPulse,
            VITAL_PULSE_SIGNAL,
            self.pulse_callback,
            qos_profile
        )
        self.publisher_ = self.create_publisher(VitalPulse, VITAL_PULSE_RESPONSE, qos_profile)
        self.display_timer = self.create_timer(1.0, self.display_loop)

    def pulse_callback(self, msg: VitalPulse):
        robot_id = msg.robot_id
        user_id = msg.user_id
        self.vc.cpu_temp = msg.cpu_temp
        self.vc.gpu_temp = msg.gpu_temp
        pulse = float(msg.vital_pulse_opm)
        timestamp = rclpy.time.Time.from_msg(msg.timestamp).nanoseconds / 1e9

        # Record identity + pulse
        self.vc.record_identity(robot_id, user_id)
        self.vc.record_remote_pulse(pulse, timestamp)

        # Send feedback
        feedback = VitalPulse()
        feedback.robot_id = ROBOT_ID
        feedback.user_id = robot_id
        feedback.timestamp = self.get_clock().now().to_msg()
        feedback.vital_pulse_opm = pulse
        self.publisher_.publish(feedback)

    def display_loop(self):
        """Continuously print pulse & connection status"""
        now = self.vc.get_now_sec()

        # Timeout / disconnection
        if self.vc.last_pulse_time is None or (now - self.vc.last_pulse_time) > VITAL_PULSE_TIMEOUT:
            self.vc.clear_identity_and_pulse()

        status = self.vc.get_status()
        pulse = status["opm"]
        color = status["color"]
        robot_id = self.vc.robot_id
        user_id = self.vc.user_id
        cpu_temp = self.vc.cpu_temp
        gpu_temp = self.vc.gpu_temp

        if robot_id == "Unknown":
            line = f"{color}Waiting for vital pulse...{RESET_COLOR}".ljust(100)
        else:
            line = f"{color}Robot [{robot_id}] | User [{user_id}] | CPU Temp: {cpu_temp:.1f}°C | GPU Temp: {gpu_temp:.1f}°C | Vital Pulse: {pulse:.1f} OPM{RESET_COLOR}".ljust(100)

        self.get_logger().info(line)

def main(args=None):
    rclpy.init(args=args)
    node = VitalPulseAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Vital Pulse Analyzer stopped.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
