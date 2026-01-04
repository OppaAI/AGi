#!/usr/bin/env python3
# Vital Terminal Core (VTC)
# - Pump: Collects raw vital data and packs into Protobuf
# - Pacemaker: Calculates pulse rhythm according to network RTT
# - Oscillator: Encodes packed vital data into vital pulse signal at the OPM rhythm
# - Regulator: Handles outgoing and incoming vital pulses and updates RTT

# System modules
import time
from jtop import jtop

# ROS 2 modules
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, LivelinessPolicy
from rclpy.duration import Duration

# AGi Vital Pulse definition
from vp.msg import VitalPulse

# Temp constants - to be moved to config file
RESET_COLOR = "\033[0m"
ASCII_HEART = "❤"

ROBOT_ID = "AuRoRA_Zero_Prototype"
SYSTEM_ID = "VCS"
USER_ID = "OppaAI"
VITAL_PULSE_SIGNAL = "vital_pulse_signal"
VITAL_PULSE_RESPONSE = "vital_pulse_response"
VITAL_PULSE_TIMEOUT = 1.0  # Timeout in seconds (Base Interval)
VITAL_PULSE_NETWORK_TIMEOUT = 500  # High latency threshold (ms)
BASELINE_OPM = 60.0

DISPLAY_INTERVAL = 0.5
BLINK_DURATION = 0.3

class Pump:
    """Collects raw system metrics for a Jetson robot using a persistent jtop instance."""
    
    def __init__(self):
        # 1. Initialize jtop monitoring (Start once)
        self.jetson = None
        if jtop:
            try:
                self.jetson = jtop()
                self.jetson.start()
                print("Pump: jtop monitoring started successfully.")
                
                # ⭐ CRITICAL FIX: Add a short delay to allow jtop to collect initial data
                # Without this, the first call to .cpu often returns empty or None data.
                time.sleep(0.5) 
                
            except Exception as e:
                print(f"Pump Initialization Error: Could not start jtop. Error: {e}")
                self.jetson = None
        
        self.data = {}        # stores raw metrics
        self.timestamp = 0    # nanoseconds of collection

    def __del__(self):
        """Ensure jtop is gracefully stopped when the object is deleted/cleaned up."""
        if self.jetson and self.jetson.ok():
            self.jetson.close()
            print("Pump: jtop monitoring stopped.")

    def gather_metrics(self):
        """Collect only CPU temperatures from the live jtop stream."""
        
        # 1. Record the timestamp BEFORE collecting the data
        now_ns = time.time_ns()
        metrics = {}
        
        if not self.jetson or not self.jetson.ok():
            # Fallback when jtop is not running
            metrics = {
                "cpu_temp_per_core": None,
                "cpu_temp_max": None
            }
        else:
            try:
                # 2. Access the currently running data stream (FAST READ)
                cpu_data = self.jetson.cpu 
                
                metrics["cpu_temp_per_core"] = {
                    core: info.get('temp', None)
                    for core, info in cpu_data.items()
                    if isinstance(info, dict)
                }
                
                # Compute max only from valid temps
                valid_temps = [t for t in metrics["cpu_temp_per_core"].values() if t is not None]
                metrics["cpu_temp_max"] = max(valid_temps) if valid_temps else None

            except Exception as e:
                print("Jetson stats data collection error:", e)
                metrics = {
                    "cpu_temp_per_core": None,
                    "cpu_temp_max": None
                }

        self.timestamp = now_ns
        self.data = metrics 
        return metrics

class PaceMaker():
    def __init__(self):
        pass

class Oscillator():
    def __init__(self):
        pass

class Regulator():
    def __init__(self):
        pass

class VitalTerminalCore(Node):
    def __init__(self):
        super().__init__("vital_terminal_core", namespace=ROBOT_ID + "/" + SYSTEM_ID)

        # 1. State Initialization
        self.heart_step = 0
        self.current_opm = BASELINE_OPM
        self.current_rtt = 0.0
        self.heartbeat_interval = 60.0 / BASELINE_OPM # 1.0 second
        self.vc_linked = False
        
        # Tracking variables for RTT and Timeout (FIX)
        self.fire_timestamp = None
        self.last_feedback_time = None # Tracks time of last successful feedback receipt (FIX)

        # Thread-safe snapshot for display
        self.display_snapshot = {}

        pump = Pump()
        data = pump.gather_metrics()
        print(f"[{pump.timestamp}] Pump collected:", data)

        # 2. QoS Profile Definition (Based on our discussion)
        qos_profile = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            liveliness=LivelinessPolicy.AUTOMATIC,
            # Use VITAL_PULSE_TIMEOUT (1.0s) as the base for timing policies
            liveliness_lease_duration=Duration(seconds=VITAL_PULSE_TIMEOUT * 1.5),
            lifespan=Duration(seconds=VITAL_PULSE_TIMEOUT),
            deadline=Duration(seconds=VITAL_PULSE_TIMEOUT * 1.1)
        )

        # 3. Communication Setup
        self.vp_signal_oscillator = self.create_publisher(VitalPulse, VITAL_PULSE_SIGNAL, qos_profile)
        self.vp_response_receptor = self.create_subscription(VitalPulse, VITAL_PULSE_RESPONSE, self.feedback_callback, qos_profile)

        # 4. Timer Setup
        self.timer = self.create_timer(self.heartbeat_interval, self.oscillate_vital_pulse)
        self.display_timer = self.create_timer(DISPLAY_INTERVAL, self.display_tick)

    def oscillate_vital_pulse(self):
        """Oscillator: publish vital pulse signal"""
        current_rclpy_time = self.get_clock().now()
        now_sec_float = current_rclpy_time.nanoseconds * 1e-9
        
        self.fire_timestamp = now_sec_float
        self.current_opm = self.compute_opm()

        msg = VitalPulse()
        msg.robot_id = ROBOT_ID
        msg.user_id = USER_ID
        
        # ⭐ FIX: Assign structured time message
        # Assuming VitalPulse.timestamp is of type builtin_interfaces/msg/Time
        msg.timestamp = now_sec_float
        
        msg.vital_pulse_opm = self.current_opm

        self.vp_signal_oscillator.publish(msg)

    def feedback_callback(self, msg: VitalPulse):
        """Regulator: handle feedback and compute RTT"""
        if msg.robot_id != ROBOT_ID:
            return

        now_sec_float = self.get_clock().now().nanoseconds * 1e-9
        
        # ⭐ FIX: Record successful feedback time
        self.last_feedback_time = now_sec_float 

        if self.fire_timestamp is not None:
            self.current_rtt = (now_sec_float - self.fire_timestamp) * 1000.0  # ms
            
        # Linked is True immediately upon successful receipt
        self.vc_linked = True

    def display_tick(self):
        """Monitor: Update snapshot for display thread, handling timeout (FIX)"""
        now = self.get_now_sec()

        # ⭐ FIX: Explicit Timeout Check (If last feedback is too old, we are offline)
        is_timeout = (
            self.last_feedback_time is None or 
            (now - self.last_feedback_time) > VITAL_PULSE_TIMEOUT
        )
        
        if is_timeout:
            # System is considered disconnected/offline
            self.current_opm = BASELINE_OPM
            self.current_rtt = 0.0
            self.vc_linked = False
        
        # Optional: Reset if RTT is extremely high (Flag high latency even if linked)
        elif self.current_rtt > VITAL_PULSE_NETWORK_TIMEOUT:
             self.vc_linked = True # Still technically linked
        
        self.heart_step += 1

        # Thread-safe snapshot update
        self.display_snapshot = {
            "time": now,
            "heart_step": self.heart_step,
            "opm": self.current_opm,
            "rtt": self.current_rtt,
            "linked": self.vc_linked
        }

        snap = self.display_snapshot.copy() if self.display_snapshot else None

        heart = ASCII_HEART if snap["heart_step"] % 2 == 0 else " "
        color = self.blink_color(snap["time"])
        status = "ONLINE" if snap["linked"] else "OFFLINE"

        self.get_logger().info(
            f"{color}{heart} VTC Monitor: "
            f"{snap['opm']:.2f} OPM | "
            f"RTT: {snap['rtt']:.2f}ms | "
            f"Server Status: {status} "
            f"{RESET_COLOR}".ljust(100)
        )

    def get_now_sec(self):
        """Helper to get current time as float seconds"""
        return self.get_clock().now().nanoseconds * 1e-9

    def compute_opm(self):
        """Pacemaker: convert interval to OPM"""
        return 60.0 / self.heartbeat_interval

    def blink_color(self, now):
        """Color indicator based on linked status"""
        # Blue for online, Dark Blue/Cyan for offline 
        return "\033[38;5;51m" if self.vc_linked else "\033[38;5;30m"

def main(args=None):
    rclpy.init(args=args)
    node = VitalTerminalCore()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\nVital Terminal Core stopped.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()