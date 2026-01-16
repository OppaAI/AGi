#!/usr/bin/env python3
# Vital Terminal Core (VTC)
# - Pump: Collects raw sensor data for different sensors using Triple-Level Polling (TLP) technique
# - Regulator: Normalizes raw sensor data into proper format and regulates the oscillation rhythm according to RTT
# - Oscillator: Packs and encodes sensor data into vital pulse signal and publishes at oscillation rhythm
# - Orchestrator: Monitors vital pulse signal and response, detects disconnections, and triggers safety interlocks

# System modules
import copy
from builtin_interfaces.msg import Time

# ROS 2 modules
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, LivelinessPolicy

# AGi VCS modules
from vp.msg import VitalPulse   # Vital Pulse message definition
from vcs.pump import Pump   # Pump: Collects raw vital data from robot sensors

# Temp constants - to be moved to config file
RESET_COLOR = "\033[0m"
ASCII_HEART = "❤"

ROBOT_ID = "AuRoRA_Zero_Prototype"
SYSTEM_ID = "VCS"
USER_ID = "OppaAI"
VITAL_PULSE_SIGNAL = "vital_pulse_signal"
VITAL_PULSE_RESPONSE = "vital_pulse_response"
VITAL_PULSE_TIMEOUT = 2.0  # Timeout in seconds (Base Interval)
VITAL_PULSE_NETWORK_TIMEOUT = 500  # High latency threshold (ms)
BASELINE_OPM = 60.0

DISPLAY_INTERVAL = 0.5
BLINK_DURATION = 0.3
  
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
        self.vital_dump = {
            "pump": {
                "timestamp": Time(),
                "duration": 0.0,
                "iteration": 0,
                "payload": {}  # raw sensor data
            },
            "regulator": {
                "timestamp": Time(),
                "payload": {}  # normalized/filtered data
            },
            "oscillator": {
                "timestamp": Time(),
                "payload": {}  # final encoded message before publish
            },
            "orchestrator": {
                "timestamp": Time(),
                "payload": {}  # decisions, alerts, safety state
            }
        }

        # Initiate the 4 modules
        self.pump = Pump()  # Pump: Collects raw vital data and packs into Protobuf
        
        # Tracking variables for RTT and Timeout (FIX)
        self.fire_timestamp = None
        self.last_feedback_time = None # Tracks time of last successful feedback receipt (FIX)

        # Thread-safe snapshot for display
        self.display_snapshot = {}

        # 2. QoS Profile Definition (Based on our discussion)
        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            liveliness=LivelinessPolicy.AUTOMATIC,
            # Use VITAL_PULSE_TIMEOUT (1.0s) as the base for timing policies
            liveliness_lease_duration=Duration(seconds=VITAL_PULSE_TIMEOUT * 1.5),
            lifespan=Duration(seconds=VITAL_PULSE_TIMEOUT),
            deadline=Duration(seconds=VITAL_PULSE_TIMEOUT * 1.3)
        )

        # 3. Communication Setup
        self.vp_signal_oscillator = self.create_publisher(VitalPulse, VITAL_PULSE_SIGNAL, qos_profile)
        self.vp_response_receptor = self.create_subscription(VitalPulse, VITAL_PULSE_RESPONSE, self.feedback_callback, qos_profile)

        # 4. Timer Setup
        # create timers for each level
        self.pump_timer = self.create_timer(1.0 / self.pump.get_poll_freq("HI"), self.run_pump_cycle)
        self.oscillation_timer = self.create_timer(self.heartbeat_interval, self.oscillate_vital_pulse)
        self.display_timer = self.create_timer(DISPLAY_INTERVAL, self.display_tick)

    def run_pump_cycle(self):
        """
        Collects vitals based on the configured polling level and updates the vital_dump.

        It performs a single pump cycle, gathering data from all streams
        (HI, MID, LO) according to the current tier and mode settings. It updates 
        the internal `vital_dump` manifest using the "update if worse/null/error" logic, 
        ensuring the worst-case scenario is always preserved until the next enrichment tick.

        Args:
            None

        Returns:
            None
        """
        # Initialize the glob and get the initial time before pumping happens
        vital_glob = {}
        vital_glob["iteration"] = self.vital_dump["pump"]["iteration"]
        vital_glob["timestamp"] = self.get_clock().now().to_msg()
        
        # Pass the glob to pump to implement Triple-Level Polling (TLP)
        vital_glob = self.pump.poll_vital_data(vital_glob)

        # Increment the step until 10 iterations, then reset
        vital_glob["iteration"] = (vital_glob["iteration"] + 1) % self.pump.get_poll_freq("HI")
      
        # Atomically commit the vital dump snapshot
        self.commit_vital_dump("pump", vital_glob)

        # for DEBUG: Print out the payload to see if collected data is correct, will DEL
        # TODO: to be removed in the future
        self.get_logger().info(f"[Pump] Collected: {self.vital_dump}")
        
    def oscillate_vital_pulse(self):
        """Oscillator: publish vital pulse signal"""
        self.fire_timestamp = self.current_rclpy_time_sec()
        self.current_opm = self.compute_opm()

        msg = VitalPulse()
        msg.robot_id = ROBOT_ID
        msg.user_id = USER_ID
        msg.cpu_temp = max(self.vital_dump["pump"]["payload"]["cpu_temp"])
        msg.gpu_temp = max(self.vital_dump["pump"]["payload"]["gpu_temp"])
        
        # ⭐ FIX: Assign structured time message
        # Assuming VitalPulse.timestamp is of type builtin_interfaces/msg/Time
        msg.timestamp = self.get_clock().now().to_msg()
        
        msg.vital_pulse_opm = self.current_opm

        self.vp_signal_oscillator.publish(msg)

    def feedback_callback(self, msg: VitalPulse):
        """Regulator: handle feedback and compute RTT"""
        if msg.robot_id != ROBOT_ID:
            return

        now_sec_float = self.current_rclpy_time_sec()
        
        # ⭐ FIX: Record successful feedback time
        self.last_feedback_time = now_sec_float 

        if self.fire_timestamp is not None:
            self.current_rtt = (now_sec_float - self.fire_timestamp) * 1000.0  # ms
            
        # Linked is True immediately upon successful receipt
        self.vc_linked = True

    def display_tick(self):
        """Monitor: Update snapshot for display thread, handling timeout (FIX)"""
        now = self.current_rclpy_time_sec()

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
        vital_manifest = copy.deepcopy(self.vital_dump["pump"]["payload"])

        # Thread-safe snapshot update
        self.display_snapshot = {
            "time": now,
            "heart_step": self.heart_step,
            "opm": self.current_opm,
            "rtt": self.current_rtt,
            "system_cpu_load%": vital_manifest["cpu_system"],
            "user_cpu_load%": vital_manifest["cpu_user"],
            "cpu_temp": vital_manifest["cpu_temp"],
            "gpu_load%": vital_manifest["gpu_load"],
            "gpu_temp": vital_manifest["gpu_temp"],
            "tj_temp": vital_manifest["tj_temp"],
            "fan_speed": vital_manifest["fan_speed"],
            "ram_used%": vital_manifest["ram_used"],
            "swap_used%": vital_manifest["swap_used"],
            "emc_load%": vital_manifest["emc_load"],
            "vdd_cpu_gpu_cv": vital_manifest["vdd_cpu_gpu_cv"],
            "vdd_soc": vital_manifest["vdd_soc"],
            "voltage_soc": vital_manifest["voltage_soc"],
            "current_soc": vital_manifest["current_soc"],
            "power_soc": vital_manifest["power_soc"],
            "disk_used%": vital_manifest["disk_used"],
            
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

    def commit_vital_dump(self, module: str, vital_dump: dict[str, any]):
        """Commit vital dump with latest data"""
        # Calculate duration time of pump cycle
        current_time = self.get_clock().now()
        start_time = rclpy.time.Time.from_msg(vital_dump["timestamp"])
        vital_dump["duration"] = (current_time - start_time).nanoseconds * 1e-9

        # Atomically commit the vital dump snapshot
        self.vital_dump[module] = copy.deepcopy(vital_dump)

    def current_rclpy_time_sec(self) -> int:
        """Helper to get current time in seconds"""
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

    # Use MultiThreadedExecutor instead of rclpy.spin(node)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("\nVital Terminal Core stopped.")

    # Clean up
    node.pump.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
