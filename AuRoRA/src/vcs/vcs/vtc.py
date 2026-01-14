#!/usr/bin/env python3
# Vital Terminal Core (VTC)
# - Pump: Collects raw sensor data for different sensors using Triple-Level Polling (TLP) technique
# - Regulator: Normalizes raw sensor data into proper format and regulates the oscillation rhythm according to RTT
# - Oscillator: Packs and encodes sensor data into vital pulse signal and publishes at oscillation rhythm
# - Orchestrator: Monitors vital pulse signal and response, detects disconnections, and triggers safety interlocks

# System modules
import copy
import time
from builtin_interfaces.msg import Time
from jtop import jtop
from typing import Any, Literal

# ROS 2 modules
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, LivelinessPolicy

# AGi Vital Pulse definition
from vp.msg import VitalPulse

# Polling Level
POLL_LEVEL = Literal["LO", "MID", "HI"]

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

class Pump():
    #TODO: to be moved to config file for settings
    # Polling Frequency (Hz)
    POLL_FREQ : dict[POLL_LEVEL, int] = {
        "LO": 1,    # 1Hz (60 ppm)
        "MID": 5,   # 5Hz (300 ppm)
        "HI": 10    # 10Hz (600 ppm)
    }

    # Keys for polling Jetson Stats
    JETSON_STATS_KEYS: dict[str, tuple[str, ...]] = {
        # CPU stats (The "Brain" load)
        "cpu_user": ("cpu", "total", "user"),
        "cpu_system": ("cpu", "total", "system"),

        # GPU stats (The "Vision/AI" load)
        "gpu_load": ("gpu", "gpu", "status", "load"),

        # Temperature stats (The "Thermals")
        "cpu_temp": ("temperature", "cpu", "temp"),
        "gpu_temp": ("temperature", "gpu", "temp"),
        "soc0_temp": ("temperature", "soc0", "temp"),
        "soc1_temp": ("temperature", "soc1", "temp"),
        "soc2_temp": ("temperature", "soc2", "temp"),
        "tj_temp": ("temperature", "tj", "temp"),
        "cv0_temp": ("temperature", "cv0", "temp"),
        "cv1_temp": ("temperature", "cv1", "temp"),
        
        # CPU stats (The "Brain" load)
        #"cpu1_load%": "CPU1",
        #"cpu2_load%": "CPU2",
        #"cpu3_load%": "CPU3",
        #"cpu4_load%": "CPU4",
        #"cpu5_load%": "CPU5",
        #"cpu6_load%": "CPU6",
        #"cpu_temp": "Temp cpu",
        #"tj_temp": "Temp tj",     # Critical: The thermal junction max temp

        # GPU stats (The "Vision/AI" load)
        #"gpu_load%": "GPU",
        #"gpu_temp": "Temp gpu",

        # SoC & Memory stats (The "Data Bus" health)
        #"soc0_temp": "Temp soc0",
        #"soc1_temp": "Temp soc1",
        #"soc2_temp": "Temp soc2",
        #"ram_used%": "RAM",
        #"swap_used%": "SWAP",
        #"emc_load%": "EMC",       # Memory controller bandwidth utilization

        # Power & Mechanical (The "Metabolism")
        #"nvp_model": "nvp model", 
        #"fan_pwm%": "Fan pwmfan0", # If fan is 100% and temp is high, it's a danger sign
        #"power_total": "Power TOT",
        #"power_cpu_gpu": "Power VDD_CPU_GPU_CV",
        #"power_soc": "Power VDD_SOC",
        
        # Temporal Reference
        #"jetson_time": "time",
        #"uptime": "uptime"
    }

    """
    Collects vital data from a Jetson robot using a persistent jtop instance.
        - get_poll_freq: Returns polling frequency for a given level
        - get_poll_rhythm: Returns polling rhythm (sec) for a given level
        - collect_vitals: Collects vitals based on polling level
        - poll_vital: Polls a vital metric from jtop stats
    """
    
    def __init__(self):
        self.jetson = None
        if jtop:
            try:
                self.jetson = jtop(interval=self.get_poll_rhythm("HI"))    # 10Hz polling at every 0.1s
                self.jetson.start()
            except Exception as e:
                #TODO: to implement logging in the future
                print(f"Pump Initialization Error: Could not start jtop. Error: {e}")
                self.jetson = None

    def close(self):
        """Ensure jtop is gracefully stopped when the object is deleted/cleaned up."""
        if self.jetson and self.jetson.ok():
            self.jetson.close()

    @classmethod
    def get_poll_freq(cls, level: POLL_LEVEL) -> int:
        """Return polling frequency (Hz) for a given level"""
        return cls.POLL_FREQ[level]
    
    @classmethod
    def get_poll_rhythm(cls, level: POLL_LEVEL) -> float:
        """Return polling rhythm (sec) for a given level"""
        return 1.0 / cls.get_poll_freq(level)
    
    def poll_vital_data(self, vital_glob: dict[str, any]) -> dict[str, any]:
        """
        Collect vital data based on polling level, and update worst case values:
            - High level: Poll all Jetson stats
            - Mid level: Poll all sensor stats
            - Low level: Poll all I/O stats
        """

        # Calculate iteration step and mid-point factor
        max_step = round(self.get_poll_freq("HI") / self.get_poll_freq("LO"))
        mid_factor = round(self.get_poll_freq("HI") / self.get_poll_freq("MID"))

        # Implement Triple-Level Polling (TLP) of polling vitals data
        # High Level: Poll all Jetson stats
        vital_glob = self.get_jetson_stats(vital_glob)

        # Mid Level: Poll all sensor stats
        if vital_glob["iteration"] % mid_factor == 0:
            # vital_glob["payload"].update(self.get_sensor_stats(vital_glob["payload"]))
            pass

        # Low Level: Poll all I/O stats
        if vital_glob["iteration"] % max_step == 0:
            # vital_glob["payload"].update(self.get_io_stats(vital_glob["payload"]))
            pass

        return vital_glob
            
    def get_jetson_stats(self, vital_glob: dict[str, any]) -> dict[str, any]:
        """
        Poll all Jetson stats and update vital blob payload
        Args:
            vital_glob (dict[str, any]): vital blob containing payload
        Returns:
            dict[str, any]: updated vital blob
        """
        t0 = time.perf_counter()
        if not self.jetson or not self.jetson.ok():
            return vital_glob

        # Use the specific properties which return the dicts you just dumped
        # This matches the structure of your printout exactly
        current_stats = {
            "cpu": self.jetson.cpu,
            "gpu": self.jetson.gpu,
            "temperature": self.jetson.temperature
        }

        if not isinstance(vital_glob["payload"], list):
            vital_glob["payload"] = []
        
        vital_glob["payload"].append(current_stats)
        vital_glob["jetson_duration"] = time.perf_counter() - t0
        print(f"[Pump] glob: {vital_glob}")
        return vital_glob

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
                "jetson_duration": 0.0,
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
        # Clone a blob of the vital dump and fill it with snapshot data
        vital_glob = copy.deepcopy(self.vital_dump["pump"])

        # Get initial time before pumping happens
        vital_glob["timestamp"] = self.get_clock().now().to_msg()
        
        # Pass vital_glob to pump to implement Triple-Level Polling (TLP)
        vital_glob = self.pump.poll_vital_data(vital_glob)

        # Increment the step until 10 iterations, then reset
        vital_glob["iteration"] = (vital_glob["iteration"] + 1) % self.pump.get_poll_freq("HI")

        # Atomically commit the vital dump snapshot
        self.commit_vital_dump(vital_glob)

        # for DEBUG: Print out the payload to see if collected data is correct, will DEL
        # TODO: to be removed in the future, and to remove Jetson duration
        self.get_logger().info(f"[Pump] Collected: {self.vital_dump}")
        
    def oscillate_vital_pulse(self):
        """Oscillator: publish vital pulse signal"""
        self.fire_timestamp = self.current_rclpy_time_sec()
        self.current_opm = self.compute_opm()

        msg = VitalPulse()
        msg.robot_id = ROBOT_ID
        msg.user_id = USER_ID
        
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
            "cpu_temp": vital_manifest["cpu_temp"],
            "gpu_temp": vital_manifest["gpu_temp"],
            "tj_temp": vital_manifest["tj_temp"],
            #"fan_pwm%": vital_manifest["fan_pwm%"],
            #"power_total": vital_manifest["power_total"],
            #"power_cpu_gpu": vital_manifest["power_cpu_gpu"],
            #"power_soc": vital_manifest["power_soc"],
            #"uptime": vital_manifest["uptime"],
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

    def commit_vital_dump(self, vital_glob: dict[str, any]):
        """Commit vital dump with latest data"""
        # Calculate duration time of pump cycle
        current_time = self.get_clock().now()
        start_time = rclpy.time.Time.from_msg(vital_glob["timestamp"])
        vital_glob["duration"] = (current_time - start_time).nanoseconds * 1e-9

        # Atomically commit the vital dump snapshot
        self.vital_dump["pump"] = vital_glob

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
