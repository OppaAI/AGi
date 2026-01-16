# System modules
from collections import deque
from jtop import jtop
from typing import Any, Literal


# Polling Level
POLL_LEVEL = Literal["LO", "MID", "HI"]

#TODO: to be moved to config file for settings
# Polling Frequency (Hz)
POLL_FREQ : dict[POLL_LEVEL, int] = {
    "LO": 1,    # 1Hz (60 ppm)
    "MID": 5,   # 5Hz (300 ppm)
    "HI": 10    # 10Hz (600 ppm)
    }

class Pump():

    MAX_POLL_HISTORY = 10

    # Keys for polling Jetson Stats
    JETSON_STATS_MAP: dict[str, tuple[str, tuple[str, ...]]] = {
        # CPU stats (The "Brain" load)
        "cpu_user": ("cpu", ("total", "user")),
        "cpu_system": ("cpu", ("total", "system")),

        # GPU stats (The "Vision/AI" load)
        "gpu_load": ("gpu", ("gpu", "status", "load")),

        # Temperature stats (The "Thermals")
        "cpu_temp": ("temperature", ("cpu", "temp")),
        "gpu_temp": ("temperature", ("gpu", "temp")),
        "soc0_temp": ("temperature", ("soc0", "temp")),
        "soc1_temp": ("temperature", ("soc1", "temp")),
        "soc2_temp": ("temperature", ("soc2", "temp")),
        "tj_temp": ("temperature", ("tj", "temp")),
        "cv0_temp": ("temperature", ("cv0", "temp")),
        "cv1_temp": ("temperature", ("cv1", "temp")),
        
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
                # Start jtop with high frequency polling at 10Hz (0.1s)
                self.jetson = jtop(interval=self.get_poll_rhythm("HI"))
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
        # Update jetson stats
        if self.jetson and self.jetson.ok():

            # Then update realized dicts
            self.jetson_source = {
                "cpu": dict(self.jetson.cpu),
                "fan": dict(self.jetson.fan),
                "gpu": dict(self.jetson.gpu),
                "memory": dict(self.jetson.memory),
                "power": dict(self.jetson.power),
                "temperature": dict(self.jetson.temperature)
            }

        # Calculate iteration step and mid-point factor
        max_step = round(self.get_poll_freq("HI") / self.get_poll_freq("LO"))
        mid_factor = round(self.get_poll_freq("HI") / self.get_poll_freq("MID"))

        # Implement Triple-Level Polling (TLP) of polling vitals data
        # High Level: Poll all Jetson stats
        vital_glob.update(self.get_jetson_stats(vital_glob))

        # Mid Level: Poll all sensor stats
        if vital_glob["iteration"] % mid_factor == 0:
            # vital_glob["payload"].update(self.get_sensor_stats(vital_glob["payload"]))
            pass

        # Low Level: Poll all I/O stats
        if vital_glob["iteration"] % max_step == 0:
            # vital_glob["payload"].update(self.get_io_stats(vital_glob["payload"]))
            pass

        print(f"[Pump] glob: {vital_glob}")

        return vital_glob
            
    def get_jetson_stats(self, vital_glob: dict[str, any]) -> dict[str, any]:
        """
        Poll all Jetson stats and update vital blob payload
        Args:
            vital_glob (dict[str, any]): vital blob containing payload
        Returns:
            dict[str, any]: updated vital blob
        """
        # Use Recursive Path Traversal to get the stats
        for key, (source_name, path) in self.JETSON_STATS_MAP.items():
            try:
                # Grab the realized dictionary
                vital_data = self.jetson_source.get(source_name)
                
                # Sanitize the data by removing "Hardware Off" noise
                if vital_data == -256 or vital_data is None:
                    vital_data = float("nan")
                else:
                    # Sift through the dictionary using your path tuple
                    vital_data = self.get_recursive(vital_data, path)

                # Initialize the deque if not exist
                if key not in vital_glob["payload"]:
                        vital_glob["payload"][key] = deque(maxlen=self.MAX_POLL_HISTORY)

                # Append the new value to the deque
                vital_glob["payload"][key].append(vital_data)
                        
            except Exception:
                # Initialize the deque if not exist
                if key not in vital_glob["payload"]:
                    vital_glob["payload"][key] = deque(maxlen=self.MAX_POLL_HISTORY)

                # Append the nan value to the deque
                vital_glob["payload"][key].append(float('nan'))
                
        return vital_glob

    def get_recursive(self, data: Any, path: tuple[str, ...]) -> Any:
        """Helper to navigate nested dictionaries safely."""
        for key in path:
            if isinstance(data, dict) and key in data:
                data = data[key]
            else:
                return float('nan')
        return data
    
if __name__ == "__main__":
    # Create the instance
    p = Pump()
    
    print("--- VTC PUMP DIAGNOSTIC START ---")
    try:
        # Run a 5-second test loop
        for i in range(5):
            # Test the 'HI' polling level (the most critical one)
            vitals = p.collect_vitals(level="HI")
            
            # Print a clean snapshot
            print(f"[{p.timestamp}] CPU: {vitals['cpu_temp']}°C | GPU: {vitals['gpu_temp']}°C")
            
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nDiagnostic stopped by user.")
    finally:
        # This ensures jtop.close() is called even if the test fails
        del p 
        print("--- VTC PUMP DIAGNOSTIC END ---")