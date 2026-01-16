# System modules
from collections import deque
from jtop import jtop
from typing import Any, Literal

class Pump():

    # Polling Level
    POLL_LEVEL = Literal["LO", "MID", "HI"]

    #TODO: to be moved to config file for settings, or to a robot spec YAML file
    # Polling Frequency (Hz)
    POLL_FREQ : dict[POLL_LEVEL, int] = {
        "LO": 1,    # 1Hz (60 ppm)
        "MID": 5,   # 5Hz (300 ppm)
        "HI": 10    # 10Hz (600 ppm)
        }

    MAX_POLL_HISTORY = 10

    # Source to poll Jetson Stats
    JETSON_STATS_SOURCES: dict[str, POLL_LEVEL] = {
        "cpu": "HI",
        "gpu": "HI",
        "temperature": "HI",
        "fan": "MID",
        "memory": "MID",
        "power": "MID",
        "disk": "LO"
    }    

    # Keys for polling Jetson Stats
    JETSON_STATS_MAP: dict[str, tuple[str, tuple[Any, ...], ...]] = {
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

        # Fan stats (The "Cooling")
        "fan_speed": ("fan", ("pwmfan", "speed", 0)),

        # Memory stats (The "RAM/Storage")
        "ram_total": ("memory", ("RAM", "tot")),
        "ram_used": ("memory", ("RAM", "used")),
        "swap_total": ("memory", ("SWAP", "tot")),
        "swap_used": ("memory", ("SWAP", "used")),
        "emc_load": ("memory", ("EMC", "cur")),

        # Power stats (The "Energy")
        "vdd_cpu_gpu_cv": ("power", ("rail", "VDD_CPU_GPU_CV", "power")),
        "vdd_soc": ("power", ("rail", "VDD_SOC", "power")),
        "voltage_soc": ("power", ("tot", "volt")),
        "current_soc": ("power", ("tot", "curr")),
        "power_soc": ("power", ("tot", "power")),

        # Disk stats (The "Storage")
        "disk_used": ("disk", ("used",)),

    }

    """
    Collects vital data from a Jetson robot using a persistent jtop instance.
        - get_poll_freq: Returns polling frequency for a given level
        - get_poll_rhythm: Returns polling rhythm (sec) for a given level
        - poll_vital_data: Collects vitals based on polling level
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
    
    def poll_vital_data(self, vital_glob: dict[str, Any]) -> dict[str, Any]:
        """
        Poll vital data based on the configured polling level and update worst case values:
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
                "temperature": dict(self.jetson.temperature),
                "disk": dict(self.jetson.disk)
            }

        # Calculate iteration step and mid-point factor
        max_step = round(self.get_poll_freq("HI") / self.get_poll_freq("LO"))
        mid_factor = round(self.get_poll_freq("HI") / self.get_poll_freq("MID"))

        # Implement Triple-Level Polling (TLP) of polling vitals data
        # High Level: Poll all Jetson stats
        vital_glob.update(self.get_jetson_stats(vital_glob, "HI"))

        # Mid Level: Poll all sensor stats
        if vital_glob["iteration"] % mid_factor == 0:
            vital_glob.update(self.get_jetson_stats(vital_glob, "MID"))

        # Low Level: Poll all I/O stats
        if vital_glob["iteration"] % max_step == 0:
            vital_glob.update(self.get_jetson_stats(vital_glob, "LO"))

        print(f"[Pump] glob: {vital_glob}")

        return vital_glob
            
    def get_jetson_stats(self, vital_glob: dict[str, Any], level: POLL_LEVEL) -> dict[str, Any]:
        """
        Poll all Jetson stats and update vital blob payload
        Args:
            vital_glob (dict[str, Any]): vital blob containing payload
        Returns:
            dict[str, Any]: updated vital blob
        """

        # Ensure jetson_source exists, even if jtop is not available
        if not hasattr(self, "jetson_source") or self.jetson_source is None:
            self.jetson_source = {}

        # Use Recursive Path Traversal to get the stats
        for key, (source_name, path) in self.JETSON_STATS_MAP.items():            
            if self.JETSON_STATS_SOURCES[source_name] == level:
                try:
                    # Grab the realized dictionary
                    vital_data = self.jetson_source.get(source_name)
                    
                    # Sanitize the data by removing "Hardware Off" noise
                    if vital_data == -256 or vital_data is None:
                        vital_data = None
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

                    # Append the None value to the deque
                    vital_glob["payload"][key].append(None)
                    
        return vital_glob

    def get_recursive(self, data: Any, path: tuple[Any, ...]) -> Any:
        """Helper to navigate nested dictionaries and lists safely."""
        for key in path:
            # Handle Dictionary access
            if isinstance(data, dict) and key in data:
                data = data[key]
            # Handle List access (if key is an integer)
            elif isinstance(data, list) and isinstance(key, int):
                try:
                    data = data[key]
                except IndexError:
                    return None
            else:
                return None
        return data
    
if __name__ == "__main__":
    import time

    # Create the instance
    pump = Pump()
    
    print("--- VTC PUMP DIAGNOSTIC START ---")
    vital_glob = {"timestamp": 0, "duration": 0.0, "iteration": 0, "payload": {}}

    try:
        for i in range(5):
            vital_glob["iteration"] = i
            vital_glob["timestamp"] = time.time()
            vitals = pump.poll_vital_data(vital_glob)
            
            cpu_temp = vitals["payload"]["cpu_temp"][-1]
            gpu_temp = vitals["payload"]["gpu_temp"][-1]
            print(f"[{vital_glob['timestamp']}] CPU: {cpu_temp}°C | GPU: {gpu_temp}°C")
            
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nDiagnostic stopped by user.")
    finally:
        pump.close()
        print("--- VTC PUMP DIAGNOSTIC END ---")
