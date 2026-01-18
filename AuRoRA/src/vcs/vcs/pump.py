# System modules
from collections import deque
from jtop import jtop
from typing import Any, cast, Literal
    
# Define the flow channel type of the lifestream
FlowChannel = Literal["LO", "MID", "HI"]

class Pump():

    """
    Pump Module:
    - Harvests useful glob from the Jetson lifestream using Triple Action Pump (TAP) technique
    - Maintains a stream of consciousness (SOC) of the harvested glob
    - Exports the harvested glob to the vital terminal core for determining the health status

    Functions:
    - which_flow_rate_for_this_channel: Return the flow rate (CPS) for a given channel
    - what_pumping_rhythm_for_this_channel: Return the pumping rhythm (sec) for a given channel
    - connect_conduits_to_collection_points: Connect the pump conduits to the collection points of the Jetson lifestream
    - engage_tap_to_harvest_glob: Harvest glob from the Jetson lifestream and put into the bin using TAP technique
    - harvest_glob_from_lifestream: Harvest glob from a specific flow channel
    - dig_deep_thru_conduit: Navigate through the conduit to extract the glob
    - close_valve: Gracefully stop the flow of the Jetson lifestream flow
    """ 

    #TODO: to be moved to config file for settings, or to a robot spec YAML file
    # The flow rate of lifestream from each flow channel in cycles per second (CPS)
    FLOW_RATE : dict[FlowChannel, int] = {
        "LO": 1,    # 1 CPS (60 ppm)
        "MID": 5,   # 5 CPS (300 ppm)
        "HI": 10    # 10 CPS (600 ppm)
        }

    # The maximum capacity of stream stored in the bin
    MAX_STREAM_CAPACITY = 10

    # The flow rate of lifestream
    LIFESTREAM_FLOW_RATE: dict[str, FlowChannel] = {
        "cpu": "HI",
        "gpu": "HI",
        "temperature": "HI",
        "fan": "MID",
        "memory": "MID",
        "power": "MID",
        "disk": "LO"
    }    

    # The mapping of the conduits leading to the reservoirs of the lifestream
    LIFESTREAM_CONDUIT_MAP: dict[str, tuple[str, tuple[Any, ...]]] = {
        # Logic/workload vitals
        "cpu_user": ("cpu", ("total", "user")),
        "cpu_system": ("cpu", ("total", "system")),

        # Neural/Sensory processing vitals
        "gpu_load": ("gpu", ("gpu", "status", "load")),

        # Body temperature vitals
        "cpu_temp": ("temperature", ("cpu", "temp")),
        "gpu_temp": ("temperature", ("gpu", "temp")),
        "soc0_temp": ("temperature", ("soc0", "temp")),
        "soc1_temp": ("temperature", ("soc1", "temp")),
        "soc2_temp": ("temperature", ("soc2", "temp")),
        "tj_temp": ("temperature", ("tj", "temp")),
        "cv0_temp": ("temperature", ("cv0", "temp")),
        "cv1_temp": ("temperature", ("cv1", "temp")),

        # Air intake vitals
        "fan_speed": ("fan", ("pwmfan", "speed", 0)),

        # STM retainment vitals
        "ram_total": ("memory", ("RAM", "tot")),
        "ram_used": ("memory", ("RAM", "used")),
        "swap_total": ("memory", ("SWAP", "tot")),
        "swap_used": ("memory", ("SWAP", "used")),
        "emc_load": ("memory", ("EMC", "cur")),

        # Energy level vitals
        "vdd_cpu_gpu_cv": ("power", ("rail", "VDD_CPU_GPU_CV", "power")),
        "vdd_soc": ("power", ("rail", "VDD_SOC", "power")),
        "voltage_soc": ("power", ("tot", "volt")),
        "current_soc": ("power", ("tot", "curr")),
        "power_soc": ("power", ("tot", "power")),

        # Long term retainment vitals
        "disk_used": ("disk", ("used",)),
    }
    
    def __init__(self):
        """
        This is the initialization of the pump:
        - Set the maximum flow rate of the Jetson lifestream and start the streamflow
        """
        
        # Connect to the Jetson lifestream to initialize the streamflow and start the streamflow
        self.jetson_lifestream = None
        self.jetson_collection_points = None
        try:
            # Set the maximum flow rate of the Jetson lifestream
            self.jetson_lifestream = jtop(interval=self.what_pumping_rhythm_for_this_channel("HI"))
            # Start the streamflow
            self.jetson_lifestream.start()

        except Exception as e:
            # In case of error, log the error and tell the pump that the Jetson lifestream is running empty
            #TODO: Log the error in the log file
            self.jetson_lifestream = None

        # Build the explicit conduit map for each flow channel
        self._CONDUITS_BY_CHANNEL = self._build_conduit_map_by_channel()
    
    def close_valve(self):
        """Gracefully stop the flow of the Jetson lifestream"""
        if self.jetson_lifestream and self.jetson_lifestream.ok():
            self.jetson_lifestream.close()

    def dig_deep_thru_conduit(self, data: Any, path: tuple[Any, ...]) -> Any:
        """Helper to navigate through the conduit to collect the glob."""
        for key in path:
            # Handle different types of conduit junctions
            if isinstance(data, dict) and key in data:
                data = data[key]
            elif isinstance(data, list) and isinstance(key, int):
                if 0 <= key < len(data):
                    data = data[key]
                else:
                    # Not a valid index, stop traversing
                    return None
            else:
                # Not a valid conduit junction, stop traversing
                return None
        return data
  
    def engage_tap_to_harvest_glob(self, bin: dict[str, Any]) -> dict[str, Any]:
        """
        Collect glob from the Jetson lifestream and put into the bin using Triple Action Pump (TAP) technique:
        - Connect the pump conduits to the collection points of the Jetson lifestream:
            - High flow channel: Collect crucial workload and energy vitals
            - Mid flow channel: Collect sensory vitals
            - Low flow channel: Collect memory vitals
        - Engage the Triple Action Pump (TAP) to harvest glob from the lifestream

        Args:
            bin (dict[str, Any]): bin for collecting the glob
        Returns:
            dict[str, Any]: bin filled with useful glob from the lifestream
        """

        # Ensure the bin is ready to collect the glob
        bin.setdefault("run", 0)
        bin.setdefault("glob", {})
        
        # Connect the pump conduits to each collection point of the Jetson lifestream, if it is running and ready
        if self.jetson_lifestream and self.jetson_lifestream.ok():
            # Connect the conduits to the collection points of the Jetson lifestream
            self.jetson_collection_points = {
                "cpu": dict(self.jetson_lifestream.cpu),
                "fan": dict(self.jetson_lifestream.fan),
                "gpu": dict(self.jetson_lifestream.gpu),
                "memory": dict(self.jetson_lifestream.memory),
                "power": dict(self.jetson_lifestream.power),
                "temperature": dict(self.jetson_lifestream.temperature),
                "disk": dict(self.jetson_lifestream.disk)
            }

        # Engage the Triple Action Pump (TAP) to harvest glob from the lifestream
        # Calculate how many runs for each pump cycle and midpoint of the cycle
        run_per_cycle = self.which_flow_rate_for_this_channel("HI") // self.which_flow_rate_for_this_channel("LO")
        cycle_midpoint = self.which_flow_rate_for_this_channel("HI") // self.which_flow_rate_for_this_channel("MID")

        # High flow channel: Collect crucial workload and energy vitals
        bin = self.harvest_glob_from_lifestream(bin, "HI")

        # Mid flow channel: Collect sensory vitals
        if bin["run"] % cycle_midpoint == 0:
            bin = self.harvest_glob_from_lifestream(bin, "MID")

        # Low flow channel: Collect memory vitals
        if bin["run"] % run_per_cycle == 0:
            bin = self.harvest_glob_from_lifestream(bin, "LO")
        
        return bin
            
    def harvest_glob_from_lifestream(self, bin: dict[str, Any], channel: FlowChannel) -> dict[str, Any]:
        """
        Harvest useful glob from a given channel of lifestream into the bin:
        - Follow the conduit map paths to locate the collection point and collect the glob
        
        Args:
            bin (dict[str, Any]): bin for collecting the glob
        Returns:
            dict[str, Any]: bin filled with useful glob from the given channel of lifestream
        """
        # Follow the conduit map paths to locate the collection point and collect the glob
        for key, (collection_point, path) in self._CONDUITS_BY_CHANNEL[channel]:
            # Ensure deque exists in the bin
            if key not in bin["glob"]:
                bin["glob"][key] = deque(maxlen=self.MAX_STREAM_CAPACITY)

            glob = None

            # Only dig into the conduit if it's connection to the Jetson lifestream is established
            if self.jetson_collection_points:
                try:
                    # Dig deep into the conduit to collect the glob
                    diverted_lifestream = self.jetson_collection_points.get(collection_point)                    
                    if diverted_lifestream is not None:
                        glob = self.dig_deep_thru_conduit(diverted_lifestream, path)

                    # Sift out the impurity or empty glob
                    if glob == -256:
                        glob = None
                except Exception:
                    # No glob to collect if conduit is not accessible
                    glob = None

                # Append the glob to the deque in the bin, even if it is None
                bin["glob"][key].append(glob)
                
        return bin

    @classmethod
    def _build_conduit_map_by_channel(cls) -> dict[FlowChannel, list[tuple[str, tuple[str, tuple[Any, ...]]]]]:
        """Return the conduit map for each flow channel"""
        return {
                cast(FlowChannel, channel): [
                    (k, v) for k, v in cls.LIFESTREAM_CONDUIT_MAP.items() 
                    if cls.LIFESTREAM_FLOW_RATE.get(v[0]) == channel
                ]
                for channel in ["LO", "MID", "HI"]
            }
            
    @classmethod
    def which_flow_rate_for_this_channel(cls, channel: FlowChannel) -> int:
        """Return the flow rate of lifestream for a given flow channel"""
        return cls.FLOW_RATE[channel]
    
    @classmethod
    def what_pumping_rhythm_for_this_channel(cls, channel: FlowChannel) -> float:
        """Return pumping rhythm (sec) for a given level"""
        return 1.0 / cls.which_flow_rate_for_this_channel(channel)
    
if __name__ == "__main__":

    # Run the diagnosis of the Pump Module to check if working as expected
    import time

    def color_temp(temp):
        """Color code the temperature based on the value."""
        if temp is None:
            return "\033[38;5;240m"  # Gray for unknown
        return "\033[38;5;196m" if temp > 80 else "\033[38;5;46m"
    
    # Initialize the Pump Module
    pump = Pump()
    
    # Start the diagnosis
    print("--- DIAGNOSING VTC PUMP MODULE ---")
    glob = {"timestamp": 0, "duration": 0.0, "run": 0, "glob": {}}

    # Run the diagnosis for 30 pump cycles
    try:
        for i in range(30):
            # Collect the glob from the lifestream
            glob["run"] = i
            glob["timestamp"] = time.strftime("%H:%M:%S")
            glob = pump.engage_tap_to_harvest_glob(glob)
            
            # Get the worst case scenario of the vitals, if any
            cpu_load_deque = glob.get("glob", {}).get("cpu_user")
            cpu_load = max([v for v in cpu_load_deque if v is not None], default=None) if cpu_load_deque else None
            cpu_temp_deque = glob.get("glob", {}).get("cpu_temp")
            cpu_temp = max([v for v in cpu_temp_deque if v is not None], default=None) if cpu_temp_deque else None
            gpu_load_deque = glob.get("glob", {}).get("gpu_load")
            gpu_load = max([v for v in gpu_load_deque if v is not None], default=None) if gpu_load_deque else None
            gpu_temp_deque = glob.get("glob", {}).get("gpu_temp")
            gpu_temp = max([v for v in gpu_temp_deque if v is not None], default=None) if gpu_temp_deque else None

            # Color code the temperature based on the value
            cpu_color = color_temp(cpu_temp)
            gpu_color = color_temp(gpu_temp)
            cpu_load_str = f"{cpu_load:.2f}%" if cpu_load is not None else "N/A"
            cpu_temp_str = f"{cpu_temp}°C" if cpu_temp is not None else "N/A"
            gpu_load_str = f"{gpu_load:.2f}%" if gpu_load is not None else "N/A"
            gpu_temp_str = f"{gpu_temp}°C" if gpu_temp is not None else "N/A"
            print(f"Run #: {glob['run']+1} | Timestamp: [{glob['timestamp']}]")
            print(f" CPU Load: {cpu_load_str} | CPU: {cpu_color}{cpu_temp_str}\033[0m")
            print(f" GPU Load: {gpu_load_str} | GPU: {gpu_color}{gpu_temp_str}\033[0m")
                        
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        # Stop the diagnosis if user interrupts
        print("\nDiagnosis is stopped by user.")
    finally:
        # Close the Pump Module and end the diagnosis
        pump.close_valve()
        print("--- VTC PUMP MODULE DIAGNOSIS COMPLETE ---")