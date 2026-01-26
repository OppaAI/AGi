# System modules
from ament_index_python.packages import get_package_share_directory
from collections import deque
from jtop import jtop
from pathlib import Path
from typing import Any, Dict, List, Literal
import yaml

# Define the flow channel type of the lifestream
FlowChannel = Literal["LO", "MID", "HI"]

# The robot spec file
ConduitJunction = tuple[Any, ...]
ConduitMap = tuple[str, ConduitJunction]

# TODO: to be moved to a global constants file
ROBOT_SPEC_FILE: str = "robot_spec.yaml"

class Pump():

    """
    Pump Module:
    - Harvests useful glob from the Jetson lifestream using Triple Action Pump (TAP) technique
    - Maintains a stream of consciousness (SOC) of the harvested glob
    - Exports the harvested glob to the vital terminal core for determining the health status

    Functions:
    - which_flow_rate_for_this_channel: Return the flow rate (CPS) for a given channel
    - what_pumping_rhythm_for_this_channel: Return the pumping rhythm (sec) for a given channel
    - engage_tap_to_harvest_glob: Harvest glob from the Jetson lifestream and put into the bin using TAP technique
    - harvest_glob_from_lifestream: Harvest glob from a specific flow channel
    - trace_thru_conduit: Trace through the conduit to extract the glob
    - close_valve: Gracefully stop the flow of the Jetson lifestream flow
    """ 

    #TODO: to be moved to config file for settings, or to a robot spec YAML file
    # The flow rate of lifestream from each flow channel in cycles per second (CPS)
    FLOW_RATE : Dict[FlowChannel, int] = {
        "LO": 1,    # 1 CPS (60 ppm)
        "MID": 5,   # 5 CPS (300 ppm)
        "HI": 10    # 10 CPS (600 ppm)
        }

    # The maximum capacity of stream stored in the bin
    MAX_STREAM_CAPACITY: int = 10

    # The flow rate of lifestream
    LIFESTREAM_FLOW_RATE: Dict[str, FlowChannel] = {
        "p_unit": "HI",
        "a_unit": "HI",
        "temp": "HI",
        "air_flow": "MID",
        "stm": "MID",
        "ltm": "LO",
        "energy": "MID"
    }    

    COLLECTION_POINT_NOT_AVAILABLE: int = -256
    
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

        # Build the explicit conduit map
        self._CONDUIT_MAP: Dict[FlowChannel, List[ConduitMap]] = self._build_conduit_map()
    
    def close_valve(self):
        """Gracefully stop the flow of the Jetson lifestream"""
        if self.jetson_lifestream and self.jetson_lifestream.ok():
            self.jetson_lifestream.close()

    def trace_thru_conduit(self, lifestream: Any, path: ConduitJunction) -> Any:
        """Helper to trace through the conduit to collect the glob."""
        for collection_point in path:
            try:
                # If the collection point is located
                lifestream = lifestream[collection_point]
            except (KeyError, IndexError, TypeError):
                # If the conduit is blocked (invalid key/index), stop traversing.
                return None
        return lifestream
  
    def engage_tap_to_harvest_glob(self, bin: Dict[str, Any]) -> Dict[str, Any]:
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
        self.jetson_collection_points = {}
        if self.jetson_lifestream and self.jetson_lifestream.ok():
            # Connect the conduits to the collection points of the Jetson lifestream
            self.jetson_collection_points = {
                "cpu": self.jetson_lifestream.cpu,
                "gpu": self.jetson_lifestream.gpu,
                "temperature": self.jetson_lifestream.temperature,
                "fan": self.jetson_lifestream.fan,
                "memory": self.jetson_lifestream.memory,
                "disk": self.jetson_lifestream.disk,
                "power": self.jetson_lifestream.power
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
            
    def harvest_glob_from_lifestream(self, bin: Dict[str, Any], channel: FlowChannel) -> Dict[str, Any]:
        """
        Harvest useful glob from a given channel of lifestream into the bin:
        - Follow the conduit map paths to locate the collection point and collect the glob
        
        Args:
            bin (dict[str, Any]): bin for collecting the glob
        Returns:
            dict[str, Any]: bin filled with useful glob from the given channel of lifestream
        """
        # Follow the conduit map paths to locate the collection point and collect the glob
        for key, (collection_point, path) in self._CONDUIT_MAP[channel]:
            # Ensure deque exists in the bin
            if key not in bin["glob"]:
                bin["glob"][key] = deque(maxlen=self.MAX_STREAM_CAPACITY)

            glob = None

            # Only trace the conduit if it's connection to the Jetson lifestream is established
            if self.jetson_collection_points:
                try:
                    # Trace through the conduit to collect the glob
                    diverted_lifestream = self.jetson_collection_points.get(collection_point)

                    # Only trace through the conduit if it's connection to the Jetson lifestream is established
                    if diverted_lifestream is not None:
                        glob = self.trace_thru_conduit(diverted_lifestream, path)

                    # Sift out the impurity or empty glob
                    if glob == self.COLLECTION_POINT_NOT_AVAILABLE:
                        glob = None
                except Exception:
                    # No glob to collect if conduit is not accessible
                    glob = None

                # Append the glob to the deque in the bin, even if it is None
                bin["glob"][key].append(glob)
                
        return bin

    @classmethod
    def _build_conduit_map(cls) -> Dict[FlowChannel, List[ConduitMap]]:
        """Return the conduit map grouped by flow channel"""
        # Read the conduit map blueprint from the robot spec file
        conduit_map_blueprint: Dict[str, Any] = cls.retrieve_conduit_map_blueprint_from_hardware_spec()
        if not conduit_map_blueprint:
            raise ValueError("Conduit map blueprint not found.")
        
        channels: Dict[FlowChannel, List[ConduitMap]] = {"LO": [], "MID": [], "HI": []}

        def locate_conduit_heads(conduit_map_blueprint: Dict[str, Any], junction: str = ""):
            """Helper function to recursively locate conduit heads from the blueprint"""
            for conduit_name, spec in conduit_map_blueprint.items():
                conduit_junction = f"{junction}.{conduit_name}" if junction else conduit_name
                if isinstance(spec, list):
                    module_name: str = spec[0]
                    junction_path: ConduitJunction = tuple(spec[1])
                    flow_rate: FlowChannel = cls.LIFESTREAM_FLOW_RATE.get(module_name, "LO")
                    channels[flow_rate].append((conduit_junction, (module_name, junction_path)))
                elif isinstance(spec, dict):
                    locate_conduit_heads(spec, conduit_junction)

        locate_conduit_heads(conduit_map_blueprint)
        return channels

    @classmethod
    def which_flow_rate_for_this_channel(cls, channel: FlowChannel) -> int:
        """Return the flow rate of lifestream for a given flow channel"""
        return cls.FLOW_RATE[channel]
    
    @classmethod
    def what_pumping_rhythm_for_this_channel(cls, channel: FlowChannel) -> float:
        """Return pumping rhythm (sec) for a given level"""
        return 1.0 / cls.which_flow_rate_for_this_channel(channel)
    
    @staticmethod
    def retrieve_conduit_map_blueprint_from_hardware_spec() -> Dict[str, Any]:
        """Retrieve the conduit map blueprint from the robot hardware spec"""
        # Use the path set in ROS2 package to locate the robot hardware spec
        spec_potential_paths = []
        try:
            spec_potential_paths.append(Path(get_package_share_directory("vcs")) / "config" / ROBOT_SPEC_FILE)
        except Exception:
            pass

        # Also check the current working directory for the robot hardware spec
        spec_potential_paths.append(Path("config") / ROBOT_SPEC_FILE)

        # Locate the robot hardware spec and extract the lifestream conduit map blueprint
        for spec_path in spec_potential_paths:
            if spec_path.exists() and spec_path.is_file():
                try:
                    with open(spec_path, 'r') as file:
                        # Read the VCS section from the robot spec YAML file
                        data = yaml.safe_load(file)
                        data = data.get("vcs") if isinstance(data, dict) else None
                        
                        # Extract the conduit map blueprint from the VCS section
                        if data and "conduit_map_blueprint" in data:
                            return data["conduit_map_blueprint"]
                except yaml.YAMLError as e:
                    # TODO: Log the error in the log file
                    raise ValueError(f"Error parsing YAML file: {e}") from e
                
        # Return an empty conduit map blueprint if no valid conduit map blueprint is found
        return {}
        
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
            cpu_load_deque = glob.get("glob", {}).get("body_load.p_unit")
            cpu_load = max([v for v in cpu_load_deque if v is not None], default=None) if cpu_load_deque else None
            temperature_cpu_deque = glob.get("glob", {}).get("body_temp.p_unit")
            temperature_cpu = max([v for v in temperature_cpu_deque if v is not None], default=None) if temperature_cpu_deque else None
            gpu_load_deque = glob.get("glob", {}).get("body_load.a_unit")
            gpu_load = max([v for v in gpu_load_deque if v is not None], default=None) if gpu_load_deque else None
            temperature_gpu_deque = glob.get("glob", {}).get("body_temp.a_unit")
            temperature_gpu = max([v for v in temperature_gpu_deque if v is not None], default=None) if temperature_gpu_deque else None

            # Color code the temperature based on the value
            cpu_color = color_temp(temperature_cpu)
            gpu_color = color_temp(temperature_gpu)
            cpu_load_str = f"{cpu_load:.2f}%" if cpu_load is not None else "N/A"
            temperature_cpu_str = f"{temperature_cpu}°C" if temperature_cpu is not None else "N/A"
            gpu_load_str = f"{gpu_load:.2f}%" if gpu_load is not None else "N/A"
            temperature_gpu_str = f"{temperature_gpu}°C" if temperature_gpu is not None else "N/A"
            print(f"Run #: {glob['run']+1} | Timestamp: [{glob['timestamp']}]")
            print(f" CPU Load: {cpu_load_str} | CPU: {cpu_color}{temperature_cpu_str}\033[0m")
            print(f" GPU Load: {gpu_load_str} | GPU: {gpu_color}{temperature_gpu_str}\033[0m")
                        
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        # Stop the diagnosis if user interrupts
        print("\nDiagnosis is stopped by user.")
    finally:
        # Close the Pump Module and end the diagnosis
        pump.close_valve()
        print("--- VTC PUMP MODULE DIAGNOSIS COMPLETE ---")
