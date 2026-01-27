# System modules
from ament_index_python.packages import get_package_share_directory
from collections import deque
from jtop import jtop
from pathlib import Path
from types import MappingProxyType
from typing import Any, Dict, List, Literal, NamedTuple
import threading
import yaml

# Define the flow channel type of the lifestream
FlowChannel = Literal["LO", "MID", "HI"]

# The robot spec file
ConduitJunction = tuple[Any, ...]
ConduitMap = tuple[str, ConduitJunction]

# TODO: to be moved to a global constants file
ROBOT_SPEC_FILE: str = "robot_spec.yaml"

# Define an immutable structure for the conduit map
class Conduit(NamedTuple):
    name: str               # Immutable name of the collection point
    collection_point: Any   # Pre-bound collection point (eg. self.jetson.cpu)
    path: tuple             # Immutable path to the glob

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
    MAX_STREAM_CAPACITY: int = 16

    COLLECTION_POINT_NOT_AVAILABLE: int = -256
    
    def __init__(self):
        """
        This is the initialization of the pump:
        - Initialize the pump state and lock
        - Set the maximum flow rate of the Jetson lifestream and start the streamflow
        - Build the explicit conduit map
        - Transition to the "RUN" state
        """

        # Initialize the pump state and lock
        self.state = "INIT"
        self._lock = threading.Lock()
        
        # Connect to the Jetson lifestream to initialize the streamflow and start the streamflow
        self.jetson_lifestream = None
        self.jetson_collection_points = None
        try:
            # Set the maximum flow rate of the Jetson lifestream
            self.jetson_lifestream = jtop(interval=self.what_pumping_rhythm_for_this_channel("HI"))
            # Start the streamflow
            self.jetson_lifestream.start()

            # Wait for the streamflow to be ready
            if not self.jetson_lifestream.ok():
                #TODO: Log the error in the log file
                raise RuntimeError("Jetson lifestream is running empty.")

        except Exception as e:
            # In case of error, log the error and tell the pump that the Jetson lifestream is running empty
            #TODO: Log the error in the log file
            print(f"Error: Jetson lifestream failed to run: {e}")
            self.state = "DEGRADED"
            self.jetson_lifestream = None

        # Build the explicit conduit map
        self._CONDUIT_MAP: Dict[FlowChannel, tuple[Conduit, ...]] = self._build_conduit_map()

        # Verify the conduit map by performing a smoke test
        self.state = "DEGRADED" if not self._smoke_test_floodgates() else "RUN"
    
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
        # Load the frozen conduit map for the given channel
        conduit_map: tuple[Conduit, ...] = self._CONDUIT_MAP.get(channel, ())
        
        # Wrap the entire flow update in a lock to ensure thread safety
        with self._lock:
            for conduit in conduit_map:
                # Ensure deque exists in the bin
                if conduit.name not in bin["glob"]:
                    bin["glob"][conduit.name] = deque(maxlen=self.MAX_STREAM_CAPACITY)

                # Harvest the glob from the lifestream via the conduit path
                glob = None
                if self.jetson_lifestream and self.jetson_lifestream.ok():
                    # Only trace the conduit if it's connection to the Jetson lifestream is established
                    if conduit.collection_point is not None:
                        glob = self.trace_thru_conduit(conduit.collection_point, conduit.path)

                    # Sift out the impurity or empty glob
                    if glob == self.COLLECTION_POINT_NOT_AVAILABLE:
                        glob = None

                # Append the glob to the deque in the bin, even if it is None
                bin["glob"][conduit.name].append(glob)               
        return bin

    def _build_conduit_map(self) -> Dict[FlowChannel, tuple[Conduit, ...]]:
        """Return the conduit map grouped by flow channel"""
        # Read the conduit map blueprint from the robot spec file
        conduit_map_blueprint: Dict[str, Any] = self.retrieve_conduit_map_blueprint_from_hardware_spec()
        if not conduit_map_blueprint:
            raise ValueError("Conduit map blueprint not found.")
        
        channels: Dict[FlowChannel, List[Conduit]] = {"HI": [], "MID": [], "LO": []}
        
        # Get ready to create a clone duplicate of the conduit map for reference
        cloned_conduit_map = {}
               
        def locate_conduit_heads(conduit_map_blueprint: Dict[str, Any], junction: str = ""):
            """Helper function to recursively locate conduit heads from the blueprint"""
            for conduit_name, path in conduit_map_blueprint.items():
                conduit_junction = f"{junction}.{conduit_name}" if junction else conduit_name
                
                if isinstance(path, list):
                    # Format of the junction: [Module, Path_Tuple, Flow_Channel]
                    module_name, junction_path = path[0], tuple(path[1])
                    # Pull the channel from robot spec YAML file, or default to LO if not available
                    flow_rate = path[2] if len(path) > 2 else "LO"

                    # Lazy cache by calling once per unique module
                    if module_name not in cloned_conduit_map:
                        if self.jetson_lifestream and self.jetson_lifestream.ok():
                            cloned_conduit_map[module_name] = getattr(self.jetson_lifestream, module_name, None)
                        else:
                            cloned_conduit_map[module_name] = None
                    
                    # Pre-bind the collection point to the conduit using the cloned conduit map
                    collection_point = cloned_conduit_map.get(module_name)
                    
                    # Create the conduit and add it to the channel
                    channels[flow_rate].append(Conduit(conduit_junction, collection_point,junction_path))
                elif isinstance(path, dict):
                    locate_conduit_heads(path, conduit_junction)

        locate_conduit_heads(conduit_map_blueprint)

        # Freeze the conduit map for each flow channel
        return MappingProxyType({channel: tuple(conduits) for channel, conduits in channels.items()})

    def _smoke_test_floodgates(self)-> bool:
        """Perform a smoke test on all floodgates to ensure they are working properly."""
        if self.state == "DEGRADED": return False

        for channel, conduits in self._CONDUIT_MAP.items():
            for conduit in conduits:
                test_bin = self.trace_thru_conduit(conduit.collection_point, conduit.path)
                if test_bin is None:
                    #TODO: Log the error in the log file
                    print(f"Error: Conduit {conduit.name} is unreachable via {conduit.path}.")
                    return False
        return True

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
