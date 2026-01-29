# System modules
from calendar import c
from ament_index_python.packages import get_package_share_directory
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from enum import Enum
from jtop import jtop
from pathlib import Path
from types import MappingProxyType
from typing import Any, cast, Dict, List, Literal, NamedTuple, Tuple, Union
import threading
import time
import yaml

# Define the flow channel type of the lifestream
FlowChannel = Literal["LO", "MID", "HI"]

# Architectural conduit type used to define each conduit component in the conduit map
# Conduit junction types used to trace through the conduit path
ConduitJunctionPoint = Union[str, int]                  # String for junction name, int for junction index
ConduitJunctionList = tuple[ConduitJunctionPoint, ...]  # List of junction points in the conduit path

# Frozen conduit junction type used to define the conduit path
class ConduitType(NamedTuple):
    conduit_id: str                    # ID of the conduit
    collection_point: Any              # Pre-bound collection point (eg. self.jetson_reservoir.cpu)
    junction_path: ConduitJunctionList # Path of junctions to trace through the conduit

# Immutable conduit map type for conduit path
ConduitMapBlueprint = MappingProxyType[FlowChannel, Tuple[ConduitType, ...]]    # Conduit map blueprint

# TODO: to be moved to a global constants file
ROBOT_SPEC_FILE: str = "robot_spec.yaml"

# Pump Module states
class PumpState(str, Enum):
    INIT = "INIT"
    RUN = "RUN"
    DEGRADED = "DEGRADED"
    OFF = "OFF"

class Pump():

    """
    Pump Module:
    - Harvests useful glob from the Jetson reservoir using Triple Action Pump (TAP) technique
    - Maintains a stream of consciousness (SOC) of the harvested glob
    - Exports the harvested glob to the vital terminal core for determining the health status

    Functions:
    - which_flow_rate_for_this_channel: Return the flow rate (CPS) for a given channel
    - what_pumping_rhythm_for_this_channel: Return the pumping rhythm (sec) for a given channel
    - engage_tap_to_harvest_glob: Harvest glob from the Jetson reservoir and put into the bin using TAP technique
    - harvest_glob_from_lifestream: Harvest glob from a specific flow channel
    - trace_thru_conduit: Trace through the conduit to extract the glob
    - close_floodgate: Gracefully stop the flow of the Jetson reservoir flow
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

    # Sentinel value indicating collection point is not available
    COLLECTION_POINT_NOT_AVAILABLE: int = -256
    
    def __init__(self):
        """
        Run the initialization of the pump module:
        - Set up "INIT" state and lock
        - Connect to the Jetson reservoir
        - Build the explicit conduit map
        - Verify the conduit map by performing a smoke test
        - Transition to the "RUN" state if all checks pass

        Returns:
            None
        """

        # Set up "INIT" state and lock
        self.state: PumpState = PumpState.INIT
        self._lock = threading.Lock()
        
        # Connect to the Jetson reservoir
        t0 = time.perf_counter()
        if not self._connect_to_jetson_reservoir():
            self.state = PumpState.DEGRADED
            return
        print(f"Connected to Jetson reservoir in {time.perf_counter() - t0:.6f} seconds")
        
        # Build the explicit conduit map
        t0 = time.perf_counter()
        self._CONDUIT_MAP: ConduitMapBlueprint = self._build_conduit_map()        
        print(f"Built conduit map in {time.perf_counter() - t0:.6f} seconds")
        
        # Verify the conduit map by performing a smoke test
        t0 = time.perf_counter()
        if not self._smoke_test_floodgates():
            self.state = PumpState.DEGRADED
            return
        print(f"Verified conduit map in {time.perf_counter() - t0:.6f} seconds")
        
        # Transition to the "RUN" state
        self.state = PumpState.RUN

    def terminate(self):
        """Terminate the pump module and close the floodgate."""
        self.close_floodgate()
        self.state = PumpState.OFF
   
    def close_floodgate(self):
        """Gracefully stop the flow of the Jetson reservoir"""
        if self.jetson_reservoir and self.jetson_reservoir.ok():
            self.jetson_reservoir.close()

    def trace_thru_conduit(self, lifestream: Any, path: ConduitJunctionList) -> Any:
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
        Collect glob from the Jetson reservoir and put into the bin using Triple Action Pump (TAP) technique:
        - Connect the pump conduits to the collection points of the Jetson reservoir:
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
        conduit_map: tuple[ConduitType, ...] = self._CONDUIT_MAP.get(channel, ())
        
        # Wrap the entire flow update in a lock to ensure thread safety
        with self._lock:
            for conduit in conduit_map:
                # Ensure deque exists in the bin
                if conduit.conduit_id not in bin["glob"]:
                    bin["glob"][conduit.conduit_id] = deque(maxlen=self.MAX_STREAM_CAPACITY)

                # Harvest the glob from the lifestream via the conduit path
                glob = None
                if self.jetson_reservoir and self.jetson_reservoir.ok():
                    # Only trace the conduit if it's connection to the Jetson lifestream is established
                    if conduit.collection_point is not None: 
                        glob = self.trace_thru_conduit(conduit.collection_point, conduit.junction_path)

                    # Sift out the impurity or empty glob
                    if glob == self.COLLECTION_POINT_NOT_AVAILABLE:
                        glob = None

                # Append the glob to the deque in the bin, even if it is None
                bin["glob"][conduit.conduit_id].append(glob)               
        return bin

    def _build_conduit_map(self) -> ConduitMapBlueprint:
        """Return the conduit map grouped by flow channel using parallel initialization"""
        # Read the conduit map blueprint from the robot spec file
        blueprint = self.retrieve_conduit_map_blueprint_from_hardware_spec()
        if not blueprint:
            raise ValueError("Conduit map blueprint not found.")
    
        # --- STEP 1: THE SCOUT (Find unique hardware modules) ---
        # Read the conduit blueprint map to find out the names of the collection point
        unique_modules = set()
        def scout(data: Dict[str, Any]):
            for val in data.values():
                if type(val) is dict:
                    scout(val)
                elif type(val) is list:
                    unique_modules.add(val[0]) # Always index 0 per your spec
        scout(blueprint)
    
        # --- STEP 2: THE HAMMER (Parallel Hardware Handshake) ---
        # Get ready to create a clone duplicate of the conduit map for reference
        self.cloned_conduit_map: Dict[str, Any] = {}
        
        def fetch_hardware(name):
            # This is where the 0.1s lag is neutralized by threading
            return name, getattr(self.jetson_reservoir, name, None)
    
        if self.floodgate_is_open() and unique_modules:
            # Spawn one thread per module to pay the hardware tax all at once
            with ThreadPoolExecutor(max_workers=len(unique_modules)) as executor:
                results = executor.map(fetch_hardware, unique_modules)
                self.cloned_conduit_map = dict(results)
    
        # --- STEP 3: THE BUILDER (Create the actual ConduitType objects) ---
        channels: Dict[FlowChannel, List[ConduitType]] = {"HI": [], "MID": [], "LO": []}
    
        self._locate_conduit_heads(blueprint, channels)
        
        # Freeze the conduit map for each flow channel
        return MappingProxyType({ch: tuple(con) for ch, con in channels.items()})
       
    def _connect_to_jetson_reservoir(self) -> bool:
        """
        Connect to the Jetson reservoir to initialize the running of the lifestream
        - Adjust the flow rate of the Jetson reservoir
        - Start running the Jetson lifestream out of the reservoir
        - Check if the floodgate is open
        Returns:
            bool: True if connection is successful, False otherwise
        """

        self.jetson_reservoir = None
        try:
            # Adjust the flow rate of the Jetson reservoir
            self.jetson_reservoir = jtop(interval=self.what_pumping_rhythm_for_this_channel("HI"))
            # Start running the Jetson lifestream out of the reservoir
            self.jetson_reservoir.start()

            # Wait for the floodgate to be open
            if not self.floodgate_is_open():
                #TODO: Log the error in the log file
                raise RuntimeError("Error: Jetson floodgate is not open.")
            
            return True

        except Exception as e:
            # In case of error, log the error and tell the pump that the Jetson reservoir is running empty
            #TODO: Log the error in the log file
            print(f"Error: Jetson reservoir failed to run: {e}")
            self.jetson_reservoir = None
            return False
        
    def _locate_conduit_heads(self, data: Dict[str, Any], channels: Dict[FlowChannel, List[ConduitType]], junction: str = ""):
        for name, path in data.items():
            conduit_junction = f"{junction}.{name}" if junction else name
            
            if type(path) is dict:
                # Recursive case: traverse deeper into the conduit map
                self._locate_conduit_heads(path, channels, conduit_junction)
            elif type(path) is list:
                # Destruct the junction path to identify each component
                module_name, junction_path, *config = path
                
                # Pull the channel from robot spec YAML file, or default to LO if not available
                flow_rate = cast(FlowChannel, config[0] if config else "LO")
                
                # Pre-bind the collection point to the conduit using the cloned conduit map
                collection_point = self.cloned_conduit_map.get(module_name)
                
                # Create the conduit and add it to the channel
                channels[flow_rate].append(ConduitType(conduit_junction, collection_point, tuple(junction_path)))

    def _smoke_test_floodgates(self) -> bool:
        """Prunes broken conduits and ensures at least some data is flowing."""
        # Give lifestream floodgates up to half a second to warm up in order to fetch the first glob of data
        time_limit = time.time() + 0.5
        broken_conduits: List[Tuple[FlowChannel, ConduitType]] = set()
        
        while time.time() < time_limit:
            broken_conduits = []
            
            # Identify the 'dry' conduits
            for channel, conduits in self._CONDUIT_MAP.items():
                for conduit in conduits:
                    # Conduct a DEEP trace to detect any anomaly in the conduit map blueprint
                    test_bin = self.trace_thru_conduit(conduit.collection_point, conduit.junction_path)
                    if test_bin is None:
                        broken_conduits.append((channel, conduit))

            # If everything is fine, we're golden
            if not broken_conduits:
                return True
                
            # If we found issues, we don't give up yet! 
            # We wait a bit to see if the hardware wakes up.
            time.sleep(0.1)

        # Final Verdict: Prune what's broken
        if broken_conduits:
            for channel, conduit in broken_conduits:
                print(f"Warning: Pruning dead conduit: {channel} -> {conduit.junction_path}")
                # Logic here to remove from self._CONDUIT_MAP
                
        # Return True if we still have at least SOME conduits left
        return len(self._CONDUIT_MAP) > 0
    
    def floodgate_is_open(self) -> bool:
        """Check if the floodgate is open (i.e., Jetson reservoir is connected and OK)"""
        return self.jetson_reservoir is not None and self.jetson_reservoir.ok()

    @classmethod
    def which_flow_rate_for_this_channel(cls, channel: FlowChannel) -> int:
        """Return the flow rate of lifestream for a given flow channel"""
        return cls.FLOW_RATE[channel]
    
    @classmethod
    def what_pumping_rhythm_for_this_channel(cls, channel: FlowChannel) -> float:
        """Return pumping rhythm (sec) for a given level"""
        return 1.0 / cls.which_flow_rate_for_this_channel(channel)
    
    @staticmethod
    def retrieve_conduit_map_blueprint_from_hardware_spec() -> ConduitMapBlueprint:
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
        
    def __enter__(self):
        """Prepare the pump for a high-speed heist."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Clean up the mess, even if we tripped the alarms."""
        if exc_type:
            # Maybe log that we're bailing out because of a meatbag error
            print(f"Bailing out! Error detected: {exc_val}")
        
        self.terminate()
        # Returning False (default) ensures the exception still propagates
        return False        
if __name__ == "__main__":

    # Run the diagnosis of the Pump Module to check if working as expected
    import time

    def color_temp(temp):
        """Color code the temperature based on the value."""
        if temp is None:
            return "\033[38;5;240m"  # Gray for unknown
        return "\033[38;5;196m" if temp > 80 else "\033[38;5;46m"
    
    # Initialize the Pump Module
    with Pump() as pump:
    
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
                disk_usage_deque = glob.get("glob", {}).get("memory.ltm.used")
                disk_usage = max([v for v in disk_usage_deque if v is not None], default=None) if disk_usage_deque else None
    
                # Color code the temperature based on the value
                cpu_color = color_temp(temperature_cpu)
                gpu_color = color_temp(temperature_gpu)
                cpu_load_str = f"{cpu_load:.2f}%" if cpu_load is not None else "N/A"
                temperature_cpu_str = f"{temperature_cpu}°C" if temperature_cpu is not None else "N/A"
                gpu_load_str = f"{gpu_load:.2f}%" if gpu_load is not None else "N/A"
                temperature_gpu_str = f"{temperature_gpu}°C" if temperature_gpu is not None else "N/A"
                disk_usage_str = f"{disk_usage:.2f}%" if disk_usage is not None else "N/A"
                print(f"Run #: {glob['run']+1} | Timestamp: [{glob['timestamp']}]")
                print(f" CPU Load: {cpu_load_str} | CPU: {cpu_color}{temperature_cpu_str}\033[0m")
                print(f" GPU Load: {gpu_load_str} | GPU: {gpu_color}{temperature_gpu_str}\033[0m")
                print(f" Disk Usage: {disk_usage_str}")
                            
                
        except KeyboardInterrupt:
            # Stop the diagnosis if user interrupts
            print("\nDiagnosis is stopped by user.")
            
    print("--- VTC PUMP MODULE DIAGNOSIS COMPLETE ---")
