# System modules
from ament_index_python.packages import get_package_share_directory
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from jtop import jtop
import os, psutil
from pathlib import Path
from types import MappingProxyType
from typing import Any, cast, Dict, List, Literal, NamedTuple, Tuple, Union
import threading
import time
import yaml

# AGi modules
from scs.igniter_temp import StateOfModule
from scs.eee import get_logger

# Identifiers of this process
PROC_ID = "VCS.VTC.Pump"

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
        Initialize the Pump module: prepare internal state and lock, connect to the Jetson reservoir, build and validate the conduit map, and transition to RUN on success.
        
        If connection to the reservoir or the smoke-test validation fails, the instance state is set to StateOfModule.DEGRADED and initialization returns early. Side effects include setting self.process, self._lock, self.logger, self._CONDUIT_MAP, and self.state, and may start the jetson reservoir.
        """

        # Set up "INIT" state and lock
        self.process = psutil.Process(os.getpid())
        self.state: StateOfModule = StateOfModule.INIT
        self._lock = threading.Lock()

        self.logger = get_logger(PROC_ID)
        self.logger.info("Pump Module Activation Sequence initiated...")

        # Connect to the Jetson reservoir
        #(TODO): to remove time logger
        t0 = time.perf_counter()
        p0 = self.process.memory_info().rss
        if not self._connect_to_jetson_reservoir():
            self.logger.error("Connection to Jetson Reservoir...[ABORT]")
            self.state = StateOfModule.DEGRADED
            return
        dp = self.process.memory_info().rss - p0
        dt = time.perf_counter() - t0
        self.logger.info(f"Connection to Jetson Reservoir...[CLEAR] - Elapsed: {dt:.6f} sec | Consumed: {dp / (1024*1024):.6f} MB")
        
        # Build the explicit conduit map
        t0 = time.perf_counter()
        p0 = self.process.memory_info().rss
        self._CONDUIT_MAP: ConduitMapBlueprint = self._build_conduit_map()
        dp = self.process.memory_info().rss - p0
        dt = time.perf_counter() - t0
        self.logger.info(f"Rebuilding of Conduit Map Blueprint...[CLEAR] - {sum(len(v) for v in self._CONDUIT_MAP.values())} conduits - Elapsed: {dt:.6f} sec | Consumed: {dp / (1024*1024):.6f} MB")
        
        # Verify the conduit map by performing a smoke test
        t0 = time.perf_counter()
        p0 = self.process.memory_info().rss
        if not self._smoke_test_floodgates():
            self.logger.error("Validation with Smoke Test...[ABORT]")
            self.state = StateOfModule.DEGRADED
            return
        dp = self.process.memory_info().rss - p0
        dt = time.perf_counter() - t0
        self.logger.info(f"Validation with Smoke Test...[CLEAR] - Elapsed: {dt:.6f} sec | Consumed: {dp / (1024*1024):.6f} MB")
        
        # Transition to the "RUN" state
        self.state = StateOfModule.RUN
        self.logger.info("Pump Module Activation Sequence completed...[SYSTEM ALL GREEN]")

    def terminate(self):
        """
        Shut down the Pump by closing the floodgate and marking the module OFF.
        
        Performs a floodgate close and sets the Pump's state to StateOfModule.OFF.
        """
        self.close_floodgate()
        self.state = StateOfModule.OFF
   
    def close_floodgate(self):
        """Gracefully stop the flow of the Jetson reservoir"""
        if self.jetson_reservoir is not None:
            self.jetson_reservoir.close()

    def trace_thru_conduit(self, lifestream: Any, path: ConduitJunctionList) -> Any:
        """
        Traverse a nested lifestream following a conduit junction path to retrieve the final value.
        
        Parameters:
            lifestream (Any): A nested mapping/sequence representing the conduit lifestream.
            path (ConduitJunctionList): Sequence of keys or indices that define the junction path to follow.
        
        Returns:
            Any: The value found at the end of the path, or `None` if any key/index is missing or the path cannot be traversed.
        """
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
        Harvest a data "glob" into the provided bin using the Triple Action Pump rhythm across HI, MID, and LO channels.
        
        Parameters:
            bin (dict[str, Any]): Mutable container for collection state. Expected keys:
                - "run" (int): current run counter (will be created with 0 if missing).
                - "glob" (dict): mapping where harvested per-conduit deques are stored (will be created if missing).
        
        Returns:
            dict[str, Any]: The same `bin` updated with harvested data from one TAP cycle. The method:
                - Ensures `bin` has "run" and "glob" defaults,
                - Always harvests from the HI channel,
                - Harvests from the MID channel when the run counter is at the cycle midpoint determined by channel flow rates,
                - Harvests from the LO channel when the run counter completes a full cycle determined by channel flow rates.
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
        Harvests data ("glob") from each conduit in the specified flow channel into the provided bin.
        
        Checks the floodgate once; for each conduit in the channel, if the floodgate is open and a collection point exists, traces the conduit path to obtain a value and appends that value (or `None` when unavailable) to a per-conduit deque stored at bin["glob"][conduit_id]. The per-conduit deque is created if missing and capped to the module's MAX_STREAM_CAPACITY; appends are performed under the Pump's internal lock.
        
        Parameters:
            bin (dict[str, Any]): Mutable container that must include a "glob" mapping; per-conduit deques are created/updated at bin["glob"][conduit_id].
            channel (FlowChannel): Flow channel whose conduits should be harvested.
        
        Returns:
            dict[str, Any]: The same `bin` object with updated per-conduit deques containing the newly appended values.
        """
        # Check the gate once here just to know if tracing is necessary.
        floodgate_is_open = self.floodgate_is_open()
        
        # Load the frozen conduit map for the given channel
        conduit_map: tuple[ConduitType, ...] = self._CONDUIT_MAP.get(channel, ())
        
        for conduit in conduit_map:
            # Harvest the glob from the lifestream via the conduit path
            glob = None
            if floodgate_is_open and conduit.collection_point is not None:
                # Only trace the conduit if it's connection to the Jetson lifestream is established
                glob = self.trace_thru_conduit(conduit.collection_point, conduit.junction_path)

                # Sift out the impurity or empty glob
                if glob == self.COLLECTION_POINT_NOT_AVAILABLE:
                    glob = None
                    
                with self._lock:
                    # Wrap the entire flow update in a lock to ensure thread safety
                    # Ensure deque exists in the bin
                    if conduit.conduit_id not in bin["glob"]:
                        bin["glob"][conduit.conduit_id] = deque(maxlen=self.MAX_STREAM_CAPACITY)

                    # Append the glob to the deque in the bin, even if it is None
                    bin["glob"][conduit.conduit_id].append(glob)               
        return bin

    def _build_conduit_map(self) -> ConduitMapBlueprint:
        """
        Builds a conduit map grouped by flow channel from the robot spec blueprint.
        
        Reads the conduit map blueprint from the hardware spec, identifies referenced hardware modules,
        optionally probes the Jetson reservoir in parallel to populate self.cloned_conduit_map, and
        constructs immutable per-channel tuples of ConduitType objects stored in a MappingProxyType.
        
        Returns:
            ConduitMapBlueprint: A mapping from each FlowChannel ("HI", "MID", "LO") to a tuple of ConduitType objects.
        
        Raises:
            ValueError: If the conduit map blueprint is not found in the robot spec.
        """
        # Read the conduit map blueprint from the robot spec file
        blueprint = self.retrieve_conduit_map_blueprint_from_hardware_spec()
        if not blueprint:
            raise ValueError("Conduit map blueprint not found.")
    
        # --- STEP 1: THE SCOUT (Find unique hardware modules) ---
        # Read the conduit blueprint map to find out the names of the collection point
        unique_modules = set()
        def scout(data: Dict[str, Any]):
            """
            Collect unique hardware module names from a nested conduit blueprint.
            
            Recursively traverses `data`, which is a mapping of dicts and lists representing part of the conduit blueprint.
            Whenever a list is encountered, its first element (index 0) is added to the `unique_modules` set.
            This function has no return value and mutates the `unique_modules` set in place.
            
            Parameters:
                data (Dict[str, Any]): Nested mapping representing a subtree of the conduit blueprint.
            """
            for val in data.values():
                if type(val) is dict:
                    scout(val)
                elif type(val) is list:
                    unique_modules.add(val[0]) # Always index 0 per your spec
        scout(blueprint)
    
        # --- STEP 2: THE HAMMER (Parallel Hardware Handshake) ---
        # Get ready to create a clone duplicate of the conduit map for reference
        self.cloned_conduit_map: Dict[str, Any] = {}

    
        if not self.floodgate_is_open():
            # (TODO) Logging here
            print("Jetson reservoir is bone dry. Skipping hardware handshake.")
        elif unique_modules:           
            def fetch_hardware(name):
                # This is where the 0.1s lag is neutralized by threading
                """
                Retrieve a named hardware module handle from the Jetson reservoir.
                
                Parameters:
                	name (str): The attribute name of the hardware module to fetch from the reservoir.
                
                Returns:
                	tuple[str, Any]: A pair (name, handle) where `handle` is the attribute value from `self.jetson_reservoir` if present, or `None` if the attribute does not exist.
                """
                return name, getattr(self.jetson_reservoir, name, None)
        
            # Spawn one thread per module to pay the hardware tax all at once
            with ThreadPoolExecutor(max_workers=len(unique_modules)) as executor:
                results = executor.map(fetch_hardware, unique_modules)
                self.cloned_conduit_map = dict(results)
    
        # --- STEP 3: THE BUILDER (Create the actual ConduitType objects) ---
        channels: Dict[FlowChannel, List[ConduitType]] = {"HI": [], "MID": [], "LO": []}
    
        self._locate_conduit_heads(blueprint, channels)
        
        # Freeze the conduit map for each flow channel
        return MappingProxyType({ch: tuple(con) for ch, con in channels.items()})
       
    def _connect_to_jetson_reservoir(self)-> bool:
        """
        Initialize and start the Jetson reservoir and verify its floodgate is open.
        
        On success stores the started reservoir instance on self.jetson_reservoir; on failure leaves self.jetson_reservoir as None.
        
        Returns:
            bool: `True` if the reservoir was started and the floodgate is open, `False` otherwise.
        """

        self.jetson_reservoir = None
        try:
            self.logger.debug(f"Establishing connection to Jetson reservoir with interval={self.what_pumping_rhythm_for_this_channel('HI')}")
            # Adjust the flow rate of the Jetson reservoir
            self.jetson_reservoir = jtop(interval=self.what_pumping_rhythm_for_this_channel("HI"))
            # Start running the Jetson lifestream out of the reservoir
            self.jetson_reservoir.start()

            # Wait for the floodgate to be open
            if not self.floodgate_is_open():
                self.logger.fatal("Opening Floodgate of Jetson Reservoir...[ABORT]")
                raise RuntimeError("CRITICAL: Floodgate of Jetson Reservoir cannot open.")
            
            self.logger.info("Opening Floodgate of Jetson Reservoir...[CLEAR]")
            return True

        except Exception as e:
            # In case of error, log the error and tell the pump that the Jetson reservoir is running empty
            self.logger.exception(f"Error: Jetson reservoir failed to run: {e}")
            self.jetson_reservoir = None
            return False
        
    def _locate_conduit_heads(self, data: Dict[str, Any], channels: Dict[FlowChannel, List[ConduitType]], junction: str = ""):
        """
        Walks a conduit blueprint and populates the provided channels mapping with ConduitType entries.
        
        This method traverses the nested `data` blueprint, resolves each conduit entry to a ConduitType (binding its collection point from self.cloned_conduit_map), and appends the conduit to the appropriate flow channel list in `channels`. Leaf entries in `data` are expected to be lists of the form [module_name, junction_path, optional_flow_channel]. If no flow channel is provided the conduit is assigned to "LO".
        
        Parameters:
            data (Dict[str, Any]): Nested conduit blueprint to traverse; keys are conduit names or groups and values are either dicts (sub-groups) or lists defining a conduit.
            channels (Dict[FlowChannel, List[ConduitType]]): Mutable mapping of flow channel to list that will be extended with discovered ConduitType objects.
            junction (str): Dot-delimited prefix used to build each conduit’s conduit_id (used when traversing nested groups). Default is "".
        """
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
        """
        Check each conduit for liveliness, remove non-responsive conduits from the conduit map, and ensure at least one functional conduit remains.
        
        This method waits up to 0.5 seconds for transient hardware wake-up and rechecks failing conduits during that period; if failures persist, it prunes non-functional conduits from self._CONDUIT_MAP.
        
        Returns:
            bool: `True` if at least one conduit remains after pruning, `False` otherwise.
        """
        # Give lifestream floodgates up to half a second to warm up in order to fetch the first glob of data
        time_limit = time.monotonic() + 0.5
        broken_conduits: List[Tuple[FlowChannel, ConduitType]] = []
        
        while time.monotonic() < time_limit:
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

        # Prune out the non-functional conduits
        if broken_conduits:
            # Rebuild the map without the broken conduits
            pruned_conduit_map = {}
            broken_conduit_ids = {conduit.conduit_id for _, conduit in broken_conduits}
            for channel, conduit_list in self._CONDUIT_MAP.items():
                functional_conduit = tuple(conduit for conduit in conduit_list if conduit.conduit_id not in broken_conduit_ids)
                if functional_conduit: pruned_conduit_map[channel] = functional_conduit
                    
            # Re-freeze the map with functional conduits
            self._CONDUIT_MAP = MappingProxyType(pruned_conduit_map)
            print(f"Warning: Pruned {len(broken_conduit_ids)} broken conduits.")
            
        # Return True if we still have at least SOME conduits left
        return len(self._CONDUIT_MAP) > 0
    
    def floodgate_is_open(self) -> bool:
        """
        Determine whether the floodgate (Jetson reservoir) is available.
        
        Returns:
            `True` if a Jetson reservoir exists and its `ok()` method returns `True`, `False` otherwise.
        """
        return self.jetson_reservoir is not None and self.jetson_reservoir.ok()

    @classmethod
    def which_flow_rate_for_this_channel(cls, channel: FlowChannel) -> int:
        """
        Get the numeric flow rate associated with a flow channel.
        
        Parameters:
            channel (FlowChannel): Channel identifier ('LO', 'MID', or 'HI').
        
        Returns:
            int: The flow rate for the specified channel.
        """
        return cls.FLOW_RATE[channel]
    
    @classmethod
    def what_pumping_rhythm_for_this_channel(cls, channel: FlowChannel) -> float:
        """
        Get the pumping rhythm in seconds for the specified flow channel.
        
        Parameters:
            channel (FlowChannel): Flow channel identifier ("LO", "MID", or "HI").
        
        Returns:
            float: Pumping rhythm in seconds for the channel (1.0 divided by the channel's flow rate).
        """
        return 1.0 / cls.which_flow_rate_for_this_channel(channel)
    
    @staticmethod
    def retrieve_conduit_map_blueprint_from_hardware_spec() -> ConduitMapBlueprint:
        """
        Load the conduit map blueprint from the robot hardware spec YAML.
        
        Searches known locations for a robot_spec.yaml, reads its top-level "vcs" section, and returns the "conduit_map_blueprint" value when present.
        
        Returns:
            ConduitMapBlueprint: mapping of flow channels to tuples of ConduitType extracted from the spec; an empty mapping if no valid blueprint is found.
        
        Raises:
            ValueError: if the YAML file is present but cannot be parsed.
        """
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
        """
        Provide the pump instance for use as a context manager.
        
        Returns:
            self (Pump): The pump instance to be bound by the with-statement.
        """
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        Perform context-manager cleanup by terminating the Pump; if an exception occurred, log it and allow it to propagate.
        
        Parameters:
            exc_type: The exception type if an exception was raised, otherwise None.
            exc_val: The exception instance if an exception was raised, otherwise None.
            exc_tb: The traceback object if an exception was raised, otherwise None.
        
        Returns:
            False: do not suppress the exception (allow it to propagate).
        """
        if exc_type:
            # Maybe log that we're bailing out because of a meatbag error
            print(f"Bailing out! Error detected: {exc_val}")
        
        self.terminate()
        # Returning False (default) ensures the exception still propagates
        return False
    
if __name__ == "__main__":

    # Run the diagnosis of the Pump Module to check if working as expected
    from typing import List
    import time
    
    def worst_case_aggregation(values: List[float]) -> float | None:
        """
        Select the maximum value from a list of vital measurements.
        
        Parameters:
            values (List[float]): Sequence of numeric vital measurements to evaluate.
        
        Returns:
            max_value (float | None): The largest value from `values`, or `None` if `values` is empty.
        """
        return max(values) if values else None
    
    # Initialize the logger
    logger = get_logger(PROC_ID)
    logger.info("Initializing VTC Pump Module Diagnosis...")

    # Initialize the Pump Module
    with Pump() as pump:
    
        # Start the diagnosis
        logger.info("Diagnosing VTC Pump Module...")
        glob = {"timestamp": 0, "duration": 0.0, "run": 0, "glob": {}}
    
        # Initialize the vitals containers
        cpu_loads = {}; cpu_temps = {}
        gpu_loads = {}; gpu_temps = {}
        disk_usages = {}

        TEST_RUNS = 10

        # Run the diagnosis for pump cycles defined in TEST_RUNS
        try:
            # Record the start time
            time_start = time.perf_counter()

            for i in range(TEST_RUNS):
                # Collect the glob from the lifestream
                glob["run"] = i
                glob["timestamp"] = time.strftime("%H:%M:%S")
                glob = pump.engage_tap_to_harvest_glob(glob)
                
                # Get the worst case scenario of the vitals, if any
                cpu_user_load_deque = glob.get("glob", {}).get("p_unit.user")
                cpu_sys_load_deque = glob.get("glob", {}).get("p_unit.system")
                cpu_user = worst_case_aggregation(cpu_user_load_deque)
                cpu_sys = worst_case_aggregation(cpu_sys_load_deque)
                cpu_load = (cpu_user + cpu_sys) if (cpu_user is not None and cpu_sys is not None) else None

                temperature_cpu_deque = glob.get("glob", {}).get("body_temp.p_unit")
                temperature_cpu = worst_case_aggregation(temperature_cpu_deque)

                gpu_load_deque = glob.get("glob", {}).get("a_unit.load")
                gpu_load = worst_case_aggregation(gpu_load_deque)

                temperature_gpu_deque = glob.get("glob", {}).get("body_temp.a_unit")
                temperature_gpu = worst_case_aggregation(temperature_gpu_deque)

                disk_usage_deque = glob.get("glob", {}).get("memory.ltm.used")
                disk_usage = worst_case_aggregation(disk_usage_deque)
    
                cpu_loads[i] = cpu_load if cpu_load is not None else -1
                cpu_temps[i] = temperature_cpu if temperature_cpu is not None else -1
                gpu_loads[i] = gpu_load if gpu_load is not None else -1
                gpu_temps[i] = temperature_gpu if temperature_gpu is not None else -1
                disk_usages[i] = disk_usage if disk_usage is not None else -1

                #time.sleep(0.1)

            if max(cpu_loads.values()) > 80 or max(gpu_loads.values()) > 80:
                logger.warn(f"CPU Load: {max(cpu_loads.values()):.2f}% | CPU Temp: {max(cpu_temps.values()):.2f}°C | GPU Load: {max(gpu_loads.values()):.2f}% | GPU Temp: {max(gpu_temps.values()):.2f}°C | Disk Usage: {max(disk_usages.values()):.2f}%")
            else:
                logger.info(f"CPU Load: {max(cpu_loads.values()):.2f}% | CPU Temp: {max(cpu_temps.values()):.2f}°C | GPU Load: {max(gpu_loads.values()):.2f}% | GPU Temp: {max(gpu_temps.values()):.2f}°C | Disk Usage: {max(disk_usages.values()):.2f}%")

        except KeyboardInterrupt:
            # Stop the diagnosis if user interrupts
            logger.info("VTC Pump Module Diagnosis is interrupted by user.")
            
    logger.info(f"VTC Pump Module Diagnosis successful. Time taken: Total {(time.perf_counter() - time_start):.6f} seconds; Average {((time.perf_counter() - time_start)/TEST_RUNS):.6f} seconds per run")
    pump.terminate()