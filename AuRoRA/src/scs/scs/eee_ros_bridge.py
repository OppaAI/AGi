"""
EEE ROS Bridge - REFLEX and AWARENESS channels.
These are OPTIONAL bridging plugins that inject ROS capabilities into EEE.
"""

import logging
from typing import Dict

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import Log
from rclpy.node import Node
from std_srvs.srv import Trigger

# Import EEE at module level (fail fast if missing)
from scs.eee import EEEAggregator


class EEEROSBridge(Node):
    """
    ROS 2 Bridge Node for EEE TALLE 2.0.
    
    Provides:
    - REFLEX channel (/diagnostics): Real-time safety alerts
    - AWARENESS channel (/rosout): Human-readable logs
    - Health query service (/eee/health): System status
    """
    
    def __init__(self):
        """
        Initialize the EEEROSBridge ROS 2 node, create and register REFLEX and AWARENESS channel plugins, expose the health query service, and install a shutdown hook.
        
        Creates ReflexPlugin and AwarenessPlugin instances and stores them in self._plugins, registers a Trigger service at /eee/health handled by _handle_health_query, and creates a dummy subscription to hook into node shutdown. Logs startup information. Raises an exception if channel initialization fails.
        """
        super().__init__('eee_ros_bridge')
        
        # Store plugins for lifecycle management
        self._plugins: list = []
        
        # Initialize channels
        try:
            self.reflex = ReflexPlugin(self)
            self.awareness = AwarenessPlugin(self)
            self._plugins.extend([self.reflex, self.awareness])
            
            self.get_logger().info("✅ REFLEX channel: /diagnostics")
            self.get_logger().info("✅ AWARENESS channel: /rosout")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize channels: {e}")
            raise  # Can't function without channels
        
        # Health query service
        self.health_srv = self.create_service(
            Trigger, 
            '/eee/health', 
            self._handle_health_query
        )
        
        # Register shutdown hook
        self.create_subscription(
            Log,  # Dummy - actually use context.on_shutdown
            '/eee/shutdown',
            lambda _: None,
            1
        )
        
        self.get_logger().info("🚀 TALLE 2.0 ROS Bridge LIVE")
        self.get_logger().info(f"   Robot: {self.get_name()}")
        self.get_logger().info(f"   Monitoring {len(self._plugins)} channels")

    def _handle_health_query(self, request, response):
        """
        Handle /eee/health Trigger service requests by summarizing REFLEX diagnostic statuses.
        
        Builds a summary of cached DiagnosticStatus entries from the REFLEX plugin, counts ERROR and WARN levels, and populates the Trigger response accordingly. Sets response.success to True only when no ERROR-level statuses are present and response.message to "Modules: <n> | Errors: <e> | Warnings: <w>". Logs a warning if any errors are detected. On exception, sets response.success to False and response.message to the error string.
        
        Parameters:
            request: The incoming Trigger request (unused).
            response: The Trigger response object to populate.
        
        Returns:
            The populated Trigger response object.
        """
        try:
            module_states = self.reflex.get_all_statuses()
            error_count = sum(
                1 for s in module_states.values() 
                if s.level == DiagnosticStatus.ERROR
            )
            warn_count = sum(
                1 for s in module_states.values() 
                if s.level == DiagnosticStatus.WARN
            )
            
            response.success = (error_count == 0)
            response.message = (
                f"Modules: {len(module_states)} | "
                f"Errors: {error_count} | "
                f"Warnings: {warn_count}"
            )
            
            if error_count > 0:
                self.get_logger().warn(f"Health check: {error_count} errors detected")
                
        except Exception as e:
            response.success = False
            response.message = f"Health check failed: {str(e)}"
        
        return response

    def destroy_node(self):
        """
        Shut down the bridge, stop registered plugins, flush the EEE disk queue, and destroy the node.
        
        Calls each plugin's shutdown method (plugin shutdown errors are logged and ignored), invokes EEEAggregator.shutdown() to flush the disk queue, and then delegates to the superclass destroy_node().
        """
        self.get_logger().info("Shutting down EEE ROS Bridge...")
        
        for plugin in self._plugins:
            try:
                plugin.shutdown()
            except Exception as e:
                self.get_logger().warn(f"Plugin shutdown error: {e}")
        
        # Flush EEE disk queue
        EEEAggregator.shutdown()
        
        super().destroy_node()


class ReflexPlugin:
    """
    REFLEX Channel: Real-time safety alerts (/diagnostics).
    """
    NAME = "reflex"
    
    def __init__(self, ros_node: Node):
        """
        Initialize the ReflexPlugin: store the parent ROS node, create the /diagnostics publisher, initialize the status cache, and register the plugin with the EEE aggregator.
        
        Parameters:
            ros_node (Node): Parent ROS 2 node used to create publishers and obtain the node name.
        """
        self.node = ros_node
        self.robot_id = ros_node.get_name()
        
        self._pub = ros_node.create_publisher(
            DiagnosticArray, 
            '/diagnostics', 
            10
        )
        self._cache: Dict[str, DiagnosticStatus] = {}
        
        # Register with EEE
        EEEAggregator.register_plugin(self.NAME, self._handle)
    
    def _handle(self, level: int, msg: str, evidence: dict, meta: dict) -> bool:
        """
        Handle an EEE event by publishing a DiagnosticStatus for warning-or-higher events and caching it.
        
        Builds a DiagnosticStatus from the provided event data, publishes it to the diagnostics topic, and stores it in the plugin cache for later health queries when the event level is WARNING or higher. If publishing fails, an error is logged and the event is not considered handled.
        
        Returns:
            `True` if the event was published and should block disk fallback (level >= WARNING and level < CRITICAL), `False` otherwise.
        """
        if level < logging.WARNING:
            return False
        
        try:
            status = self._build_status(level, msg, evidence, meta)
            self._publish(status)
            self._cache[status.name] = status
            
        except Exception as e:
            # Log error but don't crash EEE
            self.node.get_logger().error(f"REFLEX publish failed: {e}")
            return False  # Allow disk fallback
        
        # Block disk queue for non-critical (they're in cache)
        return level < logging.CRITICAL
    
    def _build_status(self, level: int, msg: str, evidence: dict, meta: dict) -> DiagnosticStatus:
        """
        Construct a ROS DiagnosticStatus representing an EEE event.
        
        Parameters:
            level (int): Numeric log level from EEE; values >= logging.ERROR map to DiagnosticStatus.ERROR, otherwise DiagnosticStatus.WARN.
            msg (str): Event message; will be truncated to 255 characters for the status.message field.
            evidence (dict): Mapping of evidence keys to values to include as KeyValue entries. The "traceback" key is skipped; values that cannot be serialized are omitted. Keys and values are truncated to 255 characters.
            meta (dict): Metadata for the event. Must contain "proc_id" (used for status.name) and "correlation_id" (used as the "cid" KeyValue, truncated to 8 characters).
        
        Returns:
            DiagnosticStatus: A DiagnosticStatus populated with level, name, message, hardware_id, and KeyValue entries derived from evidence and correlation id.
        """
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.ERROR if level >= logging.ERROR else DiagnosticStatus.WARN
        status.name = meta["proc_id"]
        status.message = msg[:255]
        status.hardware_id = self.robot_id
        
        # Add structured evidence
        if evidence:
            for key, value in evidence.items():
                if key == "traceback":
                    continue  # Too large
                try:
                    kv = KeyValue(key=str(key)[:255], value=str(value)[:255])
                    status.values.append(kv)
                except Exception:
                    pass  # Skip unserializable values
        
        # Add correlation ID
        status.values.append(KeyValue(key="cid", value=meta["correlation_id"][:8]))
        
        return status
    
    def _publish(self, status: DiagnosticStatus):
        """
        Publish a DiagnosticArray containing the given DiagnosticStatus to the /diagnostics topic.
        
        Parameters:
            status (DiagnosticStatus): The diagnostic status to include in the published DiagnosticArray.
        """
        diag = DiagnosticArray()
        diag.header.stamp = self.node.get_clock().now().to_msg()
        diag.header.frame_id = self.robot_id
        diag.status = [status]
        self._pub.publish(diag)
    
    def get_all_statuses(self) -> Dict[str, DiagnosticStatus]:
        """
        Provide a copy of the cached DiagnosticStatus entries keyed by plugin name.
        
        Returns:
            Dict[str, DiagnosticStatus]: Shallow copy of the internal cache mapping plugin names to their `DiagnosticStatus` objects.
        """
        return self._cache.copy()
    
    def shutdown(self):
        """
        Shut down the plugin and clear its cached statuses.
        
        Unregisters the plugin from EEEAggregator and clears the internal status cache (_cache).
        """
        EEEAggregator.unregister_plugin(self.NAME)
        self._cache.clear()


class AwarenessPlugin:
    """
    AWARENESS Channel: Human-readable logs (/rosout).
    """
    NAME = "awareness"
    
    def __init__(self, ros_node: Node):
        """
        Initialize the AwarenessPlugin and attach a ROS /rosout publisher plus a Python logging handler.
        
        Parameters:
            ros_node (Node): Parent ROS node used to create the /rosout publisher and to obtain the node context for the log handler.
        """
        self.node = ros_node
        self._pub = ros_node.create_publisher(Log, '/rosout', 100)
        
        # Create and register handler
        self._handler = self._create_handler()
        logging.getLogger().addHandler(self._handler)
        
        # Prevent double-registration on reload
        self._handler_added = True
    
    def _create_handler(self):
        """
        Create a logging.Handler that converts Python log records into ROS `Log` messages and publishes them to /rosout.
        
        The returned handler:
        - Maps Python log levels to ROS `Log` levels.
        - Formats logger names by replacing '.' with '::' and uppercasing.
        - Preserves messages that already contain a correlation ID substring "CID:".
        - Uses the captured node clock for message timestamps and the captured publisher to publish messages.
        - Enters a permanent local fallback on the first exception encountered to avoid repeated publish errors.
        
        Returns:
            logging.Handler: A handler instance that emits converted ROS `Log` messages and suppresses further emits after a failure.
        """
        node = self.node  # Capture for closure
        pub = self._pub
        
        class ROSLogHandler(logging.Handler):
            def __init__(self):
                """
                Initialize the ROS log handler and set the internal fallback flag.
                
                Initializes the base logging.Handler and sets an internal `_fallback` flag used to suppress further publish attempts after a handler failure.
                """
                super().__init__()
                self._fallback = False  # Flag if ROS fails
            
            def emit(self, record):
                """
                Publish a Python logging.Record to the ROS /rosout publisher as an rclpy Log message.
                
                Converts the input `record` into an rclpy Log message (mapping level, formatting the message, and setting the timestamp and logger name) and publishes it to the configured ROS publisher when the node is active. If the node is shutting down the record is skipped. On any exception the handler enters a permanent fallback state to avoid repeated failures.
                
                Parameters:
                    record (logging.LogRecord): The Python log record to convert and publish.
                
                Returns:
                    None
                """
                if self._fallback:
                    return  # ROS is dead, don't spam errors
                
                try:
                    if not node.context.ok():
                        return  # Node shutting down
                    
                    ros_log = Log()
                    ros_log.stamp = node.get_clock().now().to_msg()
                    ros_log.level = self._map_level(record.levelno)
                    ros_log.name = record.name.replace('.', '::').upper()
                    ros_log.msg = self._format_message(record)
                    
                    pub.publish(ros_log)
                    
                except Exception as e:
                    # One failure = permanent fallback (prevent spam)
                    self._fallback = True
                    print(f"AWARENESS fallback: {e}")
            
            def _map_level(self, lvl: int) -> int:
                """
                Map a Python logging level to the corresponding ROS `Log` level constant.
                
                Parameters:
                    lvl (int): Python logging level value (e.g., `logging.DEBUG`, `logging.INFO`).
                
                Returns:
                    int: The matching `rcl_interfaces.msg.Log` level constant (`Log.ERROR`, `Log.WARN`, `Log.INFO`, or `Log.DEBUG`).
                """
                if lvl >= logging.ERROR: return Log.ERROR
                if lvl >= logging.WARNING: return Log.WARN
                if lvl >= logging.INFO: return Log.INFO
                return Log.DEBUG
            
            def _format_message(self, record) -> str:
                """
                Format a log message and preserve an existing EEE correlation ID if present.
                
                Parameters:
                    record (logging.LogRecord): The log record to format.
                
                Returns:
                    str: The formatted message. If the message already contains an EEE correlation ID (contains "CID:"), it is returned unchanged.
                """
                msg = record.getMessage()
                # Extract CID if present in EEE format
                if "CID:" in msg:
                    return msg  # Already formatted by EEE
                return msg
        
        return ROSLogHandler()
    
    def shutdown(self):
        """
        Deregister the plugin's logging handler from the root logger.
        
        If the handler was previously added, remove it from the root logger and mark the plugin
        as not having an active handler.
        """
        if self._handler_added:
            logging.getLogger().removeHandler(self._handler)
            self._handler_added = False