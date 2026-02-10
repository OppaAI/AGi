"""
EEE ROS Bridge - REFLEX and AWARENESS channels.
These are OPTIONAL bridging plugins that inject ROS capabilities into EEE.

CHANGELOG (Bug Fixes):
- Fixed Bug #4: Added missing 'stales' and 'stale_count' calculation in _handle_health_query
- Fixed Bug #6: Corrected STALE_TIMEOUT -> STALE_THRESHOLD in _check_stale_modules
- Fixed Bug #7: Improved CID detection robustness in AwarenessPlugin (changed "CID:" to "| CID:")
- Added Item #8: MetricsPlugin for real-time numeric trend visualization (/eee/metrics topic)
"""

import logging
import time
from typing import Dict

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import Log
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float64MultiArray

# Import EEE at module level (fail fast if missing)
from scs.eee import EEEAggregator


class EEEROSBridge(Node):
    """
    ROS 2 Bridge Node for EEE TALLE 2.0.
    
    Provides:
    - REFLEX channel (/diagnostics): Real-time safety alerts
    - AWARENESS channel (/rosout): Human-readable logs
    - METRICS channel (/eee/metrics): Numeric trends for visualization
    - Health query service (/eee/health): System status
    """
    
    def __init__(self):
        super().__init__('eee_ros_bridge')
        
        # Store plugins for lifecycle management
        self._plugins: list = []
        
        # Initialize channels
        try:
            self.reflex = ReflexPlugin(self)
            self.awareness = AwarenessPlugin(self)
            self.metrics = MetricsPlugin(self)
            self._plugins.extend([self.reflex, self.awareness, self.metrics])
            
            self.get_logger().info("✅ REFLEX channel: /diagnostics")
            self.get_logger().info("✅ AWARENESS channel: /rosout")
            self.get_logger().info("✅ METRICS channel: /eee/metrics")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize channels: {e}")
            raise  # Can't function without channels
        
        # Health query service
        self.health_srv = self.create_service(
            Trigger, 
            '/eee/health', 
            self._handle_health_query
        )

        # Throttle status service
        self.throttle_srv = self.create_service(
            Trigger,
            '/eee/throttle_status',
            self._handle_throttle_status
        )
        self.get_logger().info("✅ THROTTLE STATUS service: /eee/throttle_status")

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

    def _handle_throttle_status(self, request, response):
        """
        Service: Get current throttling status (all proc_ids).
        
        Returns:
            success: Always True
            message: JSON string of throttle status
        """
        try:
            status = EEEAggregator.get_throttle_status()  # Your existing classmethod
            response.success = True
            response.message = json.dumps(status, indent=2)
            self.get_logger().info("Throttle status queried")
        except Exception as e:
            response.success = False
            response.message = f"Throttle status failed: {str(e)}"
            self.get_logger().error(response.message)
    
        return response
    
    def _handle_health_query(self, request, response):
        """
        Service: Instant health check via REFLEX cache.
    
        Returns:
            success: True if no ERROR-level statuses
            message: Detailed summary with errored/warned/stale modules
        """
        try:
            module_states = self.reflex.get_all_statuses()  # Dict[proc_id, DiagnosticStatus]
            
            # ✅ FIX Bug #4: Calculate all three status lists
            errors = [name for name, s in module_states.items() if s.level == DiagnosticStatus.ERROR]
            warns = [name for name, s in module_states.items() if s.level == DiagnosticStatus.WARN]
            stales = [name for name, s in module_states.items() if s.level == DiagnosticStatus.STALE]
            
            error_count = len(errors)
            warn_count = len(warns)
            stale_count = len(stales)
        
            response.success = (error_count == 0)
            
            # Richer message: counts + list of affected modules (truncated if too many)
            details = []
            if errors:
                details.append(f"Errors ({error_count}): {', '.join(errors[:10])}" + ("..." if len(errors) > 10 else ""))
            if warns:
                details.append(f"Warnings ({warn_count}): {', '.join(warns[:10])}" + ("..." if len(warns) > 10 else ""))
            if stales:
                details.append(f"STALE ({stale_count}): {', '.join(stales[:10])}" + ("..." if len(stales) > 10 else ""))

            summary = f"Modules: {len(module_states)} | " + " | ".join(details) if details else "All OK"
            response.message = summary
        
            # Logging with proper status reporting
            if error_count > 0:
                self.get_logger().warn(f"Health check: {error_count} errors ({', '.join(errors)})")
            elif warn_count > 0:
                self.get_logger().info(f"Health check: {warn_count} warnings")
            elif stale_count > 0:
                self.get_logger().info(f"Health check: {stale_count} stale modules")
            else:
                self.get_logger().info("Health check: System healthy")
            
        except Exception as e:
            response.success = False
            response.message = f"Health check failed: {str(e)}"
            self.get_logger().error(response.message)
    
        return response

    def destroy_node(self):
        """Clean shutdown of plugins."""
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
    REFLEX Channel Plugin: Real-time machine-readable safety alerts.
    
    Publishes structured DiagnosticStatus messages to /diagnostics on WARNING+ events.
    - Maps EEE level → DiagnosticStatus level (WARN/ERROR)
    - Includes PROC_ID as name, evidence as key-values, short CID
    - Caches statuses for /eee/health query
    - Enforces publish cooldown to prevent spam
    - Supports STALE detection via watchdog timer
    """

    NAME = "reflex"
    PUBLISH_COOLDOWN = 1.0  # Min 1 second between publishes per proc_id
    STALE_THRESHOLD = 10.0  # 10 seconds without update = STALE
    
    def __init__(self, ros_node: Node):
        self.node = ros_node
        self.robot_id = ros_node.get_name()
        
        self._pub = ros_node.create_publisher(
            DiagnosticArray, 
            '/diagnostics', 
            10
        )
        self._cache: Dict[str, DiagnosticStatus] = {}
        self._last_publish: Dict[str, float] = {}  # proc_id -> last publish time

        # STALE Watchdog Timer
        self._stale_timer = ros_node.create_timer(
            1.0,  # Check every 1 second
            self._check_stale_modules
        )

        # Register with EEE
        EEEAggregator.register_plugin(self.NAME, self._handle)
    
    def _handle(self, level: int, msg: str, evidence: dict, meta: dict) -> bool:
        """
        EEE plugin callback.
        
        Returns:
            True: Event handled (block disk for non-critical)
            False: Allow fall-through to disk
        """
        if level < logging.WARNING:
            return False
        
        now = time.time()
        proc_id = meta["proc_id"]
        
        # Cooldown check: min 1s between publishes per proc_id
        if proc_id in self._last_publish and now - self._last_publish[proc_id] < self.PUBLISH_COOLDOWN:
            return False  # Throttled — skip publish (disk still gets it)
        
        try:
            status = self._build_status(level, msg, evidence, meta)
            self._publish(status)
            self._cache[status.name] = status
            self._last_publish[proc_id] = now  # Update last publish time
        except Exception as e:
            self.node.get_logger().error(f"REFLEX publish failed: {e}")
            return False
        
        # Block disk queue for non-critical (they're in cache)
        return level < logging.CRITICAL
    
    def _build_status(self, level: int, msg: str, evidence: dict, meta: dict) -> DiagnosticStatus:
        """Build DiagnosticStatus from EEE event."""
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

        # Add timestamp for STALE detection
        status.values.append(KeyValue(
            key="last_update", 
            value=str(time.monotonic())
        ))
        
        return status
    
    def _publish(self, status: DiagnosticStatus):
        """Publish to /diagnostics."""
        diag = DiagnosticArray()
        diag.header.stamp = self.node.get_clock().now().to_msg()
        diag.header.frame_id = self.robot_id
        diag.status = [status]
        self._pub.publish(diag)

    def _check_stale_modules(self):
        """
        Watchdog: Mark modules as STALE if no updates in STALE_THRESHOLD seconds.
        REP-107 compliance for offline/timeout detection.
        """
        now = time.monotonic()
        
        for proc_id, status in list(self._cache.items()):
            # Extract last update time from KeyValues
            last_update = None
            for kv in status.values:
                if kv.key == "last_update":
                    try:
                        last_update = float(kv.value)
                    except (ValueError, TypeError):
                        pass
                    break
            
            if last_update is None:
                continue  # No timestamp, skip
            
            # ✅ FIX Bug #6: Use correct constant name
            # Check if stale
            elapsed = now - last_update
            if elapsed > self.STALE_THRESHOLD:  # ← Was self.STALE_TIMEOUT (wrong!)
                # Mark as STALE if not already
                if status.level != DiagnosticStatus.STALE:
                    status.level = DiagnosticStatus.STALE
                    status.message = f"[STALE] No update in {elapsed:.1f}s"
                    
                    # Publish STALE status
                    self._publish(status)
                    self._cache[proc_id] = status
                    
                    self.node.get_logger().warn(
                        f"Module {proc_id} marked STALE (no update in {elapsed:.1f}s)"
                    )

    def get_all_statuses(self) -> Dict[str, DiagnosticStatus]:
        """Public accessor for health queries."""
        return self._cache.copy()
    
    def shutdown(self):
        """Unregister from EEE."""
        # Cancel STALE watchdog timer
        if hasattr(self, '_stale_timer'):
            self._stale_timer.cancel()

        EEEAggregator.unregister_plugin(self.NAME)
        self._cache.clear()
        self._last_publish.clear()


class AwarenessPlugin:
    """
    AWARENESS Channel Plugin: Human-readable live logs.
    
    Bridges EEE formatted plain text logs to /rosout for remote viewing.
    - Publishes all levels (DEBUG+) with custom format (ISO8601 + PROC_ID + CID + data)
    - Adds handler to root logger
    - Fallback on ROS failure to prevent error spam
    """
    NAME = "awareness"
    
    def __init__(self, ros_node: Node):
        self.node = ros_node
        self._pub = ros_node.create_publisher(Log, '/rosout', 100)
        
        # Create and register handler
        self._handler = self._create_handler()
        logging.getLogger().addHandler(self._handler)
        
        # Prevent double-registration on reload
        self._handler_added = True
    
    def _create_handler(self):
        """Create ROS-aware log handler."""
        node = self.node  # Capture for closure
        pub = self._pub
        
        class ROSLogHandler(logging.Handler):
            def __init__(self):
                super().__init__()
                self._fallback = False  # Flag if ROS fails
            
            def emit(self, record):
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
                if lvl >= logging.ERROR: return Log.ERROR
                if lvl >= logging.WARNING: return Log.WARN
                if lvl >= logging.INFO: return Log.INFO
                return Log.DEBUG
            
            def _format_message(self, record) -> str:
                """
                Format message with correlation ID if available.
                
                ✅ FIX Bug #7: More robust CID detection using pipe delimiter
                """
                msg = record.getMessage()
                
                # More robust check for EEE-formatted CID (with pipe delimiter)
                if "| CID:" in msg:  # ← Was just "CID:" (too generic)
                    return msg  # Already formatted by EEE
                
                # Fallback: no CID available (non-EEE logs)
                return msg
        
        return ROSLogHandler()
    
    def shutdown(self):
        """Remove handler from logging."""
        if self._handler_added:
            logging.getLogger().removeHandler(self._handler)
            self._handler_added = False


class MetricsPlugin:
    """
    METRICS Channel Plugin: Real-time numeric trend visualization.
    
    Publishes extracted numbers from evidence/context to /eee/metrics.
    - Enables plots in Foxglove/Grafana (e.g., CPU temp, load trends)
    - Optional extension for vital monitoring
    
    Extracts numeric values from evidence dict and publishes to /eee/metrics.
    Useful for real-time monitoring of:
    - Temperature sensors
    - CPU/GPU usage  
    - Battery voltage
    - Motor currents
    - Any numeric telemetry
    
    Example evidence dict:
        {
            "temperature": 45.2,
            "cpu_usage": 0.75,
            "battery_voltage": 12.6
        }
    
    Published as Float64MultiArray: [45.2, 0.75, 12.6]
    """
    NAME = "metrics"
    
    def __init__(self, ros_node: Node):
        self.node = ros_node
        
        # Publisher for numeric metrics
        self._pub = ros_node.create_publisher(
            Float64MultiArray,
            '/eee/metrics',
            10
        )
        
        # Statistics for monitoring
        self._published_count = 0
        self._empty_count = 0
        
        # Register with EEE to receive events
        EEEAggregator.register_plugin(self.NAME, self._handle)
        
        self.node.get_logger().info("✅ METRICS channel initialized")
    
    def _handle(self, level: int, msg: str, evidence: dict, meta: dict) -> bool:
        """
        EEE plugin callback - extract and publish numeric metrics.
        
        Args:
            level: Log level (INFO, WARNING, etc.)
            msg: Log message
            evidence: Dict that may contain numeric values
            meta: Metadata (correlation_id, proc_id, etc.)
        
        Returns:
            False: Always allow disk write (don't block)
        """
        if not evidence:
            self._empty_count += 1
            return False  # No data to extract
        
        # Extract only numeric values (int or float)
        # Skip 'traceback' as it's not a metric
        metrics = {
            key: value 
            for key, value in evidence.items() 
            if isinstance(value, (int, float)) and key != "traceback"
        }
        
        if not metrics:
            self._empty_count += 1
            return False  # No numeric data
        
        try:
            # Create ROS message
            # Note: We lose key names here - just publish values in dict order
            # For production, consider creating custom msg with key-value pairs
            metrics_msg = Float64MultiArray()
            metrics_msg.data = list(metrics.values())
            
            # Publish to /eee/metrics
            self._pub.publish(metrics_msg)
            self._published_count += 1
            
            # Optional: Log what we published (for debugging)
            if self._published_count <= 5:  # Only log first 5
                self.node.get_logger().info(
                    f"Published {len(metrics)} metrics from {meta['proc_id']}: {list(metrics.keys())}"
                )
            
        except Exception as e:
            # Don't crash on publish failure
            self.node.get_logger().warn(f"Metrics publish failed: {e}")
        
        return False  # Don't block disk writes
    
    def get_stats(self) -> dict:
        """Get publishing statistics (for monitoring)."""
        return {
            "published": self._published_count,
            "empty": self._empty_count,
            "total_events": self._published_count + self._empty_count
        }
    
    def shutdown(self):
        """Unregister from EEE."""
        stats = self.get_stats()
        self.node.get_logger().info(
            f"METRICS channel shutdown - Published: {stats['published']}, "
            f"Empty: {stats['empty']}, Total: {stats['total_events']}"
        )
        EEEAggregator.unregister_plugin(self.NAME)