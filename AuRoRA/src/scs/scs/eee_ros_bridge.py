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
        Service: Instant health check via REFLEX cache.
        
        Returns:
            success: True if no ERROR-level statuses
            message: Summary of monitored modules
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
    REFLEX Channel: Real-time safety alerts (/diagnostics).
    """
    NAME = "reflex"
    
    def __init__(self, ros_node: Node):
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
        EEE plugin callback.
        
        Returns:
            True: Event handled (block disk for non-critical)
            False: Allow fall-through to disk
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
        
        return status
    
    def _publish(self, status: DiagnosticStatus):
        """Publish to /diagnostics."""
        diag = DiagnosticArray()
        diag.header.stamp = self.node.get_clock().now().to_msg()
        diag.header.frame_id = self.robot_id
        diag.status = [status]
        self._pub.publish(diag)
    
    def get_all_statuses(self) -> Dict[str, DiagnosticStatus]:
        """Public accessor for health queries."""
        return self._cache.copy()
    
    def shutdown(self):
        """Unregister from EEE."""
        EEEAggregator.unregister_plugin(self.NAME)
        self._cache.clear()


class AwarenessPlugin:
    """
    AWARENESS Channel: Human-readable logs (/rosout).
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
                """Format message with correlation ID if available."""
                msg = record.getMessage()
                # Extract CID if present in EEE format
                if "CID:" in msg:
                    return msg  # Already formatted by EEE
                return msg
        
        return ROSLogHandler()
    
    def shutdown(self):
        """Remove handler from logging."""
        if self._handler_added:
            logging.getLogger().removeHandler(self._handler)
            self._handler_added = False
