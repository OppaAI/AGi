"""
EEE ROS Bridge - REFLEX and AWARENESS channels.
These are OPTIONAL bridging plugins that inject ROS capabilities into EEE.
"""

import logging
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import Log
from rclpy.node import Node
from std_srvs.srv import Trigger

class EEEROSBridge(Node):
    def __init__(self):
        super().__init__('eee_ros_bridge')
        
        # 1. Start the Channels
        self.reflex = ReflexPlugin(self)
        self.awareness = AwarenessPlugin(self)
        
        # 2. Add the Health Service (The Ledger Query)
        self.health_srv = self.create_service(Trigger, '/eee/health', self.handle_health_query)
        
        self.get_logger().info("🚀 TALLE 2.0 ROS Bridge is LIVE")

    def handle_health_query(self, request, response):
        # We query the Reflex cache for an instant answer
        statuses = self.reflex._cache
        # If any status is ERROR (2), system is Degraded
        is_bad = any(s.level == 2 for s in statuses.values())
        response.success = not is_bad
        response.message = f"System monitoring {len(statuses)} modules."
        return response

class ReflexPlugin:
    """
    REFLEX Channel: Real-time safety alerts (/diagnostics).
    """
    NAME = "reflex"
    
    def __init__(self, ros_node):
        self.node = ros_node
        self.robot_id = ros_node.get_name()
        
        self.pub = ros_node.create_publisher(
            DiagnosticArray, '/diagnostics', 10
        )
        self._cache = {}
        
        # Register with EEE
        from scs.eee import EEEAggregator
        EEEAggregator.register_plugin(self.NAME, self.handle)
    
    def handle(self, level, msg, evidence, meta) -> bool:
        """Plugin callback for EEE."""
        if level < logging.WARNING:
            return False
        
        # Build and publish diagnostic
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.ERROR if level >= logging.ERROR else DiagnosticStatus.WARN
        status.name = meta["proc_id"]
        status.message = msg[:255]
        status.hardware_id = self.robot_id
        
        # Add evidence as KeyValue
        if evidence:
            for k, v in evidence.items():
                if k != "traceback":
                    kv = KeyValue(key=str(k)[:255], value=str(v)[:255])
                    status.values.append(kv)
        
        # Add correlation ID
        status.values.append(KeyValue(key="cid", value=meta["correlation_id"][:8]))
        
        # Publish
        diag = DiagnosticArray()
        diag.header.stamp = self.node.get_clock().now().to_msg()
        diag.status = [status]
        self.pub.publish(diag)
        
        self._cache[status.name] = status
        return level < logging.CRITICAL  # Block disk for non-critical
    
    def shutdown(self):
        from scs.eee import EEEAggregator
        EEEAggregator.unregister_plugin(self.NAME)


class AwarenessPlugin:
    """
    AWARENESS Channel: Human-readable logs (/rosout).
    """
    NAME = "awareness"
    
    def __init__(self, ros_node):
        self.node = ros_node
        
        self.pub = ros_node.create_publisher(Log, '/rosout', 100)
        
        # Hook into Python logging
        self._handler = self._create_handler()
        logging.getLogger().addHandler(self._handler)
    
    def _create_handler(self):
        class ROSLogHandler(logging.Handler):
            def __init__(self, node, pub):
                super().__init__()
                self.node = node
                self.pub = pub
            
            def emit(self, record):
                ros_log = Log()
                ros_log.stamp = self.node.get_clock().now().to_msg()
                ros_log.level = self._map_level(record.levelno)
                ros_log.name = record.name.replace('.', '::').upper()
                ros_log.msg = record.getMessage()
                self.pub.publish(ros_log)
            
            def _map_level(self, lvl):
                if lvl >= logging.ERROR: return Log.ERROR
                if lvl >= logging.WARNING: return Log.WARN
                if lvl >= logging.INFO: return Log.INFO
                return Log.DEBUG
        
        return ROSLogHandler(self.node, self.pub)
    
    def shutdown(self):
        logging.getLogger().removeHandler(self._handler)
