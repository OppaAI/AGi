"""
EEE ROS Bridge - REFLEX and AWARENESS channels.
These are OPTIONAL bridging plugins that inject ROS capabilities into EEE.
"""

import logging
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import Log


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
