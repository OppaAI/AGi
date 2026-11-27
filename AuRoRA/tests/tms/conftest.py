"""Test current AuRoRA adapters with real message blueprints and minimal ROS doubles."""
import sys
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for package in ('tms', 'hrs', 'gms'):
    sys.path.insert(0, str(ROOT / 'src' / package))

try:
    import rclpy  # noqa: F401
except ImportError:
    module = types.ModuleType('rclpy')
    node = types.ModuleType('rclpy.node')
    node.Node = type('Node', (), {})
    module.node = node
    sys.modules.update({'rclpy': module, 'rclpy.node': node})

try:
    from std_msgs.msg import String  # noqa: F401
except ImportError:
    module = types.ModuleType('std_msgs')
    message = types.ModuleType('std_msgs.msg')
    message.String = type('String', (), {'data': ''})
    module.msg = message
    sys.modules.update({'std_msgs': module, 'std_msgs.msg': message})

try:
    from cryptic_signal_core.msg import TelepathicSignal  # noqa: F401
except ImportError:
    message = types.ModuleType('cryptic_signal_core.msg')
    message.TelepathicSignal = type('TelepathicSignal', (), {})
    sys.modules['cryptic_signal_core.msg'] = message
