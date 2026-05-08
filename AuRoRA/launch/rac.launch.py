"""
RAC — Reticular Activation Compartment
========================================
AuRoRA · Semantic Cognitive System (SCS)

ROS2 launch file — replaces the plain-Python RAC bootloader.
Replaces: scs/rac.py

Responsibilities:
    1. Construct the AGi config path from HRM constants
    2. Load aurora.yaml as ROS parameters — distributed to all nodes before any start
    3. Spawn downstream ROS2 nodes in dependency order
    4. ROS2 guarantees parameters are set before nodes initialise — no race condition

Boot order (current → future):
    Stage 1 — Infrastructure : EEC  (stub — uncomment when ready)
    Stage 2 — Regulatory     : HRS  (stub — uncomment when ready)
    Stage 3 — Cognition      : CNC

Usage:
    ros2 launch scs rac.launch.py

Path convention (mirrors original RAC):
    ~/.agi/scs/aurora.yaml
"""

from pathlib import Path

from launch                            import LaunchDescription
from launch.actions                    import RegisterEventHandler, LogInfo, TimerAction
from launch.event_handlers             import OnProcessStart
from launch_ros.actions                import Node
from launch_ros.parameter_descriptions import ParameterFile

import hrs.hrm as hrm

# ─── Config Path ──────────────────────────────────────────────────────────────
#
#   Constructed from HRM constants — single source of truth, no magic strings.
#   Mirrors the path convention from the original RAC bootloader.

_YAML_PATH = (
    Path.home()
    / hrm.AGi.ENTITY_GATEWAY
    / hrm.SEMANTIC_COGNITIVE_SYSTEM
    / hrm.AGi.SCS.AURORA_SETPOINTS
)

# ─── Parameter File ───────────────────────────────────────────────────────────
#
#   allow_substs=True — enables $(env HOME) and similar substitutions in YAML.
#   Parameters are distributed to all nodes before any node starts.

_params = ParameterFile(
    param_file=str(_YAML_PATH),
    allow_substs=True,
)

# ─── Nodes ────────────────────────────────────────────────────────────────────

# Stage 3 — Cognition
_cnc = Node(
    package    = hrm.SEMANTIC_COGNITIVE_SYSTEM,     # "scs"
    executable = hrm.CENTRAL_NERVOUS_CORE,          # "cnc"
    name       = hrm.CENTRAL_NERVOUS_CORE,          # "cnc"
    namespace  = hrm.SEMANTIC_COGNITIVE_SYSTEM,     # "/scs"
    parameters = [_params],
    output     = "screen",
)

# Stage 1 — Infrastructure (stub)
# _eec = Node(
#     package    = hrm.HOMEOSTATIC_REGULATION_SYSTEM,
#     executable = hrm.EMERGENCY_EXCEPTION_CORE,
#     name       = hrm.EMERGENCY_EXCEPTION_CORE,
#     namespace  = hrm.HOMEOSTATIC_REGULATION_SYSTEM,
#     parameters = [_params],
#     output     = "screen",
# )

# Stage 2 — Regulatory (stub)
# _hrs = Node(
#     package    = hrm.HOMEOSTATIC_REGULATION_SYSTEM,
#     executable = hrm.HOMEOSTATIC_REGULATION_SYSTEM,
#     name       = hrm.HOMEOSTATIC_REGULATION_SYSTEM,
#     namespace  = hrm.HOMEOSTATIC_REGULATION_SYSTEM,
#     parameters = [_params],
#     output     = "screen",
# )

# ─── Boot Sequence ────────────────────────────────────────────────────────────
#
#   Current: CNC only — no dependencies.
#
#   When EEC and HRS are ready, replace the simple CNC action below with the
#   staged sequence using RegisterEventHandler(OnProcessStart(...)) to block
#   each stage until the previous node has started.
#
#   Staged pattern (uncomment and expand per milestone):
#
#   _start_hrs = RegisterEventHandler(
#       OnProcessStart(
#           target_action = _eec,
#           on_start      = [
#               LogInfo(msg="✅ EEC started — spawning HRS…"),
#               _hrs,
#           ],
#       )
#   )
#
#   _start_cnc = RegisterEventHandler(
#       OnProcessStart(
#           target_action = _hrs,
#           on_start      = [
#               LogInfo(msg="✅ HRS started — spawning CNC…"),
#               _cnc,
#           ],
#       )
#   )

def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([

        LogInfo(msg="=" * 60),
        LogInfo(msg="⚡ RAC — Reticular Activation Compartment igniting…"),
        LogInfo(msg=f"   Config: {_YAML_PATH}"),
        LogInfo(msg="=" * 60),

        # ── Stage 3 — Cognition ───────────────────────────────────────────────
        # Only CNC active — expand to staged sequence when EEC/HRS are ready
        _cnc,

        # ── Future staged boot (stub) ─────────────────────────────────────────
        # _eec,
        # _start_hrs,
        # _start_cnc,

        LogInfo(msg="✅ RAC — nodes launched"),
    ])
