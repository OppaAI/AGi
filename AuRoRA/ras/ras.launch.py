"""
RAS — Reticular Activation System
========================================
AuRoRA · Reticular Activation System (RAS)

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
    ros2 launch ras/ras.launch.py

Path convention:
    ~/.agi/ras/aurora.yaml
"""

from pathlib import Path

from launch                            import LaunchDescription
from launch.actions                    import LogInfo
from launch_ros.actions                import Node
from launch_ros.parameter_descriptions import ParameterFile

# Future imports — uncomment when staged boot activates
# from launch.actions        import RegisterEventHandler, TimerAction
# from launch.event_handlers import OnProcessStart

from hrs.hrm import RRR, AGi

# ─── Config Paths ─────────────────────────────────────────────────────────────
#
#   Constructed from HRM constants — single source of truth, no magic strings.

_AURORA_PATH = (
    Path.home()
    / AGi.ENTITY_GATEWAY
    / RRR.RETICULAR_ACTIVATING_COMPARTMENT
    / AGi.SCS.AURORA_SETPOINTS                      # "aurora.yaml"
)

# ─── Parameter Files ──────────────────────────────────────────────────────────
#
#   aurora.yaml  — robot-wide intrinsic params (scs.emc.*, scs.wmc.*, etc.)
#
#   Params are distributed to all nodes before any node starts.
#   allow_substs=True — enables $(env HOME) and similar substitutions in YAML.

_aurora_params = ParameterFile(
    param_file=str(_AURORA_PATH),
    allow_substs=True,
)

# ─── Nodes ────────────────────────────────────────────────────────────────────

# Stage 3 — Cognition
_cnc = Node(
    package    = RRR.SEMANTIC_COGNITIVE_SYSTEM,     # "scs"
    executable = RRR.CENTRAL_NEURAL_CORE,           # "cnc"
    name       = RRR.CENTRAL_NEURAL_CORE,           # "cnc"
    namespace  = RRR.SEMANTIC_COGNITIVE_SYSTEM,     # "/scs"
    parameters = [_aurora_params],                  # aurora params
    output     = "screen",
)

# Stage 1 — Infrastructure (stub)
# _eec = Node(
#     package    = RRR.HOMEOSTATIC_REGULATION_SYSTEM,
#     executable = RRR.EMERGENCY_EXCEPTION_CORE,
#     name       = RRR.EMERGENCY_EXCEPTION_CORE,
#     namespace  = RRR.HOMEOSTATIC_REGULATION_SYSTEM,
#     parameters = [_aurora_params],
#     output     = "screen",
# )

# Stage 2 — Regulatory (stub)
# _hrs = Node(
#     package    = RRR.HOMEOSTATIC_REGULATION_SYSTEM,
#     executable = RRR.HOMEOSTATIC_REGULATION_SYSTEM,
#     name       = RRR.HOMEOSTATIC_REGULATION_SYSTEM,
#     namespace  = RRR.HOMEOSTATIC_REGULATION_SYSTEM,
#     parameters = [_aurora_params],
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
        LogInfo(msg="⚡ RAS — Reticular Activation System igniting…"),
        LogInfo(msg=f"   Aurora  : {_AURORA_PATH}"),
        LogInfo(msg="=" * 60),

        # ── Stage 3 — Cognition ───────────────────────────────────────────────
        # Only CNC active — expand to staged sequence when EEC/HRS are ready
        _cnc,

        # ── Future staged boot (stub) ─────────────────────────────────────────
        # _eec,
        # _start_hrs,
        # _start_cnc,

        LogInfo(msg="✅ RAS — nodes launched"),
    ])
