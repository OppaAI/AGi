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

# ROS2 components — launch-action system
from launch                            import LaunchDescription       # root launch descriptor — returned by generate_launch_description
from launch.actions                    import LogInfo                 # console log action — emits messages during launch sequence
from launch_ros.actions                import Node                    # ROS2 node action — declares a node to spawn
from launch_ros.parameter_descriptions import ParameterFile           # parameter file loader — binds YAML params to nodes before start

# Future imports — uncomment when staged boot activates
# from launch.actions        import RegisterEventHandler, TimerAction
# from launch.event_handlers import OnProcessStart

# AGi components
from hrs.hrm import RRR                 # path segment constants — package, executable, namespace names
from hrs.hru import GatewayMap          # gateway paths — resolves aurora.yaml location

# ─── Parameter Files ──────────────────────────────────────────────────────────
#
#   aurora.yaml  — robot-wide intrinsic params (scs.emc.*, scs.wmc.*, etc.)
#
#   Distributed to all nodes before any node starts.
#   allow_substs=True — enables $(env HOME) and similar substitutions in YAML.

_aurora_params = ParameterFile(                             # parameter file loader — binds YAML params to all nodes before start
    param_file   = str(GatewayMap().aurora_setpoints),      # resolved path — ~/.agi/ras/aurora.yaml
    allow_substs = True,                                    # allow env substitutions in YAML
)

# ─── Nodes ────────────────────────────────────────────────────────────────────

# Stage 3 — Cognition
_cnc = Node(
    package    = RRR.SEMANTIC_COGNITIVE_SYSTEM,             # package: "scs"
    executable = RRR.CENTRAL_NEURAL_CORE,                   # executable: "cnc"
    name       = RRR.CENTRAL_NEURAL_CORE,                   # name: "cnc"
    namespace  = RRR.SEMANTIC_COGNITIVE_SYSTEM,             # namespace: "/scs"
    parameters = [_aurora_params],                          # inject aurora params before node starts
    output     = "screen",                                  # stream logs to console and /rosout
)

# TIC decodes the COC's typed RadioSignal and publishes normalized SSS to CNC.
_tic = Node(
    package    = "tms",
    executable = "tic",
    name       = "tic",
    namespace  = "tms",
    parameters = [_aurora_params],
    output     = "screen",
)

# PC integration API. Mutating endpoints require AGI_API_TOKEN.
# The COC also binds to loopback; expose it with Tailscale Serve and tailnet ACLs.
_api = Node(
    package    = "tms",
    executable = "api",
    name       = "agi_http_api",
    namespace  = "tms",
    parameters = [_aurora_params],
    output     = "screen",
)
_coc = Node(
    package    = "tms",
    executable = "coc",
    name       = "agi_coc",
    namespace  = "tms",
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
#   Current: TIC, CNC, the authenticated HTTP API, and the loopback COC server.
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
    return LaunchDescription([                                              # root launch descriptor — returned by generate_launch_description

        LogInfo(msg="=" * 60),                                              # visual separator — stdout and /rosout
        LogInfo(msg="⚡ RAS — Reticular Activation System igniting…"),       # log the startup of RAS
        LogInfo(msg=f"   Aurora  : {GatewayMap().aurora_setpoints}"),       # include resolved aurora param file path
        LogInfo(msg="=" * 60),                                              # visual separator — stdout and /rosout

        # ── Stage 3 — Cognition ───────────────────────────────────────────────
        # CNC, APIs, and COC active; expand staged cognition sequence as systems mature
        _tic,                                                               # typed COC ingress → SSS transduction
        _cnc,                                                               # launch CNC node
        _api,                                                               # authenticated PC HTTP ingress → typed ROS impulses
        _coc,                                                               # loopback WebUI + same-origin rosbridge websocket proxy

        # ── Future staged boot (stub) ─────────────────────────────────────────
        # _eec,
        # _start_hrs,
        # _start_cnc,

        LogInfo(msg="✅ RAS — nodes launched"),                             # log the successful launch of all nodes
    ])
