"""
HRP — Homeostatic Regulation Parameters
========================================
AuRoRA · Homeostatic Regulation System (HRS)

Single source of truth for all cognitive architecture parameters
across the robot's systems.

Architecture:
    Three-tier constant hierarchy by ownership and mutability:

    [STATIC]    — Frozen in code. Hardware and architecture ceilings.
                  Admin only. Never changes at runtime.
                  Lives here in hrp.py permanently.

    [INTRINSIC] — GRACE's self-tuning cognitive parameters.
                  Adaptive — GRACE may update these over time via hrs.py.
                  Future home: root/{robot_id}/hrp.yaml

    [EXTRINSIC] — Per-user preferences shaping GRACE's behaviour.
                  Relational — one set per user, loaded at session start.
                  Future home: root/Users/{user_id}/hrp.yaml

Todo:
    HRS milestone — build hrs.py startup loader to replace [INTRINSIC]
                    and [EXTRINSIC] constants with runtime YAML loading.
                    hrp.py retains [STATIC] constants only.
"""

# ── CNS — Central Nervous System ──────────────────────────────────────────────
CNS_CORTICAL_CAPACITY = 2048    # [STATIC]    Total neural capacity of the active cognitive core
CNS_COGNITIVE_RESERVE = 300     # [INTRINSIC] Cortical capacity reserved for identity and cognition

# ── EMC — Episodic Memory Cortex ──────────────────────────────────────────────
EMC_RECALL_RESERVE   = 300      # [INTRINSIC] Cortical capacity reserved for episodic recall
EMC_RECALL_DEPTH     = 3        # [INTRINSIC] Maximum number of engrams surfaced per turn
EMC_RECALL_THRESHOLD = 0.25     # [INTRINSIC] Minimum synaptic similarity to surface an engram
EMC_ENGRAM_GATEWAY   = "..."    # [STATIC]    Engram store gateway — resolved at runtime by MCC

# ── WMC — Working Memory Cortex ───────────────────────────────────────────────
UNITS_PER_CHUNK        = 4      # [STATIC]    Number of neural units per chunk
PMT_OVERHEAD           = 4      # [STATIC]    Overhead chunks per phonological memory trace
WMC_GLOBAL_CHUNK_LIMIT = (      # [INTRINSIC] Maximum number of chunks WMC can hold
    ...
)                               # = 1448
WMC_PMT_SLOT_LIMIT     = 7     # [INTRINSIC] Maximum slot vacancy for PMTs (Miller's Law 7±2)
WMC_PMT_SLOT_BUFFER    = 2     # [INTRINSIC] PMT Slot vacancy flexibility (Miller's Law ±2)
