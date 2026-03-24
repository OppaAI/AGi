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
                  — add gateway initialization for CNC to access HRS parameters via AGi_ENTITY_GATEWAY.
                  — add HRS startup/shutdown lifecycle management
                  — add recency parameter for identification of the most recent event segments in EMC
"""

# ── AGi — Amazing Grace infrastructure ────────────────────────────────────────
AGi_ENTITY_GATEWAY = "agi"      # [STATIC] Entry point for all the interactions with AGi's core systems

# ── CNS — Central Nervous System ──────────────────────────────────────────────
CNS_CORTICAL_CAPACITY = 2048    # [STATIC]    Total neural capacity of the active cognitive core
CNS_COGNITIVE_RESERVE = 300     # [INTRINSIC] Cortical capacity reserved for identity and cognition
CNC_NEURAL_GATEWAY   = "cns"    # [STATIC]    Neural gateway endpoint for inter-cortical communication

# ── EMC — Episodic Memory Cortex ──────────────────────────────────────────────
EMC_RECALL_RESERVE   = 300      # [INTRINSIC] Cortical capacity reserved for episodic recall
EMC_RECALL_DEPTH     = 3        # [INTRINSIC] Maximum number of engrams surfaced per turn
EMC_RECALL_THRESHOLD = 0.25     # [INTRINSIC] Minimum synaptic similarity to surface an engram
EMC_ENGRAM_COMPLEX   = "emc.db" # [STATIC]    Engram complex where episodic memories are stored

# ── WMC — Working Memory Cortex ───────────────────────────────────────────────
UNITS_PER_CHUNK        = 4      # [STATIC]    Number of neural units per chunk
PMT_OVERHEAD           = 4      # [STATIC]    Overhead chunks for each PMT
WMC_GLOBAL_CHUNK_LIMIT = (      # [INTRINSIC] Maximum number of chunks WMC can hold
    CNS_CORTICAL_CAPACITY - 
    CNS_COGNITIVE_RESERVE - 
    EMC_RECALL_RESERVE
)
WMC_PMT_SLOT_LIMIT     = 7     # [INTRINSIC] Maximum slot vacancy for PMTs (Miller's Law 7±2)
WMC_PMT_SLOT_BUFFER    = 2     # [INTRINSIC] PMT slot vacancy flexibility (Miller's Law ±2)
