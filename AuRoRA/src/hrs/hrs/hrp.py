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

TODO:
    HRS milestone — build hrs.py startup loader to replace [INTRINSIC]
                    and [EXTRINSIC] constants with runtime YAML loading.
                    hrp.py retains [STATIC] constants only.
                  — add gateway initialization for CNC to access HRS parameters via AGi_ENTITY_GATEWAY.
                  — add HRS startup/shutdown lifecycle management
                  — add recency parameter for identification of the most recent event segments in EMC
                  — change constant to ConfigClass
"""

class AGi:                              # Amazing Grace infrastructure
    ENTITY_GATEWAY = "agi"              # [STATIC] Entry point for all the interactions with AGi's core systems

    class CNS:                          # Central Nervous System
        CORTICAL_CAPACITY = 2048        # [STATIC]    Total neural capacity of the active cognitive core
        COGNITIVE_RESERVE = 300         # [INTRINSIC] Cortical capacity reserved for identity and cognition
        NEURAL_GATEWAY   = "cns"        # [STATIC]    Neural gateway endpoint for inter-cortical communication
        ENGRAM_COMPLEX   = "engram.db"  # [STATIC]    Engram complex where long-term memories storage

        class EMC:                      # Episodic Memory Cortex
            ENCODING_ENGINE     = "google/embeddinggemma-300m-qat-q4_0-unquantized"  # [STATIC] Encoding engine for episodic memory
            ENGRAM_CHUNK_LIMIT  = 75    # [INTRINSIC] Maximum number of chunks surfaced per engram during recall
            ENCODING_DIM        = 768   # [STATIC]    Dimensionality of the encoding vectors from the encoding engine
                                        # TODO: for future use when implementing GPU-accelerated similarity search with FAISS, Annoy, etc.

            RECALL_RESERVE      = 300   # [INTRINSIC] Cortical capacity reserved for episodic recall
            RECALL_DEPTH        = 3     # [INTRINSIC] Maximum number of engrams surfaced per turn
            RELEVANCE_THRESHOLD = 0.25  # [INTRINSIC] Minimum relevance score for an engram to be surfaced

        class WMC:                      # Working Memory Cortex
            UNITS_PER_CHUNK    = 4      # [STATIC]    Number of neural units per chunk; Todo: move to AGi.CNS if other modules need this
            PMT_OVERHEAD       = 4      # [STATIC]    Overhead chunks for each PMT
            PMT_SLOT_LIMIT     = 7      # [INTRINSIC] Maximum slot vacancy for PMTs (Miller's Law 7±2)
            PMT_SLOT_BUFFER    = 2      # [INTRINSIC] PMT slot vacancy flexibility (Miller's Law ±2)

AGi.CNS.WMC.GLOBAL_CHUNK_LIMIT = (      # [INTRINSIC] Maximum number of chunks WMC can hold, will move to hrs.py
    AGi.CNS.CORTICAL_CAPACITY - 
    AGi.CNS.COGNITIVE_RESERVE - 
    AGi.CNS.EMC.RECALL_RESERVE
)
