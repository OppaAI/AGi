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

class AGi:                                       # Amazing Grace infrastructure
    ENTITY_GATEWAY = "agi"                       # [STATIC] Entry point for all the interactions with AGi's core systems

    class CNS:                                   # Central Nervous System
        CORTICAL_CAPACITY: int = 2048            # [STATIC]    Total neural capacity of the active cognitive core
        COGNITIVE_RESERVE: int = 300             # [INTRINSIC] Cortical capacity reserved for identity and cognition
        NEURAL_GATEWAY: str    = "cns"           # [STATIC]    Neural gateway endpoint for inter-cortical communication
        ENGRAM_COMPLEX: str    = "engram.db"     # [STATIC]    Engram complex where long-term memories storage

        class EMC:                               # Episodic Memory Cortex
            ENCODING_ENGINE: str        = "google/embeddinggemma-300m-qat-q4_0-unquantized"  # [STATIC] Encoding engine for episodic memory
            ENCODING_CACHE_LIMIT: int   = 256    # [INTRINSIC] Maximum number of imprints held in encoding cache to control memory usage
            ENCODING_IMPRINT_LIMIT: int = 300    # [INTRINSIC] Maximum length of imprints to control cache hit rate vs false positive risk
            ENCODING_DIM: int           = 768    # [STATIC]    Dimensionality of the encoding vectors from the encoding engine
                                                 # TODO: for future use when implementing GPU-accelerated similarity search with FAISS, Annoy, etc.
            ENGRAM_CHUNK_LIMIT: int     = 75     # [INTRINSIC] Maximum number of chunks surfaced per engram during recall
            RECALL_RESERVE: int         = 300    # [INTRINSIC] Cortical capacity reserved for episodic recall
            RECALL_DEPTH: int           = 3      # [INTRINSIC] Maximum number of engrams surfaced per turn
            RECALL_TIMEOUT: float       = 5.0    # [INTRINSIC] Timeout for recall operations (300M param embedding model on Orin Nano CPU)
                                                 # covers encode_query (~500-900ms) + KNN search
                                                 # TODO: drop to 3.0 if model is genuinely int4 quantized
            RELEVANCE_THRESHOLD: float  = 0.25   # [INTRINSIC] Minimum relevance score for an engram to be surfaced

        class WMC:                               # Working Memory Cortex
            UNITS_PER_CHUNK: int    = 4          # [STATIC]    Number of neural units per chunk; Todo: move to AGi.CNS if other modules need this
            PMT_OVERHEAD: int       = 4          # [STATIC]    Overhead chunks for each PMT
            PMT_SLOT_LIMIT: int     = 7          # [INTRINSIC] Maximum slot vacancy for PMTs (Miller's Law 7±2)
            PMT_SLOT_BUFFER: int    = 2          # [INTRINSIC] PMT slot vacancy flexibility (Miller's Law ±2)

AGi.CNS.WMC.GLOBAL_CHUNK_LIMIT: int = (          # [INTRINSIC] Maximum number of chunks WMC can hold, will move to hrs.py
    AGi.CNS.CORTICAL_CAPACITY - 
    AGi.CNS.COGNITIVE_RESERVE - 
    AGi.CNS.EMC.RECALL_RESERVE
)
