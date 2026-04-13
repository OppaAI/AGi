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
    ENTITY_GATEWAY = ".agi"                       # [STATIC] Entry point for all the interactions with AGi's core systems

    class CNS:                                   # Central Nervous System
        CORTICAL_CAPACITY: int = 8192            # [STATIC]    Total neural capacity of the active cognitive core
        COGNITIVE_RESERVE: int = 512             # [INTRINSIC] Cortical capacity reserved for identity and cognition
        NEURAL_GATEWAY: str    = "cns"           # [STATIC]    Neural gateway endpoint for inter-cortical communication
        ENGRAM_COMPLEX: str    = "engram.db"     # [STATIC]    Engram complex where long-term memories storage
        UNITS_PER_CHUNK: int    = 4              # [STATIC]    Number of neural units per chunk; Todo: move to AGi.CNS if other modules need this

        class EMC:                               # Episodic Memory Cortex
            ENCODING_ENGINE: str        = "BAAI/bge-small-en-v1.5"  # [STATIC] Encoding engine for episodic memory
            ENCODING_CACHE_LIMIT: int   = 256    # [INTRINSIC] Maximum number of imprints held in encoding cache to control memory usage
            ENCODING_IMPRINT_LIMIT: int = 300    # [INTRINSIC] Maximum length of imprints to control cache hit rate vs false positive risk
            ENCODING_DIM: int           = 384    # [STATIC]    Dimensionality of the encoding vectors from the encoding engine
                                                 # TODO: for future use when implementing GPU-accelerated similarity search with FAISS, Annoy, etc.
            ENGRAM_CHUNK_LIMIT: int     = 300    # [INTRINSIC] Maximum number of chunks surfaced per engram during recall
            ENGRAM_CONTENT_LIMIT: int   = 6000   # [INTRINSIC] Maximum character length of a PMT bound into episodic buffer
            RECALL_RESERVE: int         = 1024   # [INTRINSIC] Cortical capacity reserved for episodic recall
            RECALL_DEPTH: int           = 3      # [INTRINSIC] Maximum number of engrams surfaced per turn
            RECALL_POOL: int            = 50     # [INTRINSIC] Candidate pool multiplier for cosine recall (RECALL_DEPTH × RECALL_POOL engrams scored)
            RECALL_TIMEOUT: float       = 2.0    # [INTRINSIC] Timeout for recall operations (300M param embedding model on Orin Nano CPU)
                                                 # covers encode_query (~500-900ms) + KNN search
                                                 # TODO: drop to 3.0 if model is genuinely int4 quantized
            RECOVERY_BATCH_SIZE: int    = 50     # [INTRINSIC] Max unencoded episodes loaded into binding stream per recovery batch
            RELEVANCE_THRESHOLD: float  = 0.25   # [INTRINSIC] Minimum relevance score for an engram to be surfaced

        class WMC:                               # Working Memory Cortex
            PMT_OVERHEAD: int       = 4          # [STATIC]    Overhead chunks per PMT for formatting and metadata
            PMT_SLOT_LIMIT: int     = 7          # [INTRINSIC] Maximum slot vacancy for PMTs (Miller's Law 7±2)
            PMT_SLOT_BUFFER: int    = 2          # [INTRINSIC] PMT slot vacancy flexibility (Miller's Law ±2)

AGi.CNS.WMC.GLOBAL_CHUNK_LIMIT: int = (          # [INTRINSIC] Maximum number of chunks WMC can hold, will move to hrs.py
    AGi.CNS.CORTICAL_CAPACITY - 
    AGi.CNS.COGNITIVE_RESERVE - 
    AGi.CNS.EMC.RECALL_RESERVE
)
