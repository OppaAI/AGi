"""
HRP — Homeostatic Regulation Parameters
========================================
AuRoRA · Homeostatic Regulation System (HRS)

Single source of truth for all STATIC cognitive architecture parameters.
All other tiers are loaded at runtime by RAS from YAML config files.

Constant tier hierarchy:

    [STATIC]    — Frozen in code. Architecture ceilings and protocol constants.
                  Admin only. Never changes at runtime. Lives here permanently.

    [INTRINSIC] — GRACE's self-tuning cognitive parameters.
                  Loaded from: ~/.agi/cns/state.yaml
                  Adaptive — GRACE may update these over time via HRS.

    [PERSONA]   — GCE model identity, inference params, and system prompt.
                  Loaded from: ~/.agi/cns/persona.yaml
                  Set by admin. One file per persona/model config.

    [EXTRINSIC] — Deployment and environment configuration.
                  Loaded from: ~/.agi/cns/config.yaml
                  Set by admin per deployment (server, backend, hardware).

    [USER]      — End-user preferences and identity.
                  Loaded from: ~/.agi/cns/profile.yaml
                  Set by user at session start.

    [DERIVED]   — Computed from other constants at class definition time.
                  Never set manually.

TODO:
    — add gateway initialization for CNC to access HRS parameters via AGi_ENTITY_GATEWAY
    — add HRS startup/shutdown lifecycle management
    — add recency parameter for identification of the most recent event segments in EMC
    — UNITS_PER_CHUNK to be obsolete after tokenizer implemented in M1.5
"""

class AGi:                                              # Amazing Grace infrastructure
    ENTITY_GATEWAY: str = ".agi"                        # [STATIC] root directory for all AGi core system data

    class SCS:                                          # Semantic Cognitive System
        NEURAL_GATEWAY: str     = "scs"                 # [STATIC] SCS node identifier for inter-cortical routing
        MEMORY_GATEWAY: str     = "mcc"                 # [STATIC] MCC node identifier for memory subsystem routing
        ENGRAM_COMPLEX: str     = "engram_complex.db"   # [STATIC] filename of the long-term engram storage database
        UNITS_PER_CHUNK: int    = 4                     # [STATIC] neural units per context chunk (obsolete after M1.5 tokenizer)

        TEXT_INPUT_GATEWAY: str     = "/scs/text_input"     # [STATIC] ROS topic — text input from user to CNC
        COGNITIVE_RESPONSE: str     = "/scs/response"       # [STATIC] ROS topic — GCE response streamed to output
        MEMORY_CONTEXT_GATEWAY: str = "/scs/memory_context" # [STATIC] ROS topic — episodic memory context injected into CNC
        MEMORY_STATS_GATEWAY: str   = "/scs/memory_stats"   # [STATIC] ROS topic — memory stats published by all memory cortices

        class GCE:                                      # Generative Cognitive Engine
            NEURAL_ENDPOINT  : str        = "http://AIVA:11434"                          # [STATIC] inference server address
            COGNITIVE_ENGINE : str        = "huihui_ai/Qwen3.6-abliterated:35b-Claude-4.7"  # [STATIC] active model — change when best model is found
            TIMEOUT           : float      = 60.0       # [STATIC] seconds before abandoning inference
            KEEP_ALIVE        : int | None = -1         # [STATIC] VRAM retention — -1 = forever, 0 = unload, n = seconds (Ollama only)
            STREAM_LEADING    : str        = "start"    # [STATIC] streaming onset marker — first cognitive fragment arriving
            STREAM_PROPAGATING: str        = "delta"    # [STATIC] streaming propagation marker — mid-stream cognitive fragments
            STREAM_TRAILING   : str        = "done"     # [STATIC] streaming completion marker — full response assembled
            STREAM_ANOMALY    : str        = "error"    # [STATIC] streaming inhibition marker — cognitive error or suppressed response

        class SMC:                                      # Semantic Memory Cortex
            ENCODING_ENGINE: str = "BAAI/bge-base-en-v1.5" # [STATIC] sentence embedding model for semantic memory
            ENCODING_DIM: int    = 768                  # [STATIC] embedding vector dimensionality — fixed by model architecture

        class EMC:                                      # Episodic Memory Cortex
            ENCODING_ENGINE: str        = "BAAI/bge-base-en-v1.5"  # [STATIC] sentence embedding model for episodic memory
            ENCODING_CUE_PREFIX: str    = "Represent this sentence for searching relevant passages  : "  # [STATIC] BGE query prefix for recall cues
            ENCODING_ENGRAM_PREFIX: str = ""            # [STATIC] BGE prefix for engram storage (intentionally empty)
            ENCODING_DIM: int           = 768           # [STATIC] embedding vector dimensionality — fixed by model architecture
                                                        # TODO: future GPU-accelerated similarity search with FAISS or Annoy

        class WMC:                                      # Working Memory Cortex
            PMT_OVERHEAD: int = 4                       # [STATIC] fixed chunk overhead per PMT for formatting and metadata


# ─── Derived Constants ────────────────────────────────────────────────────────
# Computed at import time from STATIC + INTRINSIC seeds hardcoded below.
# These will be recalculated by RAS after loading state.yaml in HRS milestone.

_CORTICAL_CAPACITY: int = 16384                         # seed — replaced by state.yaml at runtime
_COGNITIVE_RESERVE: int = 2048                          # seed — replaced by state.yaml at runtime
_RECALL_RESERVE: int    = 2048                          # seed — replaced by state.yaml at runtime

AGi.SCS.WMC.GLOBAL_CHUNK_LIMIT: int = (                 # [DERIVED] maximum total chunks WMC can hold
    _CORTICAL_CAPACITY -
    _COGNITIVE_RESERVE -
    _RECALL_RESERVE
)
