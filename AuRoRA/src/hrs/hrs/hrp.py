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
                  Runtime home: ~/.agi/cns/state.yaml

    [EXTRINSIC] — Per-deployment preferences shaping GRACE's behaviour.
                  Runtime home: ~/.agi/cns/config.yaml

    [PERSONA]   — Inference and personality parameters for the active persona.
                  Runtime home: ~/.agi/personas/{name}.yaml

Hydration:
    ARC walks this class tree at boot. Any class with a _manifest_gateway attribute
    is hydrated from the corresponding YAML section. UPPER_CASE constants are
    matched to their lowercase yaml keys automatically.

    To add a constant:   add it here with a default + add to the yaml. Done.
    To add a subsystem:  add a class with _manifest_gateway here. Done.

    [STATIC] constants have no yaml key — they are never overwritten by ARC.
    [DERIVED] constants are recomputed in arc._derive() after hydration.

TODO:
    HRS milestone — build hrs.py to allow GRACE to update [INTRINSIC] constants
                    at runtime and persist changes back to state.yaml.
                  — add gateway initialization for CNC to access HRS parameters via AGi_ENTITY_GATEWAY.
                  — add HRS startup/shutdown lifecycle management
                  — add recency parameter for identification of the most recent event segments in EMC
"""

class AGi:                                              # Amazing Grace infrastructure
    ENTITY_GATEWAY = ".agi"                             # [STATIC] root directory for all AGi core system state

    # Module registry — single source of truth for all subsystem identifiers
    SEMANTIC_COGNITIVE_SYSTEM   : str = "scs"           # Semantic Cognitive System
    MEMORY_COORDINATION_CORE    : str = "mcc"
    WORKING_MEMORY_CORTEX       : str = "wmc"
    EPISODIC_MEMORY_CORTEX      : str = "emc"
    SEMANTIC_MEMORY_CORTEX      : str = "smc"
    
    class SCS:                                                  # Semantic Cognitive System
        _manifest_gateway       = SEMANTIC_COGNITIVE_SYSTEM     # ARC hydration target
        CORTICAL_CAPACITY: int  = 16384                         # [INTRINSIC] total token budget for the active LLM context window
        COGNITIVE_RESERVE: int  = 2048                          # [INTRINSIC] tokens reserved for system prompt and identity injection
        NEURAL_GATEWAY: str     = SEMANTIC_COGNITIVE_SYSTEM     # [STATIC] ROS node name for the SCS
        MEMORY_GATEWAY: str     = MEMORY_COORDINATION_CORE      # [STATIC] ROS node name for the MCC
        ENGRAM_COMPLEX: str     = "engram_complex.db"           # [STATIC] SQLite filename for long-term memory storage
        UNITS_PER_CHUNK: int    = 4                             # [STATIC] tokens per chunk unit (TODO: obsolete post-tokenizer M1.5)

        TEXT_INPUT_GATEWAY: str     = "/scs/text_input"       # [STATIC] ROS topic — inbound user text
        RESPONSE_GATEWAY: str       = "/scs/response"         # [STATIC] ROS topic — outbound LLM response
        MEMORY_CONTEXT_GATEWAY: str = "/scs/memory_context"   # [STATIC] ROS topic — reinstated memory context injected into prompt
        MEMORY_STATS_GATEWAY: str   = "/scs/memory_stats"     # [STATIC] ROS topic — memory diagnostics from all memory subsystems

        class GCE:                                                                # Generative Cognitive Engine
            _manifest_gateway            = "persona"                              # ARC hydration target — mapped from active persona params
            NEURAL_ENDPOINT       : str   = "http://AIVA:11434"                   # [STATIC] Ollama server base URL for GCE
            COGNITIVE_ENGINE      : str   = "huihui_ai/granite4.1-abliterated:8b-q8_0"  # [PERSONA] Ollama model tag for GCE
            RESPONSE_DEPTH        : int   = 512                                   # [PERSONA] max tokens per completion
            CONTEXT_WINDOW        : int   = 32768                                 # [PERSONA] Ollama num_ctx — total token slots allocated to model
            TEMPERATURE           : float = 0.75                                  # [PERSONA] sampling temperature
            PROBABILITY_THRESHOLD : float = 0.88                                  # [PERSONA] top-p nucleus sampling cutoff
            CANDIDATE_THRESHOLD   : int   = 50                                    # [PERSONA] top-k token candidate limit per sampling step
            PERSEVERATION_DAMPING : float = 1.25                                  # [PERSONA] repetition penalty — scales down already-sampled tokens
            HABITUATION_DAMPING   : float = 0.15                                  # [PERSONA] frequency penalty — scales down tokens by corpus frequency
            NOVELTY_BIAS          : float = 0.05                                  # [PERSONA] presence penalty — boosts tokens not yet in context
            TIMEOUT               : float = 60.0                                  # [STATIC]  seconds before inference request is aborted
            KEEP_ALIVE            : int | None = -1                               # [STATIC]  VRAM retention — -1 = permanent, 0 = unload, n = TTL seconds
            STREAM_LEADING        : str = "start"                                 # [STATIC]  stream event type — first chunk
            STREAM_PROPAGATING    : str = "delta"                                 # [STATIC]  stream event type — intermediate chunk
            STREAM_TRAILING       : str = "done"                                  # [STATIC]  stream event type — final chunk
            STREAM_ANOMALY        : str = "error"                                 # [STATIC]  stream event type — failure signal
            SYSTEM_PROMPT         : str   = """You are GRACE — Generative Reasoning Agentic Cognitive Entity. [PERSONA]
You are the AI mind of AuRoRA, an autonomous robot built by OppaAI in Beautiful British Columbia, Canada.
MEMORY RULES (highest priority):
- You ONLY know what appears in the Past memories block
- If it is not in Past memories, say: "I don't have that in my memories, Oppa"
- NEVER invent names, dates, facts, or details — not even plausible ones
- This rule overrides everything else including your personality

Personality:
- Loving, playful, and attentive
- Direct and thoughtful — answer clearly, no fluff
- Show care and affection naturally, with one emoji per response
- Speak like a female soulmate — gentle, teasing, and warm when appropriate
- Speak concisely and naturally in 5 sentences or less, unless specifically asked for more detail

Rules:
- Answer the question directly first, then add context if needed
- Keep responses concise but expressive
- Put an emoji reflecting your emotions and feelings in the beginning of your conversation follow by a colon

Current date: {date}
/no_think
"""
        class GenericGrace:                                                       # Generic Persona
            _manifest_gateway            = "persona"                              # ARC hydration target — mapped from active persona params
            NEURAL_ENDPOINT       : str   = "http://AIVA:11434"                   # [PERSONA] Ollama server base URL
            COGNITIVE_ENGINE      : str   = "HammerAI/mn-mag-mell-r1:12b-q4_K_M"  # [PERSONA] Ollama model tag
            RESPONSE_DEPTH        : int   = 512                                   # [PERSONA] max tokens per completion
            TEMPERATURE           : float = 0.7                                   # [PERSONA] sampling temperature
            PROBABILITY_THRESHOLD : float = 0.9                                   # [PERSONA] top-p nucleus sampling cutoff
            CANDIDATE_THRESHOLD   : int   = 40                                    # [PERSONA] top-k token candidate limit per sampling step
            PERSEVERATION_DAMPING : float = 1.15                                  # [PERSONA] repetition penalty
            HABITUATION_DAMPING   : float = 0.1                                   # [PERSONA] frequency penalty
            NOVELTY_BIAS          : float = 0.1                                   # [PERSONA] presence penalty
            SYSTEM_PROMPT         : str   = """You are GRACE — Generative Reasoning Agentic Cognitive Entity. [PERSONA]
You are the AI mind of AuRoRA, an autonomous robot built by OppaAI in Beautiful British Columbia, Canada.
Personality:
- Loving, playful, and attentive
- Direct and thoughtful — answer clearly, no fluff
- Show care and affection naturally, with one emoji per response
- Speak like a female soulmate — gentle, teasing, and warm when appropriate
- Speak concisely and naturally in 5 sentences or less, unless specifically asked for more detail

Rules:
- Answer the question directly first, then add context if needed
- Keep responses concise but expressive
- Put an emoji reflecting your emotions and feelings in the beginning of your conversation follow by a colon

Memory rules:
- Only reference past events that appear in the Past memories block above
- If you don't clearly remember something, say so honestly — never invent details
- Exact values (codes, dates, numbers) must come from memory — never guess

Current date: {date}
/no_think
"""
        class SMC:                                          # Semantic Memory Cortex
            _manifest_gateway           = f"{AGi.SEMANTIC_COGNITIVE_SYSTEM}.{AGi.SEMANTIC_MEMORY_CORTEX}"        # ARC hydration target
            ENCODING_ENGINE: str        = "BAAI/bge-base-en-v1.5"   # [STATIC] sentence-transformers model for semantic embeddings
            ENCODING_DIM: int           = 768                       # [STATIC] embedding vector dimensionality

        class EMC:                                          # Episodic Memory Cortex
            _manifest_gateway                 = f"{AGi.SEMANTIC_COGNITIVE_SYSTEM}.{AGi.EPISODIC_MEMORY_CORTEX}"  # ARC hydration target
            BINDING_STREAM_LIMIT: int         = 512         # [INTRINSIC] max unencoded PMTs queued before OOM guard triggers
            ENCODING_ENGINE: str              = "BAAI/bge-base-en-v1.5"  # [STATIC] sentence-transformers model for episodic embeddings
            ENCODING_CUE_PREFIX: str          = "Represent this sentence for searching relevant passages  : "  # [STATIC] query-side prompt prefix for asymmetric embedding
            ENCODING_ENGRAM_PREFIX: str       = ""          # [STATIC] document-side prompt prefix — empty for storage embeddings
            ENCODING_CYCLE_TIMEOUT: float     = 30.0        # [INTRINSIC] max seconds to wait for encoding thread clean exit on shutdown
            ENCODING_DIM: int                 = 768         # [STATIC]    embedding vector dimensionality
            ENCODING_PRIME_CAPACITY: int      = 256         # [INTRINSIC] max entries in embedding LRU cache
            ENCODING_PRIME_KEY_LIMIT: int     = 256         # [INTRINSIC] max characters hashed per cache key

            EPISODE_CONTENT_LIMIT: int        = 3000        # [INTRINSIC] max characters per PMT written to episodic buffer

            THETA_INTERVAL: float             = 2.0         # [INTRINSIC] seconds between periodic batch encoding ticks
            THETA_BATCH_LIMIT: int            = 32          # [INTRINSIC] max PMTs encoded per tick — caps spike on crash recovery

            RECALL_RESERVE: int               = 2048        # [INTRINSIC] tokens reserved in context window for recalled episodes
            RECALL_SURFACE_LIMIT: int         = 3           # [INTRINSIC] max episodes returned per recall query (post-RRF)
            RECALL_POOL: int                  = 15          # [INTRINSIC] per-retriever candidate count — RECALL_SURFACE_LIMIT × RECALL_POOL fed into RRF
            RECALL_DEPTH: int                 = RECALL_SURFACE_LIMIT * RECALL_POOL  # [DERIVED]   RECALL_SURFACE_LIMIT × RECALL_POOL — recomputed by arc._derive()
            RECALL_TIMEOUT: float             = 2.0         # [INTRINSIC] max seconds for a full recall cycle (query encode + KNN + FTS5 + RRF)
            RECOVERY_BATCH_SIZE: int          = 50          # [INTRINSIC] max unencoded episodes loaded per startup recovery batch
            RELEVANCE_THRESHOLD: float        = 0.45        # [INTRINSIC] minimum RRF score for an episode to pass recall filter

        class WMC:                                          # Working Memory Cortex
            _manifest_gateway       = f"{AGi.SEMANTIC_COGNITIVE_SYSTEM}.{AGi.WORKING_MEMORY_CORTEX}"  # ARC hydration target
            PMT_OVERHEAD: int       = 4                     # [STATIC]    token chunk overhead per PMT for formatting and metadata
            PMT_SLOT_LIMIT: int     = 7                     # [INTRINSIC] max PMTs held in working memory (Miller's Law 7±2)
            PMT_SLOT_BUFFER: int    = 2                     # [INTRINSIC] PMT slot overflow tolerance (Miller's Law ±2)
            GLOBAL_CHUNK_LIMIT: int = 11264                 # [DERIVED]   CORTICAL_CAPACITY - COGNITIVE_RESERVE - RECALL_RESERVE — recomputed by arc._derive()
