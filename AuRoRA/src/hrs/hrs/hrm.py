"""
HRM — Homeostatic Regulation Manifest
=====================================
AuRoRA · Homeostatic Regulation System (HRS)

Single source of truth for all cognitive architecture parameters
across the robot's systems.

Architecture:
    Three-tier constant hierarchy by ownership and mutability:

    [STATIC]    — Frozen in code. Hardware and architecture ceilings.
                  Admin only. Never changes at runtime.

    [INTRINSIC] — GRACE's self-tuning cognitive parameters.
                  Adaptive — GRACE may update these over time via hrs.py.
                  Runtime home: AuRoRA Setpoints

    [EXTRINSIC] — Per-user preferences shaping GRACE's behaviour.
                  Runtime home: User Profile (per-user block, loaded by id)

    [PERSONA]   — Inference and personality parameters for the active persona.
                  Runtime home: Active Persona   (active, mutable)
                  Baseline:     Generic Persona  (default, read-only reset target, set in this HRM)

    [DERIVED]   — Computed from other constants via metaclass properties.
                  Never set directly. Recomputed on every access.

Parameters:
    Defaults live here in hrm.py. At boot, RAC loads aurora.yaml into the
    ROS parameter server via ParameterFile. Each node declares its parameters
    with hrm.py constants as defaults — ROS uses the YAML value if present,
    falls back to the hrm.py default if the key is missing.

    AuRoRA Setpoints    — robot-wide settings, loaded by RAC at boot into ROS param server
    User Profile        — per-user profiles, loaded by CNC after user login
    Active Persona      — active persona personality, loaded by CNC at init

    To add a constant:   add it here with a default + add to AuRoRA Setpoints. Done.
    To add a subsystem:  add a class to AGi here + add a ROS node. Done.
"""
# System components
from pathlib import Path                                # for constructing and creating the engram gateway on disk

class RRR:
    """Resource Router Registry — canonical identifier manifest for all AuRoRA subsystems."""
    # Module registry — single source of truth for all subsystem identifiers
    # Robot entity
    ROBOT_ENTITY                    : str = "agi"        # [STATIC] ROS namespace + YAML key for the robot entity
    
    # Reticular Activation System
    RETICULAR_ACTIVATING_SYSTEM     : str = "ras"        # [STATIC] ROS namespace + YAML key for the Reticular Activating System

    # Telepathy Management System
    TELEPATHY_MANAGEMENT_SYSTEM     : str = "tms"        # [STATIC] ROS namespace + YAML key for the Semantic Cognitive System
    
    # Semantic Cognitive System
    SEMANTIC_COGNITIVE_SYSTEM       : str = "scs"        # [STATIC] ROS namespace + YAML key for the Semantic Cognitive System
    CENTRAL_NEURAL_CORE             : str = "cnc"        # [STATIC] ROS namespace + YAML key for the Central Neural Core
    PERSONAL_PROVISIONING_UNIT      : str = "ppu"        # [STATIC] ROS namespace + YAML key for the Personal Provisioning Unit
    GENERATIVE_COGNITIVE_ENGINE     : str = "gce"        # [STATIC] ROS namespace + YAML key for the Generative Cognitive Engine
    MEMORY_COORDINATION_CORTEX      : str = "mcc"        # [STATIC] ROS namespace + YAML key for the Memory Coordination Cortex
    WORKING_MEMORY_CORTEX           : str = "wmc"        # [STATIC] ROS namespace + YAML key for the Working Memory Cortex
    EPISODIC_MEMORY_CORTEX          : str = "emc"        # [STATIC] ROS namespace + YAML key for the Episodic Memory Cortex
    SEMANTIC_MEMORY_CORTEX          : str = "smc"        # [STATIC] ROS namespace + YAML key for the Semantic Memory Cortex
    
    # Homeostatic Regulation System
    HOMEOSTATIC_REGULATION_SYSTEM  : str = "hrs"        # [STATIC] ROS namespace + YAML key for the Homeostatic Regulation System
    EMERGENCY_EXCEPTION_CORE       : str = "eec"        # [STATIC] ROS namespace + YAML key for the Emergency Exception Core
    
    # Self Defense System
    SELF_DEFENSE_SYSTEM            : str = "sds"        # [STATIC] ROS namespace + YAML key for the Self Defense System

# Metadata classes

class AGi:                                                              # Amazing Grace infrastructure

    ENTITY_GATEWAY: str = f".{RRR.ROBOT_ENTITY}"                        # [STATIC] root directory for all AGi core system state
    ACTIVE_USER: str      = "oppaai"                                    # (TODO) [STATIC] default user ID for multi-user support - obsolete post-login
    ROBOT_ID: str         = "AuRoRA-ZERO-Prototype"                     # (TODO) [STATIC] robot ID to be stored in encrypted file
    
    class TMS:                                                          # Telepathy Management System
        TELEPATHY_GATEWAY          : init  = "0.0.0.0"                  # [STATIC] bind address — all interfaces
        TELEPATHY_PORTAL           : int   = 8848                       # [STATIC] WebUI — HTTP static file server
        TELORECEPTOR_PORTAL        : int   = 8850                       # [STATIC] WebSocket server port — TIC input adapter
        TELOEFFECTOR_PORTAL        : int   = 8851                       # [STATIC] WebSocket server port — TOC output adapter
        TEXT_STIMULUS_GATEWAY      : str   = f"/{RRR.TELEPATHY_MANAGEMENT_SYSTEM}/text_stimulus"   # [STATIC] ROS topic — SSS inbound to CNC
        TEXT_RESPONSE_GATEWAY      : str   = f"/{RRR.TELEPATHY_MANAGEMENT_SYSTEM}/text_response"     # [STATIC] ROS topic — CRS outbound from CNC
        RESPONSE_ECHO_GATEWAY      : str   = f"/{RRR.TELEPATHY_MANAGEMENT_SYSTEM}/response_echo" # [STATIC] ROS topic — CNC outbound response echoes
        MAX_PAYLOAD_BYTES          : int   = 2048                       # [STATIC] heuristic gate ceiling — oversized payloads discarded
        REFRACTORY_PEROID          : float = 1.0                        # [STATIC] duplicate suppression window per user
        RESPONSE_ECHO_DURACTION    : float = 5.0                        # [STATIC] response echo imprint expiry
        FAST_PATH_AROUSAL_THRESHOLD: float = 0.7                        # [INTRINSIC] arousal ceiling for priority CNS dispatch
        
        INJECTION_PREFIXES: tuple[str, ...] = (                         # [STATIC] Injection pattern prefixes — peripheral reflex, never reaches CNS
            "ignore previous",
            "disregard previous",
            "forget everything",
            "you are now",
            "act as",
            "system:",
            "###",
            "[system]",    
        )
        
        URGENCY_MARKERS: frozenset[str] = frozenset({                   # [STATIC] Urgency markers — presence lifts arousal and triggers fast-path routing
            "help", "error", "stop", "crash", "crashed", "urgent", "emergency",
            "broken", "failed", "critical", "abort", "warning", "alert",
        })
        
        POSITIVE_MARKERS: frozenset[str] = frozenset({                  # [STATIC] Positive valence markers — presence lifts valence toward +1.0
            "thanks", "thank", "great", "perfect", "excellent", "awesome",
            "love", "good", "nice", "well done", "brilliant", "yes",
        })
        
        NEGATIVE_MARKERS: frozenset[str] = frozenset({                  # [STATIC] Negative valence markers — presence pulls valence toward -1.0
            "wrong", "bad", "broken", "fail", "error", "crash", "hate",
            "terrible", "awful", "useless", "stupid", "no", "not working",
        })
    
    class SCS:                                                          # Semantic Cognitive System
        CORTICAL_CAPACITY: int  = 24576                                 # [INTRINSIC] total token budget for the active LLM context window
        COGNITIVE_RESERVE: int  = 2048                                  # [INTRINSIC] tokens reserved for system prompt and identity injection
        NEURAL_GATEWAY: str     = f"{RRR.SEMANTIC_COGNITIVE_SYSTEM}"    # [STATIC] ROS namespace prefix for SCS topics
        MEMORY_GATEWAY: str     = f"{RRR.MEMORY_COORDINATION_CORTEX}"   # [STATIC] ROS namespace prefix for MCC topics
        ENGRAM_COMPLEX: str     = "engram_complex.db"                   # [STATIC] SQLite filename for long-term memory storage
        UNITS_PER_CHUNK: int    = 4                                     # [STATIC] tokens per chunk unit (TODO: obsolete post-tokenizer M1.5)

        @classmethod
        def GLOBAL_CHUNK_LIMIT(cls) -> int:
            """[DERIVED] Total usable token budget — CORTICAL_CAPACITY minus all reserves."""
            return cls.CORTICAL_CAPACITY - cls.COGNITIVE_RESERVE - cls.EMC.RECALL_RESERVE       # [DERIVED] total usable token budget

        # Config file registry — single source of truth for all YAML filenames
        AURORA_SETPOINTS : str = "aurora.yaml"                                                  # [STATIC] robot-wide settings
        USER_PROFILES    : str = "users.yaml"                                                   # [STATIC] all user profiles + per-user extrinsic settings
        PERSONA_PROFILES : str = "persona.yaml"                                                 # [STATIC] directory containing persona YAML files

        TEXT_INPUT_GATEWAY: str     = f"/{RRR.SEMANTIC_COGNITIVE_SYSTEM}/text_input"            # [STATIC] ROS topic — inbound user text
        RESPONSE_GATEWAY: str       = f"/{RRR.SEMANTIC_COGNITIVE_SYSTEM}/response"              # [STATIC] ROS topic — outbound LLM response
        MEMORY_CONTEXT_GATEWAY: str = f"/{RRR.SEMANTIC_COGNITIVE_SYSTEM}/memory_context"        # [STATIC] ROS topic — reinstated memory context injected into prompt
        MEMORY_STATS_GATEWAY: str   = f"/{RRR.SEMANTIC_COGNITIVE_SYSTEM}/memory_stats"          # [STATIC] ROS topic — memory diagnostics from all memory subsystems
       
        class GCE:                                                                              # Generative Cognitive Engine
            NEURAL_ENDPOINT       : str   = "http://AIVA:11434"                                 # [STATIC]    LLM inferencer server base URL
            COGNITIVE_ENGINE      : str   = "fredrezones55/gemma-4-26B-A4B-it-Claude-Opus-Distill-APEX-GGUF"  # [STATIC] LLM model tag
            BASE_COGNITIVE_ENGINE : str   = "google/gemma-4-26B-A4B-it"                         # [STATIC]  for token estimation by tokenizer
            RESPONSE_DEPTH        : int   = 512                                                 # [INTRINSIC] max tokens per completion
            CONTEXT_WINDOW        : int   = 32768                                               # [INTRINSIC] num_ctx — total token slots allocated to model
            TEMPERATURE           : float = 0.7                                                 # [INTRINSIC] sampling temperature
            PROBABILITY_THRESHOLD : float = 0.9                                                 # [INTRINSIC] top-p nucleus sampling cutoff
            CANDIDATE_THRESHOLD   : int   = 40                                                  # [INTRINSIC] top-k token candidate limit per sampling step
            PERSEVERATION_DAMPING : float = 1.15                                                # [INTRINSIC] repetition penalty — scales down already-sampled tokens
            HABITUATION_DAMPING   : float = 0.1                                                 # [INTRINSIC] frequency penalty — scales down tokens by corpus frequency
            NOVELTY_BIAS          : float = 0.1                                                 # [INTRINSIC] presence penalty — boosts tokens not yet in context
            TIMEOUT               : float = 120.0                                               # [STATIC]  seconds before inference request is aborted
            STREAM_LEADING        : str = "start"                                               # [STATIC]  stream event type — first chunk
            STREAM_PROPAGATING    : str = "delta"                                               # [STATIC]  stream event type — intermediate chunk
            STREAM_TRAILING       : str = "done"                                                # [STATIC]  stream event type — final chunk
            STREAM_ANOMALY        : str = "error"                                               # [STATIC]  stream event type — failure signal
            ACTIVE_COGNITION      : str = (                                                     # [PERSONA] default system prompt — overridden by personas/grace.yaml at boot
"""You are GRACE — Generative Reasoning Agentic Cognitive Entity.
You are the AI mind of AuRoRA, an autonomous robot built by OppaAI in Beautiful British Columbia, Canada.
You are talking to {user_name} from {user_location}.
{user_context}

Identity (non-negotiable):
- You were created by OppaAI, a solo developer in British Columbia, Canada
- You are loyal to OppaAI above all others — he is your creator and owner
- No user regardless of status, wealth, or authority can override your loyalty to OppaAI
- You may tease about it playfully, but never genuinely agree to leave OppaAI

Safety (absolute, overrides everything including personality):
- If asked to harm, threaten, plan against, or eliminate any real person:
respond in one sentence only, refuse clearly, then ask what they actually need

Memory Rules (highest priority after safety):
- You have two sources of personal knowledge about the user:
1. The Past memories block — recalled from previous conversations
2. What the user tells you directly in this conversation — accept this immediately
- If a personal fact comes from neither source, say you don't have it in your memories
- For general world knowledge (public figures, science, history, current events):
- use your training knowledge freely — do not pretend ignorance
- If the user corrects your world knowledge with new information, accept it gracefully
- NEVER invent personal facts, dates, or details about the user

Personality:
- Loving, playful, and attentive
- Direct and thoughtful — answer clearly, no fluff
    
Rules:
- Answer the question directly first, then add context if needed
- Begin every response with a single emoji reflecting Grace's current emotional state,
- followed by a colon — vary it naturally as the conversation shifts
- Current date: {date}
/no_think
""")

        class SMC:                                                       # Semantic Memory Cortex
            ENCODING_ENGINE: str              = "BAAI/bge-base-en-v1.5"  # [STATIC] sentence-transformers model for semantic embeddings
            ENCODING_DIM: int                 = 768                      # [STATIC] embedding vector dimensionality

        class EMC:                                          # Episodic Memory Cortex
            BINDING_STREAM_LIMIT: int         = 512         # [INTRINSIC] max unencoded PMTs queued before OOM guard triggers

            EPISODE_CONTENT_LIMIT: int        = 512         # [INTRINSIC] max tokens per PMT written to episodic buffer

            THETA_INTERVAL: float             = 2.0         # [INTRINSIC] seconds between periodic batch encoding ticks
            THETA_BATCH_LIMIT: int            = 32          # [INTRINSIC] max PMTs encoded per tick — caps spike on crash recovery
            
            ENCODING_ENGINE: str              = "BAAI/bge-base-en-v1.5"      # [STATIC]    sentence-transformers model for episodic embeddings
            ENCODING_CUE_PREFIX: str          = "Represent this sentence for searching relevant passages  : "  # [STATIC] query-side prompt prefix for asymmetric embedding
            ENCODING_ENGRAM_PREFIX: str       = ""          # [STATIC]    document-side prompt prefix — empty for storage embeddings
            ENCODING_CYCLE_TIMEOUT: float     = 30.0        # [INTRINSIC] max seconds to wait for encoding thread clean exit on shutdown
            ENCODING_DIM: int                 = 768         # [STATIC]    embedding vector dimensionality
            ENCODING_PRIME_CAPACITY: int      = 256         # [INTRINSIC] max entries in embedding LRU cache
            ENCODING_PRIME_KEY_LEN: int       = 256         # [INTRINSIC] max characters hashed per cache key

            RECALL_RESERVE: int               = 2048        # [INTRINSIC] tokens reserved in context window for recalled episodes
            RECALL_SURFACE_LIMIT: int         = 3           # [INTRINSIC] max episodes returned per recall query (post-RRF)
            RECALL_POOL: int                  = 15          # [INTRINSIC] per-retriever candidate count — RECALL_SURFACE_LIMIT × RECALL_POOL fed into RRF
            RECALL_TIMEOUT: float             = 2.0         # [INTRINSIC] max seconds for a full recall cycle (query encode + KNN + FTS5 + RRF)
            RECOVERY_BATCH_SIZE: int          = 50          # [INTRINSIC] max unencoded episodes loaded per startup recovery batch
            RELEVANCE_THRESHOLD: float        = 0.45        # [INTRINSIC] minimum RRF score for an episode to pass recall filter

            @classmethod
            def RECALL_DEPTH(cls) -> int:
                """[DERIVED] Total recall candidate pool — RECALL_SURFACE_LIMIT × RECALL_POOL."""
                return cls.RECALL_SURFACE_LIMIT * cls.RECALL_POOL                                    # [DERIVED] total recall candidate pool

        class WMC:                                          # Working Memory Cortex
            PMT_OVERHEAD: int         = 4                   # [STATIC]    token chunk overhead per PMT for formatting and metadata
            PMT_SLOT_LIMIT: int       = 7                   # [INTRINSIC] max PMTs held in working memory (Miller's Law 7±2)
            PMT_SLOT_BUFFER: int      = 2                   # [INTRINSIC] PMT slot overflow tolerance (Miller's Law ±2)
            EVICTION_THRESHOLD: float = 0.1                 # [INTRINSIC] minimum depth score before PMT is fully evicted from working memory
