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
                  Baseline:     Generic Persona  (default, read-only reset target)

    [DERIVED]   — Computed from other constants via metaclass properties.
                  Never set directly. Recomputed on every access.

Parameters:
    Defaults live here in hrm.py. At boot, RAC loads aurora.yaml into the
    ROS parameter server via ParameterFile. Each node declares its parameters
    with hrm.py constants as defaults — ROS uses the YAML value if present,
    falls back to the hrm.py default if the key is missing.

    AuRoRA Setpoints    — robot-wide settings, loaded by RAC at boot into ROS param server
    User Profile        — per-user profiles, loaded by CNC at session start
    Active Persona      — active persona inference + personality, loaded by GCE at init
    Generic Persona     — default persona baseline, read-only reset target

    To add a constant:   add it here with a default + add to AuRoRA Setpoints. Done.
    To add a subsystem:  add a class to AGi here + add a ROS node. Done.

TODO:
    HRS milestone — build hrs.py to allow GRACE to update [INTRINSIC] constants
                    at runtime and persist changes back to aurora.yaml.
                  — add HRS startup/shutdown lifecycle management
                  — add recency parameter for identification of the most recent event segments in EMC
"""

class RRR:
    """Resource Router Registry — canonical identifier manifest for all AuRoRA subsystems."""
    # Module registry — single source of truth for all subsystem identifiers
    # Robot entity
    ROBOT_ENTITY                    : str = "agi"        # [STATIC] ROS namespace + YAML key for the robot entity
    
    # Semantic Cognitive System
    SEMANTIC_COGNITIVE_SYSTEM       : str = "scs"        # [STATIC] ROS namespace + YAML key for the Semantic Cognitive System
    CENTRAL_NERVOUS_CORE            : str = "cnc"        # [STATIC] ROS namespace + YAML key for the Central Nervous Core
    RETICULAR_ACTIVATING_COMPARTMENT: str = "rac"        # [STATIC] ROS namespace + YAML key for the Reticular Activating Compartment
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
class _SCSType(type):
    @property
    def GLOBAL_CHUNK_LIMIT(cls) -> int:
        """[DERIVED] Total usable token budget — CORTICAL_CAPACITY minus all reserves."""
        return cls.CORTICAL_CAPACITY - cls.COGNITIVE_RESERVE - cls.EMC.RECALL_RESERVE        # [DERIVED] total usable token budget

class _EMCType(type):
    @property
    def RECALL_DEPTH(cls) -> int:
        """[DERIVED] Total recall candidate pool — RECALL_SURFACE_LIMIT × RECALL_POOL."""
        return cls.RECALL_SURFACE_LIMIT * cls.RECALL_POOL                                    # [DERIVED] total recall candidate pool

class AGi:                                                              # Amazing Grace infrastructure

    ENTITY_GATEWAY: str = f".{RRR.ROBOT_ENTITY}"                        # [STATIC] root directory for all AGi core system state


    class SCS(metaclass=_SCSType):                                      # Semantic Cognitive System
        CORTICAL_CAPACITY: int  = 16384                                 # [INTRINSIC] total token budget for the active LLM context window
        COGNITIVE_RESERVE: int  = 2048                                  # [INTRINSIC] tokens reserved for system prompt and identity injection
        NEURAL_GATEWAY: str     = f"{RRR.SEMANTIC_COGNITIVE_SYSTEM}"    # [STATIC] ROS namespace prefix for SCS topics
        MEMORY_GATEWAY: str     = f"{RRR.MEMORY_COORDINATION_CORTEX}"   # [STATIC] ROS namespace prefix for MCC topics
        ENGRAM_COMPLEX: str     = "engram_complex.db"                   # [STATIC] SQLite filename for long-term memory storage
        UNITS_PER_CHUNK: int    = 4                                     # [STATIC] tokens per chunk unit (TODO: obsolete post-tokenizer M1.5)

        # Config file registry — single source of truth for all YAML filenames
        AURORA_SETPOINTS : str = "aurora.yaml"                          # [STATIC] robot-wide settings
        USER_PROFILES    : str = "users.yaml"                           # [STATIC] all user profiles + per-user extrinsic settings
        PERSONA_ACTIVE   : str = "persona.yaml"                         # [STATIC] active persona — mutable
        PERSONA_GENERIC  : str = "generic.yaml"                         # [STATIC] default persona reset target — read-only

        TEXT_INPUT_GATEWAY: str     = f"/{RRR.SEMANTIC_COGNITIVE_SYSTEM}/text_input"            # [STATIC] ROS topic — inbound user text
        RESPONSE_GATEWAY: str       = f"/{RRR.SEMANTIC_COGNITIVE_SYSTEM}/response"              # [STATIC] ROS topic — outbound LLM response
        MEMORY_CONTEXT_GATEWAY: str = f"/{RRR.SEMANTIC_COGNITIVE_SYSTEM}/memory_context"        # [STATIC] ROS topic — reinstated memory context injected into prompt
        MEMORY_STATS_GATEWAY: str   = f"/{RRR.SEMANTIC_COGNITIVE_SYSTEM}/memory_stats"          # [STATIC] ROS topic — memory diagnostics from all memory subsystems

        class GCE:                                                                              # Generative Cognitive Engine
            NEURAL_ENDPOINT       : str   = "http://AIVA:11434"                                 # [STATIC]  Ollama server base URL
            COGNITIVE_ENGINE      : str   = "huihui_ai/granite4.1-abliterated:8b-q8_0"          # [PERSONA] Ollama model tag
            RESPONSE_DEPTH        : int   = 512                                                 # [PERSONA] max tokens per completion
            CONTEXT_WINDOW        : int   = 32768                                               # [PERSONA] Ollama num_ctx — total token slots allocated to model
            TEMPERATURE           : float = 0.75                                                # [PERSONA] sampling temperature
            PROBABILITY_THRESHOLD : float = 0.88                                                # [PERSONA] top-p nucleus sampling cutoff
            CANDIDATE_THRESHOLD   : int   = 50                                                  # [PERSONA] top-k token candidate limit per sampling step
            PERSEVERATION_DAMPING : float = 1.25                                                # [PERSONA] repetition penalty — scales down already-sampled tokens
            HABITUATION_DAMPING   : float = 0.15                                                # [PERSONA] frequency penalty — scales down tokens by corpus frequency
            NOVELTY_BIAS          : float = 0.05                                                # [PERSONA] presence penalty — boosts tokens not yet in context
            TIMEOUT               : float = 60.0                                                # [STATIC]  seconds before inference request is aborted
            KEEP_ALIVE            : int | None = -1                                             # [STATIC]  VRAM retention — -1 = permanent, 0 = unload, n = TTL seconds
            STREAM_LEADING        : str = "start"                                               # [STATIC]  stream event type — first chunk
            STREAM_PROPAGATING    : str = "delta"                                               # [STATIC]  stream event type — intermediate chunk
            STREAM_TRAILING       : str = "done"                                                # [STATIC]  stream event type — final chunk
            STREAM_ANOMALY        : str = "error"                                               # [STATIC]  stream event type — failure signal
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
- Put an emoji reflecting your emotions and feelings in the beginning of your response followed by a colon

Current date: {date}
/no_think
"""

        class SMC:                                                       # Semantic Memory Cortex
            ENCODING_ENGINE: str              = "BAAI/bge-base-en-v1.5"  # [STATIC] sentence-transformers model for semantic embeddings
            ENCODING_DIM: int                 = 768                      # [STATIC] embedding vector dimensionality

        class EMC(metaclass=_EMCType):                       # Episodic Memory Cortex
            BINDING_STREAM_LIMIT: int         = 512         # [INTRINSIC] max unencoded PMTs queued before OOM guard triggers
            ENCODING_ENGINE: str              = "BAAI/bge-base-en-v1.5"  # [STATIC]    sentence-transformers model for episodic embeddings
            ENCODING_CUE_PREFIX: str          = "Represent this sentence for searching relevant passages  : "  # [STATIC] query-side prompt prefix for asymmetric embedding
            ENCODING_ENGRAM_PREFIX: str       = ""          # [STATIC]    document-side prompt prefix — empty for storage embeddings
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
            RECALL_TIMEOUT: float             = 2.0         # [INTRINSIC] max seconds for a full recall cycle (query encode + KNN + FTS5 + RRF)
            RECOVERY_BATCH_SIZE: int          = 50          # [INTRINSIC] max unencoded episodes loaded per startup recovery batch
            RELEVANCE_THRESHOLD: float        = 0.45        # [INTRINSIC] minimum RRF score for an episode to pass recall filter

        class WMC:                                          # Working Memory Cortex
            PMT_OVERHEAD: int       = 4                     # [STATIC]    token chunk overhead per PMT for formatting and metadata
            PMT_SLOT_LIMIT: int     = 7                     # [INTRINSIC] max PMTs held in working memory (Miller's Law 7±2)
            PMT_SLOT_BUFFER: int    = 2                     # [INTRINSIC] PMT slot overflow tolerance (Miller's Law ±2)

    @classmethod
    def hydrate_from_ros(cls, node, prefix: str = 'scs') -> None:
        """
        Declare and read all intrinsic AGi parameters from the ROS2 param server.
        Called once per node at init — walks the class tree from the given prefix.
        
        Args:
            node: ROS2 Node instance — provides declare_parameter / get_parameter
            prefix: YAML hierarchy root to walk (default 'scs')
        """
        target = next(
            (getattr(cls, n) for n in vars(cls)
             if isinstance(getattr(cls, n), type) and n.lower() == prefix),
            None
        )
        if target is None:
            raise RuntimeError(f"❌ AGi has no subclass matching prefix '{prefix}'")
        cls._declare_and_read(node, target, prefix)

    @classmethod
    def _declare_and_read(cls, node, target, prefix: str) -> None:
        """
        Recursively walk a class tree, declare ROS2 parameters, write values back.
        Skips STATIC constants (str, None). Only touches int, float, bool.
        """
        for name in vars(target):
            if name.startswith('_'):
                continue
            val = getattr(target, name)
            key = f"{prefix}.{name.lower()}"

            if isinstance(val, type):
                cls._declare_and_read(node, val, key)
            elif isinstance(val, (int, float, bool)):
                node.declare_parameter(key, val)
                setattr(target, name, node.get_parameter(key).value)

    @classmethod
    def hydrate_from_yaml(cls, path) -> None:
        """
        Load a YAML file and apply matching keys to the AGi class tree.
        Used by CNC to load persona.yaml into AGi.SCS.GCE.
        Can be reused for any flat YAML → class override pattern.

        Args:
            path: Path to YAML file
        """
        import yaml
        raw = yaml.safe_load(open(path).read())
        cls._apply_yaml(cls.SCS.GCE, raw)

    @classmethod
    def _apply_yaml(cls, target, raw: dict) -> None:
        """Apply matching YAML keys to a target class — sets attributes in place."""
        for name in vars(target):
            if name.startswith('_'):
                continue
            key = name.lower()
            if key in raw:
                setattr(target, name, raw[key])
