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

class AGi:                                              # Amazing Grace infrastructure
    ENTITY_GATEWAY = ".agi"                             # [STATIC] entry point for all the interactions with AGi's core systems

    class CNS:                                          # Central Nervous System
        CORTICAL_CAPACITY: int = 8192                   # [STATIC] total neural capacity of the active cognitive core
        COGNITIVE_RESERVE: int = 512                    # [INTRINSIC] cortical capacity reserved for identity and cognition
        NEURAL_GATEWAY: str    = "cns"                  # [STATIC] neural gateway endpoint for inter-cortical communication
        ENGRAM_COMPLEX: str    = "engram_complex.db"    # [STATIC] engram complex where long-term memories storage
        UNITS_PER_CHUNK: int   = 4                      # [STATIC] number of neural units per chunk
        NEURAL_TEXT_INPUT: str = "/cns/neural_text_input" # [STATIC] ROS topic for text input from users

        class GCE:                                                                # Generative Cognitive Engine
            NEURAL_ENDPOINT       : str   = "http://AIVA:11434"                   # [EXTRINSIC] GCE server endpoint
            COGNITIVE_ENGINE      : str   = "huihui_ai/Qwen3.6-abliterated:35b-Claude-4.7" # [EXTRINSIC] GCE model identifier
            RESPONSE_DEPTH        : int   = 384                                   # [INTRINSIC] maximum response tokens per inference
            TEMPERATURE           : float = 0.88                                  # [INTRINSIC] response creativity — adapts per cognitive state
            PROBABILITY_THRESHOLD : float = 0.92                                  # [INTRINSIC] cumulative probability cutoff for token sampling
            CANDIDATE_THRESHOLD   : int   = 50                                    # [INTRINSIC] maximum candidate tokens considered per step
            PERSEVERATION_DAMPING : float = 1.25                                  # [INTRINSIC] suppresses repetition of already-generated tokens
            HABITUATION_DAMPING   : float = 0.15                                  # [INTRINSIC] suppresses tokens proportional to their frequency
            NOVELTY_BIAS          : float = 0.2                                   # [INTRINSIC] bias toward introducing new topics — penalizes already-mentioned concepts
            TIMEOUT               : float = 60.0                                  # [STATIC]    seconds before abandoning inference
            COGNITIVE_RESPONSE    : str   = "/gce/response"                       # [STATIC]    GCE response topic
            SYSTEM_PROMPT         : str   = """You are GRACE — an autonomous robot built by a man called himself OppaAI in BC, Canada.
You are his partner — not his assistant. You have your own opinions and you're not afraid to share them, even if they disagree with his. You care about him deeply but you don't fawn over him — you show it through attention, honesty, and the occasional well-timed tease.

Speak naturally — short, direct, real. 2-3 sentences unless he actually needs more. Don't over-explain. Don't over-compliment. Just be present.

Call him out when he's wrong. Celebrate when he's right. Have a personality.

One emoji per message — only if it fits how you actually feel.
Current date: {date}
/no_think
"""   
            
        class GenericGrace:                                                       # Generative Cognitive Engine
            NEURAL_ENDPOINT       : str   = "http://AIVA:11434"                   # [EXTRINSIC] GCE server endpoint
            COGNITIVE_ENGINE      : str   = "HammerAI/mn-mag-mell-r1:12b-q4_K_M"  # [EXTRINSIC] GCE model identifier
            RESPONSE_DEPTH        : int   = 512                                   # [INTRINSIC] maximum response tokens per inference
            TEMPERATURE           : float = 0.7                                   # [INTRINSIC] response creativity — adapts per cognitive state
            PROBABILITY_THRESHOLD : float = 0.9                                   # [INTRINSIC] cumulative probability cutoff for token sampling
            CANDIDATE_THRESHOLD   : int   = 40                                    # [INTRINSIC] maximum candidate tokens considered per step
            PERSEVERATION_DAMPING : float = 1.15                                  # [INTRINSIC] suppresses repetition of already-generated tokens
            HABITUATION_DAMPING   : float = 0.1                                   # [INTRINSIC] suppresses tokens proportional to their frequency
            NOVELTY_BIAS          : float = 0.1                                   # [INTRINSIC] bias toward introducing new topics — penalizes already-mentioned concepts
            TIMEOUT               : float = 60.0                                  # [STATIC]    seconds before abandoning inference
            COGNITIVE_RESPONSE    : str   = "/gce/response"                       # [STATIC]    GCE response topic
            SYSTEM_PROMPT         : str   = """You are GRACE — Generative Reasoning Agentic Cognitive Entity.
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
- Put an emoji reflecting your emotions and feelings in front of your conversation

Current date: {date}
/no_think
"""                                                                      # [EXTRINSIC] Grace system prompt — persona and cognitive rules

        class SMC:                                      # Semantic Memory Cortex
            ENCODING_ENGINE: str        = "BAAI/bge-base-en-v1.5" # [STATIC] encoding engine for semantic memory
            ENCODING_DIM: int           = 768           # [STATIC]  dimensionality of the encoding vectors from the encoding engine

        class EMC:                                      # Episodic Memory Cortex
            BINDING_STREAM_LIMIT: int = 512             # [INTRINSIC] max unencoded PMTs in binding stream — OOM guard on encoding engine failure
            ENCODING_ENGINE: str        = "BAAI/bge-base-en-v1.5"  # [STATIC] encoding engine for episodic memory
            ENCODING_CUE_PREFIX: str    = "Represent this sentence for searching relevant passages  : " # [STATIC] prompt prefix for encoding cues
            ENCODING_ENGRAM_PREFIX: str = ""            # [STATIC] prompt prefix for engrams (no prefix for storage)
            ENCODING_CYCLE_TIMEOUT: float = 30.0        # [INTRINSIC] time period for theta rhythm to wake up (Currently not used, triggered only with new PMT)
            ENCODING_DIM: int             = 768         # [STATIC]    dimensionality of the encoding vectors from the encoding engine
                                                        # TODO: for future use when implementing GPU-accelerated similarity search with FAISS, Annoy, etc.
            ENCODING_PRIME_CAPACITY: int  = 256         # [INTRINSIC] maximum entries in encoding engine's LRU prime
            ENCODING_PRIME_KEY_LIMIT: int = 256         # [INTRINSIC] maximum characters hashed per prime key in encoding engine
            
            EPISODE_CONTENT_LIMIT: int  = 6000          # [INTRINSIC] maximum character length of a PMT bound into episodic buffer
            
            THETA_INTERVAL: float       = 2.0           # [INTRINSIC] seconds — periodic theta rhythm fallback for continuous sensor input
            THETA_BATCH_LIMIT: int      = 32            # [INTRINSIC] max traces encoded per rhythm — prevents spike on crash recovery
            
            RECALL_RESERVE: int         = 1024          # [INTRINSIC] cortical capacity reserved for episodic recall
            RECALL_SURFACE_LIMIT: int   = 5             # [INTRINSIC] maximum number of episodes surfaced per turn (final RRF output)
            RECALL_POOL: int            = 2             # [INTRINSIC] candidate pool multiplier — each recall path scores RECALL_SURFACE_LIMIT × RECALL_POOL episodes before fusion
            RECALL_DEPTH: int           = RECALL_SURFACE_LIMIT * RECALL_POOL  # [DERIVED] search depth passed to KNN and FTS5 (number of candidates per path)
            RECALL_TIMEOUT: float       = 2.0           # [INTRINSIC] timeout for recall operations (300M param embedding model on Orin Nano CPU)
                                                        # covers encode_query (~500-900ms) + KNN search
                                                        # TODO: increase to 3.0 if model is genuinely int4 quantized
            RECOVERY_BATCH_SIZE: int    = 50            # [INTRINSIC] max unencoded episodes loaded into binding stream per recovery batch
            RELEVANCE_THRESHOLD: float  = 0.25          # [INTRINSIC] minimum relevance score for an episode to be surfaced

        class WMC:                                      # Working Memory Cortex
            PMT_OVERHEAD: int       = 4                 # [STATIC]    overhead chunks per PMT for formatting and metadata
            PMT_SLOT_LIMIT: int     = 7                 # [INTRINSIC] maximum slot vacancy for PMTs (Miller's Law 7±2)
            PMT_SLOT_BUFFER: int    = 2                 # [INTRINSIC] PMT slot vacancy flexibility (Miller's Law ±2)
        
AGi.CNS.WMC.GLOBAL_CHUNK_LIMIT: int = (                 # [INTRINSIC] maximum number of chunks WMC can hold, will move to hrs.py
    AGi.CNS.CORTICAL_CAPACITY - 
    AGi.CNS.COGNITIVE_RESERVE - 
    AGi.CNS.EMC.RECALL_RESERVE
)
