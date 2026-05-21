# gms/gms/csb.py
# Circuit Substrate Blueprints (CSB)
# ===================================
# The genome of Grace — every circuit dataclass blueprint lives here.
# Biological analogue: the genome — centralized structural blueprints
# expressed by each system at runtime. No logic, no behaviour —
# pure structure, pure definition.
#
# Namespace hierarchy:
#   module-level — all blueprints defined at top scope
#
# Usage:
#   from gms.csb import PMT, SSS, WMCState
#   pmt = PMT()

from dataclasses import dataclass, field                # for generating the centralized structural blueprints
from enum import Enum                                   # for generating the enum base for various states

# ══════════════════════════════════════════════════════════════════════════════
# AGi — Autonomous General Intelligence
# Root namespace — all Grace circuit blueprints nested here.
# ══════════════════════════════════════════════════════════════════════════════

class SensoryInputChannel(Enum):
    CLI      = "cli"
    WEBUI    = "webui"
    VOICE    = "voice"
    TELEGRAM = "telegram"
    DISCORD  = "discord"
    GMAIL    = "gmail"

class SensoryModality(Enum):
    TEXT    = "text"    # CLI, webUI, messaging bridges
    AUDIO   = "audio"   # voice pipeline — (TODO: M1.X-b)
    VISION  = "vision"  # camera — OAK-D
    SPATIAL = "spatial" # LiDAR, IMU
    TOUCH   = "touch"   # tactile sensors — (TODO: M2+)

@dataclass
class SSS:
    """
    Sensory Stimulus Signal — one complete sensory stimulus package.
    Travels the full pre-WM pipeline as a single object.
    Adapters fill raw fields. CNC enriches in-place before forwarding to MCC.
    Biological analogue: afferent signal — transduced at periphery,
    enriched at thalamus, never enters WM directly.
    Lifecycle:
        Raw → Enriched → Triaged → Dispatched → Depleted | Dropped
    """
    # ── raw — filled by adapter ────────────────────────────────────
    sss_id          : str   = ""                                    # identifier of this SSS "SSS-<source>-<induced_at>-<uuid>"
    user_id         : str   = field(default="demo")                 # identifier of the user AuRoRA is interacting with
    robot_id        : str   = field(default="AuRoRA")               # identifier of the AuRoRA robot itself
    source          : SensoryInputChannel = SensoryInputChannel.CLI   # input modality — required
    role            : str   = field(default="user")                 # speaker role — "user" or "assistant"
    text            : str   = field(default="")                     # text payload — CLI, webUI, bridges
    audio           : bytes = field(default=b"")                    # audio payload — voice pipeline (TODO: M1.X-b)
    image           : bytes = field(default=b"")                    # image payload — vision pipeline (TODO: M2+)

    # ── enriched — filled by CNC ───────────────────────────────────
    sequence        : int   = 0     # ordinal position in stimulus stream
    modality        : SensoryModality = SensoryModality.TEXT  # "text" | "audio" | "vision" — derived from source
    trace_type      : str   = ""    # WMC trace target — "pmt" | "vst" | "ast"
    proc            : str   = ""    # process that enriched this SSS
    location        : str   = ""    # physical origin — e.g. "microphone_left"

    # ── scoring — filled by CNC ────────────────────────────────────
    urgency         : float = 0.0   # dispatch priority — higher preempts lower
    confidence      : float = 0.0   # sensor confidence — 0.0 to 1.0
    vector          : list[float] = field(default_factory=list)     # pre-computed embedding if available

    # ── lifecycle — filled by CNC ──────────────────────────────────
    state           : str   = ""    # SSSState — "raw" | "enriched" | "triaged" | "dispatched" | "dropped"
    generated_at    : str   = ""    # wall-clock time of generation
    queued_at       : str   = ""    # wall-clock time of queuing
    fired_at        : str   = ""    # wall-clock time of transmission
    received_at     : str   = ""    # wall-clock time of reception
    processed_at    : str   = ""    # wall-clock time of processing
    completed_at    : str   = ""    # wall-clock time of completion
    depleted_at     : str   = ""    # wall-clock time of depletion
    dropped_at      : str   = ""    # wall-clock time of dropping
    lifecycle       : float = 0.0   # elapsed ms from generation to depletion
    drop_reason     : str   = ""    # "below_threshold" | "duplicate" | "overload"

    # ── flags — filled by adapter or CNC ──────────────────────────
    internal        : bool  = False # true if GRACE-originated — not external environment
    requires_response: bool = False # true if stimulus expects a reply
    interval        : int   = 0     # ms to generate this SSS

class TraceType(Enum):
    PMT = "pmt"   # phonological memory trace — text/audio
    VST = "vst"   # visuospatial trace — vision/spatial
    AST = "ast"   # affective state trace — (TODO: M2+)

class WMCState(Enum):
    """
    State of a PMT in the working memory.
    """
    INDUCED    = "induced"                  # User prompt received but AI response not yet generated
    FILLED     = "filled"                   # AI response received and paired with user prompt
    SUSTAINED  = "sustained"                # PMT is sustained in working memory
    RECEDING   = "receding"                 # PMT is receding from working memory
    EVICTED    = "evicted"                  # PMT is evicted from working memory

@dataclass
class PMT:
    """
    Phonological Memory Trace — one complete interaction in working memory.
    Pairs a user prompt and AI response into a single evictable unit.
    Lifecycle:
        Induction → Filling → Sustaining → Receding → Evicting
    Biological analogue: a single episode held in the phonological loop,
    tagged by the hippocampus during experience for potential consolidation.
    """

    # ── identity ──────────────────────────────────────────────────
    trace_id        : str           = ""    # identifier of this PMT "PMT-<source>-<induced_at>-<uuid>
    user_id         : str | None    = None  # identifier of the user AuRoRA is interacting with
    robot_id        : str | None    = None  # identifier of the AuRoRA robot iteself
    source          : SensoryInputChannel = SensoryInputChannel.CLI   # input modality — required
    
    # ── lifecycle ─────────────────────────────────────────────────
    state           : WMCState      = WMCState.INDUCED    # WMCState value — induced/filled/sustained/receded/evicted
    proc            : str           = ""    # process name — which process generated this PMT

    # ── content ───────────────────────────────────────────────────
    user_prompt     : str           = ""    # raw user prompt — source of truth during staging
    ai_response     : str           = ""    # raw AI response — empty until pairing complete
    content         : str           = ""    # JSON pair — WMC chat history for LLM
    trace           : str           = ""    # formatted text — for EMC embedding and reinstatement
    trace_type      : TraceType     = TraceType.PMT  # formatted text — for EMC embedding and reinstatement
    chunk_count     : int           = 0     # cached token count — O(1) eviction math, no reprobe on eviction

    # ── scoring ───────────────────────────────────────────────────
    vector          : list[float]   = field(default_factory=list)   # semantic vector — reused at EMC binding, no re-inference
    retention_score : float         = 0.0   # composite induction score — WMC eviction priority key
    salience_score  : float         = 0.0   # Factor 1 — logged at eviction boundary
    novelty_factor  : float         = 1.0   # Factor 2 multiplier — inspectable at eviction boundary
    depth_score     : float         = 0.0   # Factor 3 — logged at eviction boundary

    # ── flags ─────────────────────────────────────────────────────
    anchored        : bool          = False # true if hard-gated — salience override or explicit marker
    smc_candidate   : bool          = False # true if fact extractor flagged remainder for Dream Cycle

@dataclass
class VST:
    """
    Visuospatial Trace — one complete perceptual snapshot in working memory.
    Pairs a sensor frame with scene interpretation into a single evictable unit.
    Lifecycle:
        Induction → Filling → Sustaining → Receding → Evicting
    Biological analogue: a single episode held in the visuospatial sketchpad,
    tagged by the hippocampus during experience for potential consolidation.
    """
    # ── identity ──────────────────────────────────────────────────
    sensor_id       : str | None    = None  # source sensor — 'oak_d', 'lidar', 'imu', None if fused
    timestamp       : str           = ""    # ISO wall-clock induction time — (TODO: M1.6 replaces with ROS2 time)
    interval        : int           = 0     # time taken to generate this VST in ms — (TODO: M1.6 replaces with ROS2 time)
    # ── lifecycle ─────────────────────────────────────────────────
    state           : WMCState      = WMCState.INDUCED  # WMCState value — induced/filled/sustained/receded/evicted
    proc            : str           = ""    # process name — which process generated this VST
    # ── content ───────────────────────────────────────────────────
    raw_frame       : str           = ""    # raw sensor input — base64 image, point cloud ref, or IMU reading
    interpretation  : str           = ""    # scene description — YOLO detections, depth map summary, spatial layout
    content         : str           = ""    # JSON pair — fused frame+interpretation for LLM
    trace           : str           = ""    # formatted text — for EMC embedding and reinstatement
    # ── spatial ───────────────────────────────────────────────────
    objects         : list[dict]    = field(default_factory=list)   # detected objects — [{label, confidence, bbox, depth}]
    spatial_map     : str           = ""    # occupancy or spatial layout summary — LiDAR derived
    pose            : list[float]   = field(default_factory=list)   # GRACE pose at capture — [x, y, z, yaw]
    # ── scoring ───────────────────────────────────────────────────
    chunk_count     : int           = 0     # cached token count — O(1) eviction math, no reprobe on eviction
    vector          : list[float]   = field(default_factory=list)   # semantic vector — reused at EMC binding, no re-inference
    retention_score : float         = 0.0   # composite induction score — VSS eviction priority key
    salience_score  : float         = 0.0   # Factor 1 — logged at eviction boundary
    novelty_factor  : float         = 1.0   # Factor 2 multiplier — inspectable at eviction boundary
    depth_score     : float         = 0.0   # Factor 3 — logged at eviction boundary
    # ── flags ─────────────────────────────────────────────────────
    anchored        : bool          = False # true if hard-gated — salience override or explicit marker
    smc_candidate   : bool          = False # true if scene flagged for Dream Cycle consolidation