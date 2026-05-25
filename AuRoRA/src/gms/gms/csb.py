# gms/gms/csb.py
# Circuit Substrate Blueprints (CSB)
# ===================================
# AuRoRA · Grace Motor System (GMS)
#
# The genome of Grace — every circuit dataclass blueprint lives here.
# Biological analogue: the genome — centralized structural blueprints
# expressed by each system at runtime.
# No logic, no behaviour — pure structure, pure definition.
#
# Dataclass hierarchy:
#   SSS  — Sensory Stimulus Signal   — afferent input, TIC/ASR/CV → CNC
#   CRS  — Cognitive Response Signal — efferent output, GCE → MCC
#   PMT  — Phonological Memory Trace — WMC episode unit, SSS+CRS paired
#   VST  — Visuospatial Trace        — WMC perceptual unit, TODO: M2

from dataclasses import dataclass, field    # centralized structural blueprints expressed by each system at runtime
from datetime import datetime, timezone     # UTC timestamps — SSS reception time and PMT induction time
from enum import Enum                       # enum base for channel, modality, trace type, and WMC lifecycle states


# ══════════════════════════════════════════════════════════════════════════════
# Enums
# ══════════════════════════════════════════════════════════════════════════════

class SensoryInputChannel(Enum):
    """Originating channel of an incoming sensory stimulus."""
    CLI      = "cli"        # terminal input — developer tool
    WEBUI    = "webui"      # browser UI — TIC websocket
    VOICE    = "voice"      # ASR pipeline — TODO: M1.X
    TELEGRAM = "telegram"   # Telegram bridge — TODO: M2
    DISCORD  = "discord"    # Discord bridge — TODO: M2
    GMAIL    = "gmail"      # Gmail bridge — TODO: M2
    UNKNOWN  = "unknown"    # unknown channel — should not happen

class SensoryModality(Enum):
    TEXT    = "text"    # CLI, webUI, messaging bridges
    AUDIO   = "audio"   # voice pipeline — (TODO: M1.X-b)
    VISION  = "vision"  # camera — OAK-D
    SPATIAL = "spatial" # LiDAR, IMU
    TOUCH   = "touch"   # tactile sensors — (TODO: M2+)
    UNKNOWN = "unknown" # unknown modality — should not happen

class TraceType(Enum):
    """Target WMC trace type for a given stimulus."""
    PMT = "pmt"             # phonological memory trace — text / transcribed audio
    VST = "vst"             # visuospatial trace — vision / spatial — TODO: M2

class WMCState(Enum):
    """Lifecycle phase of a memory trace inside the Working Memory Cortex."""
    INDUCED   = "induced"   # SSS received — awaiting CRS pairing
    FILLED    = "filled"    # CRS paired — complete episode
    SUSTAINED = "sustained" # held in WMC slot
    RECEDING  = "receding"  # approaching eviction threshold
    EVICTED   = "evicted"   # removed from WMC — forwarded to EMC


# ══════════════════════════════════════════════════════════════════════════════
# SSS — Sensory Stimulus Signal
# ══════════════════════════════════════════════════════════════════════════════
# Afferent signal: TIC / ASR / CV → CNC only.
# Carries raw input from any sensory channel to the thalamic relay.
# Never enters WMC directly — CNC extracts what MCC needs.
#
# Biological analogue: peripheral afferent signal enriched at the thalamus
# before being forwarded to cortex. The thalamus (CNC) never lets raw
# peripheral signals touch memory directly.
#
# Field responsibility:
#   source, modality, trace_type, role, user_id, text  — set by adapter (TIC/ASR/CV)
#   received_at                                        — set by CNC on arrival
#   salience, valence, arousal                         — set by TIC._buffer(), refined by CNC
#   efference_suppressed                               — set by TIC._heuristic_gate()
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class SSS:
    """
    Sensory Stimulus Signal — one complete sensory stimulus package.
    Travels the full pre-WM pipeline as a single object.
    Adapters fill raw fields. CNC enriches in-place before forwarding to MCC.
    Biological analogue: afferent signal — transduced at periphery,
    enriched at thalamus, never enters WM directly.

    Lifecycle states:
        generated → transduced → triaged → buffered → dispatched → depleted | discarded
    """

    # ── identity — set by adapter ─────────────────────────────────────────────
    sss_id      : str                 = ""                           # "SSS-<source>-<generated_at>-<uuid>"
    robot_id    : str                 = "AuRoRA"                     # Grace instance identity — set by adapter
    user_id     : str                 = "demo"                       # speaker identity — resolved by IRU at CNC
    role        : str                 = "user"                       # speaker role — always "user" from external channels

    # ── sensory — set by adapter ──────────────────────────────────────────────
    source      : SensoryInputChannel = SensoryInputChannel.UNKNOWN  # originating channel — always set by adapter
    modality    : SensoryModality     = SensoryModality.UNKNOWN      # physical nature — derived from source by adapter
    trace_type  : TraceType           = TraceType.PMT                # WMC trace target — PMT or VST
    text        : str                 = ""                           # text payload — CLI, WebUI, messaging bridges
    # TODO M1.X: audio : bytes = b""  — voice pipeline (ASR)
    # TODO M2:   image : bytes = b""  — vision pipeline (OAK-D)

    # ── affective scoring — set by TIC._buffer(), refined by CNC ─────────────
    # Biological analogue: amygdala pre-tags signals with valence/arousal before
    # cortical processing. High-arousal signals route through a fast pathway.
    salience    : float               = 1.0    # dispatch priority weight — higher = more urgent; habituated signals decay toward 0.0
    valence     : float               = 0.0    # affective tone — negative (threat/error) to positive (praise/success); range [-1.0, 1.0]
    arousal     : float               = 0.0    # activation intensity — calm to urgent; range [0.0, 1.0]
    fast_path   : bool                = False  # True = bypass normal queue, alert CNS immediately (high arousal threshold)

    # ── efference copy — set by TIC._heuristic_gate() ────────────────────────
    # Biological analogue: motor efference copy — brain suppresses predicted
    # sensory consequences of self-generated actions to prevent self-tickling.
    efference_suppressed : bool       = False  # True = signal matched a CNC-published efference echo; suppressed at periphery

    # ── lifecycle — set at each pipeline stage ────────────────────────────────
    state           : str             = ""     # "generated" | "transduced" | "triaged" | "buffered" | "dispatched" | "depleted" | "discarded"
    locus           : str             = ""     # which component is currently processing this SSS
    generated_at    : str             = ""     # wall-clock time of SSS instantiation (adapter boundary)
    transduced_at   : str             = ""     # wall-clock time of transduction gate pass
    buffered_at     : str             = ""     # wall-clock time of sensory buffer entry
    triggered_at    : str             = ""     # wall-clock time of CNS dispatch
    received_at     : str             = ""     # wall-clock time of CNC receipt
    depleted_at     : str             = ""     # wall-clock time of full lifecycle completion
    dropped_at      : str             = ""     # wall-clock time of discard (any gate)
    drop_reason     : str             = ""     # "below_threshold" | "duplicate" | "overload" | "efference_echo" | "injection"
    lifecycle_ms    : float           = 0.0    # elapsed ms from generation to depletion


# ══════════════════════════════════════════════════════════════════════════════
# CRS — Cognitive Response Signal
# ══════════════════════════════════════════════════════════════════════════════
# Efferent signal: GCE → MCC only.
# Grace's generated output — efference copy for memory encoding.
# Not a sensory signal — never originates from outside Grace.
#
# Biological analogue: efference copy — motor cortex sends a copy of the
# outgoing command back to sensory cortex and hippocampus so the brain can
# distinguish self-generated output from external input, and close the
# episode for consolidation.
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class CRS:
    # ── set by CNC after GCE stream completes ─────────────────────────────────
    text        : str       = ""             # full generated response — assembled from GCE stream fragments
    trace_type  : TraceType = TraceType.PMT  # always PMT for conversational output

    # ── set by CNC ────────────────────────────────────────────────────────────
    generated_at : datetime = field(         # UTC wall-clock time GCE stream completed
                       default_factory=lambda: datetime.now(timezone.utc)
                   )


# ══════════════════════════════════════════════════════════════════════════════
# PMT — Phonological Memory Trace
# ══════════════════════════════════════════════════════════════════════════════
# One complete interaction episode in working memory.
# Pairs SSS + CRS into a single evictable unit.
# Constructed by MCC — never by CNC or WMC directly.
#
# Biological analogue: a single episode held in the phonological loop,
# tagged by the hippocampus during experience for potential consolidation
# into long-term episodic memory.
#
# Lifecycle:
#   Induction → Filling → Sustaining → Receding → Evicting → EMC
# ══════════════════════════════════════════════════════════════════════════════

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
    robot_id        : str | None    = None  # identifier of the AuRoRA robot itself
    source          : SensoryInputChannel = SensoryInputChannel.CLI   # input modality — required

    # ── lifecycle ─────────────────────────────────────────────────
    state           : WMCState      = WMCState.INDUCED    # WMCState value — induced/filled/sustained/receded/evicted
    proc            : str           = ""    # process name — which process generated this PMT

    # ── content ───────────────────────────────────────────────────
    user_prompt     : str           = ""    # raw user prompt — source of truth during staging
    ai_response     : str           = ""    # raw AI response — empty until pairing complete
    content         : str           = ""    # JSON pair — WMC chat history for LLM
    trace           : str           = ""    # formatted text — for EMC embedding and reinstatement
    trace_type      : TraceType     = TraceType.PMT  # trace classification — PMT for conversational episodes
    generated_at    : datetime      = field(default_factory=lambda: datetime.now(timezone.utc))   # UTC timestamp — set by CNC when PMT is inducted

    # ── scoring ───────────────────────────────────────────────────
    vector          : list[float]   = field(default_factory=list)   # semantic vector — reused at EMC binding, no re-inference
    retention_score : float         = 0.0   # composite induction score — WMC eviction priority key
    salience_score  : float         = 0.0   # Factor 1 — logged at eviction boundary
    novelty_factor  : float         = 1.0   # Factor 2 multiplier — inspectable at eviction boundary
    depth_score     : float         = 0.0   # Factor 3 — logged at eviction boundary

    # ── flags ─────────────────────────────────────────────────────────────────
    anchored        : bool          = False # hard-gated — protected from WMC eviction regardless of retention score
    cluster_id      : int           = -1    # assigned WMC semantic cluster — -1 = unassigned
    smc_candidate   : bool          = False # flagged for Dream Cycle semantic consolidation — TODO: SMC M2


# ══════════════════════════════════════════════════════════════════════════════
# VST — Visuospatial Trace
# ══════════════════════════════════════════════════════════════════════════════
# One complete perceptual snapshot in working memory.
# Pairs sensor frame + scene interpretation into a single evictable unit.
# Mirrors PMT structure for consistent WMC eviction logic.
#
# Biological analogue: a single episode held in the visuospatial sketchpad,
# tagged by the hippocampus for potential spatial memory consolidation.
#
# TODO: M2 — constructed when CV/LiDAR pipeline lands.
#            Scoring fields mirror PMT intentionally — WMC eviction logic is shared.
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class VST:
    # ── identity ──────────────────────────────────────────────────────────────
    sensor_id   : str | None  = None    # source sensor — "oak_d", "lidar", "imu", None if fused
    robot_id    : str         = ""      # Grace instance identity — set by CNC from AGi.ROBOT_ID
    timestamp   : datetime    = field(default_factory=lambda: datetime.now(timezone.utc))

    # ── lifecycle ─────────────────────────────────────────────────────────────
    state       : WMCState    = WMCState.INDUCED

    # ── content ───────────────────────────────────────────────────────────────
    raw_frame       : str         = ""                           # base64 image, point cloud ref, or IMU reading
    interpretation  : str         = ""                           # YOLO detections, depth map summary, spatial layout
    content         : str         = ""                           # JSON pair — fused frame+interpretation for LLM
    trace           : str         = ""                           # formatted text — EMC embedding and reinstatement
    objects         : list[dict]  = field(default_factory=list)  # [{label, confidence, bbox, depth}]
    spatial_map     : str         = ""                           # occupancy/spatial layout summary — LiDAR derived
    pose            : list[float] = field(default_factory=list)  # Grace pose at capture [x, y, z, yaw]

    # ── scoring ───────────────────────────────────────────────────────────────
    chunk_count     : int         = 0
    vector          : list[float] = field(default_factory=list)
    salience_score  : float       = 0.0
    depth_score     : float       = 0.0
    novelty_score   : float       = 0.0
    retention_score : float       = 0.0

    # ── flags ─────────────────────────────────────────────────────────────────
    anchored        : bool        = False
    cluster_id      : int         = -1
    smc_candidate   : bool        = False                        # flagged for Dream Cycle spatial consolidation — TODO: SMC M2
