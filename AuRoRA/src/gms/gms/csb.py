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


class SensoryModality(Enum):
    """Physical nature of the sensory stimulus."""
    TEXT    = "text"        # CLI, WebUI, messaging bridges
    AUDIO   = "audio"       # voice pipeline — TODO: M1.X
    VISION  = "vision"      # OAK-D camera — TODO: M2
    SPATIAL = "spatial"     # LiDAR, IMU — TODO: M2


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
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class SSS:
    # ── set by adapter ────────────────────────────────────────────────────────
    robot_id    :
    source      : SensoryInputChannel = SensoryInputChannel.CLI  # originating channel — always set by adapter
    modality    : SensoryModality     = SensoryModality.TEXT     # physical nature — derived from source by adapter
    trace_type  : TraceType           = TraceType.PMT            # WMC trace target — PMT or VST
    role        : str                 = "user"                   # speaker role — always "user" from external channels
    user_id     : str                 = "demo"                   # speaker identity — resolved by IRU at CNC
    text        : str                 = ""                       # text payload — CLI, WebUI, messaging bridges

    # ── set by CNC on arrival ─────────────────────────────────────────────────
    received_at : datetime            = field(                   # UTC wall-clock time CNC received this stimulus
                      default_factory=lambda: datetime.now(timezone.utc)
                  )

    # TODO M1.X: audio : bytes = b""  — voice pipeline (ASR)
    # TODO M2:   image : bytes = b""  — vision pipeline (OAK-D)


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
    # ── identity ──────────────────────────────────────────────────────────────
    user_id     : str | None = None                              # speaker identity — from SSS.user_id
    robot_id    : str        = ""                                # Grace instance identity — set by CNC from AGi.ROBOT_ID
    timestamp   : datetime   = field(                            # UTC induction time — drives recency decay in WMC
                                  default_factory=lambda: datetime.now(timezone.utc)
                              )

    # ── lifecycle ─────────────────────────────────────────────────────────────
    state       : WMCState   = WMCState.INDUCED                  # current WMC lifecycle phase

    # ── content ───────────────────────────────────────────────────────────────
    content     : str        = ""   # JSON pair {"user": "...", "assistant": "..."} — LLM context source of truth
    trace       : str        = ""   # plain text pair — EMC embedding and human-readable reinstatement
    trace_type  : TraceType  = TraceType.PMT

    # ── scoring ───────────────────────────────────────────────────────────────
    chunk_count     : int         = 0                            # cached token estimate — O(1) eviction math, no reprobe
    vector          : list[float] = field(default_factory=list)  # semantic embedding — set by MCC before WMC fill
    salience_score  : float       = 0.0                          # urgency signal inherited from SSS — importance of this exchange
    depth_score     : float       = 0.0                          # cosine sim to nearest static anchor — groundedness in known themes
    novelty_score   : float       = 0.0                          # 1.0 - depth — semantic distance from established themes
    retention_score : float       = 0.0                          # composite: salience * recency_decay * depth — WMC eviction priority key

    # ── flags ─────────────────────────────────────────────────────────────────
    anchored        : bool        = False   # hard-gated — protected from WMC eviction regardless of retention score
    cluster_id      : int         = -1      # assigned WMC semantic cluster — -1 = unassigned
    smc_candidate   : bool        = False   # flagged for Dream Cycle semantic consolidation — TODO: SMC M2


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
