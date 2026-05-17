# sts/msg/SSS.msg
# Sensory Stimulus Signal (SSS)
# =============================
# One complete sensory stimulus package — captures a single percept arriving
# from the physical world and prepares it for cognitive interpretation.
#
# Biological analogue: afferent signal arc —
#   Raw environmental energy is transduced by peripheral receptors into
#   electrochemical signals, relayed through sensory neurons to the thalamus,
#   and routed to the appropriate cortical region for processing. The brain
#   never receives raw stimuli — only encoded, relay-filtered representations.
#
#   stimulus → sensory receptors → sensory neurons → spinal cord/brainstem
#            → thalamus (relay + gating) → primary sensory cortex (perception)
#            → association cortex (interpretation) → prefrontal cortex (cognition)
#            → motor cortex → efferent neurons → muscles/glands → response
#
# In Grace:
#   Physical sensors transduce the environment into digital signals. The
#   Sensory Input Unit (SIU) encodes and packages each percept into an SSS,
#   which is routed to the Cognitive Node Controller (CNC) for triage and
#   modality classification. CNC acts as Grace's thalamus — gating, filtering,
#   and dispatching each stimulus into the appropriate WMC trace type.
#   Text and language stimuli become a Phonological Memory Trace (PMT).
#   Visual and spatial stimuli become a Visuospatial Trace (VST).
#   SSS is ephemeral — it is consumed by CNC and never enters WMC directly.
#
#   stimulus → SIU (transduction + encoding)
#            → CNC (triage + gating, thalamus analogue)
#            → PMT (phonological loop) | VST (visuospatial sketchpad)
#            → SCS cortices (cognition) → response
#
# Source: Sensory Input Unit (SIU)
# Target: Cognitive Node Controller (CNC)
#
# Topics: /<entity>/<system>/<source>/sts

# ── identity ──────────────────────────────────────────────────────────────────
string                  sss_id              # unique signal ID — e.g. "sss_a1b2c3"
string                  user_id             # user id AuRoRA is interacting with — e.g. "user_001"

int32                   interval            # time taken to generate this SSS in ms
int32                   sequence            # ordinal position in stimulus stream

# ── provenance ────────────────────────────────────────────────────────────────
string                  source              # originating system — e.g. "siu", "microphone", "camera"
string                  location            # physical origin — e.g. "microphone_left", "camera_front"
string                  modality            # sensory channel — e.g. "text", "audio", "vision", "touch"
string                  proc                # process that generated this SSS

# ── target ────────────────────────────────────────────────────────────────────
string                  target              # intended consumer — e.g. "cnc", "wmc"
string                  trace_type          # WMC trace to instantiate — "pmt" | "vst" | "ast"

# ── content ───────────────────────────────────────────────────────────────────
string                  raw                 # raw stimulus payload — text, path, or encoded data
string                  encoding            # payload encoding — e.g. "utf-8", "base64", "ros_image"
float64[]               vector              # pre-computed embedding if available — empty otherwise

# ── lifecycle ─────────────────────────────────────────────────────────────────
string                  state               # SSSState — e.g. "received", "triaged", "dispatched", "dropped"
builtin_interfaces/Time generated_at        # wall-clock time of generation
builtin_interfaces/Time queued_at           # wall-clock time of queuing
builtin_interfaces/Time fired_at            # wall-clock time of transmission
builtin_interfaces/Time received_at         # wall-clock time of reception
builtin_interfaces/Time processed_at        # wall-clock time of processing
builtin_interfaces/Time completed_at        # wall-clock time of completion
builtin_interfaces/Time depleted_at         # wall-clock time of depletion
builtin_interfaces/Time dropped_at          # wall-clock time of dropping
float64                 lifecycle           # time-elapsed since generation to depletion

string                  drop_reason         # why dropped — "below_threshold", "duplicate", "overload"

# ── scoring ───────────────────────────────────────────────────────────────────
float64                 urgency             # dispatch priority — higher preempts lower
float64                 confidence          # sensor confidence in this percept — 0.0 to 1.0

# ── flags ─────────────────────────────────────────────────────────────────────
bool                    internal            # true if AuRoRA-originated — not from external environment
bool                    requires_response   # true if stimulus expects a reply