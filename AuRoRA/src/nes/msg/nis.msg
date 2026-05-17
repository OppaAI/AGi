# nes/msg/nis.msg
# Neural Impulse Signal (NIS)
# ============================
# Universal signal unit — carries any structured impulse between any two
# components in Grace's nervous system. Not specific to sensory input.
#
# Biological analogue: action potential —
#   A neuron fires an all-or-nothing electrical impulse along its axon
#   regardless of whether the signal encodes pain, motor intent, a recalled
#   memory, or an internal prediction. The action potential format is fixed;
#   only the pathway, target, and payload differ. Meaning is determined by
#   the receiving neuron, not the impulse itself.
#
# AGi architecture:
#   NIS is the universal signal currency of the nervous system. Any component
#   that needs to communicate with another does so by emitting a NIS. The
#   signal carries a typed payload, a declared direction, and enough routing
#   metadata for any receiver to act without needing knowledge of the emitter.
#
#   Examples of NIS in practice:
#     SIU    → CNC   : sensory percept (afferent)
#     CNC    → WMC   : routed trace instantiation
#     CNC    → EMC   : memory recall request
#     EMC    → CNC   : recall result (efferent return)
#     SCS    → CNC   : cognitive output or decision
#     CNC    → motor : motor command (efferent)
#     CNC    → CNC   : internal self-generated impulse
#
# Source: any AGi component
# Target: any AGi component
#
# Topics: /<entity>/<system>/<source>/nis

# ── identity ──────────────────────────────────────────────────────────────────
string                  nis_id              # unique impulse ID — e.g. "nis_a1b2c3"
string                  user_id             # active user context — e.g. "user_001"; empty if system-internal
string                  session_id          # cognitive session this impulse belongs to

int32                   sequence            # ordinal position in the impulse stream for this session
int32                   interval            # time elapsed since previous impulse in this stream, in ms

# ── routing ───────────────────────────────────────────────────────────────────
string                  axon                # component that fired this impulse — e.g. "siu", "cnc", "emc"
string                  dendrite            # intended target component — e.g. "cnc", "wmc", "motor"
string                  pathway             # named signal tract — e.g. "afferent", "efferent", "recurrent", "internal"
string                  direction           # signal direction — "ascending" | "descending" | "lateral" | "recurrent"

# ── classification ────────────────────────────────────────────────────────────
string                  signal_type         # what this impulse carries — see NISType
                                            # e.g. "percept" | "recall_request" | "recall_result"
                                            #      | "cognitive_output" | "motor_command" | "internal"
string                  modality            # sensory or cognitive channel if applicable — e.g. "text", "vision", "none"
string                  trace_type          # WMC trace to instantiate if receiver is WMC — "pmt" | "vst" | "" 

# ── payload ───────────────────────────────────────────────────────────────────
string                  raw                 # signal payload — text, path, serialised struct, or encoded data
string                  encoding            # payload encoding — e.g. "utf-8", "base64", "json", "ros_image"
string                  schema              # payload schema identifier if structured — e.g. "recall_cue_v1", ""
float64[]               vector              # pre-computed embedding if available — empty otherwise

# ── scoring ───────────────────────────────────────────────────────────────────
float64                 urgency             # routing priority — higher preempts lower; 0.0 to 1.0
float64                 amplitude           # signal strength relative to threshold — 0.0 to 1.0
float64                 confidence          # emitter confidence in this payload — 0.0 to 1.0

# ── lifecycle ─────────────────────────────────────────────────────────────────
string                  state               # NISState — "fired" | "received" | "integrated" | "dropped"
builtin_interfaces/Time fired_at            # wall-clock time impulse was emitted
builtin_interfaces/Time received_at         # wall-clock time impulse arrived at receiver
builtin_interfaces/Time integrated_at       # wall-clock time receiver finished processing
builtin_interfaces/Time dropped_at          # wall-clock time of drop if applicable
float64                 latency             # time from fired_at to integrated_at in ms

string                  drop_reason         # why dropped — "below_threshold" | "refractory" | "overflow" | "timeout"

# ── flags ─────────────────────────────────────────────────────────────────────
bool                    internal            # true if Grace-originated — not from external environment
bool                    requires_response   # true if emitter expects a return NIS
bool                    refractory          # true if this pathway is in suppression — receiver should discard
string                  correlation_id      # links return NIS to its originating request — e.g. matches nis_id of request