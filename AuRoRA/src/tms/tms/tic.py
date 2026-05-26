"""
Telepathy Input Core (TIC)
==========================
System: Telepathy Management System (TMS)

This module is the afferent pathway for remote telepathic communication. It receives raw input 
from peripheral interfaces (WebUI, messaging apps, email, etc.), gates and normalizes it into a
structured Sensory Stimulus Signal (SSS), and transmits the SSS to the CNS via the Neural Gateway.

Biological analogue: Peripheral Nervous System (PNS) — transduction at the sensory organ,
local reflex handling, and afferent transmission to the thalamus.

Architecture:
    The TIC operates as a linear, unidirectional pipeline. Signals enter from decoupled 
    physical adapters, pass through structural filtration checkpoints, materialize inside 
    the buffer, and are cast onto the CNS neural bus.

[Telepathy Domain]
Stimulus (Raw user input)
        │
        ▼
SSS GENERATED
[RECEIVED]
        │
        ▼
[Transduction Threshold Gate] ──── (pass) ──────────────► [Heuristic Intercept Gate]
        │                                                            │
 (fail: discard)                                   (intercept: handle locally)
        │                                            (efference echo: suppress)
   [DISCARDED]                                                      │
                                                                 (pass)
                                                                    │
                                                                    ▼
                                                           [Sensory Buffer]
                                                           HOLD / EXTRACT / CONSOLIDATE
                                                           AFFECTIVE TAG
                                                           [BUFFERED]
                                                                    │
                                                                    ▼
            [SCS Processing Loop] ◄────────────────────────────[Neural Gateway]
                    │                                          [DISPATCHED]
                    │                (SCS response)
                    └────────────────────────────────────────► [Sensory Buffer]        ← stage 6 (planned)
                                                               [INTEGRATED]
                                                                     │
                                                                     ▼
                                                               [Neural Pathway]        ← stage 7 (planned)
                                                               [DEPLETED]
Adapters:
    Current: CLI, Web UI
    Planned: Telegram, Discord, Email, Teleops

Lifecycle:
    1. Reception
       Raw input arrives through a peripheral adapter. SSS is instantiated
       and normalized into structured schema, with raw information populated — 
       the moment external stimulus becomes an internal neural event.
       SSS marked GENERATED.

    2. Transduction Threshold Gate
       Determines whether the physical signal meets the minimum threshold
       to propagate. Failures are immediately discarded.
       Rejects: null/None, empty strings, whitespace-only, corrupted bytes.
       SSS marked TRANSDUCED on pass, DISCARDED on failure.

    3. Heuristic Intercept Gate
       Intercepts signals resolvable locally without burdening the CNS —
       peripheral equivalent of a spinal reflex loop.
       Intercepts: /commands, injection patterns, system signals (SIGINT,
       shutdown), duplicate inputs within debounce window, oversized payloads,
       efference echoes (CNC-predicted self-generated signals).
       SSS marked TRIAGED on pass, INTERCEPTED on intercept, DISCARDED on failure.

    4. Sensory Buffer
       Sensory register — analogous to iconic or echoic memory. Holds the
       signal briefly while features are extracted and the SSS is finalized
       for CNS transmission. Affective tagging (salience, valence, arousal)
       is applied here — fast amygdala-analogue pass before cortical dispatch.
       HOLD        — raw payload cached for downstream transmission
       EXTRACT     — structural primitives measured (length, density, capitals)
       AFFECT      — valence, arousal, salience derived from content signals
       CONSOLIDATE — raw trace compiled into encoded, standardized SSS
       SSS marked BUFFERED.

    5. SCS Transmission
       Consolidated SSS transmitted via Neural Gateway to SCS (Thalamic Loop)
       over the neural gateway. TIC holds the SSS in buffer awaiting
       response return — the interaction is not yet complete.
       High-arousal signals are flagged fast_path=True for priority dispatch.
       SSS marked DISPATCHED.

    6. Efferent Return
       Response generated from SCS returns via Neural Gateway and is bound to
       the originating SSS interaction ID, forming a complete interaction pair.
       SSS marked INTEGRATED.

    7. Interaction Dispatch
       Completed interaction pair transmitted via Neural Gateway for memory
       consolidation downstream. TIC's lifecycle ends here — SSS fully depleted.
       SSS marked DEPLETED.

Topics:
    Sub: TMS.EFFERENCE_ECHO (std_msgs/String) — CNC-published echo hashes for suppression
    Pub: TMS.TEXT_RESPONSE_GATEWAY (std_msgs/String) — normalized SSS JSON for SCS

Message schema (inbound from WebUI):
    {"text": "...", "user_id": "...", "role": "user", "source": "webui"}

Message schema (outbound SSS JSON):
    {"role": "user", "text": "...", "user_id": "...", "source": "webui",
     "trace_type": "pmt", "salience": 1.0, "valence": 0.0, "arousal": 0.0,
     "fast_path": false}

Terminology:
    Sensory Stimulus Signal (SSS) — the canonical internal event object that wraps a single inbound
                                    stimulus from reception through depletion. Instantiated during
                                    receiving, mutated at each pipeline stage via state/locus fields,
                                    and finally serialized for dispatch to Semantic Cognitive System (SCS).
                                    Defined in genomic substrates blueprint.
    Affective Tagging             — fast valence/arousal scoring applied before cortical dispatch;
                                    analogous to amygdala pre-processing of sensory signals
    Neural Gateway                — pathway for transmitting neural signals between subsystems.
    Response Echo                 — CNC-published prediction of expected output; suppressed at periphery
                                    to avoid processing own outputs as external stimuli.
    Stimulus Gateway              — pathway for carrying signals from sensory systems to SCS.
    Symbolic Injection            — text arrives pre-decoded, bypassing all sensory processing.
    Telepathy Gateway             — The term for direct symbolic input with no physical substrate.
    Teloreceptor Portal           — Websocket server connection for receiving symbolic input (WebUI).
"""

# System libraries
import websockets
import asyncio                                                              # async event loop — runs websocket server on its own thread, isolated from ROS2 spin
import hashlib                                                              # SHA-256 hashing for response echo imprints
import json                                                                 # serialize/deserialize message payloads at both boundaries
import threading                                                            # owns the websocket server thread — keeps asyncio loop off the ROS2 spin thread
import time                                                                 # wall-clock timestamps for debounce window and lifecycle markers
from datetime import datetime, timezone                                     # UTC timestamps for SSS lifecycle fields

# Third-party libraries
import rclpy                                                                # ROS2 Python client library — node lifecycle and spin
from rclpy.node import Node                                                 # base class for all ROS2 nodes
from std_msgs.msg import String                                             # ROS2 string message type for topic I/O
import websockets                                                           # async websocket server — WebUI connection layer
from websockets.exceptions import ConnectionClosedError, ConnectionClosedOK # clean disconnect vs unexpected drop — handled separately in _handle_connection

# AGi Libraries
from hrs.hrm import AGi                                                     # homeostatic regulation manifest namespace
from hrs.hru import hydrate_manifest                                        # binds AuRoRA parameter server values into AGi constants at boot
from gms.csb import SensoryInputChannel, SensoryModality, TraceType, SSS    # SSS dataclass + channel/trace enums

TMS = AGi.TMS                                                               # module-level alias — TMS-level constants (topic names, websocket config)

class TelepathyInputCore(Node):
    """
    Telepathy Input Core — Robot system node.

    Afferent pathway for telepathy symbolic stimulus.
    Transduces incoming telepathy signals into SSS, gates through threshold and heuristic filters,
    affectively tags, and transmits to the neural gateway for cortical processing.
    No cognitive logic — transduce, gate, tag, transmit only.
    Biological analogue: peripheral nervous system — sensory transduction, spinal reflex arcs, afferent projection to thalamus.
    """

    def __init__(self):
        """
        Initialize Telepathy Input Core.

        Establishes the neural gateways, connects to the telepathy portal via a dedicated
        peripheral thread, and arms the response echo gateway.
        Robot system operation cycle and teloreceptor never share a thread.
        """
        super().__init__("tic")                                                         # register this core with robot system
        self.get_logger().info("=" * 60)                                                # log heading title border
        self.get_logger().info("📡 TIC — Telepathy Input Core starting…")               # log the initialization of TIC
        self.get_logger().info("=" * 60)                                                # log heading title border

        hydrate_manifest(self, system="tms")                                            # hydrate manifest constants from parameter server

        # Neural Gateways infrastructure to communicate with other subsystems.
        self._stimulus_gateway = self.create_publisher(                                 # neural gateway to publish normalized SSS to SCS
            String, TMS.TEXT_STIMULUS_GATEWAY, 10,
        )
        
        # CNC publishes the SHA-256 imrpint of each outbound response text immediately
        # after GCE stream completes. Any inbound SSS whose text imprint matches within
        # the TTL window is suppressed as a self-generated echo.
        self._response_echo_gateway = self.create_subscription(                         # response echo receptor — subscribes to CNC outbound response imprints
            String, TMS.RESPONSE_ECHO_GATEWAY, self._on_response_echo, 10,
        )

        # Runtime states for processing stimuli.
        self._active_connections = set()                                                # live connections to teloreceptor to track for clean shutdown
        self._refractory_anchor = {}                                                    # to supress user repeating the exact same message within the refractory period
        self._response_echoes = {}                                                      # to supress system echoing its own responses over a short period of time per user

        # Open teloreceptor portal on its own neural thread — never competes with main robot system.
        self._teloreceptor_cycle = asyncio.new_event_loop()                             # isolated cycle to run teloreceptor
        self._teloreceptor_thread = threading.Thread(                                   # dedicated neural thread for teloreceptor
            target=self._teloreceptor_cycle.run_forever,                                # run the parallel neural cycle on its own thread — keeps it off the ROS2 spin thread
            name="tic-teloreceptor",                                                    # set the neural thread name for profilers
            daemon=True,                                                                # this thread dies with the main system — enables clean shutdown
        )
        self._teloreceptor_thread.start()                                               # ignite teloreceptor neural thread

        asyncio.run_coroutine_threadsafe(                                               # schedule server boot on websocket loop — crosses thread boundary safely
            self._ignite_telepathy_domain(), self._teloreceptor_cycle
        )

        self.get_logger().info(f"✅ Stimulus Gateway      : {TMS.TEXT_STIMULUS_GATEWAY}")                             # ROS2 topic for SSS published to SCS
        self.get_logger().info(f"✅ Response Echo Gateway : {TMS.TEXT_RESPONSE_GATEWAY}")                             # ROS2 topic for efference echoes
        self.get_logger().info(f"✅ Telepathy Domain via  : ws://{TMS.TELEPATHY_GATEWAY}:{TMS.TELORECEPTOR_PORTAL}")  # WebSocket server for WebUI connection
        self.get_logger().info("=" * 60)                                                                               # log heading title border
        self.get_logger().info("📡 TIC ready — peripheral pathway open")                                              # log ready status
        self.get_logger().info("=" * 60)                                                                               # log heading title border

    def _on_response_echo(self, response_echo: String) -> None:
        """
        Register a CNC-published response echo imprint for inbound signal suppression.
        Technical Note: ROS2 subscription callback — runs on the spin thread; dict write is GIL-safe for CPython.

        Args:
            response_echo (String): response echo in JSON schema — {"imprint": "<sha256hex>", "duration": <seconds>}
        """
        try:                                                                                                        # attempt to interpret response echo
            echo = json.loads(response_echo.data)                                                                   # deserialize echo imprint payload
            imprint = echo.get("imprint", "")                                                                       # SHA-256 hex digest of CNC outbound response
            echo_duration = float(echo.get("duration", TMS.RESPONSE_ECHO_DURATION))                                 # suppression window — falls back to module default
            if imprint:                                                                                             # if echo contains an imprint
                echo_expiry = time.monotonic() + echo_duration                                                      # absolute expiry time on monotonic clock
                self._response_echoes[imprint] = echo_expiry                                                        # register imprint with expiry duration
                self.get_logger().debug(f"🧠 Response echo registered: {imprint[:12]}… duration={echo_duration}s")  # log registry of the imprint and expiry duration
        except (json.JSONDecodeError, ValueError) as e:                                                             # malformed response echo payload
            self.get_logger().warning(f"⚠️  Response echo parse error: {e}")                                        # log parsing error and continue

    def _prune_response_echoes(self) -> None:
        """
        Prune expired response echo imprints from the registry.
        Technical Note: Called at the top of _heuristic_gate — O(n) scan, acceptable on a small dict.
        """
        current_time = time.monotonic()                                                                                     # current monotonic time — compared against imprint expiry
        echo_expired = [imprint for imprint, echo_expiry in self._response_echoes.items() if current_time > echo_expiry]    # collect stale imprints
        for imprint in echo_expired:                                                                                        # check for any expired imprints from registry
            del self._response_echoes[imprint]                                                                              # evict expired imprint from registry

    @staticmethod
    def _derive_imprint(content: str) -> str:
        """
        Derive an encoded imprint (SHA-256 hex) from normalized stimulus content.
        Normalization: strip whitespace, casefold — must match CNC imprint logic exactly.
        Technical note: casefold normalizes case — "Hello" and "hello" derive the same imprint intentionally.

        Args:
            content (str): stimulus content to derive an imprint.

        Returns:
            str: encoded imprint with SHA-256 hex digest.
        """
        normalized_content = content.strip().casefold()                                # normalize — strip whitespace, casefold for case-invariant matching
        return hashlib.sha256(normalized_content.encode("utf-8")).hexdigest()          # derive SHA-256 imprint — hex digest of normalized stimulus

    async def _ignite_telepathy_domain(self) -> None:
        """
        Ignite the teloreceptor and hold it open for the node lifetime.
        Runs entirely on the teloreceptor thread — never touches the robot system cycle.
        Technical note: blocks on wait_closed() — keeps the coroutine alive until explicit shutdown.
        """
        self._telepathy_domain = await websockets.serve(                                            # open websocket server — accepts connections from WebUI
            self._on_teloreceptor_link,                                                             # callback — fires on each new peripheral link
            TMS.TELEPATHY_GATEWAY,                                                                  # bind address — network interface for incoming stimulus
            TMS.TELORECEPTOR_PORTAL,                                                                # port — teloreceptor input channel
        )
        self.get_logger().info(f"✅ Telepathy Domain live on portal {TMS.TELORECEPTOR_PORTAL}")     # log live connection of telepathy domain
        await self._telepathy_domain.wait_closed()                                                  # suspend coroutine — telepathy domain stays active for node lifetime

    async def _on_teloreceptor_link(self, peripheral_link: websockets.ServerConnection) -> None:
        """
        Manage one peripheral link for its full lifetime.
        Each link opens an independent afferent channel — concurrent links never interfere.
        Technical note: async for loop blocks until the link closes — coroutine lives for link lifetime.
        
        Args:
            peripheral_link (websockets.ServerConnection): Active WebSocket connection from a peripheral adapter.
        """
        self._active_connections.add(peripheral_link)                           # register link — tracked for clean shutdown
        self.get_logger().info("🔗 Peripheral link established")                # log the connection of peripheral link

        try:                                                                    # attempt to connect telepathy domain
            async for raw_stimulus in peripheral_link:                          # iterate stimulus stream for this link lifetime
                sss = self._receive_stimulus(raw_stimulus)                      # stage 1 — instantiate SSS from raw stimulus
                if sss is None:                                                 # if no valid stimulus,
                    continue                                                    # skip the loop and wait for next stimulus

                sss = self._apply_transduction_gate(sss)                        # stage 2 — threshold check, discard below-minimum signals
                if sss is None:                                                 # if stimulus dropped by transduction gate,
                    continue                                                    # skip the loop and wait for next stimulus

                sss = self._apply_heuristic_gate(sss)                           # stage 3 — intercept locally resolvable signals
                if sss is None:                                                 # if stimulus dropped by heuristic gate,
                    continue                                                    # skip the loop and wait for next stimulus

                sss = self._buffer(sss)                                         # stage 4 — hold, extract, affective tag, consolidate
                self._publish_sss(sss)                                          # stage 5 — dispatch consolidated SSS to sensory gateway

                # TODO: stage 6 — response return (await SCS response, bind to SSS interaction ID)
                # TODO: stage 7 — interaction dispatch (publish completed SSS+CRS pair for memory consolidation)

        except ConnectionClosedOK:                                              # clean disconnect — normal lifecycle
            pass
        except ConnectionClosedError as e:                                      # unexpected disconnect — log but don't crash
            self.get_logger().warning(f"⚠️  Peripheral link disconnected unexpectedly: {e}")  # log the unexpected disconnect of peripheral link
        finally:
            self._active_connections.discard(peripheral_link)                   # deregister on any exit path
            self.get_logger().info("🔌 Peripheral link closed")                 # log the close of the peripheral link

    def _receive_stimulus(self, raw_stimulus: str) -> SSS | None:
        """
        Stage 1 — Reception.
        Receive raw stimulus from telepathy domain and instantiate Sensory Stimulus Signals (SSS) with filled info.
        The moment external stimulus becomes an internal neural event.
        SSS marked as GENERATED.

        Life Cycle:
              Raw Stimulus        ->    SSS (GENERATED)
            Telepathy Domain                 TIC

        Args:
            raw_stimulus (str): Raw stimulus in JSON schema received from telepathy domain.

        Returns:
            SSS | None: Freshly instantiated SSS, or None if JSON is unparseable.
        """
        try:
            stimulus = json.loads(raw_stimulus.strip())                         # parse raw JSON — reject non-JSON at the adapter boundary
            if not isinstance(stimulus, dict):                                  # non-dict JSON (array, string, etc.) — reject at boundary
                self.get_logger().debug(f"🚫 Stimulus rejected — invalid schema: {type(stimulus).__name__}")  # log the invalid stimulus
                return None                                                     # stop processing this stimulus

            generated_at = datetime.now(timezone.utc).isoformat()               # UTC birth timestamp — moment stimulus enters neural system
            return SSS(
                role         = stimulus.get("role", "user"),                    # speaker role — always "user" from UI
                content      = stimulus.get("content", ""),                     # raw stimulus content — not yet validated
                user_id      = stimulus.get("user_id", "demo"),                 # speaker identity — defaults to demo if not provided
                source       = SensoryInputChannel.UI.value,                    # adapter-stamped channel identity
                modality     = SensoryModality.TEXT.value,                      # trace type — PMT for all conversational input (TODO: M1.X - to add modality check when implement image input)
                trace_type   = TraceType.PMT.value,                             # adapter-stamped trace type (TODO: M1.X - to add VST when implement image input)
                state        = "GENERATED",                                     # lifecycle marker — SSS born
                locus        = "TIC:RECEPTION",                                 # processing locus — reception stage
                generated_at = generated_at,                                    # UTC birth timestamp — moment stimulus enters neural system
            )
        except (json.JSONDecodeError, ValueError) as e:                         # malformed JSON — drop at adapter boundary, never crash pipeline
            self.get_logger().debug(f"🚫 Stimulus rejected — malformed JSON: {e}")  # log the malformed JSON
            return None                                                         # stop processing this stimulus

    def _apply_transduction_gate(self, stimulus: SSS) -> SSS | None:
        """
        Stage 2 — Transduction Threshold Gate.
        Gate the stimulus — determine whether it meets minimum threshold to propagate.
        Rejects null, empty, whitespace-only, and corrupted byte payloads.
        SSS marked TRANSDUCED on pass, DISCARDED on failure.
        
        Life Cycle:
                SSS (GENERATED)    ->    SSS (TRANSDUCED) / SSS (DISCARDED)
                      TIC                              TIC
                    
        Args:
            stimulus (SSS): Freshly generated stimulus from reception stage.

        Returns:
            SSS | None: Stimulus marked TRANSDUCED, or None if below threshold.
        """
        content = stimulus.content.strip() if isinstance(stimulus.content, str) else ""      # guard against non-string payload — normalize to empty string

        if not content:                                                                      # null, empty, whitespace-only — below threshold
            stimulus.state       = "DISCARDED"                                               # lifecycle marker — signal dies here
            stimulus.locus       = "TIC:TRANSDUCTION"                                        # processing locus — transduction stage
            stimulus.drop_reason = "BELOW_THRESHOLD"                                         # discard reason — signal too weak to propagate
            stimulus.dropped_at  = datetime.now(timezone.utc).isoformat()                    # UTC timestamp — moment of discard
            self.get_logger().debug("🚫 Stimulus discarded — below transduction threshold")  # Log the discard of SSS
            return None                                                                      # stop processing this stimulus

        stimulus.content       = content                                                     # commit stripped content — normalized payload
        stimulus.state         = "TRANDUCED"                                                 # lifecycle marker — threshold passed
        stimulus.locus         = "TIC:TRANSDUCTION"                                          # processing locus — transduction stage
        stimulus.transduced_at = datetime.now(timezone.utc).isoformat()                      # UTC timestamp — moment of transduction
        return stimulus                                                                      # propagate SSS to heuristic gate

    def _apply_heuristic_gate(self, stimulus: SSS) -> SSS | None:
        """
        Stage 3 — Heuristic Intercept Gate.
        Intercept signals resolvable locally without burdening the CNS.
        Peripheral equivalent of a spinal reflex loop.
        SSS marked TRIAGED on pass, INTERCEPTED on intercept, DISCARDED on failure.
        Life Cycle:
                SSS (TRANSDUCED)    ->    SSS (TRIAGED) / SSS (INTERCEPTED) / SSS (DISCARDED)
                      TIC                                    TIC
        Intercepts (in order):
            1. Oversized payload     — discard before any pattern matching
            2. /commands             — local reflex handler (TODO)
            3. Injection patterns    — symbolic injection attempt
            4. Response echo         — CNC self-generated signal suppression
            5. Refractory period     — duplicate stimulus within refractory window
        
        Args:
            stimulus (SSS): Stimulus marked TRANSDUCED from transduction gate.
        Returns:
            SSS | None: Stimulus marked TRIAGED, or None if intercepted or discarded.
        """
        text       = sss.text
        text_lower = text.lower()
        now        = datetime.now(timezone.utc).isoformat()

        # prune stale efference echoes before any check — cheap housekeeping
        self._prune_response_echoes()

        # 1. oversized payload — discard before any pattern matching
        if len(text.encode("utf-8")) > _MAX_PAYLOAD_BYTES:
            sss.state       = "discarded"
            sss.locus       = "tic._heuristic_gate"
            sss.drop_reason = "overload"
            sss.dropped_at  = now
            self.get_logger().warning(f"⚠️  SSS discarded — payload exceeds {_MAX_PAYLOAD_BYTES}B ceiling")
            return None

        # 2. /command intercept — local reflex, never reaches CNS
        if text.startswith("/"):
            sss.state = "intercepted"
            sss.locus = "tic._heuristic_gate"
            self.get_logger().debug(f"🛑 SSS intercepted — command signal: {text[:40]}")
            # TODO: route /commands to local command handler
            return None

        # 3. injection pattern intercept — symbolic injection attempt, blocked at periphery
        for pattern in _INJECTION_PREFIXES:
            if text_lower.startswith(pattern):
                sss.state       = "intercepted"
                sss.locus       = "tic._heuristic_gate"
                sss.drop_reason = "injection"
                sss.dropped_at  = now
                self.get_logger().warning(f"🛑 SSS intercepted — injection pattern: '{pattern}'")
                return None

        # 4. efference echo suppression — CNC predicted this input; suppress as self-generated
        # Biological analogue: motor efference copy preventing self-tickling.
        # CNC publishes SHA-256 imprints of outbound responses to TMS.EFFERENCE_ECHO.
        # If inbound text imprint matches a live echo, the signal is suppressed here.
        imprint = self._derive_imprint(text)
        if imprint in self._efference_echoes:
            sss.state                = "intercepted"
            sss.locus                = "tic._heuristic_gate"
            sss.drop_reason          = "efference_echo"
            sss.dropped_at           = now
            sss.efference_suppressed = True
            self.get_logger().debug(f"🔇 SSS suppressed — efference echo match: {imprint[:12]}…")
            return None

        # 5. duplicate debounce — same payload from the same user within debounce window.
        # Keyed on user_id — distinct senders never suppress each other.
        # Biological analogue: sensory refractory period — organ cannot re-fire on identical
        # stimulus within recovery window.
        now_epoch = time.monotonic()
        last_text, last_time = self._refractory_anchor.get(sss.user_id, ("", 0.0))
        if text == last_text and (now_epoch - last_time) < _DEBOUNCE_WINDOW_S:
            sss.state       = "intercepted"
            sss.locus       = "tic._heuristic_gate"
            sss.drop_reason = "duplicate"
            sss.dropped_at  = now
            self.get_logger().debug("🔁 SSS intercepted — duplicate within debounce window")
            return None

        self._refractory_anchor[sss.user_id] = (text, now_epoch)                    # anchor per user — distinct senders never suppress each other

        sss.state = "triaged"                                                   # lifecycle marker — heuristic gate cleared
        sss.locus = "tic._heuristic_gate"
        return sss

    def _buffer(self, sss: SSS) -> SSS:
        """
        Stage 4 — Sensory Buffer.
        Sensory register — analogous to iconic or echoic memory.

        HOLD        — raw payload cached, SSS identity stamped
        EXTRACT     — structural primitives measured (length, density, capitals)
        AFFECT      — valence, arousal, salience derived from content signals
                      Biological analogue: amygdala pre-tagging before cortical routing.
                      Fast keyword scan + structural signals → affective dimensions.
                      High-arousal signals set fast_path=True for priority CNS dispatch.
        CONSOLIDATE — SSS finalized and encoded for CNS transmission

        SSS marked BUFFERED.

        Args:
            sss (SSS): SSS marked TRIAGED from heuristic gate.

        Returns:
            SSS: SSS marked BUFFERED, consolidated and ready for dispatch.
        """
        # HOLD — stamp buffer receipt time
        sss.locus      = "tic._buffer"
        sss.buffered_at = datetime.now(timezone.utc).isoformat()

        # EXTRACT — measure structural primitives
        text          = sss.text
        char_len      = len(text)
        word_count    = len(text.split())
        density       = word_count / char_len if char_len > 0 else 0.0         # lexical density — words per character
        capital_ratio = sum(1 for c in text if c.isupper()) / char_len if char_len > 0 else 0.0
        self.get_logger().debug(
            f"📊 SSS primitives — len={char_len} words={word_count} "
            f"density={density:.3f} capitals={capital_ratio:.3f}"
        )

        # AFFECT — derive valence, arousal, salience from fast keyword scan
        # Biological analogue: subcortical (amygdala) affective tagging before cortical awareness.
        # This is a coarse first pass — CNC may refine all three fields after semantic processing.
        words_lower: set[str] = set(text.lower().split())

        # arousal — urgency keyword hits + capital ratio (shouting signal)
        urgency_hits  = len(words_lower & TMS.URGENCY_MARKERS)
        arousal       = min(1.0, (urgency_hits * 0.25) + (capital_ratio * 0.5))

        # valence — positive vs negative keyword balance
        positive_hits = len(words_lower & TMS.POSITIVE_MARKERS)
        negative_hits = len(words_lower & TMS.NEGATIVE_MARKERS)
        raw_valence   = (positive_hits - negative_hits) * 0.3                  # each hit shifts ±0.3
        valence       = max(-1.0, min(1.0, raw_valence))                        # clamp to [-1.0, 1.0]

        # salience — base 1.0; boosted by arousal (urgency lifts priority)
        salience      = min(2.0, 1.0 + arousal)                                # range [1.0, 2.0] — CNC may apply further modulation

        sss.arousal  = round(arousal, 3)
        sss.valence  = round(valence, 3)
        sss.salience = round(salience, 3)

        # fast-path flag — high-arousal signals bypass normal queue at CNS
        sss.fast_path = arousal >= TMS._FAST_PATH_AROUSAL_THRESHOLD

        if sss.fast_path:
            self.get_logger().info(
                f"⚡ SSS fast-path — arousal={sss.arousal} valence={sss.valence} salience={sss.salience}"
            )
        else:
            self.get_logger().debug(
                f"💭 SSS affect — arousal={sss.arousal} valence={sss.valence} salience={sss.salience}"
            )

        # CONSOLIDATE — modality already set at reception; mark buffered
        sss.state = "buffered"
        return sss

    def _publish_sss(self, sss: SSS) -> None:
        """
        Stage 5 — SCS Transmission.
        Serialize consolidated SSS and publish to the sensory gateway topic.
        SSS marked DISPATCHED.

        Args:
            sss (SSS): SSS marked BUFFERED, ready for CNS dispatch.
        """
        try:
            sss.state        = "dispatched"
            sss.locus        = "tic._publish_sss"
            sss.triggered_at = datetime.now(timezone.utc).isoformat()

            signal = String()
            signal.data = json.dumps({                                          # serialize SSS fields — CNC deserializes back into SSS
                "role"        : sss.role,
                "text"        : sss.text,
                "user_id"     : sss.user_id,
                "source"      : sss.source.value,
                "trace_type"  : sss.trace_type.value,
                "state"       : sss.state,
                "salience"    : sss.salience,                                   # affective weight — CNC uses for dispatch priority
                "valence"     : sss.valence,                                    # affective tone — CNC may use for response modulation
                "arousal"     : sss.arousal,                                    # activation intensity — fast_path flag derived from this
                "fast_path"   : sss.fast_path,                                  # True = priority dispatch at CNC
                "generated_at": sss.generated_at,
                "triggered_at": sss.triggered_at,
            })
            self._stimulus_gateway.publish(signal)

            if sss.fast_path:
                self.get_logger().info(f"⚡ SSS dispatched (fast-path): {sss.text[:60]}…")
            else:
                self.get_logger().debug(f"📤 SSS dispatched: {sss.text[:60]}…")

        except Exception as e:
            self.get_logger().error(f"❌ Publish error: {e}")                   # non-fatal — publish failure must not crash the peripheral pathway

    # ── shutdown ──────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        """
        Gracefully shut down TIC — close websocket server and all active connections.
        """
        self.get_logger().info("🛑 TIC shutting down…")                             # log shutdown of TIC

        async def _close():                                                         # define a coroutine to close the websocket server
            for ws in list(self._active_connections):                               # iterate over a copy of the active connections
                await ws.close()                                                    # close each connection asynchronously
            if hasattr(self, "_ws_server"):                                         # if the websocket server exists
                self._telepathy_domain.close()                                             # close the websocket server
                await self._telepathy_domain.wait_closed()                                 # wait for the websocket server to close

        future = asyncio.run_coroutine_threadsafe(_close(), self._teloreceptor_cycle)          # run the close coroutine in the websocket loop
        try:                                                                        # try to close the websocket server
            future.result(timeout=3.0)                                              # wait for the close coroutine to complete with a timeout of 3.0 seconds
        except Exception:                                                           # handle exceptions
            pass                                                                    # ignore any exceptions raised by the close coroutine

        self._teloreceptor_cycle.call_soon_threadsafe(self._teloreceptor_cycle.stop)                      # stop the event loop
        self._teloreceptor_thread.join(timeout=3.0)                                           # wait for the thread to finish
        super().destroy_node()                                                      # destroy the ROS2 node
        self.get_logger().info("✅ TIC shutdown complete")


def main(args=None):                                            # entry point for ROS2 node
    rclpy.init(args=args)                                       # register node with ROS2
    node = TelepathyInputCore()                                 # instantiate Telepathy Input Core node
    try:                                                        # attempts to spin the node until Ctrl-C is pressed
        rclpy.spin(node)                                        # keep node alive - blocking
    except KeyboardInterrupt:                                   # catch Ctrl-C and shutdown gracefully
        node.get_logger().info("👋 Shutdown requested")         # log shutdown of TIC
    finally:                                                    # execute cleanup regardless of whether an exception was raised
        node.destroy_node()                                     # cleanup and unregister from ROS2
        rclpy.shutdown()                                        # unregister from ROS2 and shut down


if __name__ == "__main__":
    main()
