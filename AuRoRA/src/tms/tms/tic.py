"""
Telepathy Input Core (TIC)
==========================
System: Telepathy Management System (TMS)

This module is the afferent pathway for remote human communication. It receives raw input from
peripheral interfaces (WebUI, messaging apps, email, etc.), gates and normalizes it into a
structured Sensory Stimulus Signal (SSS), and transmits it to the CNS via the Neural Gateway.
Biological analogue: Peripheral Nervous System (PNS) — transduction at the sensory organ,
local reflex handling, and afferent transmission to the thalamus.

Architecture:
    The TIC operates as a linear, unidirectional pipeline. Signals enter from decoupled 
    physical adapters, pass through structural filtration checkpoints, materialize inside 
    the buffer, and are cast onto the CNS neural bus.

[Remote Adapter]
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
       efferent return — the interaction is not yet complete.
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
    Pub: TMS.TEXT_SENSORY_GATEWAY (std_msgs/String) — normalized SSS JSON for SCS

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
    Afferent Pathway              — carries signals toward the Semantic Cognitive System (SCS) (input direction)
    Symbolic Injection            — text arrives pre-decoded, bypassing all sensory processing
    Telepathy Channel             — The term for direct symbolic input with no physical substrate
    Efference Copy                — CNC-published prediction of expected input; suppressed at periphery
                                    to avoid processing own outputs as external stimuli
    Affective Tagging             — fast valence/arousal scoring applied before cortical dispatch;
                                    analogous to amygdala pre-processing of sensory signals
"""

# System libraries
import asyncio                                                              # async event loop — runs websocket server on its own thread, isolated from ROS2 spin
import hashlib                                                              # SHA-256 hashing for efference echo fingerprints
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
    Telepathy Input Core — ROS2 node.

    Afferent pathway for telepathy symbolic stimulus.
    Transduces incoming telepathy signals into SSS, gates through threshold and heuristic filters,
    affectively tags, and transmits to the neural gateway for cortical processing.
    No cognitive logic — transduce, gate, tag, transmit only.
    Biological analogue: peripheral nervous system — sensory transduction, spinal reflex arcs, afferent projection to thalamus.
    """

    def __init__(self):
        """
        Initialize Telepathy Input Core.

        Opens the neural gateway, arms the efference echo channel via the neural gateway.
        and ignites the WebSocket adapter on a dedicated afferent thread.
        ROS2 spin and WebSocket adapter never share a thread.
        """
        super().__init__("tic")                                                 # register this node with ROS2 as "tic"
        self.get_logger().info("=" * 60)
        self.get_logger().info("📡 TIC — Telepathy Input Core starting…")
        self.get_logger().info("=" * 60)

        hydrate_manifest(self, system="tms")                                    # hydrate manifest — binds TMS constants from AuRoRA parameter server

        self._active_connections: set = set()                                   # registry of live websocket connections — for clean shutdown

        # Debounce anchor keyed on user_id — prevents cross-user signal suppression.
        # Maps user_id → (last_text, monotonic_time). Distinct senders never suppress each other.
        self._last_payload: dict[str, tuple[str, float]] = {}

        # Efference echo registry — maps SHA-256 fingerprint → expiry epoch (monotonic).
        # CNC publishes expected output fingerprints here; TIC suppresses matching inbound signals.
        # Biological analogue: motor efference copy preventing self-tickling.
        self._efference_echoes: dict[str, float] = {}                           # fingerprint → expiry time (monotonic)

        # Sensory gateway — normalized SSS published here for CNC consumption
        self._sensory_gateway: rclpy.publisher.Publisher = self.create_publisher(
            String, TMS.TEXT_SENSORY_GATEWAY, 10                                # String type | topic | QoS depth 10
        )

        # Efference echo subscriber — CNC publishes fingerprints of its own outbound responses.
        # TIC suppresses inbound signals matching these fingerprints within the TTL window.
        # TODO: define TMS.EFFERENCE_ECHO topic name in hrm.py and aurora.yaml
        _efference_echo_topic = getattr(TMS, "EFFERENCE_ECHO", "tms/efference_echo")
        self._efference_echo_sub = self.create_subscription(
            String,
            _efference_echo_topic,
            self._on_efference_echo,
            10,
        )

        # Boot websocket server on its own thread — never competes with ROS2 spin
        self._ws_loop: asyncio.AbstractEventLoop = asyncio.new_event_loop()     # isolated event loop — owns the websocket server lifetime
        self._ws_thread: threading.Thread = threading.Thread(
            target=self._ws_loop.run_forever,
            name="tic-ws-server",
            daemon=True,                                                        # dies with main process — clean shutdown
        )
        self._ws_thread.start()                                                 # ignite websocket thread

        asyncio.run_coroutine_threadsafe(                                       # schedule server boot on ws loop — crosses thread boundary safely
            self._boot_ws_server(), self._ws_loop
        )

        self.get_logger().info(f"✅ Publishing  : {TMS.TEXT_SENSORY_GATEWAY}")
        self.get_logger().info(f"✅ Subscribing : {_efference_echo_topic}")
        self.get_logger().info(f"✅ WebSocket   : ws://0.0.0.0:{TMS.WS_PORT}")
        self.get_logger().info("=" * 60)
        self.get_logger().info("📡 TIC ready — afferent pathway open")
        self.get_logger().info("=" * 60)

    # ── efference copy ────────────────────────────────────────────────────────

    def _on_efference_echo(self, msg: String) -> None:
        """
        Receive a CNC-published efference echo fingerprint and register it for suppression.
        Called on the ROS2 spin thread — dict write is GIL-safe for CPython.

        CNC publishes the SHA-256 fingerprint of each outbound response text immediately
        after GCE stream completes. Any inbound SSS whose text fingerprint matches within
        the TTL window is suppressed as a self-generated echo.

        Args:
            msg (String): JSON payload — {"fingerprint": "<sha256hex>", "ttl": <seconds>}
        """
        try:
            data: dict = json.loads(msg.data)
            fingerprint: str = data.get("fingerprint", "")
            ttl: float = float(data.get("ttl", _EFFERENCE_ECHO_TTL_S))
            if fingerprint:
                expiry = time.monotonic() + ttl
                self._efference_echoes[fingerprint] = expiry
                self.get_logger().debug(f"🧠 Efference echo registered: {fingerprint[:12]}… TTL={ttl}s")
        except (json.JSONDecodeError, ValueError) as e:
            self.get_logger().warning(f"⚠️  Efference echo parse error: {e}")

    def _prune_efference_echoes(self) -> None:
        """
        Prune expired efference echo fingerprints from the registry.
        Called at the top of _heuristic_gate — cheap O(n) scan on a small dict.
        """
        now = time.monotonic()
        expired = [fp for fp, expiry in self._efference_echoes.items() if now > expiry]
        for fp in expired:
            del self._efference_echoes[fp]

    @staticmethod
    def _fingerprint(text: str) -> str:
        """
        Compute SHA-256 fingerprint of normalized text.
        Normalization: strip whitespace, casefold — matches CNC fingerprint logic.

        Args:
            text (str): Raw text to fingerprint.

        Returns:
            str: Hex-encoded SHA-256 digest.
        """
        normalized = text.strip().casefold()
        return hashlib.sha256(normalized.encode("utf-8")).hexdigest()

    # ── websocket server ──────────────────────────────────────────────────────

    async def _boot_ws_server(self) -> None:
        """
        Boot the websocket server and hold it for the node lifetime.
        Runs entirely on the tic-ws-server thread — never touches ROS2.
        """
        self._ws_server = await websockets.serve(                               # open websocket server — accepts connections from WebUI
            self._handle_connection,
            "0.0.0.0",
            TMS.WS_PORT,
        )
        self.get_logger().info(f"✅ WebSocket server live on port {TMS.WS_PORT}")
        await self._ws_server.wait_closed()                                     # hold server open until explicitly closed

    async def _handle_connection(self, websocket) -> None:
        """
        Handle one WebUI connection for its full lifetime.
        Each connection gets its own coroutine — connections are independent.

        Args:
            websocket: Active websocket connection from WebUI.
        """
        self._active_connections.add(websocket)                                 # register connection — tracked for clean shutdown
        self.get_logger().info("🔗 WebUI connected")

        try:
            async for raw_message in websocket:                                 # iterate messages for this connection lifetime
                sss = self._receive(raw_message)                                # stage 1 — instantiate SSS from raw input
                if sss is None:
                    continue

                sss = self._transduction_gate(sss)                              # stage 2 — threshold check, discard below-minimum signals
                if sss is None:
                    continue

                sss = self._heuristic_gate(sss)                                 # stage 3 — intercept locally resolvable signals
                if sss is None:
                    continue

                sss = self._buffer(sss)                                         # stage 4 — hold, extract, affective tag, consolidate
                self._publish_sss(sss)                                          # stage 5 — dispatch consolidated SSS to sensory gateway

                # TODO: stage 6 — efferent return (await SCS response, bind to SSS interaction ID)
                # TODO: stage 7 — interaction dispatch (publish completed SSS+CRS pair for memory consolidation)

        except ConnectionClosedOK:                        # clean disconnect — normal lifecycle
            pass
        except ConnectionClosedError as e:                # unexpected disconnect — log but don't crash
            self.get_logger().warning(f"⚠️  WebUI disconnected unexpectedly: {e}")
        finally:
            self._active_connections.discard(websocket)                         # deregister on any exit path
            self.get_logger().info("🔌 WebUI disconnected")

    # ── pipeline stages ───────────────────────────────────────────────────────

    def _receive(self, raw_message: str) -> SSS | None:
        """
        Stage 1 — Reception.
        Parse raw JSON from adapter and instantiate SSS with raw fields populated.
        The moment external stimulus becomes an internal neural event.
        SSS marked GENERATED.

        Args:
            raw_message (str): Raw JSON string from WebUI adapter.

        Returns:
            SSS | None: Freshly instantiated SSS, or None if JSON is unparseable.
        """
        try:
            payload: dict = json.loads(raw_message.strip())                     # parse raw JSON — reject non-JSON at the adapter boundary
            if not isinstance(payload, dict):
                return None

            now = datetime.now(timezone.utc).isoformat()
            return SSS(
                role         = payload.get("role", "user"),                     # speaker role — always "user" from WebUI
                text         = payload.get("text", ""),                         # raw stimulus content — not yet validated
                user_id      = payload.get("user_id", "demo"),                  # speaker identity — defaults to demo
                source       = SensoryInputChannel.WEBUI,                       # channel tag — always WEBUI for TIC
                modality     = SensoryModality.TEXT,                            # WebUI is always text modality
                trace_type   = TraceType.PMT,                                   # trace type — PMT for all conversational input
                state        = "generated",                                     # lifecycle marker — SSS born
                locus        = "tic._receive",                                  # processing locus — reception stage
                generated_at = now,                                             # wall-clock birth time
            )
        except (json.JSONDecodeError, ValueError):                              # malformed JSON — drop at adapter boundary
            return None

    def _transduction_gate(self, sss: SSS) -> SSS | None:
        """
        Stage 2 — Transduction Threshold Gate.
        Determines whether the physical signal meets minimum threshold to propagate.
        Rejects null, empty, whitespace-only, and corrupted byte payloads.
        SSS marked TRANSDUCED on pass, DISCARDED on failure.

        Args:
            sss (SSS): Freshly generated SSS from reception stage.

        Returns:
            SSS | None: SSS marked TRANSDUCED, or None if below threshold.
        """
        text = sss.text.strip() if isinstance(sss.text, str) else ""           # guard against non-string payload

        if not text:                                                            # null, empty, or whitespace-only — below threshold
            sss.state       = "discarded"
            sss.locus       = "tic._transduction_gate"
            sss.drop_reason = "below_threshold"
            sss.dropped_at  = datetime.now(timezone.utc).isoformat()
            self.get_logger().debug("🚫 SSS discarded — below transduction threshold")
            return None

        sss.text          = text                                                # commit stripped text — normalized payload
        sss.state         = "transduced"                                        # lifecycle marker — threshold passed
        sss.locus         = "tic._transduction_gate"
        sss.transduced_at = datetime.now(timezone.utc).isoformat()
        return sss

    def _heuristic_gate(self, sss: SSS) -> SSS | None:
        """
        Stage 3 — Heuristic Intercept Gate.
        Intercepts signals resolvable locally without burdening the CNS.
        Peripheral equivalent of a spinal reflex loop.

        Intercepts (in order):
          1. Oversized payloads        — discard before any pattern matching
          2. /commands                 — local reflex handler (TODO)
          3. Injection patterns        — symbolic injection attempt
          4. Efference echoes          — CNC self-generated signal suppression
          5. Duplicate debounce        — same payload from same user within debounce window

        SSS marked TRIAGED on pass, INTERCEPTED on intercept, DISCARDED on failure.

        Args:
            sss (SSS): SSS marked TRANSDUCED from threshold gate.

        Returns:
            SSS | None: SSS marked TRIAGED, or None if intercepted or failed.
        """
        text       = sss.text
        text_lower = text.lower()
        now        = datetime.now(timezone.utc).isoformat()

        # prune stale efference echoes before any check — cheap housekeeping
        self._prune_efference_echoes()

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
        # CNC publishes SHA-256 fingerprints of outbound responses to TMS.EFFERENCE_ECHO.
        # If inbound text fingerprint matches a live echo, the signal is suppressed here.
        fingerprint = self._fingerprint(text)
        if fingerprint in self._efference_echoes:
            sss.state                = "intercepted"
            sss.locus                = "tic._heuristic_gate"
            sss.drop_reason          = "efference_echo"
            sss.dropped_at           = now
            sss.efference_suppressed = True
            self.get_logger().debug(f"🔇 SSS suppressed — efference echo match: {fingerprint[:12]}…")
            return None

        # 5. duplicate debounce — same payload from the same user within debounce window.
        # Keyed on user_id — distinct senders never suppress each other.
        # Biological analogue: sensory refractory period — organ cannot re-fire on identical
        # stimulus within recovery window.
        now_epoch = time.monotonic()
        last_text, last_time = self._last_payload.get(sss.user_id, ("", 0.0))
        if text == last_text and (now_epoch - last_time) < _DEBOUNCE_WINDOW_S:
            sss.state       = "intercepted"
            sss.locus       = "tic._heuristic_gate"
            sss.drop_reason = "duplicate"
            sss.dropped_at  = now
            self.get_logger().debug("🔁 SSS intercepted — duplicate within debounce window")
            return None

        self._last_payload[sss.user_id] = (text, now_epoch)                    # anchor per user — distinct senders never suppress each other

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
        urgency_hits  = len(words_lower & _URGENCY_MARKERS)
        arousal       = min(1.0, (urgency_hits * 0.25) + (capital_ratio * 0.5))

        # valence — positive vs negative keyword balance
        positive_hits = len(words_lower & _POSITIVE_MARKERS)
        negative_hits = len(words_lower & _NEGATIVE_MARKERS)
        raw_valence   = (positive_hits - negative_hits) * 0.3                  # each hit shifts ±0.3
        valence       = max(-1.0, min(1.0, raw_valence))                        # clamp to [-1.0, 1.0]

        # salience — base 1.0; boosted by arousal (urgency lifts priority)
        salience      = min(2.0, 1.0 + arousal)                                # range [1.0, 2.0] — CNC may apply further modulation

        sss.arousal  = round(arousal, 3)
        sss.valence  = round(valence, 3)
        sss.salience = round(salience, 3)

        # fast-path flag — high-arousal signals bypass normal queue at CNS
        sss.fast_path = arousal >= _FAST_PATH_AROUSAL_THRESHOLD

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
            self._sensory_gateway.publish(signal)

            if sss.fast_path:
                self.get_logger().info(f"⚡ SSS dispatched (fast-path): {sss.text[:60]}…")
            else:
                self.get_logger().debug(f"📤 SSS dispatched: {sss.text[:60]}…")

        except Exception as e:
            self.get_logger().error(f"❌ Publish error: {e}")                   # non-fatal — publish failure must not crash the afferent pathway

    # ── shutdown ──────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        """
        Gracefully shut down TIC — close websocket server and all active connections.
        """
        self.get_logger().info("🛑 TIC shutting down…")

        async def _close():
            for ws in list(self._active_connections):
                await ws.close()
            if hasattr(self, "_ws_server"):
                self._ws_server.close()
                await self._ws_server.wait_closed()

        future = asyncio.run_coroutine_threadsafe(_close(), self._ws_loop)
        try:
            future.result(timeout=3.0)
        except Exception:
            pass

        self._ws_loop.call_soon_threadsafe(self._ws_loop.stop)
        self._ws_thread.join(timeout=3.0)
        super().destroy_node()
        self.get_logger().info("✅ TIC shutdown complete")


def main(args=None):
    rclpy.init(args=args)
    node = TIC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("👋 Shutdown requested")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
