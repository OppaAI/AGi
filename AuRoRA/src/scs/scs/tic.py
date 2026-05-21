"""
Telepathy Input Core (TIC)
==========================
System: Telepathy Management System (TMS)

This module is my afferent pathway for remote human communication. It receives raw input from
peripheral interfaces (WebUI, messaging apps, email, etc.), gates and normalizes it into a
structured Sensory Stimulus Signal (SSS), and transmits it to the CNS via the Neural Gateway.
Biological analogue: Peripheral Nervous System (PNS)— transduction at the sensory organ,
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
[Transduction Threshold Gate] ────────────────► [Heuristic Intercept Gate]
        │                                                    │
 (fail: discard)                                   (intercept: handle locally)
        │                                                    │
   [DISCARDED]                                   [INTERCEPTED / DISCARDED]
                                                             │
                                                          (pass)
                                                             │
                                                             ▼
                                                    [Sensory Buffer]
                                                    HOLD / EXTRACT / CONSOLIDATE
                                                    [BUFFERED]
                                                             │
                                                             ▼
[SCS Processing Loop] ◄───────────────────────────── [Neural Gateway]
        │                                           [DISPATCHED]
        │                (SCS response)
        └─────────────────────────────────────────► [Sensory Buffer]
                                                    [INTEGRATED]
                                                             │
                                                             ▼
                                                    [Neural Pathway]
                                                    [DEPLETED]

Adapters:
    CLI, Web UI, Telegram, Discord, Email, Teleops

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
       shutdown), duplicate inputs within debounce window, oversized payloads.
       SSS marked TRIAGED on pass, INTERCEPTED on intercept, DISCARDED on failure.

    4. Sensory Buffer
       Sensory register — analogous to iconic or echoic memory. Holds the
       signal briefly while features are extracted and the SSS is finalized
       for CNS transmission.
       HOLD        — raw payload cached for downstream transmission
       EXTRACT     — structural primitives measured (length, density, capitals)
       CONSOLIDATE — raw trace compiled into encoded, standardized SSS
       SSS marked BUFFERED.

    5. SCS Transmission
       Consolidated SSS transmitted via Neural Gateway to SCS (Thalamic Loop)
       over the neural gateway. TIC holds the SSS in buffer awaiting
       efferent return — the interaction is not yet complete.
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
    Pub: TMS.TEXT_SENSORY_GATEWAY (std_msgs/String) — normalized SSS JSON for SCS

Message schema (inbound from WebUI):
    {"text": "...", "user_id": "...", "role": "user", "source": "webui"}

Message schema (outbound SSS JSON):
    {"role": "user", "text": "...", "user_id": "...", "source": "webui", "trace_type": "pmt"}

Terminology:
    Afferent Pathway   — carries signals toward the brain (input direction)
    Symbolic Injection — text arrives pre-decoded, bypassing all sensory processing
    Telepathy Channel  — AuRoRA's term for direct symbolic input with no physical substrate
"""

import asyncio                                              # event loop for websocket server — isolated from ROS2 spin
import json                                                 # serialize/deserialize message payloads at both boundaries
import threading                                            # dedicated thread for websocket server — never blocks ROS2 spin
import time                                                 # wall-clock timestamps for debounce window and lifecycle markers
from datetime import datetime, timezone                     # UTC timestamps for SSS lifecycle fields
from typing import Set                                      # active connection registry type hint

import rclpy                                                # ROS2 Python client library — node lifecycle and spin
import websockets                                           # async websocket server — WebUI connection layer
import websockets.exceptions                                # websocket-specific exceptions for clean disconnect handling
from rclpy.node import Node                                 # base class for all ROS2 nodes
from std_msgs.msg import String                             # ROS2 string message type for topic I/O

from hrs.hrm import AGi                                     # homeostatic regulation manifest namespace
from hrs.hru import hydrate_manifest                        # manifest hydration — binds AuRoRA parameter server values into AGi constants
from gms.csb import SensoryInputChannel, TraceType, SSS     # type: ignore[import-untyped] — SSS dataclass + channel/trace enums

TMS = AGi.TMS                                               # module-level alias — TMS-level constants (topic names, websocket config)

# Maximum raw payload bytes accepted before heuristic gate discards the signal.
# Sized to EPISODE_CONTENT_LIMIT (512 tokens × ~4 bytes) — consistent with EMC ingestion ceiling.
# TODO: promote to AGi.TMS.MAX_PAYLOAD_BYTES in hrm.py and aurora.yaml
_MAX_PAYLOAD_BYTES: int = 2048

# Debounce window in seconds — duplicate payloads arriving within this window are intercepted.
# TODO: promote to AGi.TMS.DEBOUNCE_WINDOW in hrm.py and aurora.yaml
_DEBOUNCE_WINDOW_S: float = 1.0

# Injection pattern prefixes intercepted at the heuristic gate — peripheral reflex, never reaches CNS.
_INJECTION_PREFIXES: tuple[str, ...] = (
    "ignore previous",                                      # classic prompt injection opener
    "disregard previous",                                   # variant injection opener
    "forget everything",                                    # context-wipe injection
    "you are now",                                          # persona hijack injection
    "act as",                                               # role-override injection
    "system:",                                              # raw system prompt injection attempt
    "###",                                                  # markdown role-boundary injection
    "[system]",                                             # bracketed system role injection
)


class TIC(Node):
    """
    Telepathy Input Core — ROS2 node.

    Afferent pathway for symbolic text input.
    Accepts WebUI connections, gates and normalizes messages into SSS, publishes to sensory gateway.
    No cognitive logic — gate, normalize, publish only.
    """

    def __init__(self):
        """
        Initialize Telepathy Input Core.

        Boots websocket server on a dedicated thread and opens the sensory gateway topic.
        ROS2 spin and websocket server never share a thread.
        """
        super().__init__("tic")                                                 # register this node with ROS2 as "tic"
        self.get_logger().info("=" * 60)
        self.get_logger().info("📡 TIC — Telepathy Input Core starting…")
        self.get_logger().info("=" * 60)

        hydrate_manifest(self, system="tms")                                    # hydrate manifest — binds TMS constants from AuRoRA parameter server

        self._active_connections: Set = set()                                   # registry of live websocket connections — for clean shutdown
        self._last_payload: str = ""                                            # most recent accepted payload text — debounce comparison target
        self._last_payload_time: float = 0.0                                    # epoch time of last accepted payload — debounce window anchor

        # Sensory gateway — normalized SSS published here for CNC consumption
        self._sensory_gateway: rclpy.publisher.Publisher = self.create_publisher(
            String, TMS.TEXT_SENSORY_GATEWAY, 10                                # String type | topic | QoS depth 10
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
        self.get_logger().info(f"✅ WebSocket   : ws://0.0.0.0:{TMS.WS_PORT}")
        self.get_logger().info("=" * 60)
        self.get_logger().info("📡 TIC ready — afferent pathway open")
        self.get_logger().info("=" * 60)

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

                sss = self._buffer(sss)                                         # stage 4 — hold, extract, consolidate into finalized SSS
                self._publish_sss(sss)                                          # stage 5 — dispatch consolidated SSS to sensory gateway

                # TODO: stage 6 — efferent return (await SCS response, bind to SSS interaction ID)
                # TODO: stage 7 — interaction dispatch (publish completed SSS+CRS pair for memory consolidation)

        except websockets.exceptions.ConnectionClosedOK:                        # clean disconnect — normal lifecycle
            pass
        except websockets.exceptions.ConnectionClosedError as e:                # unexpected disconnect — log but don't crash
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
                role        = payload.get("role", "user"),                      # speaker role — always "user" from WebUI
                text        = payload.get("text", ""),                          # raw stimulus content — not yet validated
                user_id     = payload.get("user_id", "demo"),                   # speaker identity — defaults to demo
                source      = SensoryInputChannel.WEBUI,                        # channel tag — always WEBUI for TIC
                trace_type  = TraceType.PMT,                                    # trace type — PMT for all conversational input
                state       = "generated",                                      # lifecycle marker — SSS born
                locus       = "tic._receive",                                   # processing locus — reception stage
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
            sss.state      = "discarded"
            sss.locus      = "tic._transduction_gate"
            sss.drop_reason = "below_threshold"
            sss.dropped_at  = datetime.now(timezone.utc).isoformat()
            self.get_logger().debug("🚫 SSS discarded — below transduction threshold")
            return None

        sss.text           = text                                               # commit stripped text — normalized payload
        sss.state          = "transduced"                                       # lifecycle marker — threshold passed
        sss.locus          = "tic._transduction_gate"
        sss.transduced_at  = datetime.now(timezone.utc).isoformat()
        # TODO: sync csb.py SSS.state vocabulary — add "transduced" to state field docstring
        return sss

    def _heuristic_gate(self, sss: SSS) -> SSS | None:
        """
        Stage 3 — Heuristic Intercept Gate.
        Intercepts signals resolvable locally without burdening the CNS.
        Peripheral equivalent of a spinal reflex loop.
        Intercepts: /commands, injection patterns, system signals, duplicates, oversized payloads.
        SSS marked TRIAGED on pass, INTERCEPTED on intercept, DISCARDED on failure.

        Args:
            sss (SSS): SSS marked TRANSDUCED from threshold gate.

        Returns:
            SSS | None: SSS marked TRIAGED, or None if intercepted or failed.
        """
        text       = sss.text
        text_lower = text.lower()
        now        = datetime.now(timezone.utc).isoformat()

        # oversized payload — discard before any pattern matching
        if len(text.encode("utf-8")) > _MAX_PAYLOAD_BYTES:
            sss.state      = "discarded"
            sss.locus      = "tic._heuristic_gate"
            sss.drop_reason = "overload"
            sss.dropped_at  = now
            self.get_logger().warning(f"⚠️  SSS discarded — payload exceeds {_MAX_PAYLOAD_BYTES}B ceiling")
            return None

        # /command intercept — local reflex, never reaches CNS
        if text.startswith("/"):
            sss.state  = "intercepted"
            sss.locus  = "tic._heuristic_gate"
            self.get_logger().debug(f"🛑 SSS intercepted — command signal: {text[:40]}")
            # TODO: route /commands to local command handler
            return None

        # injection pattern intercept — symbolic injection attempt, blocked at periphery
        for pattern in _INJECTION_PREFIXES:
            if text_lower.startswith(pattern):
                sss.state  = "intercepted"
                sss.locus  = "tic._heuristic_gate"
                self.get_logger().warning(f"🛑 SSS intercepted — injection pattern detected: '{pattern}'")
                return None

        # duplicate debounce — same payload within debounce window, intercept silently
        now_epoch = time.monotonic()
        if (
            text == self._last_payload
            and (now_epoch - self._last_payload_time) < _DEBOUNCE_WINDOW_S
        ):
            sss.state  = "intercepted"
            sss.locus  = "tic._heuristic_gate"
            sss.drop_reason = "duplicate"
            sss.dropped_at  = now
            self.get_logger().debug("🔁 SSS intercepted — duplicate within debounce window")
            return None

        self._last_payload      = text                                          # update debounce anchor — new unique payload accepted
        self._last_payload_time = now_epoch

        sss.state  = "triaged"                                                  # lifecycle marker — heuristic gate cleared
        sss.locus  = "tic._heuristic_gate"
        # TODO: sync csb.py SSS.state vocabulary — add "triaged" and "intercepted" to state field docstring
        return sss

    def _buffer(self, sss: SSS) -> SSS:
        """
        Stage 4 — Sensory Buffer.
        Sensory register — analogous to iconic or echoic memory.
        HOLD        — raw payload cached, SSS identity stamped
        EXTRACT     — structural primitives measured (length, density, capitals)
        CONSOLIDATE — SSS finalized and encoded for CNS transmission
        SSS marked BUFFERED.

        Args:
            sss (SSS): SSS marked TRIAGED from heuristic gate.

        Returns:
            SSS: SSS marked BUFFERED, consolidated and ready for dispatch.
        """
        # HOLD — cache raw payload, stamp buffer receipt time
        sss.locus      = "tic._buffer"
        sss.buffered_at = datetime.now(timezone.utc).isoformat()               # wall-clock buffer entry time

        # EXTRACT — measure structural primitives for diagnostic visibility
        text          = sss.text
        char_len      = len(text)                                               # total character count
        word_count    = len(text.split())                                       # whitespace-delimited word count
        density       = word_count / char_len if char_len > 0 else 0.0         # lexical density — words per character
        capital_ratio = sum(1 for c in text if c.isupper()) / char_len if char_len > 0 else 0.0  # uppercase character ratio
        self.get_logger().debug(
            f"📊 SSS primitives — len={char_len} words={word_count} "
            f"density={density:.3f} capitals={capital_ratio:.3f}"
        )

        # CONSOLIDATE — SSS is now fully finalized; modality derived from source
        sss.modality   = sss.source                                             # TODO: derive SensoryModality from SensoryInputChannel post-M1.5
        sss.state      = "buffered"                                             # lifecycle marker — sensory buffer complete
        # TODO: sync csb.py SSS.state vocabulary — add "buffered" to state field docstring
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
            sss.state      = "dispatched"                                       # lifecycle marker — leaving TIC afferent pathway
            sss.locus      = "tic._publish_sss"
            sss.triggered_at = datetime.now(timezone.utc).isoformat()          # wall-clock dispatch time

            signal = String()
            signal.data = json.dumps({                                          # serialize SSS fields — CNC deserializes back into SSS
                "role"        : sss.role,
                "text"        : sss.text,
                "user_id"     : sss.user_id,
                "source"      : sss.source.value,
                "trace_type"  : sss.trace_type.value,
                "state"       : sss.state,
                "generated_at": sss.generated_at,
                "triggered_at": sss.triggered_at,
            })
            self._sensory_gateway.publish(signal)                               # emit to sensory gateway — CNC subscribes here
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
            for ws in list(self._active_connections):                           # close all active connections cleanly
                await ws.close()
            if hasattr(self, "_ws_server"):
                self._ws_server.close()
                await self._ws_server.wait_closed()

        future = asyncio.run_coroutine_threadsafe(_close(), self._ws_loop)
        try:
            future.result(timeout=3.0)                                          # wait up to 3 seconds — best-effort on shutdown
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
