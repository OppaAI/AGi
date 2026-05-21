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


class TIC(Node):
    """
    Telepathy Input Core — ROS2 node.

    Afferent pathway for symbolic text input.
    Accepts WebUI connections, normalizes messages into SSS, publishes to sensory gateway.
    No cognitive logic — validate, normalize, publish only.
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
                sss = self._normalize(raw_message)                              # validate and normalize raw input → SSS
                if sss is None:                                                 # malformed input — drop silently at boundary
                    continue
                self._publish_sss(sss)                                          # emit normalized SSS to sensory gateway

        except websockets.exceptions.ConnectionClosedOK:                        # clean disconnect — normal lifecycle
            pass
        except websockets.exceptions.ConnectionClosedError as e:                # unexpected disconnect — log but don't crash
            self.get_logger().warning(f"⚠️  WebUI disconnected unexpectedly: {e}")
        finally:
            self._active_connections.discard(websocket)                         # deregister on any exit path
            self.get_logger().info("🔌 WebUI disconnected")

    def _normalize(self, raw_message: str) -> SSS | None:
        """
        Validate and normalize raw WebUI JSON into a typed SSS.
        Drops anything that doesn't conform to the inbound contract.

        Args:
            raw_message (str): Raw JSON string from WebUI.

        Returns:
            SSS | None: Normalized sensory stimulus, or None if invalid.
        """
        try:
            payload: dict = json.loads(raw_message.strip())                     # parse raw JSON — reject non-JSON immediately
            if not isinstance(payload, dict) or not payload.get("text"):        # missing text field — not a valid stimulus
                return None

            return SSS(
                role       = payload.get("role", "user"),                       # speaker role — always "user" from WebUI
                text       = payload["text"].strip(),                           # stimulus content — stripped of whitespace
                user_id    = payload.get("user_id", "demo"),                    # speaker identity — defaults to demo
                source     = SensoryInputChannel.WEBUI,                         # channel tag — always WEBUI for TIC
                trace_type = TraceType.PMT,                                     # trace type — PMT for all conversational input
            )
        except (json.JSONDecodeError, ValueError):                              # malformed JSON or invalid enum value — drop at boundary
            return None

    def _publish_sss(self, sss: SSS) -> None:
        """
        Serialize SSS and publish to the sensory gateway topic.

        Args:
            sss (SSS): Normalized sensory stimulus ready for CNC consumption.
        """
        try:
            signal = String()
            signal.data = json.dumps({                                          # serialize SSS fields — CNC deserializes back into SSS
                "role"       : sss.role,
                "text"       : sss.text,
                "user_id"    : sss.user_id,
                "source"     : sss.source.value,
                "trace_type" : sss.trace_type.value,
            })
            self._sensory_gateway.publish(signal)                               # emit to /grace/sensory/text — CNC subscribes here
            self.get_logger().debug(f"📤 SSS published: {sss.text[:60]}…")
        except Exception as e:
            self.get_logger().error(f"❌ Publish error: {e}")                   # non-fatal — publish failure must not crash the afferent pathway

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
