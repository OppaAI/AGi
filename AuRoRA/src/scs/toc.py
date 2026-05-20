"""
TOC — Telepathy Output Core
=============================
AuRoRA · Telepathy Management System (TMS)

ROS2 node — efferent pathway for symbolic text output.
Receives CRS from CNC motor gateway, streams response fragments back to WebUI.

Architecture:
    TOC mirrors the efferent motor pathway — carries signals away from the brain.
    It is the biological analogue of the efference copy pathway: Grace's generated
    output is relayed back outward to the originating symbolic channel.

    TOC owns no state beyond active connections.
    It does not know what the response means — it only knows how to deliver it.
    If TOC goes down, CNC and TIC are unaffected — output simply stops arriving at WebUI.

    WebSocket server runs on a dedicated thread — never blocks ROS2 spin.
    ROS2 subscription callback schedules delivery onto the ws thread safely.

    Streaming semantics mirror CNC's GCE stream contract:
        {"type": "start",  "content": "<first fragment>"}
        {"type": "delta",  "content": "<fragment>"}
        {"type": "done",   "content": "<full response>"}
        {"type": "error",  "content": "<error message>"}

Topics:
    Sub: TMS.TEXT_MOTOR_GATEWAY (std_msgs/String) — CRS JSON fragments from CNC

Terminology:
    Efferent Pathway   — carries signals away from the brain (output direction)
    Efference Copy     — outgoing signal relayed back to the originating channel
    Telepathy Channel  — Grace's term for direct symbolic output with no physical substrate
    Motor Gateway      — ROS2 topic carrying CRS from CNC to TOC
"""

import asyncio                                              # event loop for websocket server — isolated from ROS2 spin
import json                                                 # serialize/deserialize payloads at both boundaries
import threading                                            # dedicated thread for websocket server
from typing import Set                                      # active connection registry type hint

import rclpy                                                # ROS2 Python client library — node lifecycle and spin
import websockets                                           # async websocket server — WebUI connection layer
import websockets.exceptions                                # websocket-specific exceptions for clean disconnect handling
from rclpy.node import Node                                 # base class for all ROS2 nodes
from std_msgs.msg import String                             # ROS2 string message type for topic I/O

from hrs.hrm import AGi                                     # homeostatic regulation manifest namespace
from hrs.hru import hydrate_manifest                        # manifest hydration — binds AuRoRA parameter server values into AGi constants

TMS = AGi.TMS                                               # module-level alias — TMS-level constants (topic names, websocket config)
GCE = AGi.SCS.GCE                                           # module-level alias — GCE stream event type constants


class TOC(Node):
    """
    Telepathy Output Core — ROS2 node.

    Efferent pathway for symbolic text output.
    Subscribes to CNC motor gateway, streams CRS fragments to all active WebUI connections.
    No cognitive logic — receive, relay, deliver only.
    """

    def __init__(self):
        """
        Initialize Telepathy Output Core.

        Boots websocket server on a dedicated thread and opens the motor gateway subscription.
        ROS2 spin and websocket server never share a thread.
        """
        super().__init__("toc")                                                 # register this node with ROS2 as "toc"
        self.get_logger().info("=" * 60)
        self.get_logger().info("📣 TOC — Telepathy Output Core starting…")
        self.get_logger().info("=" * 60)

        hydrate_manifest(self, system="tms")                                    # hydrate manifest — binds TMS constants from AuRoRA parameter server

        self._active_connections: Set = set()                                   # registry of live websocket connections — broadcast target

        # Motor gateway subscription — CRS fragments arrive here from CNC
        self._motor_gateway: rclpy.subscription.Subscription = self.create_subscription(
            String, TMS.TEXT_MOTOR_GATEWAY, self._receive_crs, 10              # String type | topic | callback | QoS depth 10
        )

        # Boot websocket server on its own thread — never competes with ROS2 spin
        self._ws_loop: asyncio.AbstractEventLoop = asyncio.new_event_loop()     # isolated event loop — owns the websocket server lifetime
        self._ws_thread: threading.Thread = threading.Thread(
            target=self._ws_loop.run_forever,
            name="toc-ws-server",
            daemon=True,                                                        # dies with main process — clean shutdown
        )
        self._ws_thread.start()                                                 # ignite websocket thread

        asyncio.run_coroutine_threadsafe(                                       # schedule server boot on ws loop — crosses thread boundary safely
            self._boot_ws_server(), self._ws_loop
        )

        self.get_logger().info(f"✅ Subscribed  : {TMS.TEXT_MOTOR_GATEWAY}")
        self.get_logger().info(f"✅ WebSocket   : ws://0.0.0.0:{TMS.WS_OUTPUT_PORT}")
        self.get_logger().info("=" * 60)
        self.get_logger().info("📣 TOC ready — efferent pathway open")
        self.get_logger().info("=" * 60)

    async def _boot_ws_server(self) -> None:
        """
        Boot the websocket server and hold it for the node lifetime.
        Runs entirely on the toc-ws-server thread — never touches ROS2.
        """
        self._ws_server = await websockets.serve(                               # open websocket server — WebUI connects here to receive responses
            self._handle_connection,
            "0.0.0.0",
            TMS.WS_OUTPUT_PORT,
        )
        self.get_logger().info(f"✅ WebSocket server live on port {TMS.WS_OUTPUT_PORT}")
        await self._ws_server.wait_closed()                                     # hold server open until explicitly closed

    async def _handle_connection(self, websocket) -> None:
        """
        Handle one WebUI output connection for its full lifetime.
        Connection is registered as a broadcast target — receives all CRS fragments.

        Args:
            websocket: Active websocket connection from WebUI.
        """
        self._active_connections.add(websocket)                                 # register as broadcast target
        self.get_logger().info("🔗 WebUI output connected")

        try:
            await websocket.wait_closed()                                       # hold open — TOC pushes, WebUI doesn't send here
        except websockets.exceptions.ConnectionClosedOK:
            pass
        except websockets.exceptions.ConnectionClosedError as e:
            self.get_logger().warning(f"⚠️  WebUI output disconnected unexpectedly: {e}")
        finally:
            self._active_connections.discard(websocket)                         # deregister on any exit path
            self.get_logger().info("🔌 WebUI output disconnected")

    def _receive_crs(self, msg: String) -> None:
        """
        Receive a CRS fragment from the motor gateway and schedule broadcast.
        ROS2 callback — schedules onto ws loop, never blocks spin.

        Args:
            msg (String): ROS2 string message carrying serialized CRS JSON.
        """
        try:
            payload: dict = json.loads(msg.data.strip())                        # deserialize CRS payload — validate at boundary
            if not isinstance(payload, dict) or "type" not in payload:          # malformed CRS — drop silently
                return

            asyncio.run_coroutine_threadsafe(                                   # schedule broadcast on ws loop — crosses thread boundary safely
                self._broadcast(payload), self._ws_loop
            )
        except (json.JSONDecodeError, ValueError):
            self.get_logger().warning("⚠️  Malformed CRS received — dropped")

    async def _broadcast(self, payload: dict) -> None:
        """
        Broadcast a CRS fragment to all active WebUI connections.
        Runs on ws loop — never touches ROS2 spin thread.
        Dead connections are pruned on send failure.

        Args:
            payload (dict): CRS fragment — type and content fields.
        """
        if not self._active_connections:                                        # no active connections — nothing to broadcast
            return

        raw: str = json.dumps(payload)                                          # serialize once — broadcast same payload to all connections
        dead: Set = set()                                                       # collect dead connections — pruned after iteration

        for websocket in self._active_connections:
            try:
                await websocket.send(raw)                                       # relay CRS fragment to this connection
            except websockets.exceptions.ConnectionClosed:
                dead.add(websocket)                                             # mark dead — prune after iteration completes
            except Exception as e:
                self.get_logger().warning(f"⚠️  Broadcast error: {e}")
                dead.add(websocket)

        self._active_connections -= dead                                         # prune dead connections — keeps registry clean

    def destroy_node(self) -> None:
        """
        Gracefully shut down TOC — close websocket server and all active connections.
        """
        self.get_logger().info("🛑 TOC shutting down…")

        async def _close():
            for ws in list(self._active_connections):                           # close all active connections cleanly
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
        self.get_logger().info("✅ TOC shutdown complete")


def main(args=None):
    rclpy.init(args=args)
    node = TOC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("👋 Shutdown requested")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
