"""
CLI — Command Line Interface
==============================
AuRoRA · Semantic Cognitive System (SCS)

Developer tool — two modes:

    python3 cli.py           — Direct mode (Option A)
                               Publishes straight to TMS sensory gateway,
                               subscribes straight to TMS motor gateway.
                               No TIC/TOC dependency — pure cognitive pipeline test.

    python3 cli.py --relay   — Relay mode (Option B)
                               Routes through TIC websocket (port 8764) and
                               TOC websocket (port 8765).
                               Full stack integration test — verifies TIC/TOC
                               are alive, normalizing, and routing correctly.

Topics (direct mode):
    Pub: TMS.TEXT_SENSORY_GATEWAY (std_msgs/String) — SSS JSON direct to CNC
    Sub: TMS.TEXT_MOTOR_GATEWAY   (std_msgs/String) — CRS JSON direct from CNC

Websockets (relay mode):
    TX:  ws://localhost:8764  — TIC input websocket
    RX:  ws://localhost:8765  — TOC output websocket
"""

import argparse                                  # --relay flag parsing
import asyncio                                   # relay mode event loop
import json                                      # serialize/deserialize payloads
import threading                                 # input thread for direct mode
import time                                      # elapsed time for stream stats

import rclpy                                     # ROS2 Python client library — direct mode only
from rclpy.node import Node                      # base class — direct mode only
from std_msgs.msg import String                  # ROS2 string message type — direct mode only

try:
    import websockets                            # relay mode websocket client — optional dep
    import websockets.exceptions
    HAS_WEBSOCKETS = True
except ImportError:
    HAS_WEBSOCKETS = False

from hrs.hrm import AGi                          # homeostatic regulation manifest namespace
TMS = AGi.TMS                                    # TMS-level constants — sensory/motor gateway topics
SCS = AGi.SCS                                    # SCS-level constants
GCE = AGi.SCS.GCE                                # GCE stream event type constants

# ── Terminal colors ────────────────────────────────────────────────────────────
RESET  = "\033[0m"
BOLD   = "\033[1m"
CYAN   = "\033[36m"
PINK   = "\033[95m"
GREY   = "\033[90m"
RED    = "\033[91m"
YELLOW = "\033[93m"

# ── Stream event aliases ───────────────────────────────────────────────────────
STREAM_LEADING     = GCE.STREAM_LEADING
STREAM_PROPAGATING = GCE.STREAM_PROPAGATING
STREAM_TRAILING    = GCE.STREAM_TRAILING
STREAM_ANOMALY     = GCE.STREAM_ANOMALY

# ── Topic aliases (direct mode) ────────────────────────────────────────────────
TOPIC_INPUT    = TMS.TEXT_SENSORY_GATEWAY        # publish SSS direct to CNC
TOPIC_RESPONSE = TMS.TEXT_MOTOR_GATEWAY          # subscribe CRS direct from CNC

# ── Relay mode websocket ports ─────────────────────────────────────────────────
WS_TIC = f"ws://localhost:{TMS.WS_PORT}"         # TIC input websocket
WS_TOC = f"ws://localhost:{TMS.WS_OUTPUT_PORT}"  # TOC output websocket

# If no trailing arrives within this many seconds, auto-unlock the prompt
STREAM_TIMEOUT_SEC = 15.0


# ══════════════════════════════════════════════════════════════════════════════
# Shared rendering — same output logic for both modes
# ══════════════════════════════════════════════════════════════════════════════

def print_header(mode: str) -> None:
    """Print boot header with active mode label."""
    mode_label = f"{'relay' if mode == 'relay' else 'direct'} mode"
    print(f"\n{BOLD}{PINK}╔══════════════════════════════════╗")
    print(f"║     Chat with GRACE — AuRoRA     ║")
    print(f"╚══════════════════════════════════╝{RESET}")
    if mode == "relay":
        print(f"{YELLOW}[relay mode] routing through TIC/TOC websockets{RESET}")
        print(f"{GREY}  TX  {WS_TIC}")
        print(f"  RX  {WS_TOC}{RESET}")
    else:
        print(f"{GREY}[direct mode] bypassing TIC/TOC — direct ROS2 topics")
        print(f"  PUB  {TOPIC_INPUT}")
        print(f"  SUB  {TOPIC_RESPONSE}{RESET}")
    print(f"\n{GREY}Type your message and press Enter.")
    print(f"/web <message> to enable web search.")
    print(f"Ctrl+C to quit.{RESET}\n")


def print_prompt() -> None:
    print(f"{BOLD}{CYAN}You{RESET}{CYAN} 💙  {RESET}", end="", flush=True)


def render_fragment(data: dict, state: dict, print_lock: threading.Lock) -> None:
    """
    Render one CRS fragment to terminal.
    Mutates state dict in place — streaming flag, timing, token count.
    state keys: streaming, turn_start, token_count, timeout_timer

    Args:
        data        (dict)            : Parsed CRS payload — type and content fields.
        state       (dict)            : Mutable stream state shared across fragments.
        print_lock  (threading.Lock)  : Mutex — prevents interleaved output.
    """
    msg_type = data.get("type", "")
    content  = data.get("content", "")

    with print_lock:
        if msg_type == STREAM_LEADING:                                   # first fragment — open stream
            _cancel_timeout(state)
            state["streaming"]   = True
            state["turn_start"]  = time.monotonic()
            state["token_count"] = 0
            _arm_timeout(state, print_lock)
            print(f"\n{BOLD}{PINK}GRACE{RESET}{PINK} 🌸  {RESET}", end="", flush=True)
            print(content, end="", flush=True)

        elif msg_type == STREAM_PROPAGATING:                             # delta fragment — accumulate
            state["token_count"] += max(1, len(content.split()))
            print(content, end="", flush=True)

        elif msg_type == STREAM_TRAILING:                                # stream complete — print stats
            _cancel_timeout(state)
            elapsed = (
                time.monotonic() - state["turn_start"]
                if state["turn_start"] is not None else 0.0
            )
            tps = state["token_count"] / elapsed if elapsed > 0 else 0.0
            stats = (
                f"[⏱ {elapsed:.2f}s | "
                f"~{state['token_count']} tok | "
                f"{tps:.1f} tok/s]"
            )
            state["streaming"]  = False
            state["turn_start"] = None
            print(f"\n{GREY}{stats}{RESET}\n")
            print_prompt()

        elif msg_type == STREAM_ANOMALY:                                 # error — surface and unlock
            _cancel_timeout(state)
            state["streaming"]  = False
            state["turn_start"] = None
            print(f"\n{RED}[error] {content}{RESET}\n")
            print_prompt()

        elif state["streaming"]:                                         # unknown type mid-stream — reset timeout, keep alive
            _cancel_timeout(state)
            _arm_timeout(state, print_lock)


def _arm_timeout(state: dict, print_lock: threading.Lock) -> None:
    """Arm stream timeout timer — fires if no trailing arrives within STREAM_TIMEOUT_SEC."""
    def _on_timeout():
        with print_lock:
            if state["streaming"]:
                state["streaming"]      = False
                state["turn_start"]     = None
                state["timeout_timer"]  = None
                print(f"\n{RED}[stream timeout — no 'done' received]{RESET}\n")
                print_prompt()

    timer = threading.Timer(STREAM_TIMEOUT_SEC, _on_timeout)
    timer.daemon = True
    timer.start()
    state["timeout_timer"] = timer


def _cancel_timeout(state: dict) -> None:
    """Cancel any pending stream timeout timer."""
    if state.get("timeout_timer") is not None:
        state["timeout_timer"].cancel()
        state["timeout_timer"] = None


def build_payload(text: str, web_search: bool = False) -> str:
    """
    Build SSS-compatible JSON payload for either direct ROS2 publish or TIC websocket send.

    Args:
        text        (str)  : User message content.
        web_search  (bool) : Whether to enable web search for this turn.

    Returns:
        str: Serialized JSON payload.
    """
    return json.dumps({
        "text"       : text,
        "role"       : "user",
        "user_id"    : "demo",
        "source"     : "cli",                                            # SensoryInputChannel.CLI
        "trace_type" : "pmt",                                            # TraceType.PMT
        "web_search" : web_search,
    })


# ══════════════════════════════════════════════════════════════════════════════
# Direct mode — ROS2 node, bypasses TIC/TOC
# ══════════════════════════════════════════════════════════════════════════════

class GraceCLI(Node):
    """
    Direct mode CLI — ROS2 node.
    Publishes SSS directly to TMS sensory gateway.
    Subscribes to TMS motor gateway for CRS fragments.
    No TIC/TOC dependency — pure cognitive pipeline test.
    """

    def __init__(self):
        super().__init__("grace_cli")

        self._pub = self.create_publisher(String, TOPIC_INPUT, 10)
        self._sub = self.create_subscription(
            String, TOPIC_RESPONSE, self._on_response, 10
        )

        self._print_lock = threading.Lock()
        self._state: dict = {                                            # mutable stream state — shared across render calls
            "streaming"    : False,
            "turn_start"   : None,
            "token_count"  : 0,
            "timeout_timer": None,
        }

    def _on_response(self, msg: String) -> None:
        """ROS2 subscription callback — deserialize and render CRS fragment."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        render_fragment(data, self._state, self._print_lock)

    def send(self, text: str, web_search: bool = False) -> None:
        """
        Publish SSS payload to TMS sensory gateway.

        Args:
            text        (str)  : User message content.
            web_search  (bool) : Whether to enable web search for this turn.
        """
        msg      = String()
        msg.data = build_payload(text, web_search)
        self._pub.publish(msg)


def input_loop_direct(node: GraceCLI) -> None:
    """
    Blocking input loop for direct mode — runs on dedicated thread.
    Reads stdin, sends SSS via ROS2 publisher.

    Args:
        node (GraceCLI): Active CLI node for publishing.
    """
    print_prompt()

    while rclpy.ok():
        try:
            text = input().strip()
        except (EOFError, KeyboardInterrupt):
            break

        if not text:
            print_prompt()
            continue

        if node._state["streaming"]:                                     # cognitive cycle busy — hold input
            print(f"{GREY}[GRACE is still thinking — please wait]{RESET}")
            print_prompt()
            continue

        web_search = False
        if text.startswith("/web "):                                     # web search shortcut
            web_search = True
            text = text[5:].strip()
            print(f"{GREY}[web search enabled]{RESET}")

        node.send(text, web_search=web_search)


def run_direct_mode(ros_args) -> None:
    """
    Boot direct mode — ROS2 node, no TIC/TOC.

    Args:
        ros_args: Filtered ROS2 args from argparse.
    """
    rclpy.init(args=ros_args)
    node = GraceCLI()

    input_thread = threading.Thread(
        target=input_loop_direct,
        args=(node,),
        daemon=True,
    )
    input_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\n{GREY}Goodbye! 👋{RESET}\n")
    finally:
        node.destroy_node()
        context = rclpy.get_default_context()
        if context.ok():
            rclpy.shutdown()


# ══════════════════════════════════════════════════════════════════════════════
# Relay mode — websocket client, routes through TIC and TOC
# ══════════════════════════════════════════════════════════════════════════════

async def relay_loop() -> None:
    """
    Relay mode async loop — connects to TIC and TOC websockets.
    TX sends SSS payloads to TIC.
    RX receives CRS fragments from TOC and renders them.
    Runs until Ctrl+C.
    """
    print_lock = threading.Lock()
    state: dict = {
        "streaming"    : False,
        "turn_start"   : None,
        "token_count"  : 0,
        "timeout_timer": None,
    }

    try:
        async with websockets.connect(WS_TIC) as tx, \
                   websockets.connect(WS_TOC) as rx:

            print(f"{YELLOW}[relay] connected to TIC and TOC{RESET}\n")
            print_prompt()

            async def receive() -> None:
                """Receive CRS fragments from TOC and render — runs concurrently."""
                try:
                    async for raw in rx:
                        try:
                            data = json.loads(raw)
                        except json.JSONDecodeError:
                            continue
                        render_fragment(data, state, print_lock)
                except websockets.exceptions.ConnectionClosed:
                    print(f"\n{RED}[relay] TOC disconnected{RESET}\n")

            asyncio.ensure_future(receive())                             # spawn receiver — runs concurrently with input loop

            loop = asyncio.get_event_loop()

            while True:
                try:
                    text = await loop.run_in_executor(None, input)      # blocking stdin read — offloaded to thread
                    text = text.strip()
                except (EOFError, KeyboardInterrupt):
                    break

                if not text:
                    print_prompt()
                    continue

                if state["streaming"]:
                    print(f"{GREY}[GRACE is still thinking — please wait]{RESET}")
                    print_prompt()
                    continue

                web_search = False
                if text.startswith("/web "):
                    web_search = True
                    text = text[5:].strip()
                    print(f"{GREY}[web search enabled]{RESET}")

                try:
                    await tx.send(build_payload(text, web_search))      # send SSS to TIC
                except websockets.exceptions.ConnectionClosed:
                    print(f"\n{RED}[relay] TIC disconnected{RESET}\n")
                    break

    except OSError as e:
        print(f"{RED}[relay] connection failed: {e}{RESET}")
        print(f"{GREY}Are TIC (:{TMS.WS_PORT}) and TOC (:{TMS.WS_OUTPUT_PORT}) running?{RESET}\n")


def run_relay_mode() -> None:
    """
    Boot relay mode — websocket client, routes through TIC/TOC.
    No ROS2 context needed.
    """
    if not HAS_WEBSOCKETS:
        print(f"{RED}[relay] websockets package not installed.{RESET}")
        print(f"{GREY}Install with: pip install websockets --break-system-packages{RESET}\n")
        return

    try:
        asyncio.run(relay_loop())
    except KeyboardInterrupt:
        print(f"\n{GREY}Goodbye! 👋{RESET}\n")


# ══════════════════════════════════════════════════════════════════════════════
# Entry point
# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    parser = argparse.ArgumentParser(
        description="Grace CLI — chat interface for AuRoRA"
    )
    parser.add_argument(
        "--relay",
        action="store_true",
        help=(
            "Route through TIC/TOC websockets instead of direct ROS2 topics. "
            "Use this to integration-test TIC and TOC are alive and routing correctly."
        ),
    )
    parsed, ros_args = parser.parse_known_args(args)

    mode = "relay" if parsed.relay else "direct"
    print_header(mode)

    if parsed.relay:
        run_relay_mode()
    else:
        run_direct_mode(ros_args)


if __name__ == "__main__":
    main()
