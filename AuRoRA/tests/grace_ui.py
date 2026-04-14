"""
CLI — Command Line Interface
==============================
AuRoRA · Semantic Cognitive System (SCS)

Simple terminal interface for chatting with GRACE during development.
No rosbridge, no browser — just two ROS2 topics and a terminal.

Usage:
    ros2 run scs cli

    Or directly:
    python3 cli.py
"""

import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ── ANSI colours ──────────────────────────────────────────────────────────────
RESET  = "\033[0m"
BOLD   = "\033[1m"
CYAN   = "\033[96m"    # You
PINK   = "\033[95m"    # GRACE
GREY   = "\033[90m"    # system messages
RED    = "\033[91m"    # errors
# ─────────────────────────────────────────────────────────────────────────────

# ── Topics ───────────────────────────────────────────────────────────────────
TOPIC_INPUT    = "/cns/neural_input"
TOPIC_RESPONSE = "/gce/response"
# ─────────────────────────────────────────────────────────────────────────────


class GraceCLI(Node):
    """
    Command Line Interface node for GRACE.

    Publishes user input to /cns/neural_input.
    Subscribes to /gce/response and prints streaming tokens inline.
    """

    def __init__(self):
        super().__init__("grace_cli")

        # ── Publisher ─────────────────────────────────────────────────────────
        self._pub = self.create_publisher(String, TOPIC_INPUT, 10)

        # ── Subscriber ────────────────────────────────────────────────────────
        self._sub = self.create_subscription(
            String, TOPIC_RESPONSE, self._on_response, 10
        )

        # ── State ─────────────────────────────────────────────────────────────
        self._streaming    = False
        self._print_lock   = threading.Lock()
        self._turn_start   = None   # float | None
        self._token_count  = 0

        self._print_header()

    # ── Startup banner ────────────────────────────────────────────────────────

    def _print_header(self):
        print(f"\n{BOLD}{PINK}╔══════════════════════════════════╗")
        print(f"║     Chat with GRACE — AuRoRA     ║")
        print(f"╚══════════════════════════════════╝{RESET}")
        print(f"{GREY}Topics:")
        print(f"  PUB  {TOPIC_INPUT}")
        print(f"  SUB  {TOPIC_RESPONSE}")
        print(f"\nType your message and press Enter.")
        print(f"Ctrl+C to quit.{RESET}\n")

    # ── Response handler ──────────────────────────────────────────────────────

    def _on_response(self, msg: String):
        """Handle streaming response chunks from GRACE."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        msg_type = data.get("type", "")
        content  = data.get("content", "")

        with self._print_lock:
            if msg_type == "start":
                self._streaming   = True
                self._turn_start  = time.monotonic()
                self._token_count = 0
                print(f"\n{BOLD}{PINK}GRACE{RESET}{PINK} 🌸  {RESET}", end="", flush=True)
                print(content, end="", flush=True)

            elif msg_type == "delta":
                # Approximate token count by whitespace-splitting the chunk.
                # Replace with exact metadata from your backend if available.
                self._token_count += max(1, len(content.split()))
                print(content, end="", flush=True)

            elif msg_type == "done":
                elapsed = (
                    time.monotonic() - self._turn_start
                    if self._turn_start is not None
                    else 0.0
                )
                tps = self._token_count / elapsed if elapsed > 0 else 0.0
                stats = (
                    f"[⏱ {elapsed:.2f}s | "
                    f"~{self._token_count} tok | "
                    f"{tps:.1f} tok/s]"
                )
                self._streaming  = False
                self._turn_start = None
                print(f"\n{GREY}{stats}{RESET}\n")
                self._print_prompt()

            elif msg_type == "error":
                self._streaming  = False
                self._turn_start = None
                print(f"\n{RED}[error] {content}{RESET}\n")
                self._print_prompt()

    # ── Send message ──────────────────────────────────────────────────────────

    def send(self, text: str, web_search: bool = False):
        """Publish a user message to GRACE."""
        payload = json.dumps({"text": text, "web_search": web_search})
        msg      = String()
        msg.data = payload
        self._pub.publish(msg)

    # ── Prompt ────────────────────────────────────────────────────────────────

    def _print_prompt(self):
        print(f"{BOLD}{CYAN}You{RESET}{CYAN} 💙  {RESET}", end="", flush=True)


# ── Input thread ──────────────────────────────────────────────────────────────

def input_loop(node: GraceCLI):
    """
    Blocking input loop — runs in a separate thread so ROS2 can spin freely.
    """
    node._print_prompt()

    while rclpy.ok():
        try:
            text = input().strip()
        except (EOFError, KeyboardInterrupt):
            break

        if not text:
            node._print_prompt()
            continue

        if node._streaming:
            print(f"{GREY}[GRACE is still thinking — please wait]{RESET}")
            node._print_prompt()
            continue

        # Check for web search prefix: /web <message>
        web_search = False
        if text.startswith("/web "):
            web_search = True
            text = text[5:].strip()
            print(f"{GREY}[web search enabled]{RESET}")

        node.send(text, web_search=web_search)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = GraceCLI()

    # Input runs in a thread — ROS2 spin runs in main thread
    input_thread = threading.Thread(
        target=input_loop,
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
        rclpy.shutdown()


if __name__ == "__main__":
    main()