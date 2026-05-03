"""
CLI — Command Line Interface
==============================
AuRoRA · Semantic Cognitive System (SCS)
"""

import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

RESET  = "\033[0m"
BOLD   = "\033[1m"
CYAN   = "\033[96m"
PINK   = "\033[95m"
GREY   = "\033[90m"
RED    = "\033[91m"

TOPIC_INPUT    = "/cns/neural_text_input"
TOPIC_RESPONSE = "/gce/response"

# If no "done" arrives within this many seconds, auto-unlock the prompt.
STREAM_TIMEOUT_SEC = 15.0


class GraceCLI(Node):

    def __init__(self):
        super().__init__("grace_cli")

        self._pub = self.create_publisher(String, TOPIC_INPUT, 10)
        self._sub = self.create_subscription(
            String, TOPIC_RESPONSE, self._on_response, 10
        )

        self._streaming    = False
        self._print_lock   = threading.Lock()
        self._turn_start   = None
        self._token_count  = 0
        self._timeout_timer = None  # threading.Timer | None

        self._print_header()

    def _print_header(self):
        print(f"\n{BOLD}{PINK}╔══════════════════════════════════╗")
        print(f"║     Chat with GRACE — AuRoRA     ║")
        print(f"╚══════════════════════════════════╝{RESET}")
        print(f"{GREY}Topics:")
        print(f"  PUB  {TOPIC_INPUT}")
        print(f"  SUB  {TOPIC_RESPONSE}")
        print(f"\nType your message and press Enter.")
        print(f"Ctrl+C to quit.{RESET}\n")

    def _cancel_timeout(self):
        """Cancel any pending stream timeout timer."""
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
            self._timeout_timer = None

    def _stream_timeout(self):
        """Called if no 'done' arrives within STREAM_TIMEOUT_SEC."""
        with self._print_lock:
            if self._streaming:
                self._streaming    = False
                self._turn_start   = None
                self._timeout_timer = None
                print(f"\n{RED}[stream timeout — no 'done' received]{RESET}\n")
                self._print_prompt()

    def _on_response(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        msg_type = data.get("type", "")
        content  = data.get("content", "")

        with self._print_lock:
            if msg_type == "start":
                self._cancel_timeout()
                self._streaming    = True
                self._turn_start   = time.monotonic()
                self._token_count  = 0
                # Arm timeout
                self._timeout_timer = threading.Timer(
                    STREAM_TIMEOUT_SEC, self._stream_timeout
                )
                self._timeout_timer.daemon = True
                self._timeout_timer.start()
                print(f"\n{BOLD}{PINK}GRACE{RESET}{PINK} 🌸  {RESET}", end="", flush=True)
                print(content, end="", flush=True)

            elif msg_type == "delta":
                self._token_count += max(1, len(content.split()))
                print(content, end="", flush=True)

            elif msg_type == "done":
                self._cancel_timeout()
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
                self._cancel_timeout()
                self._streaming  = False
                self._turn_start = None
                print(f"\n{RED}[error] {content}{RESET}\n")
                self._print_prompt()

            # ── Safety net: any message while streaming resets the timeout ──
            elif self._streaming:
                # Unknown type but we're mid-stream — keep timeout alive
                self._cancel_timeout()
                self._timeout_timer = threading.Timer(
                    STREAM_TIMEOUT_SEC, self._stream_timeout
                )
                self._timeout_timer.daemon = True
                self._timeout_timer.start()

    def send(self, text: str, web_search: bool = False):
        payload  = json.dumps({"text": text, "web_search": web_search})
        msg      = String()
        msg.data = payload
        self._pub.publish(msg)

    def _print_prompt(self):
        print(f"{BOLD}{CYAN}You{RESET}{CYAN} 💙  {RESET}", end="", flush=True)


def input_loop(node: GraceCLI):
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

        web_search = False
        if text.startswith("/web "):
            web_search = True
            text = text[5:].strip()
            print(f"{GREY}[web search enabled]{RESET}")

        node.send(text, web_search=web_search)


def main(args=None):
    rclpy.init(args=args)
    node = GraceCLI()

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
        # Guard against double-shutdown (ROS2 Humble sometimes shuts down
        # the context internally on KeyboardInterrupt before we get here).
        context = rclpy.get_default_context()
        if context.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()