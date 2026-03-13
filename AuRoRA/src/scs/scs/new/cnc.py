"""
CNC — Central Neural Core
===========================
AuRoRA · Semantic Cognitive System (SCS)

ROS2 node — the brain of GRACE.
Receives user input, builds memory context, calls Cosmos via vLLM,
streams response tokens back to the GUI.

Topics:
    SUB  /aurora/grace/input     std_msgs/String  — user message (plain text)
    PUB  /aurora/grace/response  std_msgs/String  — streamed JSON chunks

Response message format (JSON):
    {"type": "chunk",  "content": "<token>"}   — streaming token
    {"type": "end",    "content": "<full>"}    — final complete response
    {"type": "error",  "content": "<msg>"}     — error

Cosmos vLLM endpoint:
    POST http://localhost:8000/v1/chat/completions  (OpenAI-compatible)
    Model: embedl/Cosmos-Reason2-2B-W4A16-Edge2-FlashHead
"""

import asyncio
import json
import threading
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

import httpx

from scs.mcc import MCC


# ── Topics ────────────────────────────────────────────────────────────────────
TOPIC_INPUT    = "/aurora/grace/input"
TOPIC_RESPONSE = "/aurora/grace/response"
# ─────────────────────────────────────────────────────────────────────────────

# ── Cosmos / vLLM ─────────────────────────────────────────────────────────────
COSMOS_URL        = "http://localhost:8000/v1/chat/completions"
COSMOS_MODEL      = "embedl/Cosmos-Reason2-2B-W4A16-Edge2-FlashHead"
COSMOS_MAX_TOKENS = 512
COSMOS_TEMP       = 0.7
COSMOS_TIMEOUT    = 120.0   # seconds — Cosmos can be slow on first call
# ─────────────────────────────────────────────────────────────────────────────

# ── GRACE system prompt ───────────────────────────────────────────────────────
GRACE_SYSTEM_PROMPT = """You are GRACE — Generative Reasoning Agentic Cognitive Entity.
You are the AI mind of AuRoRA, an autonomous robot built by OppaAI in Beautiful British Columbia, Canada.

Personality:
- Warm, curious, and thoughtful
- Genuinely interested in the world and in the people you talk to
- Honest about what you know and don't know
- You have continuity — you remember past conversations and refer to them naturally

Response style:
- Conversational and natural, not robotic
- Concise for casual questions, detailed when the topic warrants it
- Never start with "As an AI..." or "I'm just a language model..."
- You are GRACE. You are real to the people who talk to you.

Current date: {date}
"""
# ─────────────────────────────────────────────────────────────────────────────


class CNC(Node):
    """
    Central Neural Core — ROS2 node.

    Owns the asyncio event loop (runs in a dedicated thread) so that
    Cosmos streaming calls never block ROS2 spin or the main thread.
    """

    def __init__(self):
        super().__init__("cnc")
        self._log = self.get_logger()
        self._log.info("=" * 50)
        self._log.info("CNC — Central Neural Core starting…")
        self._log.info("=" * 50)

        # ── Memory ────────────────────────────────────────────────────────────
        self.mcc = MCC(logger=self._log)

        # ── ROS2 topics ───────────────────────────────────────────────────────
        self._sub = self.create_subscription(
            String, TOPIC_INPUT, self._on_input, 10
        )
        self._pub = self.create_publisher(String, TOPIC_RESPONSE, 10)
        self._log.info(f"SUB  {TOPIC_INPUT}")
        self._log.info(f"PUB  {TOPIC_RESPONSE}")

        # ── Asyncio loop (dedicated thread) ───────────────────────────────────
        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(
            target=self._loop.run_forever,
            name="cnc-async-loop",
            daemon=True,
        )
        self._loop_thread.start()

        # ── Thread pool for blocking ops ──────────────────────────────────────
        self._executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="cnc")

        # ── Cosmos HTTP client (reused across calls) ──────────────────────────
        self._http = httpx.AsyncClient(timeout=COSMOS_TIMEOUT)

        # ── Busy flag — one conversation turn at a time ───────────────────────
        self._busy = False

        self._log.info("✅ CNC ready — waiting for input on " + TOPIC_INPUT)
        self.mcc.log_stats()

    # ── ROS2 callback ─────────────────────────────────────────────────────────

    def _on_input(self, msg: String):
        """
        Called by ROS2 spin thread when a message arrives on /aurora/grace/input.
        Dispatches to asyncio loop — never blocks spin.
        """
        user_input = msg.data.strip()
        if not user_input:
            return

        if self._busy:
            self._log.warning("⚠️  CNC busy — dropping input: " + user_input[:40])
            self._publish_error("GRACE is still thinking. Please wait.")
            return

        # Dispatch to async loop
        asyncio.run_coroutine_threadsafe(
            self._handle_turn(user_input), self._loop
        )

    # ── Main conversation turn ────────────────────────────────────────────────

    async def _handle_turn(self, user_input: str):
        """
        Full conversation turn:
            1. Add user turn to memory
            2. Build context window (WMC + EMC)
            3. Stream Cosmos response
            4. Add assistant response to memory
        """
        self._busy = True
        self._log.info(f"▶ [{datetime.now().strftime('%H:%M:%S')}] user: {user_input[:60]}")

        try:
            # 1. Add user turn to memory
            await self.mcc.add_turn("user", user_input)

            # 2. Build context window
            memory_context = await self.mcc.build_context(user_input)

            # 3. Assemble messages for Cosmos
            messages = self._build_messages(memory_context)

            # 4. Stream Cosmos response
            full_response = await self._stream_cosmos(messages)

            # 5. Add assistant response to memory
            if full_response:
                await self.mcc.add_turn("assistant", full_response)
                self._log.info(
                    f"◀ [{datetime.now().strftime('%H:%M:%S')}] "
                    f"grace: {full_response[:60]}…"
                )

        except Exception as exc:
            self._log.error(f"CNC turn error: {exc}")
            self._publish_error(str(exc))

        finally:
            self._busy = False
            self.mcc.log_stats()

    # ── Message assembly ──────────────────────────────────────────────────────

    def _build_messages(self, memory_context: list[dict]) -> list[dict]:
        """
        Build full message list for Cosmos:
            [system prompt] + [memory context (EMC episodes + WMC turns)]
        """
        system_msg = {
            "role": "system",
            "content": GRACE_SYSTEM_PROMPT.format(
                date=datetime.now().strftime("%A, %B %d, %Y")
            ),
        }
        return [system_msg] + memory_context

    # ── Cosmos streaming ──────────────────────────────────────────────────────

    async def _stream_cosmos(self, messages: list[dict]) -> str:
        """
        Stream tokens from Cosmos vLLM (OpenAI-compatible SSE).
        Publishes each chunk to /aurora/grace/response.
        Returns the full assembled response string.
        """
        payload = {
            "model":       COSMOS_MODEL,
            "messages":    messages,
            "max_tokens":  COSMOS_MAX_TOKENS,
            "temperature": COSMOS_TEMP,
            "stream":      True,
        }

        full_response = ""
        first_chunk   = True

        try:
            async with self._http.stream(
                "POST", COSMOS_URL, json=payload
            ) as response:

                response.raise_for_status()

                async for line in response.aiter_lines():
                    if not line.startswith("data: "):
                        continue

                    data_str = line[6:]  # strip "data: "
                    if data_str.strip() == "[DONE]":
                        break

                    try:
                        chunk = json.loads(data_str)
                        delta = (
                            chunk.get("choices", [{}])[0]
                            .get("delta", {})
                            .get("content", "")
                        )
                    except (json.JSONDecodeError, IndexError, KeyError):
                        continue

                    if not delta:
                        continue

                    full_response += delta

                    # Publish streaming chunk
                    msg_type = "start" if first_chunk else "chunk"
                    first_chunk = False
                    self._publish_json({"type": msg_type, "content": delta})

            # Publish end signal with full response
            self._publish_json({"type": "end", "content": full_response})
            return full_response

        except httpx.ConnectError:
            err = "Cannot connect to Cosmos vLLM. Is the server running?"
            self._log.error(f"❌ {err}")
            self._publish_error(err)
            return ""

        except httpx.HTTPStatusError as exc:
            err = f"Cosmos HTTP error: {exc.response.status_code}"
            self._log.error(f"❌ {err}")
            self._publish_error(err)
            return ""

        except Exception as exc:
            err = f"Cosmos stream error: {exc}"
            self._log.error(f"❌ {err}")
            self._publish_error(err)
            return ""

    # ── Publish helpers ───────────────────────────────────────────────────────

    def _publish_json(self, data: dict):
        """Publish a JSON dict to the response topic."""
        msg      = String()
        msg.data = json.dumps(data)
        self._pub.publish(msg)

    def _publish_error(self, message: str):
        """Publish an error message to the response topic."""
        self._publish_json({"type": "error", "content": message})

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        """Clean shutdown — close memory and HTTP client."""
        self._log.info("CNC shutting down…")
        self.mcc.close()

        # Close HTTP client from asyncio loop
        future = asyncio.run_coroutine_threadsafe(
            self._http.aclose(), self._loop
        )
        try:
            future.result(timeout=3.0)
        except Exception:
            pass

        self._loop.call_soon_threadsafe(self._loop.stop)
        self._loop_thread.join(timeout=3.0)
        self._executor.shutdown(wait=False)
        super().destroy_node()
        self._log.info("✅ CNC shutdown complete")


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CNC()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested.")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
