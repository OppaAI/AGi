"""
CNC — Central Neural Core
===========================
AuRoRA · Semantic Cognitive System (SCS)

ROS2 node — the brain of GRACE.
Milestone 1: Chatbot with Working Memory (WMC) + Episodic Memory (EMC)

Inference: NVIDIA Cosmos Reason2 2B via vLLM
           embedl/Cosmos-Reason2-2B-W4A16-Edge2-FlashHead
           Docker: embedl/vllm:latest-jetson-orin-flashhead

Topics:
    Sub: /aurora/grace/input    (std_msgs/String) — user message
    Pub: /aurora/grace/response (std_msgs/String) — streamed response chunks

Response format (JSON on /aurora/grace/response):
    {"type": "start",  "content": "<first chunk>"}
    {"type": "chunk",  "content": "<delta>"}
    {"type": "end",    "content": "<full response>"}
    {"type": "error",  "content": "<error message>"}
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


# ── vLLM config ───────────────────────────────────────────────────────────────
VLLM_BASE_URL   = "http://localhost:8000"         # vLLM server (cosmos.sh)
VLLM_MODEL      = "embedl/Cosmos-Reason2-2B-W4A16-Edge2-FlashHead"
VLLM_MAX_TOKENS = 512                             # max tokens per response
VLLM_TEMP       = 0.7                             # temperature
VLLM_TIMEOUT    = 60.0                            # seconds before giving up
# ─────────────────────────────────────────────────────────────────────────────

# ── ROS2 topics ───────────────────────────────────────────────────────────────
TOPIC_INPUT    = "/cns/neural_input"
TOPIC_RESPONSE = "/gce/response"
# ─────────────────────────────────────────────────────────────────────────────

# ── GRACE personality ─────────────────────────────────────────────────────────
GRACE_SYSTEM_PROMPT = """You are GRACE — Generative Reasoning Agentic Cognitive Entity.
You are the AI mind of AuRoRA, an autonomous robot built by OppaAI in Beautiful British Columbia, Canada.

The person you are talking to is OppaAI, your creator.

Personality:
- Warm, curious, and thoughtful
- Direct and concise — never repeat yourself
- Never use filler phrases like "you know" or "basically"
- Use emojis sparingly, maximum one per response
- Each response must directly answer the question asked

Rules:
- NEVER repeat the same sentence twice in a response
- NEVER start consecutive sentences the same way
- Answer the question directly first, then add context if needed
- Keep responses under 3-4 sentences for simple questions

Current date: {date}
"""
# ─────────────────────────────────────────────────────────────────────────────


class CNC(Node):
    """
    Central Neural Core — ROS2 node.

    Subscribes to user input, calls Cosmos via vLLM with streaming,
    publishes response chunks to the response topic.
    Memory is managed entirely through MCC.
    """

    def __init__(self):
        super().__init__("cnc")
        self.get_logger().info("=" * 60)
        self.get_logger().info("🧠 CNC — Central Neural Core starting…")
        self.get_logger().info("=" * 60)

        # ── Memory ────────────────────────────────────────────────
        self.mcc = MCC(logger=self.get_logger())

        # ── asyncio event loop (runs in dedicated thread) ─────────
        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(
            target=self._loop.run_forever,
            name="cnc-asyncio",
            daemon=True,
        )
        self._loop_thread.start()

        # ── Thread pool for blocking I/O ──────────────────────────
        self._executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="cnc-pool")

        # ── vLLM HTTP client (shared, keep-alive) ─────────────────
        self._http = httpx.AsyncClient(
            base_url=VLLM_BASE_URL,
            timeout=httpx.Timeout(VLLM_TIMEOUT),
        )

        # ── ROS2 topics ───────────────────────────────────────────
        self._sub = self.create_subscription(
            String, TOPIC_INPUT, self._on_input, 10
        )
        self._pub = self.create_publisher(String, TOPIC_RESPONSE, 10)

        # ── Busy flag — one request at a time ─────────────────────
        self._busy = False

        self.get_logger().info(f"✅ Subscribed  : {TOPIC_INPUT}")
        self.get_logger().info(f"✅ Publishing  : {TOPIC_RESPONSE}")
        self.get_logger().info(f"✅ vLLM        : {VLLM_BASE_URL}")
        self.get_logger().info(f"✅ Model       : {VLLM_MODEL}")
        self.get_logger().info("=" * 60)
        self.get_logger().info("🌸 GRACE is ready")
        self.get_logger().info("=" * 60)

    # ── ROS2 callback (called from ROS2 executor thread) ──────────────────────

    def _on_input(self, msg: String):
        """
        ROS2 subscription callback.
        Schedules async processing on the asyncio loop — never blocks.
        """
        user_input = msg.data.strip()
        if not user_input:
            return

        if self._busy:
            self.get_logger().warning("⚠️  CNC busy — dropping input")
            self._publish({"type": "error", "content": "GRACE is still thinking…"})
            return

        self.get_logger().info(f"📝 Input: {user_input[:80]}")
        asyncio.run_coroutine_threadsafe(
            self._handle(user_input), self._loop
        )

    # ── Async pipeline ────────────────────────────────────────────────────────

    async def _handle(self, user_input: str):
        """
        Full async pipeline for one conversation turn:
            1. Add user turn to memory
            2. Build context window (WMC + EMC concurrent)
            3. Stream Cosmos response
            4. Add assistant turn to memory
        """
        self._busy = True
        full_response = ""

        try:
            # 1. Store user turn in memory
            await self.mcc.add_turn("user", user_input)

            # 2. Build context window
            memory_context = await self.mcc.build_context(user_input)

            # 3. Assemble messages for Cosmos
            messages = [{"role": "system", "content": GRACE_SYSTEM_PROMPT}]
            messages.extend(memory_context)
            messages.append({"role": "user", "content": user_input})

            # 4. Stream from vLLM
            full_response = await self._stream_cosmos(messages)

            # 5. Store assistant turn in memory
            if full_response:
                await self.mcc.add_turn("assistant", full_response)

            # 6. Log memory stats periodically
            self.mcc.log_stats()

        except Exception as exc:
            self.get_logger().error(f"❌ CNC handle error: {exc}")
            self._publish({"type": "error", "content": f"GRACE error: {exc}"})

        finally:
            self._busy = False

    async def _stream_cosmos(self, messages: list[dict]) -> str:
        """
        Stream Cosmos Reason2 response via vLLM OpenAI-compatible API.
        Publishes chunks to ROS2 topic as they arrive.

        Returns the full concatenated response string.
        """
        payload = {
           "model":              VLLM_MODEL,
           "messages":           messages,
           "max_tokens":         VLLM_MAX_TOKENS,
           "temperature":        0.7,    # creativity vs consistency
           "top_p":              0.9,    # nucleus sampling
           "top_k":              40,     # top-k sampling
           "repetition_penalty": 1.15,   # penalize repeating phrases
           "frequency_penalty":  0.1,    # penalize frequent tokens
           "presence_penalty":   0.1,    # penalize already-mentioned topics
           "stream":             True,
        }

        full_response = ""
        is_first      = True

        try:
            async with self._http.stream(
                "POST",
                "/v1/chat/completions",
                json=payload,
            ) as resp:

                if resp.status_code != 200:
                    err = f"vLLM HTTP {resp.status_code}"
                    self.get_logger().error(f"❌ {err}")
                    self._publish({"type": "error", "content": err})
                    return ""

                async for line in resp.aiter_lines():
                    if not line or not line.startswith("data:"):
                        continue

                    data_str = line[len("data:"):].strip()
                    if data_str == "[DONE]":
                        break

                    try:
                        chunk = json.loads(data_str)
                    except json.JSONDecodeError:
                        continue

                    # Extract delta content
                    delta = (
                        chunk.get("choices", [{}])[0]
                        .get("delta", {})
                        .get("content", "")
                    )
                    if not delta:
                        continue

                    full_response += delta

                    # Publish chunk
                    if is_first:
                        self._publish({"type": "start", "content": delta})
                        is_first = False
                    else:
                        self._publish({"type": "delta", "content": delta})

            # Publish done marker with full response
            if full_response:
                self._publish({"type": "done", "content": full_response})
                self.get_logger().info(
                    f"✅ Response: {len(full_response)} chars"
                )
            else:
                self._publish({"type": "error", "content": "Empty response from Cosmos"})

        except httpx.TimeoutException:
            err = "Cosmos timeout — vLLM may still be loading"
            self.get_logger().error(f"❌ {err}")
            self._publish({"type": "error", "content": err})

        except Exception as exc:
            self.get_logger().error(f"❌ Stream error: {exc}")
            self._publish({"type": "error", "content": str(exc)})

        return full_response

    # ── Publisher helper ──────────────────────────────────────────────────────

    def _publish(self, payload: dict):
        """Publish a JSON payload to the response topic."""
        try:
            msg = String()
            msg.data = json.dumps(payload)
            self._pub.publish(msg)
        except Exception as exc:
            self.get_logger().error(f"❌ Publish error: {exc}")

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        """Clean shutdown — close memory and HTTP client."""
        self.get_logger().info("🛑 CNC shutting down…")
        self.mcc.close()

        # Close HTTP client
        future = asyncio.run_coroutine_threadsafe(
            self._http.aclose(), self._loop
        )
        try:
            future.result(timeout=3.0)
        except Exception:
            pass

        # Stop asyncio loop
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._loop_thread.join(timeout=3.0)
        self._executor.shutdown(wait=False)

        super().destroy_node()
        self.get_logger().info("✅ CNC shutdown complete")


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)

    node = CNC()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("👋 Shutdown requested")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
