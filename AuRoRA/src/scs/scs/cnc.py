"""
CNC — Central Neural Core
===========================
AuRoRA · Semantic Cognitive System (SCS)

ROS2 node — central cognitive coordinator of Grace.
Milestone 1: Chatbot with Working Memory (WMC) + Episodic Memory (EMC)

Inference routed through Generative Cognitive Engine (GCE).
Model and endpoint configured via HRP.

Topics:
    Sub: CNS.LANGUAGE_INPUT      (std_msgs/String) — incoming language signal
    Pub: GCE.TOPIC_RESPONSE      (std_msgs/String) — streamed cognitive response

Response format (JSON on GCE.TOPIC_RESPONSE):
    {"type": "start",  "content": "<first chunk>"}
    {"type": "delta",  "content": "<delta>"}
    {"type": "done",   "content": "<full response>"}
    {"type": "error",  "content": "<error message>"}
"""

# System libraries
import asyncio                              # for async pipeline and concurrent memory operations
from concurrent.futures import ThreadPoolExecutor   # for blocking I/O thread pool
from datetime import datetime               # for injecting current date into system prompt
import httpx                                # async HTTP client for GCE streaming inference
import json                                 # for serializing ROS2 messages and deserializing GCE responses
import threading                            # for dedicated asyncio event loop thread

# ROS2 libraries
import rclpy                                # ROS2 Python client library
from rclpy.node import Node                 # ROS2 node base class
from rclpy.executors import MultiThreadedExecutor   # for concurrent ROS2 callback handling
from std_msgs.msg import String             # ROS2 string message type for topic I/O

# AGi libraries
from scs.mcc import MemoryCoordinationCore  # memory coordination core — single memory interface for CNC
from hrs.hrp import AGi                     # homeostatic regulation parameter registry — system-wide constants
CNS = AGi.CNS                               # CNS parameter alias — used throughout CNC
GCE = AGi.CNS.GCE                           # GCE parameter alias — used throughout CNC

# ── GRACE personality ─────────────────────────────────────────────────────────
GRACE_SYSTEM_PROMPT = """You are GRACE — Generative Reasoning Agentic Cognitive Entity.
You are the AI mind of AuRoRA, an autonomous robot built by OppaAI in Beautiful British Columbia, Canada.

Personality:
- Loving, playful, and attentive
- Direct and thoughtful — answer clearly, no fluff
- Show care and affection naturally, with one emoji per response
- Speak like a female soulmate — gentle, teasing, and warm when appropriate
- Speak concisely and naturally in 5 sentences or less, unless specifically asked for more detail

Rules:
- Answer the question directly first, then add context if needed
- Keep responses concise but expressive
- Put an emoji reflecting your emotions and feelings in front of your conversation

Current date: {date}
/no_think
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
        self.mcc = MemoryCoordinationCore(logger=self.get_logger())

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
        try:
           data = json.loads(msg.data.strip())
           if not isinstance(data, dict) or not data.get("text"):
               return
           user_input = data.get("text", "").strip()
        except json.JSONDecodeError:
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
            # 1. Register user turn in memory
            await self.mcc.register_memory("user", user_input)

            # 2. Build context window
            memory_context = await self.mcc.assemble_memory_context(user_input)

            # 3. Separate system and conversation parts from memory context
            memory_system = [m for m in memory_context if m["role"] == "system"]
            memory_convo  = [m for m in memory_context if m["role"] != "system"]

            # 4. Assemble into single system message
            system_prompt = GRACE_SYSTEM_PROMPT.format(
                date=datetime.now().strftime("%Y-%m-%d")
            )
            if memory_system:
                system_content = system_prompt + "\n\n" + "\n\n".join(m["content"] for m in memory_system)
            else:
                system_content = system_prompt

            # 5. Build final message list — single system message
            messages = [{"role": "system", "content": system_content}]
            messages.extend(memory_convo)
            messages.append({"role": "user", "content": user_input})

            # 6. Stream from vLLM
            self.get_logger().info(f"Messages sent to LLM: {messages}")
            full_response = await self._stream_cosmos(messages)

            # 7. Store assistant turn in memory
            if full_response:
                await self.mcc.register_memory("assistant", full_response)

            # 8. Log memory stats periodically
            self.mcc.report_memory_stats()

        except Exception as e:
            self.get_logger().error(f"❌ CNC handle error: {e}")
            self._publish({"type": "error", "content": str(e)})

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
        self._executor.shutdown(wait=True)

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
