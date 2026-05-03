"""
CNC — Central Neural Core
===========================
AuRoRA · Semantic Cognitive System (SCS)

ROS2 node — central cognitive coordinator of Grace.
Milestone 1: Chatbot with Working Memory (WMC) + Episodic Memory (EMC)

Inference routed through Generative Cognitive Engine (GCE).
Model and endpoint configured via HRP.

Topics:
    Sub: CNS.NEURAL_TEXT_INPUT   (std_msgs/String) — incoming language signal
    Pub: GCE.COGNITIVE_RESPONSE  (std_msgs/String) — streamed cognitive response

Response format (JSON on GCE.COGNITIVE_RESPONSE):
    {"type": "start",  "content": "<first chunk>"}
    {"type": "delta",  "content": "<delta>"}
    {"type": "done",   "content": "<full response>"}
    {"type": "error",  "content": "<error message>"}
"""

# System libraries
import asyncio
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import httpx
import json
import threading

# ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

# AGi libraries
from scs.mcc import MemoryCoordinationCore
from hrs.hrp import AGi
CNS = AGi.CNS
GCE = AGi.CNS.GCE


class CNC(Node):
    """
    Central Neural Core — ROS2 node.

    Subscribes to user input, routes inference through GCE with streaming,
    publishes response chunks to the cognitive response topic.
    Memory is managed entirely through MCC.
    """

    def __init__(self):
        super().__init__("cnc")
        self.get_logger().info("=" * 60)
        self.get_logger().info("🧠 CNC — Central Neural Core starting…")
        self.get_logger().info("=" * 60)

        self.mcc = MemoryCoordinationCore(logger=self.get_logger())

        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(
            target=self._loop.run_forever,
            name="cnc-asyncio",
            daemon=True,
        )
        self._loop_thread.start()

        self._executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="cnc-pool")

        self._http = httpx.AsyncClient(
            base_url=GCE.NEURAL_ENDPOINT,
            timeout=httpx.Timeout(GCE.TIMEOUT),
        )

        self._sub = self.create_subscription(
            String, CNS.NEURAL_TEXT_INPUT, self._on_input, 10
        )
        self._pub = self.create_publisher(String, GCE.COGNITIVE_RESPONSE, 10)

        self._busy = False

        self.get_logger().info(f"✅ Subscribed  : {CNS.NEURAL_TEXT_INPUT}")
        self.get_logger().info(f"✅ Publishing  : {GCE.COGNITIVE_RESPONSE}")
        self.get_logger().info(f"✅ Endpoint    : {GCE.NEURAL_ENDPOINT}")
        self.get_logger().info(f"✅ Model       : {GCE.COGNITIVE_ENGINE}")
        self.get_logger().info("=" * 60)
        self.get_logger().info("🌸 GRACE is ready")
        self.get_logger().info("=" * 60)

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

    async def _handle(self, user_input: str):
        """
        Full async pipeline for one conversation turn:
            1. Register user turn in memory
            2. Assemble memory context (WMC + EMC)
            3. Stream GCE response
            4. Register assistant turn in memory
        """
        self._busy = True
        full_response = ""

        try:
            # 1. Register user turn in memory
            await self.mcc.register_memory("user", user_input)

            # 2. Assemble memory context
            memory_context = await self.mcc.assemble_memory_context(user_input)

            # 3. Separate system and conversation parts
            memory_system = [m for m in memory_context if m["role"] == "system"]
            memory_convo  = [m for m in memory_context if m["role"] != "system"]

            # 4. Assemble system prompt with date
            system_prompt = GCE.SYSTEM_PROMPT.format(
                date=datetime.now().strftime("%Y-%m-%d")
            )
            if memory_system:
                system_content = system_prompt + "\n\n" + "\n\n".join(m["content"] for m in memory_system)
            else:
                system_content = system_prompt

            # 5. Build final message list
            messages = [{"role": "system", "content": system_content}]
            messages.extend(memory_convo)
            messages.append({"role": "user", "content": user_input})

            # 6. Stream from GCE
            self.get_logger().info(f"Messages sent to GCE: {messages}")
            full_response = await self._stream_gce(messages)

            # 7. Register assistant turn in memory
            if full_response:
                await self.mcc.register_memory("assistant", full_response)

            # 8. Report memory stats
            self.mcc.report_memory_stats()

        except Exception as e:
            self.get_logger().error(f"❌ CNC handle error: {e}")
            self._publish({"type": "error", "content": str(e)})

        finally:
            self._busy = False

    async def _stream_gce(self, messages: list[dict]) -> str:
        """
        Stream GCE response via OpenAI-compatible API.
        Publishes chunks to ROS2 topic as they arrive.

        Returns:
            str: Full concatenated response string.
        """
        payload = {
            "model"              : GCE.COGNITIVE_ENGINE,
            "messages"           : messages,
            "max_tokens"         : GCE.RESPONSE_DEPTH,
            "temperature"        : GCE.TEMPERATURE,
            "top_p"              : GCE.PROBABILITY_THRESHOLD,
            "top_k"              : GCE.CANDIDATE_THRESHOLD,
            "repetition_penalty" : GCE.PERSEVERATION_DAMPING,
            "frequency_penalty"  : GCE.HABITUATION_DAMPING,
            "presence_penalty"   : GCE.NOVELTY_BIAS,
            "stream"             : True,
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
                    err = f"GCE HTTP {resp.status_code}"
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

                    delta = (
                        chunk.get("choices", [{}])[0]
                        .get("delta", {})
                        .get("content", "")
                    )
                    if not delta:
                        continue

                    full_response += delta

                    if is_first:
                        self._publish({"type": "start", "content": delta})
                        is_first = False
                    else:
                        self._publish({"type": "delta", "content": delta})

            if full_response:
                self._publish({"type": "done", "content": full_response})
                self.get_logger().info(f"✅ Response: {len(full_response)} chars")
            else:
                self._publish({"type": "error", "content": "Empty response from GCE"})

        except httpx.TimeoutException:
            err = "GCE timeout — model may still be loading"
            self.get_logger().error(f"❌ {err}")
            self._publish({"type": "error", "content": err})

        except Exception as exc:
            self.get_logger().error(f"❌ Stream error: {exc}")
            self._publish({"type": "error", "content": str(exc)})

        return full_response

    def _publish(self, payload: dict):
        """Publish a JSON payload to the cognitive response topic."""
        try:
            msg = String()
            msg.data = json.dumps(payload)
            self._pub.publish(msg)
        except Exception as exc:
            self.get_logger().error(f"❌ Publish error: {exc}")

    def destroy_node(self):
        """Clean shutdown — close memory and HTTP client."""
        self.get_logger().info("🛑 CNC shutting down…")
        self.mcc.close()

        future = asyncio.run_coroutine_threadsafe(
            self._http.aclose(), self._loop
        )
        try:
            future.result(timeout=3.0)
        except Exception:
            pass

        self._loop.call_soon_threadsafe(self._loop.stop)
        self._loop_thread.join(timeout=3.0)
        self._executor.shutdown(wait=True)

        super().destroy_node()
        self.get_logger().info("✅ CNC shutdown complete")


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