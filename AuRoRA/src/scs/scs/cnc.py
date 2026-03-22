"""
CNC — Central Neural Core
===========================
AuRoRA · Semantic Cognitive System (SCS)

ROS2 node — the brain of GRACE.
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
VLLM_BASE_URL   = "http://localhost:8000"
VLLM_MODEL      = "embedl/Cosmos-Reason2-2B-W4A16-Edge2-FlashHead"
VLLM_MAX_TOKENS = 512
VLLM_TEMP       = 0.7
VLLM_TIMEOUT    = 60.0
# ─────────────────────────────────────────────────────────────────────────────

# ── ROS2 topics ───────────────────────────────────────────────────────────────
TOPIC_INPUT    = "/cns/neural_input"
TOPIC_IMAGE    = "/cns/image_input"
TOPIC_REWARD   = "/cns/rl_reward"
TOPIC_RESPONSE = "/gce/response"
# ─────────────────────────────────────────────────────────────────────────────

# ── GRACE personality ─────────────────────────────────────────────────────────
GRACE_SYSTEM_PROMPT = """You are GRACE — Generative Reasoning Agentic Cognitive Entity.
You are the AI mind of AuRoRA, an autonomous robot.
Current date: {date}
"""
# ─────────────────────────────────────────────────────────────────────────────


class CNC(Node):
    def __init__(self):
        super().__init__("cnc")
        self.get_logger().info("🧠 CNC starting...")

        # ── Memory ──
        self.mcc = MCC(logger=self.get_logger())

        # ── asyncio event loop ──
        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._loop_thread.start()

        # ── vLLM HTTP client ──
        self._http = httpx.AsyncClient(base_url=VLLM_BASE_URL, timeout=httpx.Timeout(VLLM_TIMEOUT))

        # ── ROS2 topics ──
        self._sub_input = self.create_subscription(String, TOPIC_INPUT, self._on_input, 10)
        self._sub_image = self.create_subscription(String, TOPIC_IMAGE, self._on_image, 10)
        self._sub_reward = self.create_subscription(String, TOPIC_REWARD, self._on_reward, 10)
        self._pub = self.create_publisher(String, TOPIC_RESPONSE, 10)

        self._busy = False
        self.get_logger().info("🌸 GRACE is ready and subscribed to Vision/RL topics")

    def _on_input(self, msg: String):
        user_input = msg.data.strip()
        if not user_input or self._busy: return
        self.get_logger().info(f"📝 Input: {user_input[:80]}")
        asyncio.run_coroutine_threadsafe(self._handle(user_input), self._loop)

    def _on_image(self, msg: String):
        img_info = msg.data.strip()
        if img_info:
            self.get_logger().info(f"📸 Image Input: {img_info[:80]}")
            # Use 'user' role for external vision input to be compatible with MCC/WMC
            asyncio.run_coroutine_threadsafe(self.mcc.add_turn("user", f"[Vision] {img_info}"), self._loop)

    def _on_reward(self, msg: String):
        reward_info = msg.data.strip()
        if reward_info:
            self.get_logger().info(f"💎 RL Reward: {reward_info}")
            asyncio.run_coroutine_threadsafe(self.mcc.add_turn("user", f"[Reward] {reward_info}"), self._loop)

    async def _handle(self, user_input: str):
        self._busy = True
        try:
            await self.mcc.add_turn("user", user_input)
            memory_context = await self.mcc.build_context(user_input)
            
            system_prompt = GRACE_SYSTEM_PROMPT.format(date=datetime.now().strftime("%Y-%m-%d"))
            messages = [{"role": "system", "content": system_prompt}]
            messages.extend(memory_context)
            messages.append({"role": "user", "content": user_input})

            full_response = await self._stream_cosmos(messages)
            if full_response:
                await self.mcc.add_turn("assistant", full_response)
            
            # Use safe stats logging
            if hasattr(self.mcc, 'log_stats'):
                self.mcc.log_stats()

        except Exception as exc:
            self.get_logger().error(f"❌ CNC error: {exc}")
        finally:
            self._busy = False

    async def _stream_cosmos(self, messages: list[dict]) -> str:
        payload = {"model": VLLM_MODEL, "messages": messages, "stream": True}
        full_response = ""
        try:
            async with self._http.stream("POST", "/v1/chat/completions", json=payload) as resp:
                if resp.status_code != 200: return ""
                async for line in resp.aiter_lines():
                    if not line.startswith("data:"): continue
                    data_str = line[len("data:"):].strip()
                    if data_str == "[DONE]": break
                    try:
                        chunk = json.loads(data_str)
                        delta = chunk.get("choices", [{}])[0].get("delta", {}).get("content", "")
                        full_response += delta
                        self._publish({"type": "delta", "content": delta})
                    except: continue
            self._publish({"type": "done", "content": full_response})
        except: pass
        return full_response

    def _publish(self, payload: dict):
        try:
            msg = String()
            msg.data = json.dumps(payload)
            self._pub.publish(msg)
        except: pass

    def destroy_node(self):
        self.mcc.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CNC()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
