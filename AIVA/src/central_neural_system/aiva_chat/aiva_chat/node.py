"""ROS adapter between TelepathicSignal, a local LLM, and the WebUI stream."""
from __future__ import annotations

import json
import os
from concurrent.futures import ThreadPoolExecutor

import httpx
import rclpy
from aiva_cryptic_signal_core import CrypticSignalCore
from cryptic_signal_core.msg import TelepathicSignal
from rclpy.node import Node
from std_msgs.msg import String

from .memory import GraceMemory


class AivaChat(Node):
    """Decode WebUI messages, call an OpenAI-compatible LLM, and stream replies."""

    def __init__(self) -> None:
        super().__init__("aiva_chat")
        self.declare_parameter("input_topic", "/telepathic/stimulus")
        self.declare_parameter("response_topic", "/tms/text_response")
        self.declare_parameter("llm_base_url", os.getenv("AIVA_LLM_BASE_URL", "http://127.0.0.1:11434/v1"))
        self.declare_parameter("llm_model", os.getenv("AIVA_LLM_MODEL", "llama3.2"))
        self.declare_parameter("system_prompt", "You are Grace, AIVA's thoughtful and helpful companion.")
        self.declare_parameter("database_url", os.getenv("AIVA_DATABASE_URL", ""))
        self.declare_parameter("memory_turns", 8)
        self._base_url = str(self.get_parameter("llm_base_url").value).rstrip("/")
        self._model = str(self.get_parameter("llm_model").value)
        self._system_prompt = str(self.get_parameter("system_prompt").value)
        database_url = str(self.get_parameter("database_url").value)
        self._memory = GraceMemory(database_url, int(self.get_parameter("memory_turns").value))
        self._cryptic = CrypticSignalCore()
        self._api_key = os.getenv("AIVA_LLM_API_KEY", "")
        self._pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="aiva-llm")
        self._response_pub = self.create_publisher(
            String, str(self.get_parameter("response_topic").value), 10
        )
        self._input_sub = self.create_subscription(
            TelepathicSignal,
            str(self.get_parameter("input_topic").value),
            self._on_signal,
            10,
        )
        self.get_logger().info(
            f"AIVA chat listening on {self.get_parameter('input_topic').value}; "
            f"LLM={self._base_url} model={self._model}"
        )

    def _on_signal(self, signal: TelepathicSignal) -> None:
        if signal.channel not in {"aiva", "agi", ""}:
            return
        try:
            text = self._cryptic.decode(bytes(signal.signal), int(signal.pulse), signal.scheme).strip()
        except (TypeError, ValueError) as error:
            self.get_logger().warning(f"Ignoring invalid TelepathicSignal: {error}")
            return
        if not text:
            return
        self._pool.submit(self._answer, text, signal.identifier)

    def _publish(self, event: dict[str, str]) -> None:
        message = String()
        message.data = json.dumps(event, ensure_ascii=False)
        self._response_pub.publish(message)

    def _answer(self, text: str, identifier: str) -> None:
        try:
            messages = [{"role": "system", "content": self._system_prompt}]
            messages.extend(self._memory.recent())
            messages.append({"role": "user", "content": text})
            headers = {"Authorization": f"Bearer {self._api_key}"} if self._api_key else {}
            response = httpx.post(
                f"{self._base_url}/chat/completions",
                json={"model": self._model, "messages": messages, "stream": False},
                headers=headers,
                timeout=120,
            )
            response.raise_for_status()
            answer = response.json()["choices"][0]["message"]["content"].strip()
            if not answer:
                raise ValueError("LLM returned an empty response")
            self._memory.add_exchange(text, answer)
            self._publish({"type": "start", "content": ""})
            self._publish({"type": "delta", "content": answer})
            self._publish({"type": "done", "content": "", "radio_id": identifier})
        except Exception as error:  # report provider/network failures to the chat UI
            self.get_logger().error(f"AIVA LLM request failed: {error}")
            self._publish({"type": "error", "content": f"AIVA could not reach its LLM: {error}"})

    def destroy_node(self) -> None:
        self._pool.shutdown(wait=False, cancel_futures=True)
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AivaChat()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
