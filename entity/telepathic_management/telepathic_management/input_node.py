"""Normalize shared telepathic input messages and publish canonical stimuli."""
from __future__ import annotations

import json
import os
import time
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TelepathicInput(Node):
    """Receive JSON input from adapters and publish canonical SSS JSON."""

    def __init__(self) -> None:
        super().__init__("telepathic_input")
        self.declare_parameter("input_topic", "/telepathic/input")
        self.declare_parameter("stimulus_topic", "/telepathic/stimulus")
        self.declare_parameter("response_echo_topic", "/telepathic/response_echo")
        self.declare_parameter("max_payload_bytes", 8192)
        self.declare_parameter("refractory_period", 1.0)
        self._max_payload_bytes = int(self.get_parameter("max_payload_bytes").value)
        self._refractory_period = float(self.get_parameter("refractory_period").value)
        self._last_content = ""
        self._last_received = 0.0
        self._stimulus_pub = self.create_publisher(
            String, str(self.get_parameter("stimulus_topic").value), 10
        )
        self._echo_sub = self.create_subscription(
            String, str(self.get_parameter("response_echo_topic").value), self._on_echo, 10
        )
        self._input_sub = self.create_subscription(
            String, str(self.get_parameter("input_topic").value), self._on_input, 10
        )
        self._response_echoes: dict[str, float] = {}
        self.get_logger().info(
            f"Ready: {self.get_parameter('input_topic').value} -> "
            f"{self.get_parameter('stimulus_topic').value}"
        )

    def _on_echo(self, message: String) -> None:
        try:
            value = json.loads(message.data)
            imprint = value.get("imprint") if isinstance(value, dict) else None
            if isinstance(imprint, str):
                self._response_echoes[imprint] = time.monotonic() + float(value.get("duration", 30))
        except (ValueError, TypeError):
            self.get_logger().warning("Ignoring malformed response echo")

    def _on_input(self, message: String) -> None:
        try:
            raw = json.loads(message.data)
        except ValueError:
            raw = {"text": message.data}
        if isinstance(raw, str):
            raw = {"text": raw}
        if not isinstance(raw, dict) or not isinstance(raw.get("text"), str):
            self.get_logger().warning("Ignoring input without text")
            return
        text = raw["text"].strip()
        if not text or len(text.encode("utf-8")) > self._max_payload_bytes:
            self.get_logger().warning("Ignoring empty or oversized input")
            return
        now = time.monotonic()
        if text == self._last_content and now - self._last_received < self._refractory_period:
            self.get_logger().info("Suppressing repeated input")
            return
        self._last_content, self._last_received = text, now
        source = raw.get("source", "webui")
        stimulus = {
            "sss_id": str(raw.get("radio_id") or uuid.uuid4()),
            "role": "user",
            "text": text,
            "user_id": str(raw.get("user_id", "user")),
            "source": str(source),
            "trace_type": "pmt",
            "generated_at": time.time(),
        }
        out = String()
        out.data = json.dumps(stimulus, ensure_ascii=False)
        self._stimulus_pub.publish(out)


def main(args: list[str] | None = None) -> None:
    """Run the shared telepathic input ROS node."""
    rclpy.init(args=args)
    node = TelepathicInput()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
