"""Authenticated HTTP ingress for PC telemetry and robot requests.

This is a thin edge adapter: validate a request, serialize a typed
NeuralImpulse, and publish it to ROS 2. It does not execute homing itself.
"""
from __future__ import annotations

import hmac
import json
import os
import threading
import time
import uuid
from typing import Annotated

from fastapi import Depends, FastAPI, Header, HTTPException, Request
from pydantic import BaseModel, ConfigDict, Field
import rclpy
from rclpy.node import Node
from nes.msg import NeuralImpulse
import uvicorn

from hrs.hrm import AGi


class TelemetryRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    robot_id: str = Field(min_length=1, max_length=64, pattern=r"^[A-Za-z0-9_.:-]+$")
    timestamp: str | None = Field(default=None, max_length=40)
    metrics: dict[str, float | bool] = Field(min_length=1, max_length=32)


class HomingRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    robot_id: str = Field(min_length=1, max_length=64, pattern=r"^[A-Za-z0-9_.:-]+$")
    request_id: str = Field(default_factory=lambda: str(uuid.uuid4()), max_length=64)
    target: str = Field(default="all", min_length=1, max_length=32, pattern=r"^[A-Za-z0-9_.:-]+$")
    operation: str = Field(default="request", pattern=r"^request$")


class ApiNode(Node):
    def __init__(self):
        super().__init__("agi_http_api")
        self._impulses = self.create_publisher(NeuralImpulse, AGi.TMS.API_IMPULSE_GATEWAY, 10)

    def publish_impulse(self, signal_type: str, robot_id: str, payload: dict, *, correlation_id: str = "") -> str:
        msg = NeuralImpulse()
        msg.nis_id = f"nis_{uuid.uuid4().hex}"
        msg.user_id = ""  # Network clients cannot choose a trusted user identity.
        msg.session_id = ""
        msg.sequence = 0
        msg.interval = 0
        msg.axon = "pc_api"
        msg.dendrite = "vcs" if signal_type == "health_telemetry" else "homing_controller"
        msg.pathway = "afferent"
        msg.direction = "ascending"
        msg.signal_type = signal_type
        msg.modality = "spatial" if signal_type == "health_telemetry" else "motor_request"
        msg.trace_type = ""
        msg.raw = json.dumps({"robot_id": robot_id, **payload}, separators=(",", ":"), allow_nan=False)
        msg.encoding = "json"
        msg.schema = "agi.health_telemetry.v1" if signal_type == "health_telemetry" else "agi.homing_request.v1"
        msg.vector = []
        msg.urgency = 0.0
        msg.amplitude = 1.0
        msg.confidence = 1.0
        msg.state = "fired"
        msg.fired_at = self.get_clock().now().to_msg()
        msg.internal = False
        msg.requires_response = signal_type == "homing_request"
        msg.refractory = False
        msg.correlation_id = correlation_id
        self._impulses.publish(msg)
        return msg.nis_id


def create_app(node: ApiNode, token: str | None = None) -> FastAPI:
    app = FastAPI(title="AGi PC Integration API", version="1.0.0", docs_url=None, redoc_url=None)
    secret = token if token is not None else os.getenv("AGI_API_TOKEN", "")
    last_request: dict[str, float] = {}
    rate_lock = threading.Lock()

    async def authenticate(authorization: Annotated[str | None, Header()] = None) -> str:
        if not secret:
            raise HTTPException(status_code=503, detail="AGI_API_TOKEN is not configured")
        scheme, _, supplied = (authorization or "").partition(" ")
        if scheme.lower() != "bearer" or not hmac.compare_digest(supplied, secret):
            raise HTTPException(status_code=401, detail="Bearer token required")
        return "authenticated-pc"

    async def rate_limit(request: Request) -> None:
        host = request.client.host if request.client else "unknown"
        now = time.monotonic()
        with rate_lock:
            last = last_request.get(host, 0.0)
            if now - last < 0.1:
                raise HTTPException(status_code=429, detail="Request rate limited")
            last_request[host] = now
            if len(last_request) > 64:
                for address, stamp in list(last_request.items()):
                    if stamp < now - 60:
                        last_request.pop(address, None)

    @app.get("/health/live")
    async def live():
        return {"status": "ok", "service": "agi-http-api"}

    @app.post("/api/v1/telemetry/health", status_code=202, dependencies=[Depends(rate_limit)])
    async def health_telemetry(body: TelemetryRequest, _identity: str = Depends(authenticate)):
        impulse_id = node.publish_impulse("health_telemetry", body.robot_id,
            {"timestamp": body.timestamp, "metrics": body.metrics})
        return {"status": "accepted", "impulse_id": impulse_id}

    @app.post("/api/v1/robot/homing", status_code=202, dependencies=[Depends(rate_limit)])
    async def homing_request(body: HomingRequest, _identity: str = Depends(authenticate)):
        impulse_id = node.publish_impulse("homing_request", body.robot_id,
            {"request_id": body.request_id, "target": body.target, "operation": body.operation},
            correlation_id=body.request_id)
        return {"status": "queued", "impulse_id": impulse_id,
                "note": "A homing controller must subscribe to the AGi API impulse topic to execute this request."}

    return app


def main(args=None):
    rclpy.init(args=args)
    node = ApiNode()
    host = os.getenv("AGI_API_HOST", AGi.TMS.API_GATEWAY)
    port = int(os.getenv("AGI_API_PORT", str(AGi.TMS.API_PORTAL)))
    cert = os.getenv("AGI_API_TLS_CERT", "")
    key = os.getenv("AGI_API_TLS_KEY", "")
    remote_bind = host not in {"127.0.0.1", "localhost", "::1"}
    if remote_bind and (not os.getenv("AGI_API_TOKEN") or not cert or not key):
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError("A non-loopback API bind requires AGI_API_TOKEN and AGI_API_TLS_CERT/AGI_API_TLS_KEY")
    server = uvicorn.Server(uvicorn.Config(create_app(node), host=host, port=port,
        log_level="info", access_log=False, ssl_certfile=cert or None, ssl_keyfile=key or None,
        limit_concurrency=32, backlog=64))
    thread = threading.Thread(target=server.run, name="agi-http-api", daemon=True)
    thread.start()
    node.get_logger().info(f"AGi HTTP API listening on http://{host}:{port}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.should_exit = True
        thread.join(timeout=5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
