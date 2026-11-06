"""Serve the Web Teleops Console interface and proxy its browser WebSocket to local rosbridge."""
from __future__ import annotations

import asyncio
import os
import time
import uuid
import tempfile
from pathlib import Path

import uvicorn
import websockets
from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response
from fastapi.staticfiles import StaticFiles

from telepathic_management.radio_encoding import encode_text

try:
    from hrs.hrm import AGi
except ImportError:
    class AGi:
        class TMS:
            RADIO_CHANNEL = "grace"
            API_GATEWAY = "127.0.0.1"
            API_PORTAL = 8852

try:
    import yaml
except ImportError:  # pragma: no cover - ROS images normally provide python3-yaml
    yaml = None


def _interface_dir() -> Path:
    """Resolve the source checkout or the installed ROS package interface."""
    configured = os.getenv("AGI_WEB_TELEOPS_CONSOLE_INTERFACE_DIR")
    if configured:
        return Path(configured)
    try:
        from ament_index_python.packages import get_package_share_directory
        installed = Path(get_package_share_directory("telepathic_management")) / "telepathic_console"
        if (installed / "frontend" / "index.html").is_file():
            return installed
    except (ImportError, LookupError):
        pass
    return Path(__file__).resolve().parent


INTERFACE_DIR = _interface_dir()
FRONTEND_DIR = INTERFACE_DIR / "frontend"
app = FastAPI(title="AGi Web Teleops Console", docs_url=None, redoc_url=None)
app.mount("/static", StaticFiles(directory=INTERFACE_DIR / "backend" / "static"), name="static")
_aiva_last_seen: float | None = None

# Only HRM values explicitly marked [INTRINSIC] are editable here. They are
# hydrated into ROS nodes at launch, and AGi has no dynamic parameter callback
# yet, so changes are persisted for the next AuRoRA restart.
_SETTINGS = [
    ("tms.fast_path_arousal_threshold", "Fast path arousal threshold", "TMS", "Arousal above this level receives priority dispatch.", 0.7, 0.0, 1.0),
    ("scs.cortical_capacity", "Cortical capacity", "Context", "Total token budget for the active model context.", 24576, 256, 262144),
    ("scs.cognitive_reserve", "Cognitive reserve", "Context", "Tokens reserved for system prompt and identity.", 2048, 0, 65536),
    ("scs.gce.response_depth", "Response depth", "Inference", "Maximum generated tokens per completion.", 512, 1, 32768),
    ("scs.gce.context_window", "Model context window", "Inference", "Context slots requested from the inference server.", 32768, 256, 262144),
    ("scs.gce.temperature", "Temperature", "Inference", "Sampling randomness.", 0.7, 0.0, 2.0),
    ("scs.gce.probability_threshold", "Top-p", "Inference", "Nucleus sampling cutoff.", 0.9, 0.0, 1.0),
    ("scs.gce.candidate_threshold", "Top-k", "Inference", "Candidate token limit per sampling step.", 40, 0, 1000),
    ("scs.gce.perseveration_damping", "Repetition penalty", "Inference", "Scales down tokens already sampled.", 1.15, 0.0, 3.0),
    ("scs.gce.habituation_damping", "Frequency penalty", "Inference", "Scales down tokens by corpus frequency.", 0.1, -2.0, 2.0),
    ("scs.gce.novelty_bias", "Presence penalty", "Inference", "Adjusts preference for tokens not yet in context.", 0.1, -2.0, 2.0),
    ("scs.emc.binding_stream_limit", "Binding stream limit", "Episodic memory", "Maximum queued unencoded PMTs.", 512, 1, 100000),
    ("scs.emc.episode_content_limit", "Episode content limit", "Episodic memory", "Maximum tokens retained per episode.", 512, 1, 32768),
    ("scs.emc.theta_interval", "Encoding interval (s)", "Episodic memory", "Seconds between periodic encoding batches.", 2.0, 0.1, 3600.0),
    ("scs.emc.theta_batch_limit", "Encoding batch limit", "Episodic memory", "Maximum episodes encoded per batch.", 32, 1, 10000),
    ("scs.emc.encoding_cycle_timeout", "Encoding shutdown timeout (s)", "Episodic memory", "Maximum graceful encoding-thread shutdown wait.", 30.0, 0.1, 3600.0),
    ("scs.emc.encoding_prime_capacity", "Embedding cache capacity", "Episodic memory", "Maximum embeddings held in the LRU cache.", 256, 0, 1000000),
    ("scs.emc.encoding_prime_key_len", "Embedding cache key length", "Episodic memory", "Maximum characters hashed for a cache key.", 256, 1, 100000),
    ("scs.emc.recall_reserve", "Recall token reserve", "Recall", "Tokens reserved for recalled episodes.", 2048, 0, 65536),
    ("scs.emc.recall_surface_limit", "Recall surface limit", "Recall", "Maximum episodes returned after ranking.", 3, 0, 1000),
    ("scs.emc.recall_pool", "Recall candidate pool", "Recall", "Candidates retrieved per retriever.", 15, 1, 10000),
    ("scs.emc.recall_timeout", "Recall timeout (s)", "Recall", "Maximum duration for a complete recall cycle.", 2.0, 0.1, 300.0),
    ("scs.emc.recovery_batch_size", "Recovery batch size", "Recall", "Unencoded episodes recovered per startup batch.", 50, 1, 100000),
    ("scs.emc.relevance_threshold", "Relevance threshold", "Recall", "Minimum reciprocal-rank score accepted for recall.", 0.45, 0.0, 1.0),
    ("scs.wmc.pmt_slot_limit", "Working memory slots", "Working memory", "Maximum thought units held in working memory.", 7, 1, 100),
    ("scs.wmc.pmt_slot_buffer", "Working memory overflow buffer", "Working memory", "Temporary overflow allowance for thought units.", 2, 0, 100),
    ("scs.wmc.eviction_threshold", "Eviction threshold", "Working memory", "Minimum depth score before a thought is evicted.", 0.1, 0.0, 1.0),
]
_SETTING_INDEX = {entry[0]: entry for entry in _SETTINGS}


def _settings_path() -> Path:
    from hrs.hru import GatewayMap
    return GatewayMap().aurora_setpoints


def _read_settings() -> dict:
    if yaml is None:
        raise HTTPException(status_code=503, detail="YAML support is unavailable")
    path = _settings_path()
    if not path.exists():
        return {}
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError) as error:
        raise HTTPException(status_code=500, detail=f"Could not read AuRoRA settings: {error}") from error
    if not isinstance(data, dict):
        raise HTTPException(status_code=500, detail="AuRoRA settings file must contain a YAML mapping")
    return data


def _parameter_values(data: dict) -> dict:
    # ROS 2 allows node-specific selectors as well as the /** wildcard.
    # Merge them for display; wildcard values are the editable source of truth.
    merged: dict = {}
    for block in data.values():
        params = block.get("ros__parameters", {}) if isinstance(block, dict) else {}
        if isinstance(params, dict):
            merged.update(params)
    return merged


@app.get("/api/settings")
async def get_settings() -> dict:
    """Return editable intrinsic parameters and read-only runtime settings."""
    data = _read_settings()
    current = _parameter_values(data)
    settings = []
    for key, label, group, description, default, minimum, maximum in _SETTINGS:
        value = current.get(key, default)
        settings.append({"key": key, "label": label, "group": group, "description": description,
                         "value": value, "default": default, "min": minimum, "max": maximum,
                         "apply": "restart"})
    runtime = [
        {"name": "Web Teleops Console bind", "value": os.getenv("AGI_WEB_TELEOPS_CONSOLE_HOST", "127.0.0.1"), "apply": "restart", "source": "Environment"},
        {"name": "Web Teleops Console port", "value": os.getenv("AGI_WEB_TELEOPS_CONSOLE_PORT", "8853"), "apply": "restart", "source": "Environment"},
        {"name": "Web Teleops Console interface directory", "value": os.getenv("AGI_WEB_TELEOPS_CONSOLE_INTERFACE_DIR", "auto-detected from ROS package or source checkout"), "apply": "restart", "source": "Environment"},
        {"name": "ROS bridge URL", "value": os.getenv("AGI_ROSBRIDGE_URL", "ws://127.0.0.1:9090"), "apply": "restart", "source": "Environment"},
        {"name": "AIVA presence timeout", "value": os.getenv("AGI_AIVA_PRESENCE_TIMEOUT", "12"), "apply": "restart", "source": "Environment"},
        {"name": "HTTP API host", "value": os.getenv("AGI_API_HOST", AGi.TMS.API_GATEWAY), "apply": "restart", "source": "Environment"},
        {"name": "HTTP API port", "value": os.getenv("AGI_API_PORT", str(AGi.TMS.API_PORTAL)), "apply": "restart", "source": "Environment"},
        {"name": "HTTP API TLS certificate", "value": os.getenv("AGI_API_TLS_CERT") or "not configured", "apply": "restart", "source": "Environment · path only"},
        {"name": "HTTP API TLS key", "value": "configured" if os.getenv("AGI_API_TLS_KEY") else "not configured", "apply": "restart", "source": "Environment · private key path is hidden"},
        {"name": "HTTP API bearer token", "value": "configured" if os.getenv("AGI_API_TOKEN") else "not configured", "apply": "restart", "source": "Environment · secret value is never displayed"},
    ]
    return {"path": str(_settings_path()), "settings": settings, "runtime": runtime,
            "apply_note": "Saved ROS parameters are read at AuRoRA launch; restart the ROS launch to apply them.",
            "read_only_note": "Environment configuration is shown for reference and must be edited in the service environment."}


@app.put("/api/settings")
async def put_settings(request: Request) -> dict:
    """Validate and atomically persist allowlisted intrinsic ROS parameters."""
    if yaml is None:
        raise HTTPException(status_code=503, detail="YAML support is unavailable")
    try:
        payload = await request.json()
    except ValueError as error:
        raise HTTPException(status_code=400, detail="Request body must be JSON") from error
    if not isinstance(payload, dict) or not isinstance(payload.get("settings"), dict):
        raise HTTPException(status_code=400, detail="Expected an object with a settings mapping")
    values = payload["settings"]
    unknown = set(values) - set(_SETTING_INDEX)
    if unknown:
        raise HTTPException(status_code=400, detail=f"Unknown or read-only setting: {sorted(unknown)[0]}")
    current = _read_settings()
    params = _parameter_values(current).copy()
    for key, raw in values.items():
        _, label, _, _, default, minimum, maximum = _SETTING_INDEX[key]
        if isinstance(default, bool):
            valid = isinstance(raw, bool)
        elif isinstance(default, int):
            valid = isinstance(raw, int) and not isinstance(raw, bool)
        else:
            valid = isinstance(raw, (int, float)) and not isinstance(raw, bool)
        if not valid or not minimum <= raw <= maximum:
            raise HTTPException(status_code=422, detail=f"{label} must be between {minimum} and {maximum}")
        params[key] = raw
        # Keep explicit ROS node selectors in sync: ROS gives a node-specific
        # selector precedence over the /** wildcard block.
        for selector, block in current.items():
            if selector == "/**" or not isinstance(block, dict):
                continue
            selected_params = block.get("ros__parameters")
            if isinstance(selected_params, dict) and key in selected_params:
                selected_params[key] = raw
    effective = {key: params.get(key, entry[4]) for key, entry in _SETTING_INDEX.items()}
    effective.update(values)
    if effective["scs.cortical_capacity"] <= effective["scs.cognitive_reserve"] + effective["scs.emc.recall_reserve"]:
        raise HTTPException(status_code=422, detail="Cortical capacity must exceed cognitive and recall reserves combined")
    if effective["scs.gce.context_window"] < effective["scs.gce.response_depth"]:
        raise HTTPException(status_code=422, detail="Model context window must be at least the response depth")
    current["/**"] = current.get("/**") if isinstance(current.get("/**"), dict) else {}
    current["/**"]["ros__parameters"] = params
    path = _settings_path()
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        serialized = yaml.safe_dump(current, sort_keys=False, allow_unicode=True)
        with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=path.parent, delete=False) as temp:
            temp.write(serialized)
            temp.flush()
            os.fsync(temp.fileno())
            temp_path = Path(temp.name)
        os.replace(temp_path, path)
    except OSError as error:
        raise HTTPException(status_code=500, detail=f"Could not save AuRoRA settings: {error}") from error
    return {"saved": sorted(values), "path": str(path), "apply": "restart"}


@app.post("/api/presence/aiva", status_code=204)
async def aiva_presence() -> None:
    """Refresh AIVA's heartbeat; tailnet ACLs restrict callers to trusted devices."""
    global _aiva_last_seen
    _aiva_last_seen = time.monotonic()


@app.get("/api/presence")
async def presence() -> dict[str, bool | int | None]:
    """Report whether AIVA refreshed its heartbeat before the TTL expired."""
    timeout_s = max(5, int(os.getenv("AGI_AIVA_PRESENCE_TIMEOUT", "12")))
    age = None if _aiva_last_seen is None else time.monotonic() - _aiva_last_seen
    return {
        "aiva_online": age is not None and age <= timeout_s,
        "aiva_last_seen_seconds_ago": None if age is None else int(age),
        "timeout_seconds": timeout_s,
    }


@app.post("/api/radio/encode")
async def encode_radio_input(request: Request) -> Response:
    """Encode plain UTF-8 input for the typed RadioSignal ROS message."""
    raw_text = await request.body()
    if len(raw_text) > 2048:
        raise HTTPException(status_code=413, detail="Text input exceeds 2048 bytes")
    try:
        text = raw_text.decode("utf-8")
    except UnicodeDecodeError as error:
        raise HTTPException(status_code=400, detail="Input must be UTF-8 text") from error
    if not text.strip():
        raise HTTPException(status_code=400, detail="Text input is empty")

    if request.query_params.get("web_search", "false").lower() in {"1", "true", "yes"}:
        text = f"/web {text}"
    try:
        signal, symbol_count = encode_text(text)
    except (TypeError, ValueError) as error:
        raise HTTPException(status_code=400, detail=str(error)) from error

    return Response(
        content=signal,
        media_type="application/octet-stream",
        headers={
            "X-Radio-ID": uuid.uuid4().hex,
            "X-Radio-Channel": AGi.TMS.RADIO_CHANNEL,
            "X-Radio-Symbol-Count": str(symbol_count),
            "Cache-Control": "no-store",
        },
    )


@app.get("/")
async def index() -> FileResponse:
    return FileResponse(FRONTEND_DIR / "index.html")


@app.get("/settings")
async def settings_page() -> FileResponse:
    return FileResponse(FRONTEND_DIR / "settings.html")


@app.websocket("/rosbridge")
async def rosbridge_proxy(browser: WebSocket) -> None:
    """Keep rosbridge on loopback; the browser connects through this origin."""
    await browser.accept()
    target = os.getenv("AGI_ROSBRIDGE_URL", "ws://127.0.0.1:9090")
    try:
        async with websockets.connect(target, max_size=None) as ros:
            async def browser_to_ros() -> None:
                while True:
                    message = await browser.receive_text()
                    await ros.send(message)

            async def ros_to_browser() -> None:
                async for message in ros:
                    if isinstance(message, bytes):
                        await browser.send_bytes(message)
                    else:
                        await browser.send_text(message)

            tasks = [asyncio.create_task(browser_to_ros()), asyncio.create_task(ros_to_browser())]
            done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for task in pending:
                task.cancel()
            await asyncio.gather(*pending, return_exceptions=True)
            for task in done:
                task.result()
    except (WebSocketDisconnect, websockets.WebSocketException, OSError, RuntimeError):
        pass
    finally:
        try:
            await browser.close()
        except RuntimeError:
            pass


def main() -> None:
    """Run loopback-only Web Teleops Console; use Tailscale Serve for tailnet access."""
    uvicorn.run(
        app,
        host=os.getenv("AGI_WEB_TELEOPS_CONSOLE_HOST", "127.0.0.1"),
        port=int(os.getenv("AGI_WEB_TELEOPS_CONSOLE_PORT", "8853")),
        log_level="info",
    )


if __name__ == "__main__":
    main()
