"""Serve the COC interface and proxy its browser WebSocket to local rosbridge."""
from __future__ import annotations

import asyncio
import os
import time
import uuid
from pathlib import Path

import uvicorn
import websockets
from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response
from fastapi.staticfiles import StaticFiles

from hrs.hrm import AGi
from tms.radio_encoding import encode_text


def _interface_dir() -> Path:
    """Resolve the source checkout or the installed ROS package interface."""
    try:
        from ament_index_python.packages import get_package_share_directory
        installed = Path(get_package_share_directory("tms")) / "coc"
        if (installed / "index.html").is_file():
            return installed
    except (ImportError, LookupError):
        pass
    return Path(__file__).resolve().parents[4] / "interface" / "coc"


INTERFACE_DIR = _interface_dir()
app = FastAPI(title="AGi Communication Operation Console", docs_url=None, redoc_url=None)
app.mount("/assets", StaticFiles(directory=INTERFACE_DIR / "assets"), name="assets")
_aiva_last_seen: float | None = None


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
    return FileResponse(INTERFACE_DIR / "index.html")


@app.get("/{filename}")
async def static_file(filename: str) -> FileResponse:
    allowed = {"styles.css", "script.js", "avatar.js", "theme.js", "presence.js"}
    if filename not in allowed:
        from fastapi import HTTPException
        raise HTTPException(status_code=404)
    return FileResponse(INTERFACE_DIR / filename)


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
    """Run loopback-only COC; use Tailscale Serve for tailnet access."""
    uvicorn.run(
        app,
        host=os.getenv("AGI_COC_HOST", "127.0.0.1"),
        port=int(os.getenv("AGI_COC_PORT", "8853")),
        log_level="info",
    )


if __name__ == "__main__":
    main()
