"""Serve the COC interface and proxy its browser WebSocket to local rosbridge."""
from __future__ import annotations

import asyncio
import os
from pathlib import Path

import uvicorn
import websockets
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles


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


@app.get("/")
async def index() -> FileResponse:
    return FileResponse(INTERFACE_DIR / "index.html")


@app.get("/{filename}")
async def static_file(filename: str) -> FileResponse:
    allowed = {"styles.css", "script.js", "avatar.js"}
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
