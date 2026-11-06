"""Checks that Web Teleops Console's local encoder endpoint returns packed bytes, not a JSON string."""

import asyncio

from starlette.requests import Request

from tms.web_teleops_console import encode_radio_input
from tms.radio_encoding import decode_text


def _request(body: bytes, query: bytes = b"") -> Request:
    scope = {
        "type": "http",
        "asgi": {"version": "3.0"},
        "http_version": "1.1",
        "method": "POST",
        "scheme": "http",
        "path": "/api/radio/encode",
        "raw_path": b"/api/radio/encode",
        "query_string": query,
        "headers": [],
        "client": ("127.0.0.1", 12345),
        "server": ("127.0.0.1", 8853),
    }
    received = False

    async def receive() -> dict:
        nonlocal received
        if received:
            return {"type": "http.disconnect"}
        received = True
        return {"type": "http.request", "body": body, "more_body": False}

    return Request(scope, receive)


def test_encoder_endpoint_returns_binary_radio_signal() -> None:
    response = asyncio.run(encode_radio_input(_request(b"Hello world!", b"web_search=true")))

    assert response.media_type == "application/octet-stream"
    assert response.headers["cache-control"] == "no-store"
    assert response.headers["x-radio-id"]
    assert response.headers["x-radio-channel"] == "grace"
    assert decode_text(response.body, int(response.headers["x-radio-symbol-count"])) == "/web Hello world!"
