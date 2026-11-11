"""Checks that Web Teleops Console's local encoder endpoint returns packed bytes, not a JSON string."""

import asyncio

from starlette.requests import Request

import json
from types import SimpleNamespace
from telepathic_management.telepathic_console import server
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


def test_encoder_endpoint_publishes_typed_signal(monkeypatch) -> None:
    monkeypatch.setenv("AGI_TELEPATHIC_CHANNEL", "grace")
    messages = []
    class Connection:
        async def __aenter__(self):
            return self
        async def __aexit__(self, *_):
            pass
        async def send(self, raw):
            messages.append(json.loads(raw))
    monkeypatch.setattr(server, 'websockets', SimpleNamespace(connect=lambda *a, **k: Connection(), WebSocketException=Exception))
    response = asyncio.run(server.send_telepathic_input(_request(b"Hello world!", b"web_search=true")))
    assert response['identifier']
    assert messages[0]['op'] == 'advertise'
    envelope = messages[1]
    assert envelope['topic'] == '/telepathic/stimulus'
    assert envelope['msg']['identifier'] == response['identifier']
    assert decode_text(bytes(envelope['msg']['signal']), envelope['msg']['pulse']) == '/web Hello world!'
