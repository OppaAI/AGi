"""
AuRoRA — LLM Interface (Cosmos via vLLM)
=========================================
Replaces Ollama with vLLM serving Cosmos over the OpenAI-compatible API.

vLLM endpoint: POST /v1/chat/completions  (same shape as OpenAI)
Streaming:     supported via SSE chunks

Usage:
    from llm import llm_chat, llm_generate, llm_chat_stream

All callers use the same functions — no Ollama-specific code anywhere else.
"""

import json
import logging
import requests
from typing import Iterator, Optional

from config import (
    VLLM_BASE_URL,
    VLLM_MODEL,
    VLLM_FAST_MODEL,
    VLLM_API_KEY,
    MAX_REQUEST_TOKENS,
)

log = logging.getLogger("aurora.llm")

_HEADERS = {
    "Content-Type": "application/json",
    "Authorization": f"Bearer {VLLM_API_KEY}",
}

# ── Core call — blocking ──────────────────────────────────────────────────────

def llm_chat(
    messages: list,
    *,
    model: str = None,
    max_tokens: int = 1500,
    temperature: float = 0.75,
    timeout: int = 120,
) -> str:
    """
    Blocking chat completion via vLLM.
    messages: list of {"role": "system"|"user"|"assistant", "content": str}
    Returns the assistant reply string, or "" on error.
    """
    model = model or VLLM_MODEL
    try:
        resp = requests.post(
            f"{VLLM_BASE_URL}/chat/completions",
            headers=_HEADERS,
            json={
                "model":       model,
                "messages":    messages,
                "max_tokens":  max_tokens,
                "temperature": temperature,
                "stream":      False,
            },
            timeout=timeout,
        )
        resp.raise_for_status()
        return resp.json()["choices"][0]["message"]["content"].strip()
    except Exception as e:
        log.error(f"❌ llm_chat failed: {e}")
        return ""


def llm_generate(
    prompt: str,
    *,
    system: str = "",
    model: str = None,
    max_tokens: int = 400,
    temperature: float = 0.5,
    timeout: int = 30,
) -> str:
    """
    Convenience wrapper for single-turn generation.
    Equivalent to the old Ollama /api/generate calls.
    """
    messages = []
    if system:
        messages.append({"role": "system", "content": system})
    messages.append({"role": "user", "content": prompt})
    return llm_chat(
        messages,
        model=model or VLLM_FAST_MODEL,
        max_tokens=max_tokens,
        temperature=temperature,
        timeout=timeout,
    )


def llm_chat_stream(
    messages: list,
    *,
    model: str = None,
    max_tokens: int = MAX_REQUEST_TOKENS,
    temperature: float = 0.75,
    timeout: int = 120,
) -> Iterator[str]:
    """
    Streaming chat completion — yields text delta strings.
    Caller iterates and assembles full_response.

    Example:
        full = ""
        for delta in llm_chat_stream(messages):
            full += delta
            publish(delta)
    """
    model = model or VLLM_MODEL
    try:
        with requests.post(
            f"{VLLM_BASE_URL}/chat/completions",
            headers=_HEADERS,
            json={
                "model":       model,
                "messages":    messages,
                "max_tokens":  max_tokens,
                "temperature": temperature,
                "stream":      True,
            },
            stream=True,
            timeout=timeout,
        ) as resp:
            resp.raise_for_status()
            for raw_line in resp.iter_lines():
                if not raw_line:
                    continue
                line = raw_line.decode("utf-8") if isinstance(raw_line, bytes) else raw_line
                if line.startswith("data: "):
                    line = line[6:]
                if line.strip() == "[DONE]":
                    break
                try:
                    chunk = json.loads(line)
                    delta = chunk["choices"][0].get("delta", {}).get("content", "")
                    if delta:
                        yield delta
                except (json.JSONDecodeError, KeyError):
                    continue
    except Exception as e:
        log.error(f"❌ llm_chat_stream failed: {e}")


def llm_health_check() -> bool:
    """
    Verify vLLM is reachable and the model is loaded.
    Called at startup — logs result, returns True/False.
    """
    try:
        resp = requests.get(f"{VLLM_BASE_URL}/models", headers=_HEADERS, timeout=10)
        resp.raise_for_status()
        models = [m["id"] for m in resp.json().get("data", [])]
        if VLLM_MODEL in models or any(VLLM_MODEL in m for m in models):
            log.info(f"✅ vLLM ready — model: {VLLM_MODEL}")
            return True
        log.warning(f"⚠️  vLLM reachable but model '{VLLM_MODEL}' not listed: {models}")
        return False
    except Exception as e:
        log.error(f"❌ vLLM health check failed: {e}")
        log.error(f"   Is vLLM running at {VLLM_BASE_URL}?")
        log.error(f"   Start with: vllm serve {VLLM_MODEL} --port 8000")
        return False


def llm_with_tools(
    messages: list,
    tools: list,
    *,
    model: str = None,
    max_tokens: int = 1500,
    temperature: float = 0.75,
    timeout: int = 60,
) -> dict:
    """
    Non-streaming call with tool definitions.
    Returns the full choices[0].message dict so caller can inspect tool_calls.

    vLLM supports OpenAI-style function/tool calling for compatible models.
    """
    model = model or VLLM_MODEL
    try:
        resp = requests.post(
            f"{VLLM_BASE_URL}/chat/completions",
            headers=_HEADERS,
            json={
                "model":       model,
                "messages":    messages,
                "tools":       tools,
                "max_tokens":  max_tokens,
                "temperature": temperature,
                "stream":      False,
            },
            timeout=timeout,
        )
        resp.raise_for_status()
        return resp.json()["choices"][0]["message"]
    except Exception as e:
        log.error(f"❌ llm_with_tools failed: {e}")
        return {"content": "", "tool_calls": []}
