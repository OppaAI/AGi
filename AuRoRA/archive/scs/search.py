"""
AuRoRA — Web Search
====================
Dual-hybrid search:
  Priority 1: MCP proxy  → full page content (if igniter runs it)
  Priority 2: SearXNG    → snippets (always available locally)

Auto-detection: keyword triggers + context-aware sports/follow-up detection.
Skills topics (weather, aurora, wildlife) are excluded — handled elsewhere.
"""

import logging
import json
import requests
from typing import Union

from config import (
    SEARXNG_URL,
    ENABLE_AUTO_SEARCH,
    MAX_SEARCH_RESULTS,
    SEARCH_CONTENT_CHARS,
)

log = logging.getLogger("aurora.search")

_MCP_SEARCH_URL = "http://127.0.0.1:51622/search"

# ── Skills topics — DO NOT web-search these ───────────────────────────────────
_SKILLS_TOPICS = [
    "weather", "temperature", "sunrise", "sunset", "moon",
    "aurora", "northern lights", "stargazing", "wildlife",
    "hiking conditions", "camping", "outdoor conditions",
]

# ── Keyword triggers ──────────────────────────────────────────────────────────
_TIME_TRIGGERS   = ["latest news", "recent events", "breaking", "this week"]
_INFO_TRIGGERS   = ["who is", "what happened", "when did", "where is"]
_SEARCH_TRIGGERS = ["search for", "look up", "find out", "google"]

_SPORTS_FOLLOWUP = [
    "how about", "what about", "and him", "and her",
    "how did he", "how did she", "how did they", "his stats", "her stats",
]
_SPORTS_KEYWORDS = [
    "score", "game", "stats", "nba", "nhl", "nfl", "mlb",
    "raptors", "basketball", "hockey", "football", "soccer",
    "points", "goals", "assists", "wins", "playoff",
]
_VAGUE_FOLLOWUPS = [
    "what else", "tell me more", "and then", "what happened next",
    "more about that", "more details",
]
_REALTIME_PATTERNS = [
    ("stock",    ["price", "trading", "market", "shares"]),
    ("crypto",   ["price", "btc", "eth", "bitcoin", "ethereum"]),
    ("score",    ["final", "result", "win", "lose", "beat"]),
    ("election", ["result", "winner", "vote", "poll"]),
    ("today",    ["happen", "news", "update", "announce"]),
    ("now",      ["happen", "going on", "situation"]),
]


def detect_search_need(
    user_message: str,
    conversation_history: list = None,
    use_skills: bool = False,
) -> Union[str, bool]:
    """
    Returns:
      str  — specific search query to use
      True — search with user_message as query
      False — no search needed
    """
    if use_skills:
        return False

    kw = _keyword_search(user_message)
    if kw:
        log.info("🔍 Keyword search trigger")
        return user_message

    if ENABLE_AUTO_SEARCH:
        auto = _auto_search(user_message, conversation_history or [])
        if auto:
            return auto

    return False


def _keyword_search(message: str) -> bool:
    lower = message.lower()
    if any(t in lower for t in _SKILLS_TOPICS):
        return False
    if any(t in lower for t in _TIME_TRIGGERS):
        return True
    if any(t in lower for t in _INFO_TRIGGERS):
        if not any(t in lower for t in _SKILLS_TOPICS):
            return True
    if any(t in lower for t in _SEARCH_TRIGGERS):
        return True
    return False


def _auto_search(message: str, context: list) -> Union[str, bool]:
    lower = message.lower().strip()

    # Sports follow-up
    if context and any(p in lower for p in _SPORTS_FOLLOWUP):
        for hist in reversed(context[-6:]):
            if any(kw in hist.get("content", "").lower() for kw in _SPORTS_KEYWORDS):
                subject = message
                for prefix in ["how about ", "what about ", "and "]:
                    if lower.startswith(prefix):
                        subject = message[len(prefix):].strip()
                        break
                sport = next((kw for kw in _SPORTS_KEYWORDS if kw in hist.get("content", "").lower()), "game")
                query = f"{subject} stats {sport} today"
                log.info(f"🤖 Sports follow-up → '{query}'")
                return query

    # Vague follow-up after search response
    if context and any(p in lower for p in _VAGUE_FOLLOWUPS):
        for hist in reversed(context[-4:]):
            if hist.get("role") == "assistant":
                content = hist.get("content", "")
                if "http" in content or "[1]" in content:
                    for h in reversed(context[-6:]):
                        if h.get("role") == "user" and h.get("content") != message:
                            query = f"{h['content']} more details"
                            log.info(f"🤖 Follow-up search → '{query}'")
                            return query
                    break

    # Real-time data patterns
    for anchor, companions in _REALTIME_PATTERNS:
        if anchor in lower and any(c in lower for c in companions):
            log.info(f"🤖 Real-time pattern: '{message[:50]}'")
            return message

    return False


def web_search(query: str, num_results: int = MAX_SEARCH_RESULTS) -> list:
    """
    Try MCP proxy first (full content), fall back to SearXNG (snippets).
    Returns list of result dicts or empty list on failure.
    """
    if not query:
        return []

    # Clean malformed JSON queries
    if isinstance(query, str) and query.strip().startswith("{"):
        try:
            parsed = json.loads(query)
            query  = parsed.get("text", query)
        except Exception:
            pass

    # ── Try 1: MCP proxy ─────────────────────────────────────────────────────
    try:
        resp = requests.post(
            _MCP_SEARCH_URL,
            json={"query": query, "num_results": num_results},
            timeout=35,
        )
        if resp.status_code == 200:
            results = resp.json()
            if results:
                log.info(f"🔍 MCP (full content): '{query}' → {len(results)} results")
                return results
    except requests.exceptions.ConnectionError:
        log.debug("MCP proxy not reachable, falling back to SearXNG")
    except Exception as e:
        log.warning(f"MCP proxy error: {e}")

    # ── Try 2: SearXNG ───────────────────────────────────────────────────────
    try:
        resp = requests.get(
            f"{SEARXNG_URL}/search",
            params={"q": query, "format": "json", "categories": "general"},
            timeout=10,
        )
        resp.raise_for_status()
        raw = resp.json().get("results", [])[:num_results]
        results = [
            {
                "number":           i + 1,
                "title":            r.get("title", ""),
                "url":              r.get("url", ""),
                "snippet":          r.get("content", "")[:SEARCH_CONTENT_CHARS],
                "full_content":     "",
                "has_full_content": False,
            }
            for i, r in enumerate(raw)
        ]
        log.info(f"🔍 SearXNG (snippets): '{query}' → {len(results)} results")
        return results
    except Exception as e:
        log.error(f"❌ SearXNG failed: {e}")
        return []


def check_searxng_available() -> bool:
    try:
        resp = requests.get(f"{SEARXNG_URL}/search",
                            params={"q": "test", "format": "json"}, timeout=5)
        return resp.status_code == 200
    except Exception:
        return False
