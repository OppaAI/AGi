"""
WMC — Working Memory Cortex
============================
AuRoRA · Semantic Cognitive System (SCS)

Active conversation context for GRACE.
Mirrors human working memory — fast, limited capacity, current focus only.

Capacity: token-counted, stays under chunk_capacity to fit LLM context window with room for system prompt + GRACE personality + EMC context injection.
Overflow:  oldest event segments pushed to MCC → EMC buffer (async)

Miller's Law: 7±2 chunks — chunk capacity and event segment count are monitored in hybrid
to ensure optimal context window construction for LLM inference while maintaining conversation coherence.
"""

from collections import deque
from datetime import datetime
from typing import Optional


# ── Token budget ──────────────────────────────────────────────────────────────
# LLM: max_model_len = 2048 (cosmos.sh)
# Reserve ~30% for system prompt + GRACE personality + EMC context injection

# (TODO) add MAX_TURN = 7 for better 7+/-2 Miller's law
# (TODO) put this chunk_capacity and MAX_TURN to mcc.yaml
chunk_capacity = 1400   # tokens reserved for raw conversation event segments
# (TODO) put this CHARS_PER_TOKEN to hrs/hrs/hrp.py (homeostatic regulation system/parameters)
CHARS_PER_TOKEN   = 4      # rough English approximation (1 token ≈ 4 chars)
# ─────────────────────────────────────────────────────────────────────────────


def _estimate_tokens(text: str) -> int:
    """Rough token estimate: 1 token ≈ 4 English characters."""
    return max(1, len(text) // CHARS_PER_TOKEN)


def _event_segment_tokens(event_segment: dict) -> int:
    """Estimate tokens for a single conversation event segment."""
    content = event_segment.get("content", "")
    # Add small overhead per event segment for role label + formatting
    return _estimate_tokens(content) + 4


class WMC:
    """
    Working Memory Cortex.

    Maintains the active conversation window sent to LLM on every event segment.
    Oldest event segments are evicted when the token budget is exceeded and returned
    to MCC for async forwarding to EMC.

    Thread-safety: NOT thread-safe by design — only CNC/MCC touches WMC,
    always from the asyncio event loop.
    """

    def __init__(self, logger, token_budget: int = chunk_capacity):
        self.logger       = logger
        self.token_budget = token_budget
        self._turns: deque[dict] = deque()
        self._used_tokens: int   = 0

        self.logger.info(
            f"✅ WMC initialised — token budget: {self.token_budget} tokens"
        )

    # ── Public API ────────────────────────────────────────────────────────────

    def add_event_segment(self, role: str, content: str) -> list[dict]:
        """
        Add a new conversation event segment to working memory.

        Returns a list of evicted event segments (may be empty) so MCC can forward
        them to EMC asynchronously.

        Args:
            role:    "user" or "assistant"
            content: The message text

        Returns:
            List of evicted event segments [{role, content, timestamp}]
        """
        event_segment = {
            "role":      role,
            "content":   content,
            "timestamp": datetime.now().isoformat(),
        }
        event_segment_cost = _event_segment_tokens(event_segment)

        # Evict oldest event segments until new event segment fits
        evicted: list[dict] = []
        while self._turns and (self._used_tokens + event_segment_cost > self.token_budget):
            oldest = self._turns.popleft()
            self._used_tokens -= _event_segment_tokens(oldest)
            evicted.append(oldest)
            self.logger.debug(
                f"WMC evict → EMC: [{oldest['role']}] "
                f"{oldest['content'][:40]}…"
            )

        self._turns.append(event_segment)
        self._used_tokens += event_segment_cost

        self.logger.debug(
            f"WMC + event segment [{role}] | "
            f"event segments={len(self._turns)} | "
            f"tokens={self._used_tokens}/{self.token_budget} | "
            f"evicted={len(evicted)}"
        )

        return evicted

    def get_event_segments(self) -> list[dict]:
        """
        Return active event segments for context window construction.
        Returns event segments in chronological order (oldest first).
        Only includes role + content — timestamp stripped for LLM.
        """
        return [
            {"role": t["role"], "content": t["content"]}
            for t in self._turns
        ]

    def peek_last(self, n: int = 1) -> list[dict]:
        """Return the last N event segments (most recent) with timestamps."""
        event_segments = list(self._turns)
        return event_segments[-n:] if n <= len(event_segments) else event_segments

    def token_usage(self) -> dict:
        """Return current token usage stats."""
        return {
            "used":    self._used_tokens,
            "budget":  self.token_budget,
            "free":    self.token_budget - self._used_tokens,
            "event_segments":   len(self._turns),
            "percent": round(self._used_tokens / self.token_budget * 100, 1),
        }

    def clear(self):
        """
        Clear all event segments from working memory.
        Called at conversation end or on explicit reset.
        Does NOT evict to EMC — caller is responsible for saving if needed.
        """
        count = len(self._turns)
        self._turns.clear()
        self._used_tokens = 0
        self.logger.info(f"🧹 WMC cleared ({count} event segments discarded)")

    def is_empty(self) -> bool:
        return len(self._turns) == 0

    def __len__(self) -> int:
        return len(self._turns)

    def __repr__(self) -> str:
        u = self.token_usage()
        return (
            f"WMC(event_segments={u['event_segments']}, "
            f"tokens={u['used']}/{u['budget']} "
            f"[{u['percent']}%])"
        )
