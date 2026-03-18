"""
WMC — Working Memory Cortex
============================
AuRoRA · Semantic Cognitive System (SCS)

Active conversation context for GRACE.
Mirrors human working memory — fast, limited capacity, current focus only.

Capacity: token-counted, stays under MAX_TOKENS budget
Overflow:  oldest turns pushed to MCC → EMC buffer (async)

Miller's Law: 7±2 chunks — we use token budget instead of turn count
since Cosmos 2B has a hard 2048 token context window.
"""

from collections import deque
from datetime import datetime
from typing import Optional


# ── Token budget ──────────────────────────────────────────────────────────────
# Cosmos Reason2 2B: max_model_len = 2048 (cosmos.sh)
# Reserve ~30% for system prompt + GRACE personality + EMC context injection

# (TODO) add MAX_TURN = 7 for better 7+/-2 Miller's law
# (TODO) put this WMC_TOKEN_BUDGET and MAX_TURN to mcc.yaml
WMC_TOKEN_BUDGET  = 1400   # tokens reserved for raw conversation turns
# (TODO) put this CHARS_PER_TOKEN to hrs/hrs/hrp.py (homeostatic regulation system/parameters)
CHARS_PER_TOKEN   = 4      # rough English approximation (1 token ≈ 4 chars)
# ─────────────────────────────────────────────────────────────────────────────


def _estimate_tokens(text: str) -> int:
    """Rough token estimate: 1 token ≈ 4 English characters."""
    return max(1, len(text) // CHARS_PER_TOKEN)


def _turn_tokens(turn: dict) -> int:
    """Estimate tokens for a single conversation turn."""
    content = turn.get("content", "")
    # Add small overhead per turn for role label + formatting
    return _estimate_tokens(content) + 4


class WMC:
    """
    Working Memory Cortex.

    Maintains the active conversation window sent to Cosmos on every turn.
    Oldest turns are evicted when the token budget is exceeded and returned
    to MCC for async forwarding to EMC.

    Thread-safety: NOT thread-safe by design — only CNC/MCC touches WMC,
    always from the asyncio event loop.
    """

    def __init__(self, logger, token_budget: int = WMC_TOKEN_BUDGET):
        self.logger       = logger
        self.token_budget = token_budget
        self._turns: deque[dict] = deque()
        self._used_tokens: int   = 0

        self.logger.info(
            f"✅ WMC initialised — token budget: {self.token_budget} tokens"
        )

    # ── Public API ────────────────────────────────────────────────────────────

    def add_turn(self, role: str, content: str) -> list[dict]:
        """
        Add a new conversation turn to working memory.

        Returns a list of evicted turns (may be empty) so MCC can forward
        them to EMC asynchronously.

        Args:
            role:    "user" or "assistant"
            content: The message text

        Returns:
            List of evicted turns [{role, content, timestamp}]
        """
        turn = {
            "role":      role,
            "content":   content,
            "timestamp": datetime.now().isoformat(),
        }
        turn_cost = _turn_tokens(turn)

        # Evict oldest turns until new turn fits
        evicted: list[dict] = []
        while self._turns and (self._used_tokens + turn_cost > self.token_budget):
            oldest = self._turns.popleft()
            self._used_tokens -= _turn_tokens(oldest)
            evicted.append(oldest)
            self.logger.debug(
                f"WMC evict → EMC: [{oldest['role']}] "
                f"{oldest['content'][:40]}…"
            )

        self._turns.append(turn)
        self._used_tokens += turn_cost

        self.logger.debug(
            f"WMC +turn [{role}] | "
            f"turns={len(self._turns)} | "
            f"tokens={self._used_tokens}/{self.token_budget} | "
            f"evicted={len(evicted)}"
        )

        return evicted

    def get_turns(self) -> list[dict]:
        """
        Return active turns for context window construction.
        Returns turns in chronological order (oldest first).
        Only includes role + content — timestamp stripped for Cosmos.
        """
        return [
            {"role": t["role"], "content": t["content"]}
            for t in self._turns
        ]

    def peek_last(self, n: int = 1) -> list[dict]:
        """Return the last N turns (most recent) with timestamps."""
        turns = list(self._turns)
        return turns[-n:] if n <= len(turns) else turns

    def token_usage(self) -> dict:
        """Return current token usage stats."""
        return {
            "used":    self._used_tokens,
            "budget":  self.token_budget,
            "free":    self.token_budget - self._used_tokens,
            "turns":   len(self._turns),
            "percent": round(self._used_tokens / self.token_budget * 100, 1),
        }

    def clear(self):
        """
        Clear all turns from working memory.
        Called at conversation end or on explicit reset.
        Does NOT evict to EMC — caller is responsible for saving if needed.
        """
        count = len(self._turns)
        self._turns.clear()
        self._used_tokens = 0
        self.logger.info(f"🧹 WMC cleared ({count} turns discarded)")

    def is_empty(self) -> bool:
        return len(self._turns) == 0

    def __len__(self) -> int:
        return len(self._turns)

    def __repr__(self) -> str:
        u = self.token_usage()
        return (
            f"WMC(turns={u['turns']}, "
            f"tokens={u['used']}/{u['budget']} "
            f"[{u['percent']}%])"
        )
