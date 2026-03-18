"""
WMC — Working Memory Cortex
============================
AuRoRA · Semantic Cognitive System (SCS)

Active conversation context for GRACE.
Mirrors human working memory — fast, limited capacity, current focus only.

Capacity: chunk-counted, stays under chunk capacity to fit LLM context window with room for system prompt + GRACE personality + EMC context injection.
Overflow:  oldest event segments pushed to MCC → EMC buffer (async)

Miller's Law: 7±2 chunks — chunk capacity and event segment count are monitored in hybrid
to ensure optimal context window construction for LLM inference while maintaining conversation coherence.
"""

from datetime import datetime            # (TODO) replace with hrs.blc when BioLogic Clock is built
from collections import deque            # for use in memory management

# ── Chunk Capacity / Event Segment Limit ──────────────────────────────────────────────────────────────
# LLM: max_model_len = 2048 (cosmos.sh)
# Reserve ~30% for system prompt + GRACE personality + EMC context injection

# (TODO) add MAX_CHUNK = 7 for better 7+/-2 Miller's law
# (TODO) put this chunk_capacity and MAX_CHUNK to mcc.yaml
chunk_capacity = 1440   # chunks reserved for raw conversation event segments

# Retrieve the chunk size from homeostatic regulation system
# Fallback to default if HRS cannot be called
try:                                        # attempt to reach HRS
    from hrs.hrp import UNITS_PER_CHUNK     # retrieve chunk size from HRS
except ImportError:                         # if error cannot reach HRS
    UNITS_PER_CHUNK = 4                     # fallback default value of 4
# ─────────────────────────────────────────────────────────────────────────────

def _estimate_chunks_in_content(content: str) -> int:
    """
    Roughly estimate the number of chunks in the given content.

    Args:
        content (str): Text to estimate chunks for

    Returns:
        int: Number of chunks, minimum 1
    """
    return max(1, len(content) // UNITS_PER_CHUNK)            # minimum 1 chunk even empty content, otherwise break the content into chunks

def _event_segment_chunks(event_segment: dict) -> int:
    """Estimate chunks for a single conversation event segment."""
    content = event_segment.get("content", "")
    # Add small overhead per event segment for role label + formatting
    return _estimate_chunks(content) + 4

class WMC:
    """
    Working Memory Cortex.

    Maintains the active conversation window sent to LLM on every event segment.
    Oldest event segments are evicted when the chunk capacity is exceeded and returned
    to MCC for async forwarding to EMC.

    Thread-safety: NOT thread-safe by design — only CNC/MCC touches WMC,
    always from the asyncio event loop.
    """

    def __init__(self, logger, chunk_capacity: int = chunk_capacity):
        self.logger       = logger
        self.chunk_capacity = chunk_capacity
        self._event_segments: deque[dict] = deque()
        self._active_chunks: int   = 0

        self.logger.info(
            f"✅ WMC initialised — chunk capacity: {self.chunk_capacity} chunks"
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
        event_segment_load = _event_segment_chunks(event_segment)

        # Evict oldest event segments until new event segment fits
        evicted: list[dict] = []
        while self._event_segments and (self._active_chunks + event_segment_load > self.chunk_capacity):
            oldest = self._event_segments.popleft()
            self._active_chunks -= _event_segment_chunks(oldest)
            evicted.append(oldest)
            self.logger.debug(
                f"WMC evict → EMC: [{oldest['role']}] "
                f"{oldest['content'][:40]}…"
            )

        self._event_segments.append(event_segment)
        self._active_chunks += event_segment_load

        self.logger.debug(
            f"WMC + event segment [{role}] | "
            f"event segments={len(self._event_segments)} | "
            f"chunks={self._active_chunks}/{self.chunk_capacity} | "
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
            for t in self._event_segments
        ]

    def peek_last(self, n: int = 1) -> list[dict]:
        """Return the last N event segments (most recent) with timestamps."""
        event_segments = list(self._event_segments)
        return event_segments[-n:] if n <= len(event_segments) else event_segments

    def chunk_usage(self) -> dict:
        """Return current chunk usage stats."""
        return {
            "used":    self._active_chunks,
            "capacity":  self.chunk_capacity,
            "free":    self.chunk_capacity - self._active_chunks,
            "event_segments":   len(self._event_segments),
            "percent": round(self._active_chunks / self.chunk_capacity * 100, 1),
        }

    def clear(self):
        """
        Clear all event segments from working memory.
        Called at conversation end or on explicit reset.
        Does NOT evict to EMC — caller is responsible for saving if needed.
        """
        count = len(self._event_segments)
        self._event_segments.clear()
        self._active_chunks = 0
        self.logger.info(f"🧹 WMC cleared ({count} event segments discarded)")

    def is_empty(self) -> bool:
        return len(self._event_segments) == 0

    def __len__(self) -> int:
        return len(self._event_segments)

    def __repr__(self) -> str:
        u = self.chunk_usage()
        return (
            f"WMC(event_segments={u['event_segments']}, "
            f"chunks={u['used']}/{u['capacity']} "
            f"[{u['percent']}%])"
        )
