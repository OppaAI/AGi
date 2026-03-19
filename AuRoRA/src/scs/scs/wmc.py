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

# LLM: max_model_len = 2048 (cosmos.sh)
# Reserve ~30% for system prompt + GRACE personality + EMC context injection

# (TODO) add MAX_CHUNK = 7 for better 7+/-2 Miller's law
# (TODO) put this chunk_capacity and MAX_CHUNK to mcc.yaml

# Retrieve the chunk size and segment overhead from homeostatic regulation system
# Fallback to default if HRS cannot be called
try:                                    # attempt to reach HRS
    from hrs.hrp import (               # import parameters from HRS
    UNITS_PER_CHUNK,                    # retrieve number of units per chunk
    SEGMENT_OVERHEAD,                   # retrieve overhead in chunks for each event segment (for role label and formatting)        
    DEFAULT_CHUNK_CAPACITY,             # retrieve default chunk capacity for WMC, tunable based on LLM context window and hardware constraints
    DEFAULT_EVENT_SEGMENT_CAPACITY,     # retrieve default event segment capacity for WMC, based on Miller's Law 7±2
    DEFAULT_EVENT_SEGMENT_BUFFER        # retrieve default event segment buffer for WMC, based on Miller's Law 7±2 
)
except ImportError:                     # if error cannot reach HRS
    UNITS_PER_CHUNK = 4                 # fallback to default value for units per chunk
    SEGMENT_OVERHEAD = 4                # fallback to default value for overhead per event segment
    DEFAULT_CHUNK_CAPACITY = 1440       # fallback to default chunk capacity for WMC, tunable based on LLM context window and hardware constraints
    DEFAULT_EVENT_SEGMENT_CAPACITY = 7  # fallback to default event segment capacity for WMC, based on Miller's Law 7±2
    DEFAULT_EVENT_SEGMENT_BUFFER = 2    # fallback to default event segment buffer for WMC, based on Miller's Law 7±2

def _estimate_chunks_in_segment(event_segment: dict) -> int:
    """
    Estimate the number of chunks in the given event segment.

    Args:
        event_segment (dict): Event segment with 'role' and 'content'

    Returns:
        int: Number of chunks, including overhead for role label and formatting
    """
    role = event_segment.get("role", "")                                # retrieve role from event segment
    content = event_segment.get("content", "")                          # retrieve conversation content from event segment
    role_chunks = len(role) // UNITS_PER_CHUNK                          # calculate chunks for role label
    content_chunks = max(1, len(content) // UNITS_PER_CHUNK)            # calculate chunks for content, minimum 1 chunk even for empty content
    return role_chunks + content_chunks + SEGMENT_OVERHEAD              # return total chunk count including overhead for formatting and role label

class WorkingMemoryCortex:
    """
    Working Memory Cortex.

    Maintains the active conversation window sent to LLM on every event segment.
    Oldest event segments are evicted when the chunk capacity is exceeded and returned
    to MCC for async forwarding to EMC.

    Thread-safety: NOT thread-safe by design — only CNC/MCC touches WMC,
    always from the asyncio event loop.
    """

    def __init__(self, logger, 
                chunk_capacity: int = DEFAULT_CHUNK_CAPACITY, 
                event_segment_capacity: int = DEFAULT_EVENT_SEGMENT_CAPACITY,
                event_segment_buffer: int = DEFAULT_EVENT_SEGMENT_BUFFER
                ):
        """
        Initialize WMC with chunk capacity and event segment capacity.
        Args:
            logger: Logger instance from CNC for logging WMC operations
            chunk_capacity: Maximum number of chunks WMC can hold (tunable based on LLM context window and hardware constraints)
            event_segment_capacity: Maximum number of event segments WMC can hold (based on Miller's Law 7±2, but can be adjusted as needed)
            event_segment_buffer: Additional buffer for event segments beyond Miller's Law limit to allow flexibility if chunks are small (default 2)
        """
        self.logger                 = logger                    # retrieve logger from CNC for logging WMC operations
        self.chunk_capacity         = chunk_capacity            # retrieve chunk capacity from MCC configuration for WMC
        self.event_segment_capacity = event_segment_capacity    # retrieve event segment capacity from MCC configuration for WMC
        self.event_segment_buffer   = event_segment_buffer      # retrieve event segment buffer from MCC configuration for WMC
        self._event_segments: deque[dict] = deque()             # set up deque for holding event segments in working memory
        self._active_chunks: int    = 0                         # start with empty working memory with 0 active chunks

        self.logger.info(                                       # log entry on WMC initialization with configured capacities
            f"✅ WorkingMemoryCortex initialised — "
            f"chunk capacity: {self.chunk_capacity} | "
            f"event segment capacity: {self.event_segment_capacity} (Miller's Law 7±2)"
        )

    def register_event_segment(self, role: str, content: str) -> list[dict]:
        """
        Register a new conversation event segment in working memory.

        Returns a list of evicted event segments (may be empty) so MCC can forward
        them to EMC asynchronously.

        Args:
            role:    The role of the speaker ("user" or "assistant")
            content: The content of the conversation turn

        Returns:
            List of evicted event segments [{role, content, timestamp}]
        """
        event_segment = {
            "role":      role,
            "content":   content,
            "timestamp": datetime.now().isoformat(),
        }
        event_segment_load = _estimate_chunks_in_segment(event_segment)

        # Evict oldest event segments until new event segment fits or event segment capacity is reached
        evicted: list[dict] = []
        while self._event_segments and (
            self._active_chunks + event_segment_load > self.chunk_capacity
            or len(self._event_segments) >= self.event_segment_capacity + self.event_segment_buffer
        ):
            oldest = self._event_segments.popleft()
            self._active_chunks -= _estimate_chunks_in_segment(oldest)
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
