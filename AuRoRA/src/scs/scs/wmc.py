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

from datetime import datetime            # (TODO) Replace with hrs.blc when BioLogic Clock is built
from collections import deque            # For use in memory management

# LLM: max_model_len = 2048 (cosmos.sh)
# Reserve ~30% for system prompt + GRACE personality + EMC context injection

# Retrieve the chunk size and segment overhead from homeostatic regulation system
# Fallback to default if HRS cannot be called
try:                                    # Attempt to reach HRS
    from hrs.hrp import (               # Import parameters from HRS
    UNITS_PER_CHUNK,                    # Retrieve number of units per chunk
    SEGMENT_OVERHEAD,                   # Retrieve overhead in chunks for each event segment (for role label and formatting)        
    DEFAULT_CHUNK_CAPACITY,             # Retrieve default chunk capacity for WMC, tunable based on LLM context window and hardware constraints
    DEFAULT_EVENT_SEGMENT_CAPACITY,     # Retrieve default event segment capacity for WMC, based on Miller's Law 7±2
    DEFAULT_EVENT_SEGMENT_BUFFER        # Retrieve default event segment buffer for WMC, based on Miller's Law 7±2 
)
except ImportError:                     # If error cannot reach HRS,
    UNITS_PER_CHUNK = 4                 # Fallback to default value for units per chunk
    SEGMENT_OVERHEAD = 4                # Fallback to default value for overhead per event segment
    DEFAULT_CHUNK_CAPACITY = 1440       # Fallback to default chunk capacity for WMC, tunable based on LLM context window and hardware constraints
    DEFAULT_EVENT_SEGMENT_CAPACITY = 7  # Fallback to default event segment capacity for WMC, based on Miller's Law 7±2
    DEFAULT_EVENT_SEGMENT_BUFFER = 2    # Fallback to default event segment buffer for WMC, based on Miller's Law 7±2

def _estimate_chunks_in_segment(event_segment: dict) -> int:
    """
    Estimate the number of chunks in the given event segment.

    Args:
        event_segment (dict): Event segment with 'role' and 'content'

    Returns:
        int: Number of chunks, including overhead for role label and formatting
    """
    role = event_segment.get("role", "")                                # Retrieve role from event segment
    content = event_segment.get("content", "")                          # Retrieve conversation content from event segment
    role_chunks = len(role) // UNITS_PER_CHUNK                          # Calculate chunks for role label
    content_chunks = max(1, len(content) // UNITS_PER_CHUNK)            # Calculate chunks for content, minimum 1 chunk even for empty content
    return role_chunks + content_chunks + SEGMENT_OVERHEAD              # Return total chunk count including overhead for formatting and role label

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
        self.logger                 = logger                    # Retrieve logger from CNC for logging WMC operations
        self.chunk_capacity         = chunk_capacity            # Retrieve chunk capacity from MCC configuration for WMC
        self.event_segment_capacity = event_segment_capacity    # Retrieve event segment capacity from MCC configuration for WMC
        self.event_segment_buffer   = event_segment_buffer      # Retrieve event segment buffer from MCC configuration for WMC
        self._memory: deque[dict] = deque()                     # Set up buffer for holding the working memory
        self._active_chunks: int    = 0                         # Start with empty working memory with 0 active chunks

        self.logger.info(                                       # Log entry on WMC initialization with configured capacities
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
        event_segment = {                                # Combine the elements of event_segment into one package for registration
            "role":      role,                           # Register the role of the speaker
            "content":   content,                        # Register content of the conversation
            "timestamp": datetime.now().isoformat(),     # Register the time of the event
        }
        event_segment_size = _estimate_chunks_in_segment(event_segment)    # Calculate how many chunks in the event segment to be registered

        # Evict oldest event segments until new event segment fits or event segment capacity is reached
        # And then append new event segment, to keep working memory alway within the capacities
        decaying_memory: list[dict] = []                 # Set up buffer for holding decaying memory
        
        while self._memory and (                                                                # Evict loop when the working memory is not empty, and one of the following condition is met
            self._active_chunks + event_segment_size > self.chunk_capacity                      # Chunk capacity exceeded after registration of the given event segment, or
            or len(self._memory) >= self.event_segment_capacity + self.event_segment_buffer     # Event segment count exceeds the configured capacity (Miller's Law 7±2, tunable)
        ):
            evicted_event_segment = self._memory.popleft()                                      # Evict the oldest part of the working memory
            decaying_memory.append(evicted_event_segment)                                       # Append the evicted part to the list of decaying memory
            evicted_event_segment_size = _estimate_chunks_in_segment(evicted_event_segment)     # Calculate the chunk size of the evicted event segment
            self._active_chunks -= evicted_event_segment_size                                   # Update the chunk size of the working memory
            self.logger.debug(                                                                  # Log the eviction for development/troubleshooting
                f"WMC evict → EMC: [{evicted_event_segment['role']}] "                          # Log the role of the event segment evicted
                f"size={evicted_event_segment_size} chunks"                                     # Log the chunk size of the event segment evicted
            )

        self._memory.append(event_segment)                                # Register the given event segment into working memory
        self._active_chunks += event_segment_size                         # Update the chunk size of the working memory

        self.logger.debug(                                                # Log the registration and eviction for development/troubleshooting
            f"WMC registered [{role}] | "                                 # Log the role of the event segment registered
            f"segments={len(self._memory)} | "                            # Log the event segment count of the working memory
            f"chunks={self._active_chunks}/{self.chunk_capacity} | "      # Log the chunk size of the working memory over chunk capacity
            f"evicted={len(decaying_memory)}"                             # Log the event segment count of the decaying memory
        )

        return decaying_memory                                            # Return the list of decaying memory back to MCC

    def get_event_segments(self) -> list[dict]:
        """
        Return active event segments for context window construction.
        Returns event segments in chronological order (oldest first).
        Only includes role + content — timestamp stripped for LLM.
        """
        return [
            {"role": t["role"], "content": t["content"]}
            for t in self._memory
        ]

    def peek_last(self, n: int = 1) -> list[dict]:
        """Return the last N event segments (most recent) with timestamps."""
        event_segments = list(self._memory)
        return event_segments[-n:] if n <= len(event_segments) else event_segments

    def chunk_usage(self) -> dict:
        """Return current chunk usage stats."""
        return {
            "used":    self._active_chunks,
            "capacity":  self.chunk_capacity,
            "free":    self.chunk_capacity - self._active_chunks,
            "event_segments":   len(self._memory),
            "percent": round(self._active_chunks / self.chunk_capacity * 100, 1),
        }

    def clear(self):
        """
        Clear all event segments from working memory.
        Called at conversation end or on explicit reset.
        Does NOT evict to EMC — caller is responsible for saving if needed.
        """
        count = len(self._memory)
        self._memory.clear()
        self._active_chunks = 0
        self.logger.info(f"🧹 WMC cleared ({count} event segments discarded)")

    def is_empty(self) -> bool:
        return len(self._memory) == 0

    def __len__(self) -> int:
        return len(self._memory)

    def __repr__(self) -> str:
        u = self.chunk_usage()
        return (
            f"WMC(event_segments={u['event_segments']}, "
            f"chunks={u['used']}/{u['capacity']} "
            f"[{u['percent']}%])"
        )
