"""
WMC — Working Memory Cortex
============================
AuRoRA · Semantic Cognitive System (SCS)

Active conversation context for the robot.
Mirrors human working memory — fast, limited capacity, current focus only.

Capacity: utterance-counted, stays under utterance capacity to fit LLM context window with room for system prompt + robot personality + EMC context injection.
Overflow: oldest utterances pushed to MCC → EMC buffer (async)

Miller's Law: 7±2 utterances — both utterance count and chunk capacity are enforced
as hybrid limits to ensure the context window stays within LLM constraints
while maintaining conversation coherence.
"""

from datetime import datetime            # (TODO) Replace with hrs.blc when BioLogic Clock is built
from collections import deque            # For use in memory management

# Reserve ~30% for system prompt + robot personality + EMC context injection

# Retrieve the chunk size and utterance overhead from homeostatic regulation system parameters (HRS.HRP) for dynamic configuration of WMC based on LLM context window and hardware constraints
# Fallback to default if HRS cannot be called
try:                                    # Attempt to reach HRS
    from hrs.hrp import (               # Import parameters from HRS
        UNITS_PER_CHUNK,                # Retrieve number of units per chunk
        PHONO_TRACE_OVERHEAD,           # Retrieve overhead in chunks for each utterance (for role label and formatting)        
        DEFAULT_WMC_GLOBAL_LIMIT,       # Retrieve default chunk capacity for WMC, tunable based on LLM context window and hardware constraints
        DEFAULT_WMC_PHONO_LIMIT,        # Retrieve default utterance capacity for WMC
        DEFAULT_WMC_PHONO_BUFFER        # Retrieve default utterance buffer for WMC
    )
except ImportError:                     # If error cannot reach HRS,
    UNITS_PER_CHUNK = 4                 # Fallback to default value for units per chunk
    PHONO_TRACE_OVERHEAD = 4            # Fallback to default value for overhead per utterance
    DEFAULT_WMC_GLOBAL_LIMIT = 1440     # Fallback to default chunk capacity for WMC, tunable based on LLM context window and hardware constraints
    DEFAULT_WMC_PHONO_LIMIT = 7         # Fallback to default utterance capacity for WMC, based on Miller's Law 7±2
    DEFAULT_WMC_PHONO_BUFFER = 2        # Fallback to default utterance buffer for WMC, based on Miller's Law 7±2

def _estimate_chunk_count(phono_trace: dict) -> int:
    """
    Estimate the number of chunks in the given phonological memory trace.

    Args:
        phono_trace (dict): a conversation turn with 'role' and 'content'

    Returns:
        int: Number of chunks, including overhead for role label and formatting
    """
    role = phono_trace.get("role", "")                                  # Retrieve role from utterance
    content = phono_trace.get("content", "")                            # Retrieve conversation content from utterance
    role_chunk_count = len(role) // UNITS_PER_CHUNK                     # Calculate chunks for role label
    content_chunk_count = max(1, len(content) // UNITS_PER_CHUNK)       # Calculate chunks for content, minimum 1 chunk even for empty content
    return role_chunk_count + content_chunk_count + PHONO_TRACE_OVERHEAD  # Return total chunk count including overhead for formatting and role label

class WorkingMemoryCortex:
    """
    Working Memory Cortex.

    Maintains the active conversation window sent to LLM on every utterance.
    Oldest utterances are evicted when the chunk capacity is exceeded and returned
    to MCC for async forwarding to EMC.

    Thread-safety: Single-threaded by design — protected by CNC._busy flag.
    Only one conversation turn is processed at a time, ensuring WMC is
    always accessed from the asyncio event loop thread only.
    """

    def __init__(self, logger, 
                global_limit: int = DEFAULT_WMC_GLOBAL_LIMIT, 
                phono_limit: int = DEFAULT_WMC_PHONO_LIMIT,
                phono_buffer: int = DEFAULT_WMC_PHONO_BUFFER
                ):
        """
        Initialize WMC with chunk capacity and utterance capacity.

        Args:
            logger (Logger): Logger instance from CNC for logging WMC operations
            global_limit (int): Maximum number of chunks WMC can hold (tunable based on LLM context window and hardware constraints)
            phono_limit (int): Maximum number of utterances WMC can hold (based on Miller's Law 7±2, but can be adjusted as needed)
            phono_buffer (int): Additional buffer for utterances beyond Miller's Law limit to allow flexibility if chunks are small (default 2)
        """
        self.logger               = logger                  # Retrieve logger from CNC for logging WMC operations
        self.global_limit         = global_limit            # Retrieve global limit of from MCC configuration for WMC
        self.phono_limit          = phono_limit             # Retrieve utterance capacity from MCC configuration for WMC
        self.phono_buffer         = phono_buffer            # Retrieve utterance buffer from MCC configuration for WMC
        self._phono_slot: deque[dict] = deque()             # Set up buffer for holding the working memory
        self._sustained_chunks: int  = 0                       # Start with empty working memory with 0 active chunks

        self.logger.info(                                   # Log entry on WMC initialization with configured capacities
            f"✅ WorkingMemoryCortex initialised — "
            f"chunk capacity: {self.global_limit} | "
            f"utterance capacity: {self.phono_limit} (based on Miller's Law 7±2)"
        )

    def __len__(self) -> int:
        """
        Return number of active utterances in working memory.

        Returns:
            int: Number of active utterances in working memory
        """
        return len(self._phono_slot)                    # Return the number of active utterances in working memory

    def __repr__(self) -> str:
        phono_schema_stats = self.phono_schema_stats()
        return (                                    # Return the string representation of the working memory status for debugging and logging purposes
            f"WorkingMemoryCortex(phono_traces={phono_schema_stats['phono_trace_count']}, "
            f"chunks={phono_schema_stats['sustained_chunks']}/{phono_schema_stats['global_limit']} "
            f"[{phono_schema_stats['load_percent']}%])"
        )
        
    def register_phono_trace(self, role: str, content: str) -> list[dict]:
        """
        Register a new conversation utterance in working memory.

        Returns a list of evicted utterances (may be empty) so MCC can forward
        them to EMC asynchronously.

        Args:
            role (str): The role of the speaker ("user" or "assistant")
            content (str): The content of the utterance to be registered in working memory

        Returns:
            list[dict]: List of evicted utterances [{role, content, timestamp}]
        """
        phono_trace = {                                  # Combine the elements of utterance into one package for registration
            "role":      role,                           # Register the role of the speaker
            "content":   content,                        # Register content of the conversation
            "timestamp": datetime.now().isoformat(),     # Register the time of the event
        }
        phono_trace_chunks = _estimate_chunk_count(phono_trace)     # Calculate how many chunks in the utterance to be registered

        # Evict oldest utterances until new utterance fits or utterance capacity is reached
        # And then append new utterance, to keep working memory always within the capacities
        decayed_phono_slot: list[dict] = []                         # Set up buffer for holding decaying memory
        
        while self._phono_slot and (                                                        # Evict loop when the working memory is not empty, and one of the following condition is met
            self._sustained_chunks + phono_trace_chunks > self.global_limit                   # Chunk capacity exceeded after registration of the given utterance, or
            or len(self._phono_slot) >= self.phono_limit + self.phono_buffer                # utterance count exceeds the configured capacity (Miller's Law 7±2, tunable)
        ):
            evicted_phono_trace = self._phono_slot.popleft()                                # Evict the oldest utterance from the working memory 
            decayed_phono_slot.append(evicted_phono_trace)                                  # Append the evicted utterance to the list of decaying memory
            evicted_chunks = _estimate_chunk_count(evicted_phono_trace)                     # Calculate the chunk size of the evicted utterance
            self._sustained_chunks -= evicted_chunks                                        # Update the chunk size of the working memory
            self.logger.debug(                                                              # Log the eviction of the oldest utterance
                f"WMC evict → EMC: [{evicted_phono_trace['role']}] "
                f"size={evicted_chunks} chunks"
            )

        self._phono_slot.append(phono_trace)                  # Register the new utterance into working memory
        self._sustained_chunks += phono_trace_chunks          # Update the chunk size of the working memory

        self.logger.debug(                                    # Log the registration and eviction for development/troubleshooting
            f"WMC registered [{role}] | "
            f"phono_trace={len(self._phono_slot)} | "
            f"chunks={self._sustained_chunks}/{self.global_limit} | "
            f"evicted={len(decayed_phono_slot)}"
        )

        return decayed_phono_slot                             # Return the list of decaying memory back to MCC

    def recall_phono_schema(self) -> list[dict]:
        """
        Recall active utterances for context window construction.
        Return utterances in chronological order (oldest first).
        Only includes role + content — timestamp stripped for LLM.
        Timestamps are only used for logging and memory management.

        Returns:
            list[dict]: List of active utterances [{role, content}]
        """
        return [                                                            # Return the list of active utterances in chronological order (oldest first)
            {"role": phono_trace["role"], "content": phono_trace["content"]}
            for phono_trace in self._phono_slot
        ]

    def phono_schema_stats(self) -> dict:
        """
        Return current memory usage stats.

        Returns:
            dict: Current memory usage stats including utterance count, active chunks, free chunks, chunk capacity, and load percentage
        """
        return {                                                            # Return the number of utterances and chunk usage in the working memory
            "phono_trace_count":  len(self._phono_slot),
            "sustained_chunks":   self._sustained_chunks,
            "free_chunks":        self.global_limit - self._sustained_chunks,
            "global_limit":       self.global_limit,
            "load_percent":       round(self._sustained_chunks / self.global_limit * 100, 1) if self.global_limit > 0 else 0.0
        }

    def discard_phono_schema(self) -> list[dict]:
        """
        Discard all utterances from working memory.
        Called at conversation end or on explicit reset.
        Does NOT forward to EMC — utterances are permanently discarded unless
        caller chooses to save the returned list.

        Returns:
            list[dict]: List of discarded utterances [{role, content, timestamp}]
        """
        discarded_phono_schema = list(self._phono_slot)    # Capture utterances before discarding, # Safe — single-threaded access guaranteed by CNC._busy flag
        self._phono_slot.clear()                           # Wipe out working memory
        self._sustained_chunks = 0                         # Zero out active chunk count
        self.logger.info(                                  # Log the discarding of all utterances from working memory
            f"🧹 WMC discarded ({len(discarded_phono_schema)} utterances)"
        )
        return discarded_phono_schema                     # Return discarded utterances to caller

    def is_empty(self) -> bool:
        """
        Check if working memory is empty.

        Returns:
            bool: True if working memory is empty, False otherwise
        """
        return len(self._phono_slot) == 0               # Return True if working memory is empty, False otherwise
