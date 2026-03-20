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
        UTTERANCE_OVERHEAD,             # Retrieve overhead in chunks for each utterance (for role label and formatting)        
        DEFAULT_CHUNK_CAPACITY,         # Retrieve default chunk capacity for WMC, tunable based on LLM context window and hardware constraints
        DEFAULT_UTTERANCE_CAPACITY,     # Retrieve default utterance capacity for WMC
        DEFAULT_UTTERANCE_BUFFER        # Retrieve default utterance buffer for WMC
    )
except ImportError:                     # If error cannot reach HRS,
    UNITS_PER_CHUNK = 4                 # Fallback to default value for units per chunk
    UTTERANCE_OVERHEAD = 4              # Fallback to default value for overhead per utterance
    DEFAULT_CHUNK_CAPACITY = 1440       # Fallback to default chunk capacity for WMC, tunable based on LLM context window and hardware constraints
    DEFAULT_UTTERANCE_CAPACITY = 7      # Fallback to default utterance capacity for WMC, based on Miller's Law 7±2
    DEFAULT_UTTERANCE_BUFFER = 2        # Fallback to default utterance buffer for WMC, based on Miller's Law 7±2

def _estimate_chunk_count(utterance: dict) -> int:
    """
    Estimate the number of chunks in the given utterance.

    Args:
        utterance (dict): a conversation turn with 'role' and 'content'

    Returns:
        int: Number of chunks, including overhead for role label and formatting
    """
    role = utterance.get("role", "")                                    # Retrieve role from utterance
    content = utterance.get("content", "")                              # Retrieve conversation content from utterance
    role_chunk_count = len(role) // UNITS_PER_CHUNK                     # Calculate chunks for role label
    content_chunk_count = max(1, len(content) // UNITS_PER_CHUNK)       # Calculate chunks for content, minimum 1 chunk even for empty content
    return role_chunk_count + content_chunk_count + UTTERANCE_OVERHEAD  # Return total chunk count including overhead for formatting and role label

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
                chunk_capacity: int = DEFAULT_CHUNK_CAPACITY, 
                utterance_capacity: int = DEFAULT_UTTERANCE_CAPACITY,
                utterance_buffer: int = DEFAULT_UTTERANCE_BUFFER
                ):
        """
        Initialize WMC with chunk capacity and utterance capacity.

        Args:
            logger (Logger): Logger instance from CNC for logging WMC operations
            chunk_capacity (int): Maximum number of chunks WMC can hold (tunable based on LLM context window and hardware constraints)
            utterance_capacity (int): Maximum number of utterances WMC can hold (based on Miller's Law 7±2, but can be adjusted as needed)
            utterance_buffer (int): Additional buffer for utterances beyond Miller's Law limit to allow flexibility if chunks are small (default 2)
        """
        self.logger               = logger                  # Retrieve logger from CNC for logging WMC operations
        self.chunk_capacity       = chunk_capacity          # Retrieve chunk capacity from MCC configuration for WMC
        self.utterance_capacity   = utterance_capacity      # Retrieve utterance capacity from MCC configuration for WMC
        self.utterance_buffer     = utterance_buffer        # Retrieve utterance buffer from MCC configuration for WMC
        self._memory: deque[dict] = deque()                 # Set up buffer for holding the working memory
        self._active_chunks: int  = 0                       # Start with empty working memory with 0 active chunks

        self.logger.info(                                   # Log entry on WMC initialization with configured capacities
            f"✅ WorkingMemoryCortex initialised — "
            f"chunk capacity: {self.chunk_capacity} | "
            f"utterance capacity: {self.utterance_capacity} (Miller's Law 7±2)"
        )

    def __len__(self) -> int:
        """
        Return number of active utterances in working memory.

        Returns:
            int: Number of active utterances in working memory
        """
        return len(self._memory)                    # Return the number of active utterances in working memory

    def __repr__(self) -> str:
        memory_status = self.memory_status()
        return (                                    # Return the string representation of the working memory status for debugging and logging purposes
            f"WMC(utterances={memory_status['utterance_count']}, "
            f"chunks={memory_status['active_chunks']}/{memory_status['chunk_capacity']} "
            f"[{memory_status['load_percent']}%])"
        )
        
    def register_utterance(self, role: str, content: str) -> list[dict]:
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
        utterance = {                                    # Combine the elements of utterance into one package for registration
            "role":      role,                           # Register the role of the speaker
            "content":   content,                        # Register content of the conversation
            "timestamp": datetime.now().isoformat(),     # Register the time of the event
        }
        utterance_chunks = _estimate_chunk_count(utterance)     # Calculate how many chunks in the utterance to be registered

        # Evict oldest utterances until new utterance fits or utterance capacity is reached
        # And then append new utterance, to keep working memory always within the capacities
        decaying_memory: list[dict] = []                     # Set up buffer for holding decaying memory
        
        while self._memory and (                                                            # Evict loop when the working memory is not empty, and one of the following condition is met
            self._active_chunks + utterance_chunks > self.chunk_capacity                    # Chunk capacity exceeded after registration of the given utterance, or
            or len(self._memory) >= self.utterance_capacity + self.utterance_buffer         # utterance count exceeds the configured capacity (Miller's Law 7±2, tunable)
        ):
            evicted_utterance = self._memory.popleft()                                      # Evict the oldest utterance from the working memory 
            decaying_memory.append(evicted_utterance)                                       # Append the evicted utterance to the list of decaying memory
            evicted_chunks = _estimate_chunk_count(evicted_utterance)                       # Calculate the chunk size of the evicted utterance
            self._active_chunks -= evicted_chunks                                           # Update the chunk size of the working memory
            self.logger.debug(                                                              # Log the eviction of the oldest utterance
                f"WMC evict → EMC: [{evicted_utterance['role']}] "
                f"size={evicted_chunks} chunks"
            )

        self._memory.append(utterance)                                  # Register the new utterance into working memory
        self._active_chunks += utterance_chunks                         # Update the chunk size of the working memory

        self.logger.debug(                                              # Log the registration and eviction for development/troubleshooting
            f"WMC registered [{role}] | "
            f"utterances={len(self._memory)} | "
            f"chunks={self._active_chunks}/{self.chunk_capacity} | "
            f"evicted={len(decaying_memory)}"
        )

        return decaying_memory                                          # Return the list of decaying memory back to MCC

    def recall_active_utterances(self) -> list[dict]:
        """
        Recall active utterances for context window construction.
        Return utterances in chronological order (oldest first).
        Only includes role + content — timestamp stripped for LLM.
        Timestamps are only used for logging and memory management.

        Returns:
            list[dict]: List of active utterances [{role, content}]
        """
        return [                                                            # Return the list of active utterances in chronological order (oldest first)
            {"role": utterance["role"], "content": utterance["content"]}
            for utterance in self._memory
        ]

    def memory_status(self) -> dict:
        """
        Return current memory usage stats.

        Returns:
            dict: Current memory usage stats including utterance count, active chunks, free chunks, chunk capacity, and load percentage
        """
        return {                                                            # Return the number of utterances and chunk usage in the working memory
            "utterance_count":  len(self._memory),
            "active_chunks":    self._active_chunks,
            "free_chunks":      self.chunk_capacity - self._active_chunks,
            "chunk_capacity":   self.chunk_capacity,
            "load_percent":     round(self._active_chunks / self.chunk_capacity * 100, 1) if self.chunk_capacity > 0 else 0.0
        }

    def discard_memory(self) -> list[dict]:
        """
        Discard all utterances from working memory.
        Called at conversation end or on explicit reset.
        Does NOT forward to EMC — utterances are permanently discarded unless
        caller chooses to save the returned list.

        Returns:
            list[dict]: List of discarded utterances [{role, content, timestamp}]
        """
        discarded_memory = list(self._memory)       # Capture utterances before discarding
        self._memory.clear()                        # Wipe out working memory
        self._active_chunks = 0                     # Zero out active chunk count
        self.logger.info(                           # Log the discarding of all utterances from working memory
            f"🧹 WMC discarded ({len(discarded_memory)} utterances)"
        )
        return discarded_memory                     # Return discarded utterances to caller

    def is_empty(self) -> bool:
        """
        Check if working memory is empty.

        Returns:
            bool: True if working memory is empty, False otherwise
        """
        return len(self._memory) == 0               # Return True if working memory is empty, False otherwise
