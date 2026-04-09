"""
WMC — Working Memory Cortex
============================
AuRoRA · Semantic Cognitive System (SCS)

Working memory layer of the CNS — mirrors human working memory.
Fast, limited capacity, current focus only.

Responsibilities:
    - Receive new PMTs from MCC and fill into active buffer (induction → filling)
    - Sustain PMTs in active buffer for context construction (sustaining)
    - Evict receding PMTs back to MCC when capacity exceeded (receding → evicting)
    - Provide sustained PMT schema to MCC for context assembly on each turn (recalling)
    - Forget PMT schema on conversation end or explicit reset (forgetting)

Architecture:
    Deque-based PMT slot, dual-guard eviction policy:
        PMT slot  — deque of sustained PMTs, bounded by Miller's Law limit + buffer
        Chunks    — running count of sustained chunks for capacity management

    Capacity:
        Phonological limit — Miller's Law (7±2 PMTs), tunable via HRS config
        Global chunk limit — LLM context window minus system prompt,
                             GRACE personality, and EMC injection reserve

    Eviction triggers (both checked on every fill):
        - Sustained chunks + incoming PMT chunks > global chunk limit
        - PMT slot count >= Miller's Law limit + buffer

    Overflow:
        - Evicted PMTs returned to MCC → forwarded to EMC buffer (async)
        - Incoming PMT content truncated to fit global chunk limit before filling

    Thread safety:
        - Single-threaded by design — protected by CNC._busy flag
        - No async — all operations are synchronous and in-memory

Terminology:
    Chunk   — unit of LLM context window size (~4 neural units per chunk)
    PMT     — phonological memory trace (one conversation turn, user or assistant)
    Schema  — a subset of PMTs evicted or recalled together as a group
    Slot    — the active deque buffer holding all sustained PMTs

Lifecycle:
    Induction → Filling → Sustaining → Receding → Evicting

Public interface:
    wmc.fill_pmt(speaker, content) → list[evicted_pmt]
    wmc.recall_pmt_schema() → list[dict]
    wmc.forget_pmt_schema() → list[dict]
    wmc.assess_pmt_schema() → dict
    wmc.is_empty → bool

TODO:
    M2 — integrate HRS.BLC for biological clock timestamps
    M3 — add salience weighting to eviction policy
"""

# System libraries
from datetime import datetime            # (TODO) Replace with hrs.blc when BioLogic Clock is built
from collections import deque

# AGi libraries
from hrs.hrp import AGi                  # Import AGi homeostatic regulation parameters
WMC = AGi.CNS.WMC                        # Channel for interfacing with Working Memory Cortex (WMC)

def _estimate_chunk_count(pmt: dict) -> int:
    """
    Estimate the number of chunks in the given PMT.

    Args:
        pmt (dict): a conversation turn with 'speaker' and 'content'

    Returns:
        int: Number of chunks, including overhead for speaker ID and formatting
    """
    speaker: str = pmt.get("speaker", "")                                                                # Retrieve speaker ID from PMT. 
    content: str = pmt.get("content", "")                                                                # Retrieve content from PMT.
    speaker_chunk_count: int = (len(speaker) + WMC.UNITS_PER_CHUNK - 1 ) // WMC.UNITS_PER_CHUNK          # Calculate chunks for speaker ID
    content_chunk_count: int = max(1, (len(content) + WMC.UNITS_PER_CHUNK - 1 ) // WMC.UNITS_PER_CHUNK)  # Calculate chunks for content, minimum 1 chunk even for empty content
    return speaker_chunk_count + content_chunk_count + WMC.PMT_OVERHEAD                                  # Return total chunk count (speaker ID + content + overhead)

class WorkingMemoryCortex:
    """
    Working Memory Cortex.

    Maintains the active conversation window sent to LLM on every PMT.
    Receding PMT schema are evicted when the global chunk limit or PMT slot limit is exceeded,
    and returned to MCC for async forwarding to EMC.

    Thread-safety: Use only one main neural thread by design — protected by CNC._busy flag.
    Only one PMT is processed at a time, ensuring WMC is always accessed from the main neural thread only.
    """

    def __init__(self, logger, 
                global_chunk_limit: int = WMC.GLOBAL_CHUNK_LIMIT, 
                pmt_slot_limit: int = WMC.PMT_SLOT_LIMIT,
                pmt_slot_buffer: int = WMC.PMT_SLOT_BUFFER
                ) -> None:
        """
        Initialize the WMC with global chunk limit and PMT slot limit.

        Args:
            logger: Logger instance from CNC for logging WMC operations
            global_chunk_limit (int): Maximum number of chunks WMC can hold (tunable based on LLM context window and hardware constraints)
            pmt_slot_limit (int): Maximum number of PMTs WMC can hold (based on Miller's Law 7±2, but can be adjusted as needed)
            pmt_slot_buffer (int): Additional buffer for PMTs beyond Miller's Law limit to allow flexibility if chunks are small (default 2)
        """
        self.logger                  = logger               # Retrieve logger from CNC for logging WMC operations
        self.global_chunk_limit: int = global_chunk_limit   # Retrieve global chunk limit of from MCC configuration for WMC
        self.pmt_slot_limit: int     = pmt_slot_limit       # Retrieve PMT slot limit from MCC configuration for WMC
        self.pmt_slot_buffer: int    = pmt_slot_buffer      # Retrieve PMT slot buffer from MCC configuration for WMC
        self._pmt_slot: deque[dict]  = deque()              # Set up slot for holding the phonological memory, Safe — single-threaded access guaranteed by CNC._busy flag
        self._sustained_chunks: int  = 0                    # Start with empty working memory with no sustained chunks

        self.logger.info(                                   # Log entry on WMC initialization with configured capacity
            f"   [Working Memory Cortex]  ONLINE ✅ — "
            f"{self.pmt_slot_limit}±{self.pmt_slot_buffer} PMT slots | {self.global_chunk_limit} chunks allocated"
        )
        
    def fill_pmt(self, speaker: str, content: str) -> list[dict]:
        """
        Fill an induced PMT into working memory.

        Returns the evicted PMT schema (may be empty) so MCC can forward
        them to EMC asynchronously.

        Args:
            speaker (str): The speaker ID ("user" or "assistant") to be induced in working memory
            content (str): The content of the PMT to be induced in working memory

        Returns:
            list[dict]: list of evicted PMTs [{speaker, content, timestamp}]
        """
        # Truncate content if it exceeds the global chunk limit to prevent overflow, ensuring at least 1 chunk for speaker and overhead
        speaker_chunk_count: int   = (len(speaker) + WMC.UNITS_PER_CHUNK - 1) // WMC.UNITS_PER_CHUNK  # Calculate chunks for speaker ID to determine remaining capacity for content
        max_content_chunks: int = self.global_chunk_limit - WMC.PMT_OVERHEAD - speaker_chunk_count # Calculate maximum chunks available for content
        max_content_length: int = max_content_chunks * WMC.UNITS_PER_CHUNK                      # Calculate maximum content length based on remaining chunk capacity
        if len(content) > max_content_length:                                                   # If content exceeds maximum content length,
            content = content[:max_content_length]                                              # Truncate content to fit within global chunk limit
            self.logger.warning(f"WMC content truncated to {max_content_chunks} chunks")        # Log the warning of truncation of the content
        
        induced_pmt: dict = {                          # Combine the elements of induced PMT into one register for filling
            "speaker":   speaker,                      # Register the speaker ID of PMT
            "content":   content,                      # Register the content of the PMT
            "timestamp": datetime.now().isoformat(),   # Register the inducing time of the PMT
        }
        induced_pmt_chunks: int = _estimate_chunk_count(induced_pmt)    # Calculate how many chunks in the PMT to be induced

        # Evict receding PMT schema until induced PMT fits or the limit of PMT slot is reached
        # And then fill the induced PMT, to keep working memory always within the capacities
        evicted_pmt_slot: list[dict] = []                         # Set up buffer for holding receding memory
        
        while self._pmt_slot and (                                                # Evict loop when the working memory is not empty, and one of the following condition is met
            self._sustained_chunks + induced_pmt_chunks > self.global_chunk_limit # - Global chunk limit exceeded after filling of the induced PMT, or
            or len(self._pmt_slot) >= self.pmt_slot_limit + self.pmt_slot_buffer  # - PMT schema exceeds the limit of PMT slot (ie. Miller's Law 7±2, tunable)
        ):
            evicted_pmt: dict = self._pmt_slot.popleft()                                    # Evict the receding PMT from the working memory 
            evicted_pmt_slot.append(evicted_pmt)                                            # Fill the evicted PMT to the evicted slot
            evicted_chunks: int = _estimate_chunk_count(evicted_pmt)                        # Calculate the chunk size of the evicted PMT
            self._sustained_chunks: int = max(0, self._sustained_chunks - evicted_chunks)   # Update the chunk size of the working memory, cannot be negative
            self.logger.debug(                                                              # Log the eviction of the receding PMT
                f"WMC evict → EMC: [{evicted_pmt['speaker']}] "
                f"size={evicted_chunks} chunks"
            )

        self._pmt_slot.append(induced_pmt)                # Fill in the induced PMT into working memory
        self._sustained_chunks += induced_pmt_chunks      # Update the chunk size of the working memory

        self.logger.debug(                                # Log the filling and eviction for development/troubleshooting
            f"WMC filled [{speaker}] | "
            f"sustained={len(self._pmt_slot)} | "
            f"chunks={self._sustained_chunks}/{self.global_chunk_limit} | "
            f"evicted={len(evicted_pmt_slot)}"
        )

        return evicted_pmt_slot                            # Return the list of evicted PMT schema back to MCC

    def recall_pmt_schema(self) -> list[dict]:
        """
        Recall sustaining PMT schema for context window construction.
        Return PMT schema in ascending chronological order.
        Only includes speaker + content — timestamp stripped for LLM.
        Timestamps are only used for logging and memory management.

        Returns:
            list[dict]: List of sustained PMTs [{speaker, content}]
        """
        return [                                                            # Return the list of sustained PMT schema in ascending chronological order
            {"speaker": pmt["speaker"], "content": pmt["content"]}
            for pmt in self._pmt_slot
        ]

    def forget_pmt_schema(self) -> list[dict]:
        """
        Forget all sustaining PMT schema in working memory.
        Called at conversation end or on explicit reset.
        Does NOT forward to EMC — PMT schema are permanently forgotten unless
        caller chooses to save the returned list.

        Returns:
            list[dict]: List of forgotten PMTs [{speaker, content, timestamp}]
        """
        forgotten_pmt_schema: list[dict] = list(self._pmt_slot)     # Capture PMT schema before forgetting, Safe — single-threaded access guaranteed by CNC._busy flag
        self._pmt_slot.clear()                                      # Wipe out working memory
        self._sustained_chunks: int = 0                             # Zero out sustained chunk count
        self.logger.info(                                           # Log the forgetting of the PMT schema from working memory
            f"🧹 WMC forgotten ({len(forgotten_pmt_schema)} PMTs)"
        )
        return forgotten_pmt_schema                                 # Return forgotten PMT schema to caller for optional saving or forwarding to EMC
    
    def assess_pmt_schema(self) -> dict:
        """
        Assess the current status of sustaining PMT schema for logging and monitoring.

        Returns:
            dict: Current memory usage stats including PMT count, sustained chunks, free chunks, global chunk limit, and load percentage
        """
        return {                                     # Return the occupancy of PMT and chunk sustaining in the working memory
            "pmt_count"          : len(self._pmt_slot),
            "pmt_slot_limit"     : self.pmt_slot_limit,
            "slot_occupancy"     : round(len(self._pmt_slot) / self.pmt_slot_limit * 100, 1) if self.pmt_slot_limit > 0 else 0.0,
            "sustained_chunks"   : self._sustained_chunks,
            "free_chunks"        : self.global_chunk_limit - self._sustained_chunks,
            "global_chunk_limit" : self.global_chunk_limit,
            "chunk_occupancy"    : round(self._sustained_chunks / self.global_chunk_limit * 100, 1) if self.global_chunk_limit > 0 else 0.0
        }

    @property
    def is_empty(self) -> bool:
        """
        Check if working memory is empty.

        Returns:
            bool: True if working memory is empty, False otherwise
        """
        return len(self._pmt_slot) == 0               # Return True if working memory is empty, False otherwise
