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
        Global chunk limit — Cognitive engine context window minus system prompt,
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
    Chunk   — unit of cognitive engine context window size (~4 neural units per chunk)
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
from collections import deque            # For PMT slot — fast FIFO eviction of receding PMTs
import json                              # For structured PMT storage — crash-safe serialization and recall

# AGi libraries
from hrs.hrp import AGi                  # Import AGi homeostatic regulation parameters
CNS = AGi.CNS                            # Channel for interfacing with Central Nervous System (CNS)
WMC = CNS.WMC                            # Channel for interfacing with Working Memory Cortex (WMC)

def _estimate_chunk_count(pmt: dict) -> int:
    """
    Estimate the number of chunks in the given PMT.
    Each PMT is one complete interaction — user prompt + AI response paired.

    Args:
        pmt (dict): a complete interaction with 'timestamp' and 'content'

    Returns:
        int: Number of chunks, including overhead for formatting
    """
    content: str = pmt.get("content", "")                                                                   # Retrieve content from PMT.
    content_chunk_count: int = max(1, (len(content) + CNS.UNITS_PER_CHUNK - 1 ) // CNS.UNITS_PER_CHUNK)     # Calculate chunks for content, minimum 1 chunk even for empty content
    return content_chunk_count + WMC.PMT_OVERHEAD                                                           # Return total chunk count (content + overhead)

class WorkingMemoryCortex:
    """
    Working Memory Cortex.

    Maintains the active conversation window sent to cognitive engine on every PMT.
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
            global_chunk_limit (int): Maximum number of chunks WMC can hold (tunable based on cognitive engine context window and hardware constraints)
            pmt_slot_limit (int): Maximum number of PMTs WMC can hold (based on Miller's Law 7±2, but can be adjusted as needed)
            pmt_slot_buffer (int): Additional buffer for PMTs beyond Miller's Law limit to allow flexibility if chunks are small (default 2)
        """
        self.logger                  = logger               # Retrieve logger from CNC for logging WMC operations
        self.global_chunk_limit: int = global_chunk_limit   # For holding the maximum number of chunks WMC can hold
        self.pmt_slot_limit: int     = pmt_slot_limit       # For holding the maximum number of PMTs WMC can hold
        self.pmt_slot_buffer: int    = pmt_slot_buffer      # For holding the additional buffer for PMTs beyond the slot limit
        self._induced_pmt: dict | None = None               # For holding the induced user prompt pending pairing with AI response
        self._pmt_slot: deque[dict]  = deque()              # For holding the sustained PMTs, Safe — single-threaded access guaranteed by CNC._busy flag
        self._sustained_chunks: int  = 0                    # For tracking the number of sustained chunks in WMC

        self.logger.info(                                   # Log entry on WMC initialization with configured capacity
            f"   [Working Memory Cortex]  ONLINE ✅ — "
            f"{self.pmt_slot_limit}±{self.pmt_slot_buffer} PMT slots | {self.global_chunk_limit} chunks allocated"
        )
        
    def fill_pmt(self, speaker: str, content: str) -> list[dict]:
        """
        Induce a user prompt and pair it with AI response into a complete interaction internally.
        Fill the complete interaction into working memory.
        Evict receding PMT schema until induced PMT fits or the limit of PMT slot is reached.

        Args:
            speaker (str): The user ID of the speaker to be induced in working memory
            content (str): The content of the PMT to be induced in working memory

        Returns:
            list[dict]: list of evicted PMTs [{timestamp, content}], empty if no eviction occurred
        """
        if speaker == "user":                                   # If this is user prompt,
            # Induce unpaired user prompt — pending for AI response
            self._induced_pmt = {                               # Induce the user prompt into PMT
                "timestamp": datetime.now().isoformat(),        # Register the inducing time of the PMT (TODO M2: use ROS2 time)
                "content": {                                    # Embed both user prompt and AI response in the same PMT
                    "speaker": speaker,                         # Register the user ID of the interaction (TODO M2: replace with real user_id)
                    "prompt": content,                          # Register the user prompt of the interaction
                    "response": ""                              # Register the empty response until AI responds
                }
            }
            self.logger.debug(                                  # Log the induced unpaired user prompt
                "WMC induced unpaired user prompt — pending for AI response"
            )
            return []                                           # Exchange incomplete — nothing to induce or evict

        elif speaker == "assistant":                            # If this is AI response,
            if self._induced_pmt is None:                       # If there is no induced PMT (unpaired AI response),
                self.logger.warning(                            # Log the warning of unpaired AI response
                    "WMC: AI response induced without user prompt — wrapping with placeholder"
                )
                self._induced_pmt = {                           # Wrap the unpaired AI response with placeholder
                    "timestamp": datetime.now().isoformat(),    # Register the inducing time of the PMT
                    "content": {                                # Embed the unpaired AI response with placeholder
                        "speaker": "user",                      # Register the user ID of the interaction
                        "prompt": "[context missing]",          # Register the placeholder for missing context
                        "response": content                     # Register the AI response
                    }
                }
                # Fall through to complete the pairing — unpaired AI response wrapped with placeholder, proceed normally

            else:
                # Complete the pairing of user prompt and AI response to form a complete interaction
                self._induced_pmt["content"]["response"]: str = content  # Register the AI response to the induced PMT

            # Decay induced PMT into evictable PMT
            content: dict = self._induced_pmt["content"]            # Extract the complete interaction
            induced_pmt: dict = {                                   # Prepare the PMT to be filled
                "timestamp": self._induced_pmt["timestamp"],        # Register the inducing time of the PMT
                "content":   json.dumps({                           # Combine user prompt and AI response into one content
                                 "user":      content["prompt"],
                                 "assistant": content["response"],
                             }),
            }
            self._induced_pmt = None                            # Clear the induced PMT — exchange complete

            induced_pmt_chunks: int = _estimate_chunk_count(induced_pmt)    # Calculate how many chunks in the PMT to be induced

            # Evict receding PMT schema until induced PMT fits or the limit of PMT slot is reached
            # And then fill the induced PMT, to keep working memory always within the capacities
            evicted_pmt_slot: list[dict] = []                                         # Set up buffer for holding receding memory
            while self._pmt_slot and (                                                # Evict loop when the working memory is not empty, and one of the following condition is met
                self._sustained_chunks + induced_pmt_chunks > self.global_chunk_limit # - Global chunk limit exceeded after filling of the induced PMT, or
                or len(self._pmt_slot) >= self.pmt_slot_limit + self.pmt_slot_buffer  # - PMT schema exceeds the limit of PMT slot (ie. Miller's Law 7±2, tunable)
            ):
                evicted_pmt: dict           = self._pmt_slot.popleft()                          # Evict the receding PMT from the working memory
                evicted_pmt_slot.append(evicted_pmt)                                            # Fill the evicted PMT to the evicted slot
                evicted_chunks: int         = _estimate_chunk_count(evicted_pmt)                # Calculate the chunk size of the evicted PMT
                self._sustained_chunks: int = max(0, self._sustained_chunks - evicted_chunks)   # Update the chunk size of the working memory, cannot be negative
                self.logger.debug(                                                              # Log the eviction of the receding PMT
                    f"WMC evict → EMC: size={evicted_chunks} chunks"
                )

            # Fill the induced PMT into working memory
            self._pmt_slot.append(induced_pmt)                # Fill the induced PMT into working memory
            self._sustained_chunks += induced_pmt_chunks      # Update the chunk size of the working memory

            self.logger.debug(                                # Log the filling and eviction for development/troubleshooting
                f"WMC filled [{speaker}] | "
                f"sustained={len(self._pmt_slot)} | "
                f"chunks={self._sustained_chunks}/{self.global_chunk_limit} | "
                f"evicted={len(evicted_pmt_slot)}"
            )

            return evicted_pmt_slot                            # Return the list of evicted PMT schema back to MCC

        return []                                              # Return empty list if no PMT schema is induced

    def recall_pmt_schema(self) -> list[dict]:
        """
        Recall sustaining PMT schema for context window construction.
        Unpacks the PMT schema into pair of user prompt and AI response.
        Return PMT schema in ascending chronological order.
        Only includes role + content — timestamp stripped for cognitive engine.
        Timestamps are only used for logging and memory management.

        Returns:
            list[dict]: List of sustained PMTs unpacked for cognitive engine inference [{role, content}]
        """
        sustained_pmts = []                                                                        # For collecting recalled pmts for cognitive engine memory context
        for pmt in self._pmt_slot:                                                                 # Iterate over each sustained PMT in the slot
            # Each pmt["content"] is a JSON string — deserialize into user prompt/AI response pairs
            try:                                                                                   # Attempt to deserialize the PMT content
                content = json.loads(pmt["content"])                                               # Deserialize PMT content into user prompt/AI response pair
                sustained_pmts.append({"role": "user",      "content": content["user"]})           # Unpack user prompt from the content
                sustained_pmts.append({"role": "assistant", "content": content["assistant"]})      # Unpack AI response from the content
            except (json.JSONDecodeError, KeyError):                                               # Malformed — surface as-is rather than silent drop
                sustained_pmts.append({"role": "user", "content": pmt["content"]})                 # Use the PMT content as-is

        # Return the list of sustained PMT schema in ascending chronological order
        return sustained_pmts                                                                      # Return the list of unpacked user prompt/AI response pairs

    def forget_pmt_schema(self) -> list[dict]:
        """
        Forget all sustaining PMT schema in working memory.
        Called at conversation end or on explicit reset.
        Does NOT forward to EMC — PMT schema are permanently forgotten unless
        caller chooses to save the returned list.

        Returns:
            list[dict]: List of forgotten PMTs [{timestamp, content}]
        """
        forgotten_pmt_schema: list[dict] = list(self._pmt_slot)     # Capture PMT schema before forgetting, Safe — single-threaded access guaranteed by CNC._busy flag
        self._pmt_slot.clear()                                      # Wipe out working memory
        self._induced_pmt = None                                    # Clear any incomplete induced PMT  
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
