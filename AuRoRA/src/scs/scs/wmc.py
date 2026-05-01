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
        Global chunk limit — cognitive engine context window minus system prompt,
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
from datetime import datetime            # for PMT timestamps — (TODO) replaced by hrs.blc when BioLogic Clock is built
from collections import deque            # for PMT slot — O(1) append and popleft on eviction
import json                              # for structured PMT storage — serialization and recall

# AGi libraries
from hrs.hrp import AGi                  # homeostatic regulation parameter registry — system-wide constants
CNS = AGi.CNS                            # CNS parameter namespace alias — keeps constant references concise
WMC = CNS.WMC                            # WMC parameter namespace alias — keeps WMC constant references concise

def _estimate_chunk_count(pmt: dict) -> int:
    """
    Estimate the number of chunks in a given PMT.
    Each PMT is one complete interaction — user prompt + AI response paired.

    Args:
        pmt (dict) : A complete interaction with 'timestamp' and 'content'

    Returns:
        int : Number of chunks, including overhead for formatting
    """
    content: str = pmt.get("content", "")                                                                   # extract raw content string from PMT
    content_chunk_count: int = max(1, (len(content) + CNS.UNITS_PER_CHUNK - 1 ) // CNS.UNITS_PER_CHUNK)     # ceiling division — minimum 1 chunk even for empty content
    return content_chunk_count + WMC.PMT_OVERHEAD                                                           # add fixed overhead for PMT formatting

class WorkingMemoryCortex:
    """
    Working Memory Cortex — the active conversation window of the CNS.

    Maintains the sustained PMT slot sent to the cognitive engine on every turn.
    Receding PMT schema are evicted when the global chunk limit or PMT slot limit is exceeded,
    and returned to MCC for async forwarding to EMC.

    Thread-safety: single-threaded by design — protected by CNC._busy flag.
    Only one PMT is processed at a time, ensuring WMC is always accessed from the main neural thread.
    """

    def __init__(self, logger, 
                global_chunk_limit: int = WMC.GLOBAL_CHUNK_LIMIT, 
                pmt_slot_limit: int = WMC.PMT_SLOT_LIMIT,
                pmt_slot_buffer: int = WMC.PMT_SLOT_BUFFER
                ) -> None:
        """
        Initialize the Working Memory Cortex with configured capacity limits.

        Args:
            logger                   : Logger instance forwarded from MCC
            global_chunk_limit (int) : Maximum chunks WMC can sustain — tuned to cognitive engine context window
            pmt_slot_limit (int)     : Maximum PMTs WMC can hold — based on Miller's Law (7±2)
            pmt_slot_buffer (int)    : Additional PMT buffer beyond Miller's Law limit for flexibility
        """
        self.logger                  = logger               # logger forwarded from MCC — all WMC methods emit through this handle
        self.global_chunk_limit: int = global_chunk_limit   # maximum chunks WMC can sustain before eviction
        self.pmt_slot_limit: int     = pmt_slot_limit       # maximum PMTs WMC can hold before eviction
        self.pmt_slot_buffer: int    = pmt_slot_buffer      # additional PMT buffer beyond Miller's Law limit
        self._induced_pmt: dict | None = None               # induced user prompt pending pairing with AI response
        self._pmt_slot: deque[dict]  = deque()              # sustained PMT slot — single-threaded access guaranteed by CNC._busy flag
        self._sustained_chunks: int  = 0                    # running count of sustained chunks across all PMTs

        self.logger.info(                                   # log entry on WMC initialization with configured capacity
            f"   [Working Memory Cortex]  ONLINE ✅ — "
            f"{self.pmt_slot_limit}±{self.pmt_slot_buffer} PMT slots | {self.global_chunk_limit} chunks allocated"
        )
        
    def fill_pmt(self, speaker: str, content: str) -> list[dict]:
        """
        Induce a conversation turn and pair it into a complete interaction.
        Fill the complete interaction into working memory.
        Evict receding PMTs until the induced PMT fits within capacity.

        Args:
            speaker (str) : User ID of the speaker — 'user' or 'assistant'
            content (str) : Content of the conversation turn

        Returns:
            list[dict] : List of evicted PMTs [{timestamp, content}], empty if no eviction occurred
        """
        if speaker == "user":                                   # user turn — stage as induced PMT pending AI response
            if self._induced_pmt is not None:                   # unpaired user prompt already pending — append rather than overwrite
                unpaired_user_prompt: str = self._induced_pmt["content"]["prompt"]   # retrieve existing unpaired prompt
                self._induced_pmt["content"]["prompt"] = (      # append new message — preserve both
                    unpaired_user_prompt + "\n" + content
                )
                self.logger.warn(                               # log the double message append
                    "WMC: second user message before AI response — appended to induced PMT"
                )
                return []                                       # still incomplete — wait for AI response

            # Induce unpaired user prompt — pending for AI response
            self._induced_pmt = {                               # stage induced PMT — pending AI response
                "timestamp": datetime.now().isoformat(),        # wall-clock induction time (TODO M2: use ROS2 time)
                "content": {                                    # embed both user prompt and AI response in the same PMT
                    "speaker": speaker,                         # user ID of the speaker
                    "prompt": content,                          # user prompt — paired with AI response on next turn
                    "response": ""                              # empty until AI responds
                }
            }
            self.logger.debug(                                  # log the induced unpaired user prompt
                "WMC induced unpaired user prompt — pending for AI response"
            )
            return []                                           # exchange incomplete — nothing to fill or evict

        elif speaker == "assistant":                            # assistant turn — complete the pairing
            if self._induced_pmt is None:                       # no induced PMT — unpaired AI response
                self.logger.warning(                            # log the warning of unpaired AI response
                    "WMC: AI response induced without user prompt — wrapping with placeholder"
                )
                self._induced_pmt = {                           # wrap unpaired AI response with placeholder
                    "timestamp": datetime.now().isoformat(),    # wall-clock induction time
                    "content": {                                # embed the unpaired AI response with placeholder
                        "speaker": "user",                      # placeholder speaker
                        "prompt": "[context missing]",          # placeholder for missing user prompt
                        "response": content                     # AI response preserved
                    }
                }
                # Fall through to complete the pairing — unpaired AI response wrapped with placeholder, proceed normally

            else:
                # Complete the pairing of user prompt and AI response to form a complete interaction
                self._induced_pmt["content"]["response"]: str = content  # pair AI response into induced PMT — exchange complete

            # Decay induced PMT into evictable PMT
            content: dict = self._induced_pmt["content"]            # extract complete interaction
            induced_pmt: dict = {                                   # assemble evictable PMT
                "timestamp": self._induced_pmt["timestamp"],        # preserve original induction timestamp
                "content":   json.dumps({                           # serialize user/assistant pair into single content string
                                 "assistant": content["response"],
                             }),
            }
            self._induced_pmt = None                            # clear induced PMT — exchange complete

            induced_pmt_chunks: int = _estimate_chunk_count(induced_pmt)    # estimate chunk cost of incoming PMT


            # Evict receding PMT schema until induced PMT fits or the limit of PMT slot is reached
            # And then fill the induced PMT, to keep working memory always within the capacities
            evicted_pmt_slot: list[dict] = []                                         # buffer for evicted PMTs returned to MCC
            while self._pmt_slot and (                                                # evict until incoming PMT fits within both limits
                self._sustained_chunks + induced_pmt_chunks > self.global_chunk_limit # global chunk limit would be exceeded, or
                or len(self._pmt_slot) >= self.pmt_slot_limit + self.pmt_slot_buffer  # PMT slot limit reached
            ):
                evicted_pmt: dict           = self._pmt_slot.popleft()                          # evict oldest PMT from working memory
                evicted_pmt_slot.append(evicted_pmt)                                            # stage for return to MCC
                evicted_chunks: int         = _estimate_chunk_count(evicted_pmt)                # calculate chunk cost of evicted PMT
                self._sustained_chunks: int = max(0, self._sustained_chunks - evicted_chunks)   # decrement sustained chunks — floor at 0
                self.logger.debug(                                                              # log the eviction of the receding PMT
                    f"WMC evict → EMC: size={evicted_chunks} chunks"
                )

            # Fill the induced PMT into working memory
            self._pmt_slot.append(induced_pmt)                # fill induced PMT into working memory
            self._sustained_chunks += induced_pmt_chunks      # increment sustained chunk count

            self.logger.debug(                                # log the filling and eviction for development/troubleshooting
                f"WMC filled [{speaker}] | "
                f"sustained={len(self._pmt_slot)} | "
                f"chunks={self._sustained_chunks}/{self.global_chunk_limit} | "
                f"evicted={len(evicted_pmt_slot)}"
            )

            return evicted_pmt_slot                            # return evicted PMTs to MCC for async forwarding to EMC

        return []                                              # unknown speaker — nothing to fill or evict

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
            except (json.JSONDecodeError, KeyError):                                               # When error occurs during deseralization - malformed content format
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
