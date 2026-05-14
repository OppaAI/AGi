"""
WMC — Working Memory Cortex
===========================
AuRoRA · Semantic Cognitive System (SCS)

Working memory layer of the SCS — mirrors human working memory.
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
    wmc.fill_pmt(user_id, role, content) -> list[evicted_pmt]
    wmc.recall_pmt_schema() -> list[dict]
    wmc.forget_pmt_schema() -> list[dict]
    wmc.assess_pmt_schema() -> dict
    wmc.is_empty -> bool
"""

# System components
from datetime import datetime            # for PMT timestamps — (TODO) M1.6 replaced by hrs.blc when BioLogic Clock is built
from collections import deque            # for PMT slot — O(1) append and popleft on eviction
import json                              # for structured PMT storage — serialization and recall

# AGi components
from hrs.hru import ChunkSampler         # probes and truncates cognitive context for budget management
from hrs.hrm import AGi                  # homeostatic regulation manifest namespace — system-wide constants
SCS = AGi.SCS                            # SCS parameter namespace alias — keeps constant references concise
WMC = SCS.WMC                            # WMC parameter namespace alias — keeps WMC constant references concise
from scs.niu import PMT                  # Phonological Memory Trace — typed PMT contract for working memory

class WorkingMemoryCortex:
    """
    Working Memory Cortex — the active conversation window of the SCS.

    Maintains the sustained PMT slot sent to the cognitive engine on every turn.
    Receding PMT schema are evicted when the global chunk limit or PMT slot limit is exceeded,
    and returned to MCC for async forwarding to EMC.

    Thread-safety: single-threaded by design — protected by CNC._busy flag.
    Only one PMT is processed at a time, ensuring WMC is always accessed from the main neural thread.
    """

    def __init__(self, logger, chunk_sampler: ChunkSampler, 
                global_chunk_limit: int = SCS.GLOBAL_CHUNK_LIMIT, 
                pmt_slot_limit: int = WMC.PMT_SLOT_LIMIT,
                pmt_slot_buffer: int = WMC.PMT_SLOT_BUFFER
                ) -> None:
        """
        Initialize the Working Memory Cortex with configured capacity limits.

        Args:
            logger                   : Logger instance forwarded from MCC
            chunk_sampler            : Chunk sampler for accurate chunk counting of the context
            global_chunk_limit (int) : Maximum chunks WMC can sustain — tuned to cognitive engine context window
            pmt_slot_limit (int)     : Maximum PMTs WMC can hold — based on Miller's Law (7±2)
            pmt_slot_buffer (int)    : Additional PMT buffer beyond Miller's Law limit for flexibility
        """
        self.logger                  = logger               # logger forwarded from MCC — all WMC methods emit through this handle
        self.chunk_sampler           = chunk_sampler        # token estimated forwarded from HRM for more accurate token count
        self.global_chunk_limit: int = global_chunk_limit   # maximum chunks WMC can sustain before eviction
        self.pmt_slot_limit: int     = pmt_slot_limit       # maximum PMTs WMC can hold before eviction
        self.pmt_slot_buffer: int    = pmt_slot_buffer      # additional PMT buffer beyond Miller's Law limit
        self._induced_pmt: PMT | None = None                # induced user prompt pending pairing with AI response
        self._pmt_slot: deque[PMT]  = deque()               # sustained PMT slot — single-threaded access guaranteed by CNC._busy flag
        self._sustained_chunks: int  = 0                    # running count of sustained chunks across all PMTs


        self.logger.info(                                   # log entry on WMC initialization with configured capacity
            f"   [Working Memory Cortex]  ONLINE ✅ — "
            f"{self.pmt_slot_limit}±{self.pmt_slot_buffer} PMT slots | {self.global_chunk_limit} chunks allocated"
        )
        
    def fill_pmt(self, user_id: str | None, role: str, content: str) -> tuple[PMT | None, list[PMT]]:
        """
        Induce a conversation turn and pair it into a complete interaction.
        Fill the complete interaction into working memory.
        Evict receding PMTs until the induced PMT fits within capacity.

        Args:
            user_id (str | None) : User ID of the interacting user; None for assistant
            role (str)           : Role of the speaker — 'user' or 'assistant'
            content (str)        : Content of the conversation turn

        Returns:
            list[dict] : List of evicted PMTs [{timestamp, content}], empty if no eviction occurred
        """
        if role == "user":                                      # user turn — stage as induced PMT pending AI response
            if self._induced_pmt is not None:                   # unpaired user prompt already pending — append rather than overwrite
                self._induced_pmt.raw_text = self._induced_pmt.raw_text + "\n" + content  # append new message — preserve both user turns
                self.logger.warn(                               # log the double message append
                    "WMC: second user message before AI response — appended to induced PMT"
                )
                return [], None                                 # still incomplete — wait for AI response

            # Induce unpaired user prompt — pending for AI response
            self._induced_pmt = PMT(
                user_id    = user_id,                       # speaker identity
                timestamp  = datetime.now().isoformat(),    # wall-clock induction time (TODO M1.6: use ROS2 time)
                content    = "",                            # filled on assistant turn — JSON pair
                raw_text   = "",                            # filled on assistant turn — plain concat for EMC
                chunk_cost = 0,                             # filled on assistant turn — cached chunk count
                vector     = [],                            # filled by MCC at induction scoring — reused at EMC binding
                anchored   = False,                         # hard-gate flag
            )

            self.logger.debug(                                  # log the induced unpaired user prompt
                "WMC induced unpaired user prompt — pending for AI response"
            )
            return []                                           # exchange incomplete — nothing to fill or evict

        elif role == "assistant":                               # assistant turn — complete the pairing
            if self._induced_pmt is None:                       # no induced PMT — unpaired AI response
                self.logger.warning(                            # log the warning of unpaired AI response
                    "WMC: AI response induced without user prompt — wrapping with placeholder"
                )
                self._induced_pmt = {                           # wrap unpaired AI response with placeholder
                    "user_id": None,                            # None since no user prompt was provided
                    "timestamp": datetime.now().isoformat(),    # wall-clock induction time
                    "content": {                                # embed the unpaired AI response with placeholder
                        "prompt": "[context missing]",          # placeholder for missing user prompt
                        "response": content                     # AI response preserved
                    }
                }
                # Fall through to complete the pairing — unpaired AI response wrapped with placeholder, proceed normally

            else:
                # Complete the pairing of user prompt and AI response to form a complete interaction
                self._induced_pmt["content"]["response"] = content  # pair AI response into induced PMT — exchange complete
                user_prompt = self._induced_pmt.raw_text                                                    # user prompt stored at induction
                ai_response = content                                                                       # current AI response completes the pair
                self._induced_pmt.content  = json.dumps({"user": user_prompt, "assistant": ai_response})    # serialize pair — WMC chat history format
                self._induced_pmt.raw_text = f"{user_prompt} {ai_response}"                                 # plain concat — EMC encoding and storage

            # Decay induced PMT into evictable PMT
            induced_pmt: PMT = self._induced_pmt                        # promote staged PMT to evictable
            induced_pmt.chunk_count = self.chunk_sampler.probe(         # cache chunk count — avoid reprobe on eviction
                content=induced_pmt.content,
                overhead=WMC.PMT_OVERHEAD
            )
            self._induced_pmt = None                                    # clear induced PMT — exchange complete
            induced_pmt_chunks: int = induced_pmt.chunk_cost            # read cached value — no reprobe


            # Evict receding PMT schema until induced PMT fits or the limit of PMT slot is reached
            # And then fill the induced PMT, to keep working memory always within the capacities
            evicted_pmt_slot: list[dict] = []                                                            # buffer for evicted PMTs returned to MCC
            while self._pmt_slot and (                                                                   # evict until incoming PMT fits within both limits
                self._sustained_chunks + induced_pmt_chunks > self.global_chunk_limit                    # global chunk limit would be exceeded, or
                or len(self._pmt_slot) >= self.pmt_slot_limit + self.pmt_slot_buffer                     # PMT slot limit reached
            ):
                evicted_pmt: dict           = self._pmt_slot.popleft()                                   # evict oldest PMT from working memory
                evicted_pmt_slot.append(evicted_pmt)                                                     # stage for return to MCC
                evicted_chunks: int = evicted_pmt.chunk_cost                                             # cached at induction — no reprobe on eviction
                self._sustained_chunks: int = max(0, self._sustained_chunks - evicted_chunks)            # decrement sustained chunks — floor at 0
                self.logger.debug(                                                                       # log the eviction of the receding PMT
                    f"WMC evict → EMC: size={evicted_chunks} chunks"
                )

            # Fill the induced PMT into working memory
            self._pmt_slot.append(induced_pmt)                # fill induced PMT into working memory
            self._sustained_chunks += induced_pmt_chunks      # increment sustained chunk count

            self.logger.debug(                                # log the filling and eviction for development/troubleshooting
                f"WMC filled [{user_id}] | "
                f"sustained={len(self._pmt_slot)} | "
                f"chunks={self._sustained_chunks}/{self.global_chunk_limit} | "
                f"evicted={len(evicted_pmt_slot)}"
            )

            return induced_pmt, evicted_pmt_slot              # filled PMT + any evicted PMTs returned to MCC

        return []                                             # unknown speaker — nothing to fill or evict

    def recall_pmt_schema(self) -> list[dict]:
        """
        Recall sustaining PMT schema for context window construction.
        Unpacks the PMT schema into interleaved pair of user prompt and AI response.
        Timestamps stripped — only role and content surfaced for inference.
        Return PMT schema in ascending chronological order.

        Returns:
            list[dict]: Sustained PMT schema unpacked as turn pairs [{role, content}], in ascending chronological order
        """
        sustained_pmts = []                                                                        # accumulate unpacked turn pairs for inference

        for pmt in self._pmt_slot:                                                                 # traverse sustaining PMTs oldest-first
            # Each pmt["content"] is a JSON string — deserialize into user prompt/AI response pairs
            try:                                                                                   # attempt to deserialize the PMT content

                content = json.loads(pmt.content)                                                  # deserialize — each PMT encodes a user/assistant pair
                sustained_pmts.append({"role": "user",      "content": content["user"]})           # Unpack user turn from the content
                sustained_pmts.append({"role": "assistant", "content": content["assistant"]})      # Unpack assistant turn from the content
            except (json.JSONDecodeError, KeyError):                                               # malformed PMT — surface raw rather than drop
                sustained_pmts.append({"role": "user", "content": pmt.content")                    # use the PMT content as-is

        # Return the list of sustained PMT schema in ascending chronological order
        return sustained_pmts                                                                      # ascending chronological order

    def forget_pmt_schema(self) -> list[PMT]:
        """
        Forget all sustaining PMT schema — returns working memory to rest.
        Called at conversation end or on explicit reset.
        Forwarding the forgotten schema to EMC is the caller's responsibility.
    
        Returns:
            list[dict] : Forgotten PMT schema [{timestamp, content}]
        """
        forgotten_pmt_schema: list[dict] = list(self._pmt_slot)     # snapshot before wipe — safe under CNC._busy
        self._pmt_slot.clear()                                      # evict all sustaining PMTs
        self._induced_pmt = None                                    # discard any incomplete induced PMT
        self._sustained_chunks: int = 0                             # reset sustained chunk count
        self.logger.info(                                           # log the forgetting of the PMT schema from working memory
            f"🧹 WMC forgotten ({len(forgotten_pmt_schema)} PMTs)"
        )
        return forgotten_pmt_schema                                 # return forgotten PMT schema to caller for optional saving or forwarding to EMC
    
    def assess_pmt_schema(self) -> dict:
        """
        Assess current occupancy of the sustaining PMT schema.
        Used for monitoring and GRACE cognitive state display.

        Returns:
            dict: Current memory usage stats including PMT count, sustained chunks, free chunks, global chunk limit, and load percentage
        """
        return {                                     # return the occupancy of PMT and chunk sustaining in the working memory
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
        True if no PMTs are sustaining in working memory.
        Expected at conversation start and after forgetting.

        Returns:
            bool:  True if PMT slot is empty, False if any PMTs are sustaining
        """
        return len(self._pmt_slot) == 0               # return True if working memory is empty, False otherwise
