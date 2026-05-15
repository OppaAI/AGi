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
    Induction → Filling → Sustaining → Receding → Evicting ---> EMC
                              ↓
                            Recalling → Reinstatement ---> MCC ---> CNC
                              

Public interface:
    wmc.fill_pmt(user_id, role, content) -> tuple[PMT | None, list[PMT]]
    wmc.recall_pmt_schema() -> list[PMT]
    wmc.reinstate_pmt_schema() -> list[dict]
    wmc.forget_pmt_schema() -> list[PMT]
    wmc.assess_pmt_schema() -> dict
    wmc.is_empty -> bool
"""

# System components
from huggingface_hub.inference._generated.types import zero_shot_image_classification
from collections import deque            # for PMT slot — O(1) append and popleft on eviction
from dataclasses import dataclass       # for PMT — phonological memory trace schema
from datetime import datetime            # for PMT timestamps — (TODO) M1.6 replaced by hrs.blc when BioLogic Clock is built
import json                              # for structured PMT storage — serialization and recall

# AGi components
from hrs.hru import ChunkSampler         # probes and truncates cognitive context for budget management
from hrs.hrm import AGi                  # homeostatic regulation manifest namespace — system-wide constants
SCS = AGi.SCS                            # SCS parameter namespace alias — keeps constant references concise
WMC = SCS.WMC                            # WMC parameter namespace alias — keeps WMC constant references concise

@dataclass
class PMT:
    """
    Phonological Memory Trace — one complete interaction in working memory.
    Pairs a user prompt and AI response into a single evictable unit.

    Lifecycle:
        Induction → Filling → Sustaining → Receding → Evicting

    Biological analogue: a single episode held in the phonological loop,
    tagged by the hippocampus during experience for potential consolidation.
    """
    user_id         : str | None        # speaker identity — None for assistant-originated
    timestamp       : str = ""          # ISO wall-clock induction time — M1.6 replaces with ROS2 time
    user_prompt     : str = ""          # raw user prompt — source of truth during staging
    ai_response     : str = ""          # raw AI response — empty until pairing complete
    content         : str = ""          # JSON pair — derived at pairing, WMC chat history for LLM
    trace           : str = ""          # formatted text — for EMC embedding and reinstatement
    chunk_count     : int = 0           # cached token count — O(1) eviction math, no reprobe on eviction
    vector          : list[float]       # semantic vector computed at induction — reused at EMC binding, no re-inference
    retention_score : float = 0.0       # composite induction score — WMC eviction priority key
    salience_score  : float = 0.0       # Factor 1 score — logged and inspectable at eviction boundary
    novelty_factor  : float = 1.0       # Factor 2 multiplier — inspectable at eviction boundary
    depth_score     : float = 0.0       # Factor 3 score — logged and inspectable at eviction boundary
    anchored        : bool  = False     # True if hard-gated (explicit marker or salience override)
    smc_candidate   : bool  = False     # True if regex fact extractor flagged remainder for Dream Cycle
    
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
        self._pmt_slot: deque[PMT]   = deque()              # sustained PMT slot — single-threaded access guaranteed by CNC._busy flag
        self._sustained_chunks: int  = 0                    # running count of sustained chunks across all PMTs


        self.logger.info(                                   # log entry on WMC initialization with configured capacity
            f"   [Working Memory Cortex]  ONLINE ✅ — "
            f"{self.pmt_slot_limit}±{self.pmt_slot_buffer} PMT slots | {self.global_chunk_limit} chunks allocated"
        )

    def _induce_pmt(self, user_id: str | None, role: str, content: str) -> PMT | None:
        """
        Induce a conversation turn into a staged or completed PMT.
        User turn stages the prompt into an induced PMT pending AI response.
        Assistant turn completes the pairing and promotes to an evictable PMT.

        Biological analogue: phonological loop encoding — the user prompt is held
        in active rehearsal until the AI response arrives to complete the episode.
        The completed pair is then tagged by the hippocampus for potential consolidation.

        Args:
            user_id (str | None) : Speaker identity — None for assistant-originated
            role (str)           : Role of the speaker — 'user' or 'assistant'
            content (str)        : Content of the conversation turn

        Returns:
            PMT | None : Completed evictable PMT on assistant turn — None on user turn or unknown speaker
        """
        if role == "user":
            if self._induced_pmt is not None:                                           # unpaired prompt already pending — append
                self._induced_pmt.user_prompt += "\n" + content                         # plain append — no json.loads needed
                self._induced_pmt.trace       += "\n" + content                         # mirror in trace
                self.logger.warn("WMC: second user message — appended to induced PMT")  # log double message
                return None                                                             # still incomplete

            self._induced_pmt = PMT(                                                    # stage user prompt — pending for AI response
                user_id     = user_id,
                timestamp   = datetime.now().isoformat(),                               # wall-clock induction time (TODO M1.6: use ROS2 time)
                user_prompt = content,                                                  # raw user prompt — source of truth during staging
                trace       = f'{user_id} said: "{content}"',                           # partial trace — assistant appended on pairing
                chunk_count = 0,                                                        # filled on assistant turn — cached chunk count
                vector      = [],                                                       # filled by MCC at induction scoring — reused at EMC binding
                anchored    = False,                                                    # hard-gate flag
            )
            self.logger.debug("WMC induced unpaired user prompt — pending for AI response")
            return None                                                                 # exchange incomplete — nothing to fill or evict

        elif role == "assistant":
            if self._induced_pmt is None:                                               # no induced PMT — unpaired AI response
                self.logger.warning("WMC: AI response without user prompt — wrapping with placeholder")
                self._induced_pmt = PMT(                                                # wrap with placeholder
                    user_id     = user_id,
                    timestamp   = datetime.now().isoformat(),                           # wall-clock induction time
                    user_prompt = "[context missing]",                                  # placeholder for missing user prompt
                    ai_response = content,                                              # AI response preserved
                    content     = json.dumps({"user": "[context missing]", "assistant": content}), # derived immediately — pair already complete
                    trace       = f'[context missing]\nYou replied: "{content}"',       # formatted — consistent with paired trace format
                    chunk_count = 0,                                                    # filled below — cached chunk count
                    vector      = [],                                                   # filled by MCC at induction scoring
                    anchored    = False,                                                # hard-gate flag
                )
            else:                                                                       # normal path — complete the pairing
                self._induced_pmt.ai_response = content                          # store raw AI response
                self._induced_pmt.content = json.dumps({                                # derive JSON once — pairing complete
                    "user"      : self._induced_pmt.user_prompt,                        # raw user prompt
                    "assistant" : content                                               # raw AI response
                })
                self._induced_pmt.trace = f'{self._induced_pmt.trace}\nYou replied: "{content}"'  # complete trace

            induced_pmt             = self._induced_pmt                                 # promote staged PMT to evictable
            induced_pmt.chunk_count = self.chunk_sampler.probe(                         # cache chunk count — avoid reprobe on eviction
                content  = induced_pmt.content,
                overhead = WMC.PMT_OVERHEAD
            )
            self._induced_pmt = None                                                    # clear induced PMT — exchange complete
            return induced_pmt                                                          # completed PMT — ready for eviction and filling

        return None                                                                     # unknown speaker

    def _evict_pmt(self, induced_pmt_chunks: int) -> list[PMT]:
        """
        Evict receding PMTs from the slot until the incoming PMT fits within capacity.
        Eviction triggers when either the global chunk limit or PMT slot limit is exceeded.
        Evicted PMTs are returned to MCC for async forwarding to EMC episodic buffer.
        Biological analogue: displacement from the phonological loop —
        oldest traces fade when capacity is exceeded by incoming stimuli.

        Args:
            induced_pmt_chunks (int) : Cached chunk count of the incoming PMT

        Returns:
            list[PMT] : Evicted PMTs — returned to MCC for episodic binding
        """
        evicted_pmt_slot: list[PMT] = []                                                             # buffer for evicted PMTs returned to MCC
        while self._pmt_slot and (                                                                   # evict until incoming PMT fits within both limits
            self._sustained_chunks + induced_pmt_chunks > self.global_chunk_limit                    # global chunk limit would be exceeded, or
            or len(self._pmt_slot) >= self.pmt_slot_limit + self.pmt_slot_buffer                     # PMT slot limit reached
        ):
            evicted_pmt: PMT    = self._pmt_slot.popleft()                                           # evict oldest PMT from working memory
            evicted_pmt_slot.append(evicted_pmt)                                                     # stage for return to MCC
            evicted_chunks: int = evicted_pmt.chunk_count                                            # cached at induction — no reprobe on eviction
            self._sustained_chunks = max(0, self._sustained_chunks - evicted_chunks)                 # decrement sustained chunks — floor at 0
            self.logger.debug(                                                                       # log the eviction of the receding PMT
                f"WMC evict → EMC: size={evicted_chunks} chunks"
            )
        return evicted_pmt_slot                                                                      # return evicted PMTs to caller for episodic binding

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
            tuple[PMT | None, list[PMT]] : (filled_pmt, evicted_pmts) — None for filled_pmt on incomplete user turn
        """
        induced_pmt = self._induce_pmt(user_id, role, content)                       # stage or complete PMT — None on user turn, evictable PMT on assistant turn

        if induced_pmt is None:                                                      # user turn or unknown speaker — nothing to fill or evict
            return None, []

        evicted_pmt_slot = self._evict_pmt(induced_pmt.chunk_count)                  # evict receding PMTs until induced PMT fits
        self._pmt_slot.append(induced_pmt)                                           # fill induced PMT into working memory
        self._sustained_chunks += induced_pmt.chunk_count                            # increment sustained chunk count

        self.logger.debug(                                                           # log filling and eviction
            f"WMC filled [{user_id}] | "
            f"sustained={len(self._pmt_slot)} | "
            f"chunks={self._sustained_chunks}/{self.global_chunk_limit} | "
            f"evicted={len(evicted_pmt_slot)}"
        )
        return induced_pmt, evicted_pmt_slot                                         # filled PMT + evicted PMTs returned to MCC
    
    def recall_pmt_schema(self) -> list[PMT]:
        """
        Recall sustaining PMT schema for context window construction.
        Return PMT schema in ascending chronological order.

        Returns:
            list[PMT]: Sustained PMT schema, in ascending chronological order
        """
        return list(self._pmt_slot)                           # return PMT schema in ascending chronological order

    def reinstate_pmt_schema(self) -> list[dict]:
        """
        Reinstate sustaining PMT schema into the context window for inference.
        Unpacks the PMT schema into interleaved pair of user prompt and AI response.
        Timestamps stripped — only role and content surfaced for inference.
        Return PMT schema in ascending chronological order.

        Returns:
            list[dict]: Sustained PMT schema unpacked as turn pairs [{role, content}], in ascending chronological order
        """
        sustained_pmts: list[dict] = []                                                            # accumulate unpacked turn pairs for inference

        for pmt in self.recall_pmt_schema():                                                       # traverse sustaining PMTs oldest-first
            # Each pmt["content"] is a JSON string — deserialize into user prompt/AI response pairs
            try:                                                                                   # attempt to deserialize the PMT content

                content = json.loads(pmt.content)                                                  # deserialize — each PMT encodes a user/assistant pair
                sustained_pmts.append({"role": "user",      "content": content["user"]})           # Unpack user turn from the content
                sustained_pmts.append({"role": "assistant", "content": content["assistant"]})      # Unpack assistant turn from the content
            except (json.JSONDecodeError, KeyError):                                               # malformed PMT — surface raw rather than drop
                sustained_pmts.append({"role": "user", "content": pmt.content})                    # use the PMT content as-is

        # Return the list of sustained PMT schema in ascending chronological order
        return sustained_pmts                                                                      # ascending chronological order

    def forget_pmt_schema(self) -> list[PMT]:
        """
        Forget all sustaining PMT schema — returns working memory to rest.
        Called at conversation end or on explicit reset.
        Forwarding the forgotten schema to EMC is the caller's responsibility.
    
        Returns:
           list[PMT] : Forgotten PMT schema — caller decides whether to forward to EMC
        """
        forgotten_pmt_schema: list[PMT] = list(self._pmt_slot)      # snapshot before wipe — safe under CNC._busy
        self._pmt_slot.clear()                                      # evict all sustaining PMTs
        self._induced_pmt = None                                    # discard any incomplete induced PMT
        self._sustained_chunks = 0                                  # reset sustained chunk count
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
