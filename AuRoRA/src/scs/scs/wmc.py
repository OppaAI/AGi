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
from collections import deque            # for PMT slot — O(1) append and popleft on eviction
from datetime import datetime            # for PMT timestamps — (TODO) M1.6 replaced by hrs.blc when BioLogic Clock is built
import json                              # for structured PMT storage — serialization and recall

# AGi components
from gms.csb.AGi.SIU import SensoryInputChannel, SensoryModality, SSS  # sensory input channel — input gateway for all peripheral stimuli (text, speech, vision, etc.)
from gms.csb.AGi.SCS import TraceType  # trace type — PMT, VST, or AST
from gms.csb.AGi.SCS.WMC import PMT, VST, WMCState,       # PMT and PMTState classes — phonological and visuospatial memory traces
from hrs.hru import ChunkSampler         # probes and truncates cognitive context for budget management
from hrs.hrm import AGi                  # homeostatic regulation manifest namespace — system-wide constants
SCS = AGi.SCS                            # SCS parameter namespace alias — keeps constant references concise
WMC = SCS.WMC                            # WMC parameter namespace alias — keeps WMC constant references concise

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

    def _semantic_encode(self, sss: SSS) -> PMT | VST | None:
        """
        Transform SSS into a staged or completed memory trace.
    
        Routes to _populate on first turn (user/raw) to construct and stage
        an incomplete trace, or to _pair on second turn (assistant/interpreted)
        to complete the staged trace and return it for the gating pipeline.
    
        A PMT or VST does not exist until both turns are encoded.
    
        Args:
            sss : SSS — incoming sensory stimulus, either first or second turn
    
        Returns:
            PMT | VST | None — completed trace on second turn, None on first turn
        """
        match sss.role:
            case "user" | "raw":                return self._populate(sss)
            case "assistant" | "interpreted":   return self._pair(sss)
            case _: raise ValueError(f"Unknown SSS role: {sss.role}")
    
    
    def _populate(self, sss: SSS) -> None:
        """
        First turn — construct an incomplete trace shell from SSS and stage it
        in the modality induction slot, pending the second turn.
    
        PMT : user SSS       → populate user fields    → stage in _induced_pmt
        VST : raw frame SSS  → populate sensor fields  → stage in _induced_vst
    
        Always returns None — trace is incomplete until second turn arrives.
    
        Args:
            sss : SSS — first turn sensory stimulus, role 'user' or 'raw'
        """
        match sss.trace_type:
            case TraceType.PMT:
                if self._induced_pmt is not None:                                       # unpaired prompt already pending — append
                    self._induced_pmt.user_prompt += "\n" + sss.text                    # plain append — no json.loads needed
                    self._induced_pmt.trace       += "\n" + sss.text                    # mirror in trace
                    self.logger.warning("WMC: second user SSS — appended to staged PMT")
                    return None                                                         # still incomplete
    
                self._induced_pmt = PMT(
                    # ── identity ──────────────────────────────────────────────────
                    user_id         = sss.user_id,          # SSS.user_id → PMT.user_id
                    timestamp       = sss.generated_at,     # SSS.generated_at → PMT.timestamp
                    interval        = sss.interval,         # SSS.interval → PMT.interval
                    proc            = sss.proc,             # SSS.proc → PMT.proc
                    # ── lifecycle ─────────────────────────────────────────────────
                    state           = WMCState.INDUCED,
                    # ── content ───────────────────────────────────────────────────
                    user_prompt     = sss.text,             # SSS.text → PMT.user_prompt — source of truth during staging
                    ai_response     = "",                   # empty — assistant turn completes this
                    content         = "",                   # empty — derived on pairing completion
                    trace           = f'{sss.user_id} said: "{sss.text}"',  # partial — assistant turn appends
                    # ── scoring ───────────────────────────────────────────────────
                    vector          = sss.vector,           # SSS.vector → PMT.vector — reused at EMC binding
                    salience_score  = sss.urgency,          # SSS.urgency → PMT.salience_score
                    chunk_count     = 0,                    # filled on pairing completion — full pair needed
                    # ── flags ─────────────────────────────────────────────────────
                    anchored        = False,
                )
                self.logger.debug("WMC: user SSS staged — pending assistant turn")
                return None
    
            case TraceType.VST:
                if self._induced_vst is not None:                                       # unpaired frame already pending — warn and replace
                    self.logger.warning("WMC: second raw VST — replacing staged VST")
    
                self._induced_vst = VST(
                    # ── identity ──────────────────────────────────────────────────
                    sensor_id       = sss.location,         # SSS.location → VST.sensor_id
                    timestamp       = sss.generated_at,     # SSS.generated_at → VST.timestamp
                    interval        = sss.interval,         # SSS.interval → VST.interval
                    proc            = sss.proc,             # SSS.proc → VST.proc
                    # ── lifecycle ─────────────────────────────────────────────────
                    state           = WMCState.INDUCED,
                    # ── content ───────────────────────────────────────────────────
                    raw_frame       = sss.raw,              # SSS.raw → VST.raw_frame — path/ref only
                    interpretation  = "",                   # empty — interpreted turn completes this
                    content         = "",                   # empty — derived on pairing completion
                    trace           = "",                   # empty — derived on pairing completion
                    # ── spatial ───────────────────────────────────────────────────
                    objects         = [],                   # empty — interpreted turn fills this
                    spatial_map     = "",                   # empty — interpreted turn fills this
                    pose            = [],                   # empty — filled from ROS2 tf on interpreted turn
                    # ── scoring ───────────────────────────────────────────────────
                    vector          = sss.vector,           # SSS.vector → VST.vector
                    salience_score  = sss.urgency,          # SSS.urgency → VST.salience_score
                    chunk_count     = 0,                    # filled on pairing completion
                    # ── flags ─────────────────────────────────────────────────────
                    anchored        = False,
                )
                self.logger.debug("WMC: raw VST staged — pending interpreted turn")
                return None
    
            case _:
                raise ValueError(f"Unknown trace type in _populate: {sss.trace_type}")
    
    
    def _pair(self, sss: SSS) -> PMT | VST | None:
        """
        Second turn — complete the staged trace with the response or interpretation
        and return the fully formed memory object for the gating pipeline.
    
        PMT : assistant SSS      → complete _induced_pmt → return completed PMT
        VST : interpreted SSS    → complete _induced_vst → return completed VST
    
        Orphan handling:
            PMT — assistant turn without staged PMT wraps with placeholder, preserved.
            VST — interpreted turn without staged VST is discarded, nothing to recover.
    
        Args:
            sss : SSS — second turn sensory stimulus, role 'assistant' or 'interpreted'
    
        Returns:
            PMT | VST | None — completed trace, or None if VST orphan discarded
        """
        match sss.trace_type:
            case TraceType.PMT:
                if self._induced_pmt is None:                                           # orphaned assistant turn — wrap with placeholder
                    self.logger.warning("WMC: assistant SSS without staged PMT — wrapping with placeholder")
                    self._induced_pmt = PMT(
                        # ── identity ──────────────────────────────────────────────
                        user_id         = sss.user_id,
                        timestamp       = sss.generated_at,
                        interval        = sss.interval,
                        proc            = sss.proc,
                        # ── lifecycle ─────────────────────────────────────────────
                        state           = WMCState.INDUCED,
                        # ── content ───────────────────────────────────────────────
                        user_prompt     = "[context missing]",                          # placeholder — user turn was lost
                        ai_response     = sss.text,
                        content         = json.dumps({                                  # derive immediately — pair already complete
                            "user"      : "[context missing]",
                            "assistant" : sss.text,
                        }),
                        trace           = f'[context missing]\nYou replied: "{sss.text}"',
                        # ── scoring ───────────────────────────────────────────────
                        vector          = sss.vector,
                        salience_score  = sss.urgency,
                        chunk_count     = 0,                                            # filled below
                        # ── flags ─────────────────────────────────────────────────
                        anchored        = False,
                    )
                else:                                                                   # normal path — complete staged PMT
                    self._induced_pmt.ai_response   = sss.text                          # SSS.text → PMT.ai_response
                    self._induced_pmt.content       = json.dumps({                      # derive JSON — pairing complete
                        "user"      : self._induced_pmt.user_prompt,
                        "assistant" : sss.text,
                    })
                    self._induced_pmt.trace        += f'\nYou replied: "{sss.text}"'    # complete trace — append assistant turn
    
                self._induced_pmt.chunk_count = self.chunk_sampler.probe(               # cache chunk count — avoid reprobe on eviction
                    content  = self._induced_pmt.content,
                    overhead = WMC.PMT_OVERHEAD,
                )
                completed           = self._induced_pmt                                 # promote staged PMT
                self._induced_pmt   = None                                              # clear staging slot — ready for next exchange
                return completed                                                        # fully formed PMT — ready for gating pipeline
    
            case TraceType.VST:
                if self._induced_vst is None:                                           # orphaned interpretation — nothing to recover
                    self.logger.warning("WMC: interpreted SSS without staged VST — discarding")
                    return None
    
                self._induced_vst.interpretation    = sss.text                          # SSS.text → VST.interpretation
                self._induced_vst.objects           = sss.objects                       # SSS.objects → VST.objects
                self._induced_vst.spatial_map       = sss.spatial_map                   # SSS.spatial_map → VST.spatial_map
                self._induced_vst.pose              = sss.pose                          # SSS.pose → VST.pose
                self._induced_vst.content           = json.dumps({                      # derive JSON — pairing complete
                    "raw_frame"     : self._induced_vst.raw_frame,
                    "interpretation": sss.text,
                })
                self._induced_vst.trace             = (                                 # derive trace — for EMC embedding
                    f'Sensor {self._induced_vst.sensor_id} captured: "{sss.text}"'
                )
                self._induced_vst.chunk_count = self.chunk_sampler.probe(               # cache chunk count — avoid reprobe on eviction
                    content  = self._induced_vst.content,
                    overhead = WMC.VST_OVERHEAD,
                )
                completed           = self._induced_vst                                 # promote staged VST
                self._induced_vst   = None                                              # clear staging slot — ready for next exchange
                return completed                                                        # fully formed VST — ready for gating pipeline
    
            case _:
                raise ValueError(f"Unknown trace type in _pair: {sss.trace_type}")
        
    def _cosine_sim(self, a: list[float], b: list[float]) -> float:
        """Cosine similarity between two vectors."""
        dot = sum(x * y for x, y in zip(a, b))
        mag_a = sum(x * x for x in a) ** 0.5
        mag_b = sum(x * x for x in b) ** 0.5
        if mag_a == 0.0 or mag_b == 0.0:
            return 0.0                                    # degenerate vector — no similarity
        return dot / (mag_a * mag_b)
    
    def _static_anchor_gate(self, pmt: PMT,
                             static_anchors: list[list[float]] | None) -> tuple[float, float]:
        """
        Factor #2 Novelty + Factor #3 Depth from static anchors.
        
        Depth   — cosine sim to nearest static anchor centroid — base score
                  High depth = PMT connects to established episodic themes
        Novelty — 1.0 - depth — polar opposite of depth
                  High novelty = PMT is far from any known theme
        
        Both scores returned for composite weighting in _score_pmt_at_induction.
        Thresholds tuned during testing — not applied here.
    
        Args:
            pmt             : PMT being scored at induction
            static_anchors  : Consolidated episodic centroids from bootup — None if no history
    
        Returns:
            tuple[float, float] : (depth_score, novelty_score) — both in [0.0, 1.0]
        
        Biological analogue:
            Depth   — levels of processing (Craik & Lockhart) — deeper encoding for
                      semantically rich material connecting to established knowledge
            Novelty — dopamine novelty signal — heightened encoding for unexpected stimuli
        """
        if static_anchors is None or not pmt.vector:
            return 0.0, 1.0                                         # no anchor or no vector — depth undefined, novelty maximum
    
        depth = max(
            self._cosine_sim(pmt.vector, anchor)
            for anchor in static_anchors                            # nearest cluster wins — any topic match counts
        )
        novelty = 1.0 - depth                                       # polar opposite — one axis, two poles
    
        return depth, novelty

    def _semantic_clustering(self, pmt: PMT) -> bool:
        """
        Semantic grouping: group related PMTs together.
        """
        pass

    def _dynamic_workspace_gate(self, sss: SSS) -> bool:  
        """
        Late gate to filter out irrelevant PMTs.
        """
        return sss.urgency > self.urgency_threshold

    def induce(self, sss: SSS, 
           static_anchors: list[list[float]] | None,
           dynamic_anchor: list[float] | None) -> PMT | None:
        """
        Induction: prepare a user/assistant interaction to be added to the working memory.

        Biological Analogue: Prefrontal Cortex (PFC) working memory -
        
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
        self._semantic_encode(sss)
        self._static_anchors_gate(sss, static_anchors)
        self._semantic_cluster(sss)
        self._dynamic_workspace_gate(sss, dynamic_anchors)
        self.sss.state = WMCState.READY
        
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
