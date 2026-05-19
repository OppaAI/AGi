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
    wmc.induce(sss, static_anchors)    -> PMT | None
    wmc.fill_pmt(induced_pmt)          -> list[PMT]
    wmc.recall_pmt_schema()            -> list[PMT]
    wmc.reinstate_pmt_schema()         -> list[dict]
    wmc.forget_pmt_schema()            -> list[PMT]
    wmc.assess_pmt_schema()            -> dict
    wmc.is_empty                       -> bool
"""

# Standard library
import math                            # for exponential recency decay — half-life formula
from collections import deque          # for PMT slot — O(1) append and popleft on eviction
from datetime import datetime, timezone  # for PMT timestamps and UTC-aware age calculations
import json                            # for structured PMT storage — serialization and recall

# AGi components
from hrs.hru import ChunkSampler       # probes and truncates cognitive context for budget management
from hrs.hrm import AGi                # homeostatic regulation manifest namespace — system-wide constants
from gms.csb import SSS, PMT, VST, TraceType, WMCState  # type: ignore[import-untyped] PMT and WMCState classes — phonological and visuospatial memory traces

SCS = AGi.SCS                          # SCS parameter namespace alias — keeps constant references concise
WMC = SCS.WMC                          # WMC parameter namespace alias — keeps WMC constant references concise


class WorkingMemoryCortex:
    """
    Working Memory Cortex — the active conversation window of the SCS.

    Maintains the sustained PMT slot sent to the cognitive engine on every turn.
    Receding PMT schema are evicted when the global chunk limit or PMT slot limit is exceeded,
    and returned to MCC for async forwarding to EMC.

    Thread-safety: single-threaded by design — protected by CNC._busy flag.
    Only one PMT is processed at a time, ensuring WMC is always accessed from the main neural thread.

    New fields required on PMT dataclass (add to AGi/SCS/WMC/__init__.py):
        cluster_id      : int   = -1   # assigned cluster slot — -1 = unassigned
        retention_score : float = 0.0  # salience * recency_decay * depth — drives eviction priority
    """

    # ══════════════════════════════════════════════════════════════════════════════
    #  STUB CONSTANTS  (replace with WMC.* references once hrm.py is shared)
    # ══════════════════════════════════════════════════════════════════════════════

    _CLUSTER_LIMIT             = 4      # TODO: replace with WMC.CLUSTER_LIMIT
    _NOVELTY_CLUSTER_THRESHOLD = 0.35   # TODO: replace with WMC.NOVELTY_CLUSTER_THRESHOLD
    _EVENT_BOUNDARY_THRESHOLD  = 0.30   # TODO: replace with WMC.EVENT_BOUNDARY_THRESHOLD
    _RECENCY_HALF_LIFE_SECONDS = 300.0  # TODO: replace with WMC.RECENCY_HALF_LIFE_SECONDS

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
        self.logger                  = logger                # logger forwarded from MCC — all WMC methods emit through this handle
        self.chunk_sampler           = chunk_sampler         # token estimator forwarded from HRM for more accurate token count
        self.global_chunk_limit: int = global_chunk_limit    # maximum chunks WMC can sustain before eviction
        self.pmt_slot_limit: int     = pmt_slot_limit        # maximum PMTs WMC can hold before eviction
        self.pmt_slot_buffer: int    = pmt_slot_buffer       # additional PMT buffer beyond Miller's Law limit
        self._active_PMT: PMT | None = None                  # induced user prompt pending pairing with AI response
        self._active_vst: VST | None = None                  # induced raw frame pending pairing with interpreted turn
        self._pmt_slot: deque[PMT]   = deque()               # sustained PMT slot — single-threaded access guaranteed by CNC._busy flag
        self._sustained_chunks: int  = 0                     # running count of sustained chunks across all PMTs

        # Clustering and dynamic anchor state — initialized empty at boot
        self._dynamic_anchor: list[float] | None           = None   # running mean vector of all sustained PMTs — updated on fill and eviction
        self._cluster_registry: dict[int, list[list[float]]] = {}   # maps cluster_id → member PMT vectors — used to recompute centroids
        self._next_cluster_id: int                         = 0      # monotonic counter — never reused, avoids id collision after collapse
        self._cluster_ages: dict[int, datetime]            = {}     # maps cluster_id → timestamp of first PMT — oldest cluster evicted on overflow

        self.logger.info(                                    # log entry on WMC initialization with configured capacity
            f"   [Working Memory Cortex]  ONLINE ✅ — "
            f"{self.pmt_slot_limit}±{self.pmt_slot_buffer} PMT slots | {self.global_chunk_limit} chunks allocated"
        )

    # ══════════════════════════════════════════════════════════════════════════════
    #  STEP 1 — SEMANTIC ENCODING
    #  SSS → PMT | VST — construct, stage, and pair memory traces
    # ══════════════════════════════════════════════════════════════════════════════

    def _semantic_encode(self, sss: SSS) -> PMT | None:
        """
        Transform SSS into a staged or completed PMT.
        Routes to _pair_trace which drives all staging and promotion decisions.
        VST encoding deferred — implemented when CV/sensor pipeline is wired.
    
        Args:
            sss : SSS — incoming sensory stimulus
    
        Returns:
            PMT | None — completed PMT on assistant turn, None on user turn
        """
        if sss.trace_type != TraceType.PMT:
            self.logger.warning(f"WMC: non-PMT trace type received — skipping ({sss.trace_type})")
            return None                                             # VST deferred — skip silently
        return self._pair_trace(sss)
    
    def _construct_memory_trace(self, stimulus: SSS, content: str = "", trace: str = "") -> PMT | VST:
        """
        Working Memory Cycle -> Phase 1 - Induction -> 1.1 - Semantic Encoding -> 1.1.2 - Construct Memory Trace 
        
        Construct a memory trace from an incoming sensory stimulus passed from MCC,
        capturing relevant information across two sequential stages.
            
        Construction goes through 2 stages:
        The stage is determined from the presence/absence of the active memory trace.
        Stage 1: Staged construction — active memory trace is being initialized and constructed;
                                       captures all the necessary information from the received stimulus.
        Stage 2: Committed construction — active memory trace is already constructed; 
                                          captures additional memory content from AI response;
                                          passed to next for verification of completeness and correctness.
   
        Life cycle: 
        Stage 1: Stimulus (from MCC) -> (activates) -> Phonological Memory Trace (PMT) (for text/transcibed text)
                                                    -> Visuospatial Memory Trace (VST) (for vision/spatial related)
        Stage 2: Active PMT/VST (incomplete) -> (completes) -> Primed PMT/VST
        
        Incoming substrate(s):
            stimulus    : SSS — Incoming sensory stimulus signal passed from SIU -> CNC -> MCC
            content     : str — Stimulus information to be filled into memory trace
                                (empty in Phase 1, used in Phase 2 only; in JSON format)
            trace       : str — Stimulus information to be filled into memory trace
                                (empty in Phase 1, used in Phase 2 only; in plain text specialized format)
    
        Outgoing substrate(s):
            PMT / VST         — Staged (Phase 1) / Finalized (Phase 2) version of memory trace with stimulus information

        TODO: VST is planned for vision/spatial sensors in future milestones
        """
        if self._active_pmt is None:                                               # no active PMT present — 
            return PMT(                                                            # return the constructed 
                # ── identity ──────────────────────────────────────────────────
                user_id         = sss.user_id,                                     # identified of the user AuRoRA is interacting with
                # ── lifecycle ─────────────────────────────────────────────────
                state           = WMCState.INDUCED,                                # state: which phrase PMT is in (ie. induced to working memory)
                status          = "Staged"                                         # status of the PMT 
                induced_at      = sss.generated_at,                                # initial time of PMT induced
                filled_at       =                                                  # time when PMT filled into PMT slot
                sustained_interval =                                               # interval how long memory trace is sustained in PMT slot
                evicted_at      =                                                  # time of PMT being evicted from PMT slot
                destroyed_at    =                                                  # time of PMT destruction
                lifetime        = sss.interval,                                    # interval of the PMT construction to 
                # ── content ───────────────────────────────────────────────────
                content         = content,                                         # empty on user turn — derived on pairing
                trace           = trace,                                           # partial on user turn — completed on pairing
                # ── scoring ───────────────────────────────────────────────────
                vector          = sss.vector,                                      # 
                salience_score  = sss.urgency,                                     #
                chunk_count     = 0,                                               # filled on pairing — full pair needed
                # ── flags ─────────────────────────────────────────────────────
                anchored        = False,
            )
        else:                                                                      # staged PMT exists — update fill
            self._active_pmt.content     = content                                 # derived by _pair_trace
            self._active_pmt.trace       = trace                                   # derived by _pair_trace
            self._active_pmt.chunk_count = self.chunk_sampler.probe(               # cache chunk count — avoid reprobe on eviction
                content  = content,
                overhead = WMC.PMT_OVERHEAD,
            )
            return self._active_pmt                                                # updated in place — ready for promotion
    
    
    def _pair_trace(self, sss: SSS) -> PMT | None:
        """
        All staging and promotion decisions for PMT encoding.
    
        Handles:
            Double user   — append to staged PMT, return None
            Normal user   — full fill, stage, return None
            Orphan asst   — full fill with placeholder, then update fill, promote
            Normal asst   — update fill on staged PMT, promote, return PMT
    
        Derives content and trace — passes to _populate_trace as args.
        Never touches PMT fields directly.
    
        Args:
            sss : SSS — incoming sensory stimulus
    
        Returns:
            PMT | None — promoted PMT on assistant turn, None on user turn
        """
        if sss.role == "user":
            if self._active_pmt is not None:                                       # double user — append to staged PMT
                self._active_pmt.trace        += f'\n{sss.user_id} said: "{sss.text}"'  # consistent format — mirror initial trace
                self._active_pmt.vector        = sss.vector                        # latest user turn wins
                self._active_pmt.salience_score = max(                             # keep highest urgency across appended turns
                    self._active_pmt.salience_score, sss.urgency
                )
                self.logger.warning("WMC: second user SSS — appended to staged PMT")
                return None                                                         # still incomplete — pair not ready
    
            self._active_pmt = self._populate_trace(                               # full fill — new PMT shell
                sss,
                trace = f'{sss.user_id} said: "{sss.text}"',                        # partial trace — assistant turn appends
            )
            self.logger.debug("WMC: user SSS staged — pending assistant turn")
            return None                                                             # always None on user turn
    
        elif sss.role == "assistant":
            if self._active_pmt is None:                                           # orphan — no staged user turn
                self.logger.warning("WMC: assistant SSS without staged PMT — recovering")
                self._active_pmt = self._populate_trace(                           # full fill with placeholder identity
                    sss,
                    trace = "[context missing]",                                    # placeholder — user turn was lost
                )
    
            # derive content and trace — same path for normal and orphan
            staged_trace = self._active_pmt.trace                                  # partial trace from user turn or placeholder
            content      = json.dumps({                                             # derive JSON pair — pairing complete
                "user"      : staged_trace,                                         # user turn trace as source of truth
                "assistant" : sss.text,
            })
            trace        = f'{staged_trace}\nYou replied: "{sss.text}"'             # complete trace — append assistant turn
            completed    = self._populate_trace(                                    # update fill — changed fields only
                sss,
                content = content,
                trace   = trace,
            )
            self._active_pmt = None                                                # clear staging slot — ready for next exchange
            self.logger.debug("WMC: assistant SSS paired — PMT promoted")
            return completed                                                        # fully formed PMT — ready for gating pipeline

    # ══════════════════════════════════════════════════════════════════════════════
    #  PRIVATE HELPERS
    # ══════════════════════════════════════════════════════════════════════════════

    def _cosine_sim(self, a: list[float], b: list[float]) -> float:
        """
        Cosine similarity between two equal-length vectors.
        Returns 0.0 on zero-norm input — degenerate vectors never match.

        Args:
            a : list[float] — first vector
            b : list[float] — second vector

        Returns:
            float — similarity in [-1.0, 1.0], clamped to 0.0 on degenerate input
        """
        dot  = sum(x * y for x, y in zip(a, b))                         # dot product of the two vectors
        norm = math.sqrt(sum(x * x for x in a)) * math.sqrt(sum(y * y for y in b))
        return dot / norm if norm > 0.0 else 0.0                         # guard against zero-norm — undefined similarity

    def _cluster_mean(self, cluster_id: int) -> list[float] | None:
        """
        Compute the mean vector of all PMTs in a cluster.
        Returns None if cluster is empty or all vectors are degenerate.

        Args:
            cluster_id : int — target cluster

        Returns:
            list[float] | None — centroid of the cluster, or None if uncomputable
        """
        vectors = [v for v in self._cluster_registry.get(cluster_id, []) if v]   # drop empty vectors
        if not vectors:
            return None
        dim  = len(vectors[0])
        mean = [0.0] * dim
        for v in vectors:
            for i, x in enumerate(v):
                mean[i] += x
        n = len(vectors)
        return [x / n for x in mean]                                              # element-wise mean — cluster centroid

    def _recompute_dynamic_anchor(self) -> None:
        """
        Recompute the running mean vector of all sustained PMTs in the slot.
        Called after every slot mutation — fill_pmt append and _evict_pmt remove.
        Not called during induction — the slot is unchanged until fill.
     
        Biological analogue: the dynamic anchor is the aggregate semantic context
        of working memory — the attentional focus of all currently sustained traces.
        """
        vectors = [p.vector for p in self._pmt_slot if p.vector]   # collect non-empty vectors only
        if not vectors:
            self._dynamic_anchor = None                             # empty slot — no anchor
            return
        dim  = len(vectors[0])
        mean = [0.0] * dim
        for v in vectors:
            for i, x in enumerate(v):
                mean[i] += x
        n = len(vectors)
        self._dynamic_anchor = [x / n for x in mean]               # replaces previous anchor

    def _recency_decay(self, pmt_timestamp: datetime) -> float:
        """
        Exponential recency decay based on PMT age.
        Returns 1.0 for a brand-new PMT, approaches 0.0 as age grows.

        Biological analogue: Ebbinghaus forgetting curve —
        memory traces decay exponentially without rehearsal.

        Args:
            pmt_timestamp : datetime — when the PMT was created

        Returns:
            float — decay factor in (0.0, 1.0]
        """
        now = datetime.now(timezone.utc)
        if pmt_timestamp.tzinfo is None:                             # defensive — make tz-aware if bare
            pmt_timestamp = pmt_timestamp.replace(tzinfo=timezone.utc)
        age_sec   = (now - pmt_timestamp).total_seconds()
        half_life = self._RECENCY_HALF_LIFE_SECONDS                  # TODO: replace with WMC.RECENCY_HALF_LIFE_SECONDS
        return math.exp(-0.693 * age_sec / half_life)                # ln(2) ≈ 0.693 — standard half-life decay

    def _score_retention(self, pmt: PMT, depth: float) -> float:
        """
        Compute and cache the retention score on the PMT.
        Drives eviction priority — lowest score evicted first.

        Formula: salience * recency_decay * depth
            salience      — urgency signal from SSS — how important was this exchange
            recency_decay — exponential age decay — older PMTs fade without rehearsal
            depth         — cosine sim to nearest static anchor — grounded in known themes

        Multiplicative: all three factors must be non-negligible for a PMT to stay.
        A zero on any axis collapses the score — correct for WM eviction.

        Args:
            pmt   : PMT — target trace (salience_score and timestamp must be set)
            depth : float — from _static_anchor_gate, already computed at induction

        Returns:
            float — retention score in [0.0, 1.0], cached on pmt.retention_score
        """
        decay               = self._recency_decay(pmt.timestamp)
        score               = pmt.salience_score * decay * depth     # multiplicative — all three must hold
        pmt.retention_score = round(score, 6)                        # cache on PMT — no reprobe on eviction
        return pmt.retention_score
        
    def _static_anchor_gate(self, pmt: PMT,
                             static_anchors: list[list[float]] | None) -> tuple[float, float]:
        """
        Compute depth and novelty scores from static episodic anchors.
    
        Depth   — cosine sim to nearest static anchor centroid
                  High depth = PMT connects to established episodic themes
        Novelty — 1.0 - depth — polar opposite of depth
                  High novelty = PMT is far from any known theme
    
        Both scores feed into _score_retention via the depth parameter.
        Novelty available for future PMT.novelty_score field if added.
    
        Biological analogue:
            Depth   — levels of processing (Craik & Lockhart) — deeper encoding
                      for semantically rich material connecting to known knowledge
            Novelty — dopamine novelty signal — heightened encoding for unexpected stimuli
    
        Args:
            pmt            : PMT — incoming trace being scored at induction
            static_anchors : list[list[float]] | None — episodic centroids from bootup
    
        Returns:
            tuple[float, float] — (depth_score, novelty_score), both in [0.0, 1.0]
        """
        if static_anchors is None or not pmt.vector:
            return 0.0, 1.0                                         # no anchor or no vector — depth undefined, novelty maximum
    
        depth = max(
            self._cosine_sim(pmt.vector, anchor)
            for anchor in static_anchors                            # nearest cluster wins — any topic match counts
        )
        novelty = 1.0 - depth                                       # polar opposite — one axis, two poles
    
        return depth, novelty
    # ══════════════════════════════════════════════════════════════════════════════
    #  GATE 1 — _dynamic_workspace_gate
    #  Admission decision: does this PMT belong in WM at all?
    # ══════════════════════════════════════════════════════════════════════════════
     
    def _dynamic_workspace_gate(self, pmt) -> bool:
        """
        Admission gate — decide whether the incoming PMT belongs in working memory.
     
        Compares the PMT's vector against self._dynamic_anchor (the running mean of
        all currently sustained PMTs). A cosine sim below EVENT_BOUNDARY_THRESHOLD
        signals that the topic has shifted too far from the current WM context —
        the PMT is rejected to prevent workspace fragmentation.
     
        Pass conditions (PMT admitted):
            - No dynamic anchor yet (slot empty — first PMT always admitted)
            - No vector on PMT (unmeasurable — admit by default)
            - Cosine sim >= EVENT_BOUNDARY_THRESHOLD (topic continuous)
     
        Fail condition (PMT rejected):
            - Cosine sim < EVENT_BOUNDARY_THRESHOLD (event boundary detected)
     
        Rejection means the PMT is dropped from the induction pipeline entirely.
        MCC receives None from induce() and decides whether to buffer or discard.
     
        Biological analogue: prefrontal gating of working memory (O'Reilly & Frank) —
        the PFC actively filters inputs to maintain coherent task representations,
        blocking topic discontinuities that would cause interference.
     
        Args:
            pmt : PMT — incoming completed trace with vector set
     
        Returns:
            bool — True if admitted, False if rejected at event boundary
        """
        if self._dynamic_anchor is None or not pmt.vector:
            return True                                             # empty slot or no vector — admit unconditionally
     
        sim      = self._cosine_sim(pmt.vector, self._dynamic_anchor)
        admitted = sim >= self._EVENT_BOUNDARY_THRESHOLD                 # TODO: replace with WMC.EVENT_BOUNDARY_THRESHOLD
     
        if not admitted:
            self.logger.debug(
                f"WMC gate: event boundary — PMT rejected "
                f"(sim={sim:.3f} < threshold={_EVENT_BOUNDARY_THRESHOLD})"
            )
        return admitted
     
     
    # ══════════════════════════════════════════════════════════════════════════════
    #  GATE 2 — _semantic_clustering
    #  Assignment: which cluster does this admitted PMT belong to?
    # ══════════════════════════════════════════════════════════════════════════════
     
    def _semantic_clustering(self, pmt) -> int:
        """
        Assign a cluster_id to the incoming PMT without touching the deque.
        Called only on PMTs that have already passed _dynamic_workspace_gate.
     
        Two paths determine assignment:
     
        Path A — Explicit marker (hard override)
            pmt.anchored=True → assign to reserved anchor cluster (id=0).
            Skips centroid comparison entirely.
            Biological analogue: hippocampal tagging — explicitly marked events
            bypass encoding thresholds and are bound with priority.
     
        Path B — Nearest centroid match
            Compare pmt.vector against each existing cluster's centroid.
            Join if best sim >= NOVELTY_CLUSTER_THRESHOLD (familiar topic).
            Open new cluster if best sim < threshold (novel topic).
            Cluster overflow (> CLUSTER_LIMIT) → collapse oldest into nearest neighbour.
     
        cluster_id is written to pmt.cluster_id.
        Actual deque insertion happens in fill_pmt — not here.
     
        Args:
            pmt : PMT — incoming trace, already admitted by _dynamic_workspace_gate
     
        Returns:
            int — assigned cluster_id (also written to pmt.cluster_id)
        """
     
        # ── Path A: explicit marker — hard override ────────────────────────────
        if pmt.anchored:
            anchor_cluster_id = 0                                           # reserved id — never auto-assigned
            if anchor_cluster_id not in self._cluster_registry:
                self._cluster_registry[anchor_cluster_id] = []
                self._cluster_ages[anchor_cluster_id]     = pmt.timestamp
            if pmt.vector:
                self._cluster_registry[anchor_cluster_id].append(pmt.vector)
            pmt.cluster_id = anchor_cluster_id
            self.logger.debug("WMC cluster: anchored → cluster 0 (hard override)")
            return anchor_cluster_id
     
        # ── Path B: nearest centroid match ────────────────────────────────────
        assigned_id: int | None = None
     
        if pmt.vector and self._cluster_registry:
            best_sim = -1.0
            best_id  = -1
            for cid in self._cluster_registry:
                centroid = self._cluster_mean(cid)
                if centroid is None:
                    continue
                sim = self._cosine_sim(pmt.vector, centroid)
                if sim > best_sim:
                    best_sim = sim
                    best_id  = cid
     
            if best_sim >= self._NOVELTY_CLUSTER_THRESHOLD and best_id >= 0:    # TODO: WMC.NOVELTY_CLUSTER_THRESHOLD
                assigned_id = best_id                                       # join existing cluster — topic familiar
                self.logger.debug(
                    f"WMC cluster: joined cluster {assigned_id} (sim={best_sim:.3f})"
                )
     
        # ── Open new cluster if no match ──────────────────────────────────────
        if assigned_id is None:
            assigned_id           = self._next_cluster_id
            self._next_cluster_id += 1                                      # monotonic — never reused
            self._cluster_registry[assigned_id] = []
            self._cluster_ages[assigned_id]     = pmt.timestamp
            self.logger.debug(f"WMC cluster: new cluster opened → id={assigned_id}")
     
        # ── Cluster overflow guard ─────────────────────────────────────────────
        non_anchor_clusters = [cid for cid in self._cluster_registry if cid != 0]
        if len(non_anchor_clusters) > self._CLUSTER_LIMIT:                       # TODO: WMC.CLUSTER_LIMIT
            self._collapse_oldest_cluster(exclude_id=assigned_id)
     
        # ── Register vector in assigned cluster ───────────────────────────────
        if pmt.vector:
            self._cluster_registry[assigned_id].append(pmt.vector)
     
        pmt.cluster_id = assigned_id
        return assigned_id

    def _collapse_oldest_cluster(self, exclude_id: int) -> None:
        """
        Collapse the oldest non-anchor cluster when CLUSTER_LIMIT is exceeded.
        Re-assigns its PMTs to their nearest remaining neighbour by centroid similarity.
        PMTs with no remaining neighbour keep their old id and evict naturally.

        Biological analogue: schema consolidation — when topic capacity is saturated,
        the least-recent context is absorbed into the nearest existing schema.

        Args:
            exclude_id : int — the newly opened cluster, protected from immediate collapse
        """
        candidates = {
            cid: ts for cid, ts in self._cluster_ages.items()
            if cid != 0 and cid != exclude_id                               # never collapse anchor or new cluster
        }
        if not candidates:
            return

        oldest_id     = min(candidates, key=lambda cid: candidates[cid])    # oldest by first-PMT timestamp
        remaining_ids = [cid for cid in self._cluster_registry if cid != oldest_id]

        for pmt in self._pmt_slot:                                           # re-point affected PMTs in the deque
            if pmt.cluster_id != oldest_id:
                continue
            best_sim = -1.0
            best_id  = oldest_id                                             # fallback — keeps old id if no neighbour
            for cid in remaining_ids:
                centroid = self._cluster_mean(cid)
                if centroid is None or not pmt.vector:
                    continue
                sim = self._cosine_sim(pmt.vector, centroid)
                if sim > best_sim:
                    best_sim = sim
                    best_id  = cid
            pmt.cluster_id = best_id

        del self._cluster_registry[oldest_id]
        del self._cluster_ages[oldest_id]
        self.logger.debug(f"WMC cluster: collapsed cluster {oldest_id} → redistributed to neighbours")

    # ══════════════════════════════════════════════════════════════════════════════
    #  EVICTION — retention-score-ordered, anchor-protected
    # ══════════════════════════════════════════════════════════════════════════════

    def _evict_pmt(self, induced_pmt_chunks: int) -> list[PMT]:
        """
        Evict PMTs until the incoming PMT fits within both capacity limits.
        Eviction order: lowest retention_score first (salience * recency * depth).

        Cluster-awareness: eviction naturally drains low-score clusters first —
        retention score already encodes relevance, no explicit cluster selection needed.

        Anchored PMTs (cluster_id=0) are eviction-protected — skipped unless slot is anchor-only.
        Calls _recompute_dynamic_anchor after each remove — slot has mutated.

        Args:
            induced_pmt_chunks : int — cached chunk count of the incoming PMT

        Returns:
            list[PMT] : Evicted PMTs — returned to MCC for episodic binding
        """
        evicted: list[PMT] = []

        while self._pmt_slot and (
            self._sustained_chunks + induced_pmt_chunks > self.global_chunk_limit   # global chunk limit would be exceeded, or
            or len(self._pmt_slot) >= self.pmt_slot_limit + self.pmt_slot_buffer    # PMT slot limit reached
        ):
            candidates = [p for p in self._pmt_slot if p.cluster_id != 0]          # prefer non-anchored PMTs
            if not candidates:
                candidates = list(self._pmt_slot)                                   # all anchored — must evict anyway

            target = min(candidates, key=lambda p: p.retention_score)              # lowest retention goes first

            self._pmt_slot.remove(target)                                           # O(n) — slot ≤9 PMTs, acceptable
            evicted.append(target)
            self._sustained_chunks = max(0, self._sustained_chunks - target.chunk_count)  # decrement chunk count — floor at 0

            # clean vector from cluster registry — keeps centroids accurate after eviction
            if target.vector and target.cluster_id in self._cluster_registry:
                vecs = self._cluster_registry[target.cluster_id]
                try:
                    vecs.remove(target.vector)
                except ValueError:
                    pass                                                             # already gone — harmless
                if not vecs:                                                        # cluster emptied by this eviction
                    del self._cluster_registry[target.cluster_id]
                    self._cluster_ages.pop(target.cluster_id, None)

            self._recompute_dynamic_anchor()                                        # slot mutated — anchor must reflect new state

            self.logger.debug(
                f"WMC evict → EMC: cluster={target.cluster_id} "
                f"retention={target.retention_score:.4f} chunks={target.chunk_count}"
            )

        return evicted

    # ══════════════════════════════════════════════════════════════════════════════
    #  PUBLIC INTERFACE
    # ══════════════════════════════════════════════════════════════════════════════

    # ══════════════════════════════════════════════════════════════════════════════
    #  induce — full induction pipeline
    # ══════════════════════════════════════════════════════════════════════════════
    def induce(self, sss, static_anchors: list | None, dynamic_anchor: list[float] | None = None):
        """
        Full induction pipeline for one SSS turn.
    
        Pipeline:
            1. _semantic_encode        — stage user turn or complete assistant turn
            2. _dynamic_workspace_gate — event boundary check vs self._dynamic_anchor
            3. _semantic_clustering    — assign cluster_id, no deque touch
            4. _static_anchor_gate     — depth + novelty from static episodic centroids
            5. _score_retention        — salience * recency_decay * depth → cached on PMT
    
        Returns None on user turn (staged, not yet complete).
        Returns None if PMT rejected at dynamic gate (event boundary detected).
        Returns completed, scored, clustered PMT — caller calls fill_pmt next.
    
        dynamic_anchor is read from self._dynamic_anchor — not passed as parameter.
        self._dynamic_anchor reflects the slot state before this induction,
        which is the correct comparison point (PMT not yet in the slot).
    
        Args:
            sss            : SSS — incoming sensory stimulus
            static_anchors : list[list[float]] | None — episodic centroids from bootup
    
        Returns:
            PMT | VST | None — completed trace ready for fill_pmt, or None
        """
        # Stage 1 — encode: stage on user turn, complete on assistant turn
        completed = self._semantic_encode(sss)
        if completed is None:
            return None                                         # user turn — staged, nothing to gate or score yet
    
        # Stage 2 — dynamic workspace gate: cheapest rejection — kill before any centroid math
        if not self._dynamic_workspace_gate(completed):
            self.logger.debug("WMC induce: PMT rejected — event boundary detected")
            return None                                         # topic too distant — MCC decides what to do
    
        # Stage 3 — cluster assignment: tag with cluster_id before scoring — no deque insert
        self._semantic_clustering(completed)
    
        # Stage 4 — static anchor gate: depth + novelty from episode history
        depth, novelty = self._static_anchor_gate(completed, static_anchors)
        # TODO: if PMT gains novelty_score field → completed.novelty_score = novelty
    
        # Stage 5 — retention score: salience * recency_decay * depth
        self._score_retention(completed, depth)
    
        self.logger.debug(
            f"WMC induced: cluster={completed.cluster_id} "
            f"retention={completed.retention_score:.4f} "
            f"depth={depth:.3f} novelty={novelty:.3f}"
        )
        return completed                                        # scored + clustered — ready for fill_pmt
    
    def fill_pmt(self, induced_pmt: PMT) -> list[PMT]:
        """
        Fill a completed, scored, clustered PMT into the slot.
        Evict receding PMTs first until the incoming PMT fits within capacity.
        Recompute dynamic anchor after append — slot has grown.

        Called by MCC after induce() returns a non-None PMT.
        chunk_count must already be set on induced_pmt before calling.

        Args:
            induced_pmt : PMT — completed trace from induce()

        Returns:
            list[PMT] : Evicted PMTs — returned to MCC for episodic binding
        """
        evicted = self._evict_pmt(induced_pmt.chunk_count)                  # evict until incoming fits

        self._pmt_slot.append(induced_pmt)                                  # fill into slot
        self._sustained_chunks += induced_pmt.chunk_count                   # increment sustained chunk count
        self._recompute_dynamic_anchor()                                     # slot grew — update anchor

        self.logger.debug(
            f"WMC filled: cluster={induced_pmt.cluster_id} "
            f"sustained={len(self._pmt_slot)} PMTs | "
            f"chunks={self._sustained_chunks}/{self.global_chunk_limit} | "
            f"evicted={len(evicted)}"
        )
        return evicted

    def recall_pmt_schema(self) -> list[PMT]:
        """
        Recall sustaining PMT schema for context window construction.
        Return PMT schema in ascending chronological order.

        Returns:
            list[PMT]: Sustained PMT schema, in ascending chronological order
        """
        return list(self._pmt_slot)                            # return PMT schema in ascending chronological order

    def reinstate_pmt_schema(self) -> list[dict]:
        """
        Reinstate sustaining PMT schema into the context window for inference.
        Unpacks the PMT schema into interleaved pair of user prompt and AI response.
        Timestamps stripped — only role and content surfaced for inference.
        Return PMT schema in ascending chronological order.

        Returns:
            list[dict]: Sustained PMT schema unpacked as turn pairs [{role, content}], in ascending chronological order
        """
        sustained_pmts: list[dict] = []                                                             # accumulate unpacked turn pairs for inference

        for pmt in self.recall_pmt_schema():                                                        # traverse sustaining PMTs oldest-first
            # Each pmt["content"] is a JSON string — deserialize into user prompt/AI response pairs
            try:                                                                                    # attempt to deserialize the PMT content
                content = json.loads(pmt.content)                                                   # deserialize — each PMT encodes a user/assistant pair
                sustained_pmts.append({"role": "user",      "content": content["user"]})            # unpack user turn from the content
                sustained_pmts.append({"role": "assistant", "content": content["assistant"]})       # unpack assistant turn from the content
            except (json.JSONDecodeError, KeyError):                                                # malformed PMT — surface raw rather than drop
                sustained_pmts.append({"role": "user", "content": pmt.content})                    # use the PMT content as-is

        return sustained_pmts                                                                       # ascending chronological order

    def forget_pmt_schema(self) -> list[PMT]:
        """
        Forget all sustaining PMT schema — returns working memory to rest.
        Called at conversation end or on explicit reset.
        Forwarding the forgotten schema to EMC is the caller's responsibility.

        Returns:
           list[PMT] : Forgotten PMT schema — caller decides whether to forward to EMC
        """
        forgotten_pmt_schema: list[PMT] = list(self._pmt_slot)      # snapshot before wipe — safe under CNC._busy
        self._pmt_slot.clear()                                       # evict all sustaining PMTs
        self._active_pmt    = None                                  # discard any incomplete induced PMT
        self._induced_vst    = None                                  # discard any incomplete induced VST
        self._sustained_chunks = 0                                   # reset sustained chunk count
        self._dynamic_anchor = None                                  # clear anchor — slot is empty
        self.logger.info(                                            # log the forgetting of the PMT schema from working memory
            f"🧹 WMC forgotten ({len(forgotten_pmt_schema)} PMTs)"
        )
        return forgotten_pmt_schema                                  # return forgotten PMT schema to caller for optional saving or forwarding to EMC

    def assess_pmt_schema(self) -> dict:
        """
        Assess current occupancy of the sustaining PMT schema.
        Used for monitoring and GRACE cognitive state display.

        Returns:
            dict: Current memory usage stats including PMT count, sustained chunks, free chunks, global chunk limit, and load percentage
        """
        return {                                      # return the occupancy of PMT and chunk sustaining in the working memory
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
        return len(self._pmt_slot) == 0                # return True if working memory is empty, False otherwise
