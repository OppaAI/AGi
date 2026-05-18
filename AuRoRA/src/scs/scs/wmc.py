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

    """
    WMC — Clustering, Retention Scoring, and Induction Pipeline
    ============================================================
    AuRoRA · Semantic Cognitive System (SCS) — WorkingMemoryCortex additions
    
    New fields required on PMT dataclass (add to AGi/SCS/WMC/__init__.py):
        cluster_id      : int   = -1   # assigned cluster slot — -1 = unassigned
        retention_score : float = 0.0  # salience * recency_decay * depth — drives eviction priority
    
    New fields required on WorkingMemoryCortex.__init__ (after existing assignments):
        self._dynamic_anchor    : list[float] | None            = None
            # running mean vector of all sustained PMTs — updated on fill and eviction
        self._cluster_registry  : dict[int, list[list[float]]]  = {}
            # maps cluster_id → member PMT vectors — used to recompute centroids
        self._next_cluster_id   : int                           = 0
            # monotonic counter — never reused, avoids id collision after collapse
        self._cluster_ages      : dict[int, datetime]           = {}
            # maps cluster_id → timestamp of first PMT — oldest cluster evicted on overflow
    
    New HRS constants required (stubs used until hrm.py is shared):
        WMC.CLUSTER_LIMIT             — max topic clusters before oldest is collapsed
        WMC.NOVELTY_CLUSTER_THRESHOLD — cosine sim floor below which a new cluster is opened
        WMC.EVENT_BOUNDARY_THRESHOLD  — cosine sim drop vs dynamic anchor that signals topic shift
        WMC.RECENCY_HALF_LIFE_SECONDS — half-life for exponential recency decay
    
    Induction pipeline (induce):
        1. _semantic_encode        — stage user turn or complete assistant turn into PMT/VST
        2. _static_anchor_gate     — depth + novelty scores from static episodic centroids
        3. _dynamic_workspace_gate — admission gate: event boundary check vs _dynamic_anchor
        4. _score_retention        — salience * recency_decay * depth → cached on PMT
        5. _semantic_clustering    — assign cluster_id, no deque touch
        fill_pmt / _evict_pmt      — actual slot mutation + _dynamic_anchor recompute
    """
    
    import math
    from datetime import datetime, timezone
    
    
    # ══════════════════════════════════════════════════════════════════════════════
    #  STUB CONSTANTS  (replace with WMC.* references once hrm.py is shared)
    # ══════════════════════════════════════════════════════════════════════════════
    
    _CLUSTER_LIMIT             = 4      # TODO: replace with WMC.CLUSTER_LIMIT
    _NOVELTY_CLUSTER_THRESHOLD = 0.35   # TODO: replace with WMC.NOVELTY_CLUSTER_THRESHOLD
    _EVENT_BOUNDARY_THRESHOLD  = 0.30   # TODO: replace with WMC.EVENT_BOUNDARY_THRESHOLD
    _RECENCY_HALF_LIFE_SECONDS = 300.0  # TODO: replace with WMC.RECENCY_HALF_LIFE_SECONDS
    
    
    # ══════════════════════════════════════════════════════════════════════════════
    #  PRIVATE HELPERS
    # ══════════════════════════════════════════════════════════════════════════════
    
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
        if pmt_timestamp.tzinfo is None:                            # defensive — make tz-aware if bare
            pmt_timestamp = pmt_timestamp.replace(tzinfo=timezone.utc)
        age_sec   = (now - pmt_timestamp).total_seconds()
        half_life = _RECENCY_HALF_LIFE_SECONDS                      # TODO: replace with WMC.RECENCY_HALF_LIFE_SECONDS
        return math.exp(-0.693 * age_sec / half_life)               # ln(2) ≈ 0.693 — standard half-life decay
    
    
    def _score_retention(self, pmt, depth: float) -> float:
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
        score               = pmt.salience_score * decay * depth    # multiplicative — all three must hold
        pmt.retention_score = round(score, 6)                       # cache on PMT — no reprobe on eviction
        return pmt.retention_score
    
    
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
        admitted = sim >= _EVENT_BOUNDARY_THRESHOLD                 # TODO: replace with WMC.EVENT_BOUNDARY_THRESHOLD
    
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
    
            if best_sim >= _NOVELTY_CLUSTER_THRESHOLD and best_id >= 0:    # TODO: WMC.NOVELTY_CLUSTER_THRESHOLD
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
        if len(non_anchor_clusters) > _CLUSTER_LIMIT:                       # TODO: WMC.CLUSTER_LIMIT
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
            if cid != 0 and cid != exclude_id                              # never collapse anchor or new cluster
        }
        if not candidates:
            return
    
        oldest_id     = min(candidates, key=lambda cid: candidates[cid])  # oldest by first-PMT timestamp
        remaining_ids = [cid for cid in self._cluster_registry if cid != oldest_id]
    
        for pmt in self._pmt_slot:                                         # re-point affected PMTs in the deque
            if pmt.cluster_id != oldest_id:
                continue
            best_sim = -1.0
            best_id  = oldest_id                                           # fallback — keeps old id if no neighbour
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
    
    def _evict_pmt(self, induced_pmt_chunks: int) -> list:
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
        evicted: list = []
    
        while self._pmt_slot and (
            self._sustained_chunks + induced_pmt_chunks > self.global_chunk_limit
            or len(self._pmt_slot) >= self.pmt_slot_limit + self.pmt_slot_buffer
        ):
            candidates = [p for p in self._pmt_slot if p.cluster_id != 0]  # prefer non-anchored
            if not candidates:
                candidates = list(self._pmt_slot)                           # all anchored — must evict anyway
    
            target = min(candidates, key=lambda p: p.retention_score)      # lowest retention goes first
    
            self._pmt_slot.remove(target)                                   # O(n) — slot ≤9 PMTs, acceptable
            evicted.append(target)
            self._sustained_chunks = max(0, self._sustained_chunks - target.chunk_count)
    
            # clean vector from cluster registry — keeps centroids accurate after eviction
            if target.vector and target.cluster_id in self._cluster_registry:
                vecs = self._cluster_registry[target.cluster_id]
                try:
                    vecs.remove(target.vector)
                except ValueError:
                    pass                                                    # already gone — harmless
                if not vecs:                                                # cluster emptied by this eviction
                    del self._cluster_registry[target.cluster_id]
                    self._cluster_ages.pop(target.cluster_id, None)
    
            self._recompute_dynamic_anchor()                                # slot mutated — anchor must reflect new state
    
            self.logger.debug(
                f"WMC evict → EMC: cluster={target.cluster_id} "
                f"retention={target.retention_score:.4f} chunks={target.chunk_count}"
            )
    
        return evicted
    
    
    # ══════════════════════════════════════════════════════════════════════════════
    #  fill_pmt — slot mutation entry point
    #  (replaces the original fill_pmt in wmc.py)
    # ══════════════════════════════════════════════════════════════════════════════
    
    def fill_pmt(self, induced_pmt) -> list:
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
        evicted = self._evict_pmt(induced_pmt.chunk_count)                 # evict until incoming fits
    
        self._pmt_slot.append(induced_pmt)                                 # fill into slot
        self._sustained_chunks += induced_pmt.chunk_count                  # increment sustained chunk count
        self._recompute_dynamic_anchor()                                    # slot grew — update anchor
    
        self.logger.debug(
            f"WMC filled: cluster={induced_pmt.cluster_id} "
            f"sustained={len(self._pmt_slot)} PMTs | "
            f"chunks={self._sustained_chunks}/{self.global_chunk_limit} | "
            f"evicted={len(evicted)}"
        )
        return evicted
    
    
    # ══════════════════════════════════════════════════════════════════════════════
    #  induce — full induction pipeline
    # ══════════════════════════════════════════════════════════════════════════════
    
    def induce(self, sss, static_anchors: list | None):
        """
        Full induction pipeline for one SSS turn.
    
        Pipeline:
            1. _semantic_encode        — stage user turn or complete assistant turn
            2. _static_anchor_gate     — depth + novelty from static episodic centroids
            3. _dynamic_workspace_gate — event boundary check vs self._dynamic_anchor
            4. _score_retention        — salience * recency_decay * depth → cached on PMT
            5. _semantic_clustering    — assign cluster_id, no deque touch
    
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
    
        # Stage 2 — static anchor gate: depth + novelty from episode history
        depth, novelty = self._static_anchor_gate(completed, static_anchors)
        # TODO: if PMT gains novelty_score field → completed.novelty_score = novelty
    
        # Stage 3 — dynamic workspace gate: event boundary admission check
        if not self._dynamic_workspace_gate(completed):
            self.logger.debug("WMC induce: PMT rejected — event boundary detected")
            return None                                         # topic too distant — MCC decides what to do
    
        # Stage 4 — retention score: salience * recency_decay * depth
        self._score_retention(completed, depth)
    
        # Stage 5 — cluster assignment: tag with cluster_id, no deque insert
        self._semantic_clustering(completed)
    
        self.logger.debug(
            f"WMC induced: cluster={completed.cluster_id} "
            f"retention={completed.retention_score:.4f} "
            f"depth={depth:.3f} novelty={novelty:.3f}"
        )
        return completed                                        # scored + clustered — ready for fill_pmt

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
