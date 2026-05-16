"""
MCC — Memory Coordination Cortex
================================
AuRoRA · Semantic Cognitive System (SCS)

Single memory interface for CNC — coordinates WMC and EMC.
CNC never touches WMC or EMC directly; all memory operations route through MCC.

Responsibilities:
    - Receive induced PMTs from CNC and fill them into WMC
    - Bind evicted PMTs to episodic buffer for EMC encoding (async, non-blocking)
    - Assemble unified memory context from sustained WMC PMTs and recalled EMC episodes
    - Manage cortical capacity across the full memory core

Architecture:
    MCC mirrors the human prefrontal cortex — the brain's active workspace
    where working memory and episodic recall converge into a single conscious field.

    WMC and EMC operate independently but are unified in assemble_memory_context()
    into one memory context passed to the cognitive engine — exactly as fresh thoughts
    and recalled memories surface together into the same prefrontal awareness.
    The cognitive engine cannot distinguish a sustained WMC PMT from a recalled EMC
    engram; to it, they are all active cognition.

    Cortical capacity budget:
        Identity and cognition          →  SCS.COGNITIVE_RESERVE
        EMC recalled engrams            →  SCS.EMC.RECALL_RESERVE
        WMC sustained PMTs              →  SCS.GLOBAL_CHUNK_LIMIT
        ─────────────────────────────────────────────────────────────────────
        Total active cognitive core     →  SCS.CORTICAL_CAPACITY

Terminology:
    Buffer      — temporary staging area for memory traces in transition
                  (evicted PMTs from WMC awaiting EMC encoding, or recalled EMC 
                  episodes awaiting reinstatment into the memory context)
    Context     — the full active memory presented to the cognitive engine
                  (WMC PMTs + relevant EMC episodes, assembled each turn)
    Engram      — one physical memory record stored in the engram complex
    PMT         — phonological memory trace; one complete interaction
                  (user prompt + AI response). WMC pairs turns internally —
                  MCC forwards complete evicted PMTs to EMC.
    Reserve     — cortical capacity allocated to a specific memory function
    Threshold   — minimum relevancy score required for an EMC episode
                  to enter the active memory context

Public interface:
    MemoryCoordinationCore:
        await register_memory(user_id: str, content: str) -> None
        context = await assemble_memory_context(user_prompt: str) -> list[dict]
        assess_memory_schema() -> dict
        report_memory_stats() -> None
        forget_memory() -> None
        close() -> None
"""

# System components
import asyncio                                      # for fire-and-forget episodic binding and EMC recall timeout racing
from concurrent.futures import ThreadPoolExecutor   # for type hint on executor parameter
from datetime import datetime, timedelta
import numpy as np                                  # for building vector anchor

# AGi components
from hrs.hru import GatewayMap, ChunkSampler        # establish engram gateway, and probe and truncate cognitive context for budget management
from hrs.hrm import AGi                             # homeostatic regulation manifest namespace — system-wide constants
SCS = AGi.SCS                                       # SCS parameter namespace alias — keeps constant references concise
EMC = AGi.SCS.EMC                                   # EMC parameter namespace alias — keeps WMC constant references concise

# TODO: MCC imports EncodingEngine directly from msb.py as a temporary layering exception.
# Encoding engine construction is owned here to avoid multiple model loads across cortices.
# M2 cleanup: move EncodingEngine to scs/eee.py — dedicated encoding layer between MCC and MSB.
from scs.msb import EncodingEngine                  # shared encoding engine — owned by MCC, passed to all cortices
from scs.wmc import PMT, WorkingMemoryCortex        # Working Memory Cortex — sustains active PMTs in hot short-term memory
from scs.emc import EpisodicMemoryCortex            # Episodic Memory Cortex — encodes evicted PMTs and recalls past episodes

class MemoryCoordinationCore:
    """
    Memory Coordination Core — the memory manager of the SCS.

    Central relay between CNC and the memory cortex layers (WMC, EMC).
    Instantiated once at startup. All memory operations go through MCC
    — CNC never touches WMC or EMC directly.
    """

    def __init__(self, logger, executor: ThreadPoolExecutor) -> None:
        """
        Initiatiate the Memory Coordination Core and prepare its memory layers for operation.

        Args:
            logger: Logger instance forwarded from caller
            executor: Thread pool for blocking operations
        """
        self.logger = logger            # logger forwarded from CNC — all MCC methods emit through this handle
        self._executor = executor       # thread pool for blocking operations

        # Construct shared encoding engine — owned by MCC, passed to all cortices
        self._encoding_engine = EncodingEngine(             # one model load — shared across EMC, SMC, PMC
            logger          = logger,                       # logger instance forwarded from caller
            encoding_engine = EMC.ENCODING_ENGINE,          # model name — e.g. BAAI/bge-small-en-v1.5
            cue_prefix      = EMC.ENCODING_CUE_PREFIX,      # query prefix for recall cues
            engram_prefix   = EMC.ENCODING_ENGRAM_PREFIX,   # document prefix for engrams
            prime_capacity  = EMC.ENCODING_PRIME_CAPACITY,  # max LRU prime entries before eviction
            prime_key_len   = EMC.ENCODING_PRIME_KEY_LEN,   # max chars hashed per prime key
        )

        # Ensure engram gateway exists
        _gateway = GatewayMap()                                                            # absolute path to the engram complex — constructed by HRS
        self.engram_gateway = _gateway.connect_gateway(_gateway.engram_complex)            # create all missing parent dirs — no-op if already exists

        # Initialize ChunkSampler for consistent token counting
        self._chunk_sampler = ChunkSampler(logger)                                         # use chunk sampler for consistent token counting

        # Initialize memory cortex layers
        self.logger.info("🔄 Activating Memory Coordination Core…")                        # log entry on MCC activation
        self.wmc = WorkingMemoryCortex(                                                    # boot WMC — owns the active PMT slot
            logger=logger,                                                                 # logger instance forwarded from caller 
            chunk_sampler=self._chunk_sampler,                                             # pass down tokenizer for accurate token count
        )
        self.emc = EpisodicMemoryCortex(                                                   # boot EMC — owns the engram complex on disk
            logger          = logger,                                                      # logger instance forwarded from caller
            engram_gateway  = self.engram_gateway,                                         # path to the engram complex database
            chunk_sampler   = self._chunk_sampler,                                         # pass down tokenizer for accurate token count
            encoding_engine = self._encoding_engine,                                       # shared encoding engine — one model load across all cortices
        )

        self.logger.info("✅ Memory Coordination Core Activated")                          # log entry on successful MCC activation

    async def register_memory(self, user_id: str | None, role: str, content: str) -> None:
        """
        Receive a new conversation turn and commit it to working memory.
        Any PMTs evicted by WMC are handed off to the episodic buffer — non-blocking.
        Eviction only fires at interaction boundary — never mid-exchange.

        1. Pair to complete interaction and fill induced PMT into WMC
        2. Bind any evicted PMT to episodic buffer (non-blocking)

        Args:
            user_id (str | None) : User ID of the speaker interacting with AI; None if the speaker is AI
            role (str)           : Role of the speaker - "user" or "assistant"
            content (str)        : Content of the conversation turn
        """

        chunks = self._chunk_sampler.probe(content)                                                 # estimate token cost of the content
        if role == "user":                                                                          # if user prompt,
            self.logger.info(f"📝 stimulus: {chunks} tokens")                                       # log token cost of user prompt
        elif role == "assistant":                                                                   # if AI response,
            self.logger.info(f"🧠 response: {chunks} tokens")                                       # log token cost of AI response
        
        # Fill induced PMT to WMC — returns evicted PMTs synchronously (fast, in-memory)
        filled_pmt, evicted_pmts = self.wmc.fill_pmt(user_id=user_id, role=role, content=content)   # induce turn into WMC — returns filled PMT and any displaced PMTs

        # Score filled PMT at induction — only fires on assistant turn (filled_pmt is None on user turn)
        if filled_pmt is not None:                                                                  # assistant turn only — user turn returns None
            filled_pmt.vector      = self._encoding_engine.encode_engram(filled_pmt.trace)          # encode trace — clean formatted text, no JSON noise
            pending_binding, score = self._score_pmt_at_induction(filled_pmt)                       # 5+1-factor induction gate — biological analogue: hippocampal tagging during experience
            filled_pmt.retention_score = score                                                      # cache composite score — WMC eviction priority key
            if pending_binding:                                                                     # PMT scored above induction threshold — bind to EMC immediately
                filled_pmt.anchored = True                                                          # mark as hard-gated — protected from WMC eviction
                _ = asyncio.get_running_loop().run_in_executor(                                     # recruit dormant thread — binding never blocks active cognition
                    self._executor, self.emc.bind_pmt,                                              # async bind to EMC via executor
                    filled_pmt.user_id, filled_pmt.timestamp, filled_pmt.trace                      # pass PMT identity, timestamp, and formatted trace
                )
                self.logger.debug("MCC induction path → EMC binding (high salience)")               # log immediate binding via induction gate

        # Bind evicted PMTs to episodic buffer
        # Run and forget — never blocks active cognition
        if evicted_pmts:                                                                            # evicted PMTs present — hand off to episodic buffer
            _ = asyncio.get_running_loop().run_in_executor(                                         # recruit a dormant thread — binding never blocks active cognition
                self._executor, self._bind_to_episodic_buffer, evicted_pmts                         # bind evicted PMTs to episodic buffer
            )
            self.logger.debug(                                                                      # log the binding transition of evicted PMTs to episodic buffer
                f"MCC bound {len(evicted_pmts)} evicted PMT(s) → episodic buffer"
            )

    def _bind_to_episodic_buffer(self, evicted_pmts: list[PMT]) -> None:
        """
        Bind evicted PMTs from WMC into the episodic buffer for encoding and consolidation.
        Runs on a dormant thread — never blocks active cognition.
        Anchor vector safety net (M1.5): single depth check at eviction boundary — last-chance gate.
        PMTs that scored below INDUCTION_THRESHOLD at induction get one more chance on depth alone.
    
        Args:
            evicted_pmts (list[PMT]) : List of evicted PMTs from WMC
        """
        try:                                                         # attempt binding evicted PMTs to episodic buffer
            for evicted_pmt in evicted_pmts:                         # iterate through each evicted PMT
                # Anchor vector safety net — depth check only, no full composite re-run
                depth_score = self._cosine_sim(                      # single cosine sim — last-chance gate at eviction boundary
                    evicted_pmt.vector, self._dynamic_anchor
                )
                if depth_score < SCS.MCC.EVICTION_THRESHOLD:         # below safety net threshold — truly forgotten
                    self.logger.debug(                               # log the discarded PMT at eviction boundary
                        "MCC eviction safety net — PMT discarded"
                    )
                    continue                                         # skip — not worth encoding into episodic memory
                self.emc.bind_pmt(                                   # bind evicted PMT into episodic buffer
                    user_id=evicted_pmt.user_id,                     # speaker identity of the original PMT
                    timestamp=evicted_pmt.timestamp,                 # induction timestamp of the original PMT
                    trace=evicted_pmt.trace,                         # formatted trace — embedding input and reinstatement
                )
        except Exception as e:                                       # if binding lapse occurs, log and continue
            self.logger.error(                                       # Log the binding lapse
                f"MCC binding lapse — {len(evicted_pmts)} PMT(s) unbound: {e}",
            )
            
    async def assemble_memory_context(self, user_id: str, user_prompt: str) -> list[dict]:
        """
        Assemble the full memory context for the cognitive engine.
        EMC reinstatement runs on a dormant thread — awaited before returning.
        Awaits both before returning — inference requires full memory context.

        Structure:
            [EMC reinstated episodes (system block, if relevant)]
            + [WMC PMTs (chronological, chat turns)]

        Args:
            user_prompt (str) : Current user message — used as EMC recall cue

        Returns:
            list[dict] : List of message dicts [{role, content}] ready for inference
        """
        # Recall WMC PMTs directly in main neural pathway
        active_pmts: list[PMT] = self.wmc.recall_pmt_schema()                # retrieve the sustained PMT schema
        recalled_pmts: list[dict[str, str]] = self.wmc.reinstate_pmt_schema() # recall the sustained PMT schema in dict structure
        
        # EMC reinstatement on isolated neural pathway — recall, filter, format, inject
        self.emc.episodic_buffer.clear_recall_stream()                       # clear recall stream before reinstating fresh episodes
        reinstated_episodes: list[Episode] = []                              # holds reinstated episodes from EMC
        try:                                                                 # attempt to recall EMC episodes
            future = asyncio.get_running_loop().run_in_executor(             # recruit a dormant thread — EMC recall blocks on encoding engine
                self._executor, self.emc.reinstate_episodes, user_id, user_prompt     # reinstate relevant episodes using current prompt as cue
            )
            reinstated_episodes = await asyncio.wait_for(                    # await with timeout — recall must not stall inference
                future, timeout=SCS.EMC.RECALL_TIMEOUT                       # set a time limit for recall of episodic memory traces
            )
        except asyncio.TimeoutError:                                         # recall exceeded time limit — cancel dormant thread
            self.logger.warning("⚠️  EMC recall timed out — proceeding without episodic context")    # log the timeout error while recalling EMC episodes

        # Reinstate WMC PMTs after EMC episodes — for chronological order in memory context
        self.emc.episodic_buffer.stage_episode_list(recalled_pmts)           # inject recalled WMC PMTs into episodic buffer after EMC episodes — preserves chronological order

        # Build dynamic anchor vector for scoring PMT for binding decision
        _build_dynamic_anchor(active_pmts, reinstated_episodes)              # build a dynamic anchor vector from current memory context
        
        self.logger.debug(                                                   # log the memory context assembled
            f"MCC context assembled: "
            f"{len(recalled_pmts)} WMC PMTs + {len(reinstated_episodes)} EMC episodes reinstated"
        )

        # Cortical capacity assessment  — aggregate WMC + EMC chunks vs total cortical capacity
        assembled_memory_context = self.emc.episodic_buffer.assess_recall_stream()       # retrieve fully assembled memory context for token budget audit
        memory_chunks = sum(                                                             # probe each message — combined cost of WMC + EMC streams
            self._chunk_sampler.probe(m["content"])
            for m in assembled_memory_context
        )
        if memory_chunks > SCS.CORTICAL_CAPACITY:                                        # aggregate exceeds cortical budget — individual reserves are bounded but can stack
            self.logger.warning(                                                         # log warning message of chunk counts of memory context over cortical capacity
                f"⚠️  MCC cortical overflow — "
                f"{memory_chunks} chunks memory context vs {SCS.CORTICAL_CAPACITY} capacity "
                f"(WMC: {len(wmc_pmts)} PMTs | EMC: {len(reinstated_episodes)} episodes)"
            )
        else:                                                                            # if token cost within budget
            self.logger.debug(                                                           # log load for tuning visibility
                f"MCC cortical capacity: {memory_chunks}/{SCS.CORTICAL_CAPACITY} chunks "
                f"({round(memory_chunks / SCS.CORTICAL_CAPACITY * 100, 1)}% load)"
            )
        
        return assembled_memory_context                                                  # return fully assembled memory context

    def assess_memory_schema(self) -> dict:
        """
        Assess the current state of all memory cortex layers for logging and health checks.

        Returns:
            dict : Combined schema from WMC and EMC cortex layers
        """
        wmc_schema: dict = self.wmc.assess_pmt_schema()      # assess current PMT state of working memory
        emc_schema: dict = self.emc.assess_emc()             # assess current engram state of episodic memory
        return {                                             # return the current stats of all memory cortex layers
            "wmc": wmc_schema,                               # current stats of working memory
            "emc": emc_schema,                               # current stats of engram complex
        }

    def report_memory_stats(self) -> dict:
        """
        Report current memory cortex stats to the log.
        Called by CNC after every turn for health monitoring.

        Returns:
            dict : Combined memory stats from WMC and EMC cortex layers
        """
        stats: dict = self.assess_memory_schema()           # assess current state of all memory cortex layers
        wmc_stats: dict = stats["wmc"]                      # extract working memory stats
        emc_stats: dict = stats["emc"]                      # extract episodic memory stats
        self.logger.info(                                   # log the current stats of all memory cortex layers 
            f"🧠 Memory stats:\n"
            f"   WMC: {wmc_stats['pmt_count']} PMTs ({wmc_stats['slot_occupancy']}%) | "
            f"{wmc_stats['sustained_chunks']}/{wmc_stats['global_chunk_limit']} chunks ({wmc_stats['chunk_occupancy']}%)\n"
            f"   EMC: {emc_stats.get('engram_count', 0)} episodes | "
            f"{emc_stats.get('binding_pending', 0)} binding | "
            f"{emc_stats.get('buffer_count', 0)} staged | "
            f"{emc_stats.get('physical_volume', 0)} MB"
        )
        return stats                                       # return the stats collected from all memory cortex layers

    def forget_memory(self) -> None:
        """
        Forget active memory at session end or conversation reset.

        Clears:
            - Working memory (WMC pmt_slot)
        Preserves:
            - Episodic memory (EMC) — permanent
            - Semantic memory (SMC) — permanent (future)
            - Procedural memory (PMC) — permanent (future)
        """
        self.wmc.forget_pmt_schema()                        # clear all sustained PMTs from working memory
        self.logger.info("🧹 Working memory forgotten")     # log entry on successful working memory forgetting

    def close(self) -> None:
        """
        Gracefully close MCC and its memory cortex layers.
        Flushes remaining WMC PMTs to EMC before shutdown,
        then waits for encoding cycle to drain before terminating.
        """
        # Flush remaining WMC PMTs to EMC before shutdown
        # M1.5 — full induction scoring replaces unconditional flush
        memory_residue = self.wmc.forget_pmt_schema()                         # retrieve the remaining PMTs in the slots
        if memory_residue:                                                    # remaining PMTs in WMC at shutdown
            for i, pmt in enumerate(memory_residue):                          # score each remaining PMT through full induction gate
                pending_binding, score = self._score_pmt_at_induction(pmt)    # same gate as wake — consistent scoring model
                if pending_binding:                                           # PMT scored above induction threshold — bind before shutdown
                    self.emc.bind_pmt(                                        # bind high-salience PMT to episodic buffer
                        user_id=pmt.user_id,                                  # speaker identity — preserved from original PMT
                        timestamp=pmt.timestamp,                              # induction timestamp — temporal anchor for recall
                        trace=pmt.trace,                                      # formatted trace — embedding input and reinstatement
                    )
            self.logger.info(f"🧠 MCC flushed {len(memory_residue)} WMC PMT(s) → EMC on shutdown")  # log flush completion before encoding drain
            self.emc.drain_encoding_cycle()                                   # wait for binding stream to encode before terminating

        self.emc.terminate()                                                  # release EMC engram gateway file handles
        self.logger.info("🗄️  MCC shutdown sequence complete")                # log completion of MCC shutdown
        
    def _build_dynamic_anchor(self, active_pmts: list[PMT], reinstated_episodes: list[Episode]) -> None:
        """
        Build dynamic session anchor from current WM and EM vectors.
        Stores result on self._dynamic_anchor for induction scoring.
        Biological analogue: hippocampal-prefrontal coherence signal — 
        mean population vector of all currently active memory traces.
    
        Args:
            active_pmts (list[PMT])              : Sustained PMTs from WMC — live vectors
            reinstated_episodes (list[Episode])  : Reinstated episodes from EMC — encoding blobs
        """
        vectors: list[np.ndarray] = []                                           # accumulator for all active trace vectors
    
        for pmt in active_pmts:                                                  # walk sustained WM traces
            if pmt.vector:                                                       # skip PMTs that never received an embedding
                vectors.append(np.array(pmt.vector, dtype=np.float32))           # list[float] → ndarray for uniform stack
    
        for episode in reinstated_episodes:                                      # walk reinstated EM traces
            if episode.encoding:                                                 # skip episodes with no stored encoding
                vectors.append(np.frombuffer(episode.encoding, dtype=np.float32))# packed bytes → ndarray view, no copy
    
        if not vectors:                                                          # nothing active — anchor is undefined
            return                                                               # leave self._dynamic_anchor unchanged; caller handles absence
    
        self._dynamic_anchor = np.mean(vectors, axis=0).tolist()                 # mean population vector → list[float] to match PMT.vector contract
        
    def _score_pmt_at_induction(self, pmt: PMT) -> tuple[bool, float]:
        """
        5+1-factor WMC→EMC encoding gate — stub pending anchor init.
        Biological analogue: hippocampal tagging during experience, in parallel with PFC maintenance.
    
        Args:
            pmt (PMT)                : PMT being scored at induction
            prev_pmt (PMT | None)    : Previous PMT in slot — None if first turn
    
        Returns:
            tuple[bool, float] : (should_encode, composite_score)
        """
        return False, 0.0                                                 # stub — replaced when anchor vectors are initialized

    def _build_static_anchors(self, days: int = 7) -> None:
        """
        Build static session anchors from consolidated episodic memory.
        Stores result on self._static_anchors for induction scoring.
        Biological analogue: schema consolidation during sleep — distinct
        experiential themes preserved as separate centroids rather than
        dissolved into a single mean.
    
        Intended to run during dreaming cycle, not on the hot path.
        Results are persisted to SQLite and loaded at bootup.
    
        Args:
            days (int) : Lookback window in days (1=today, 7=week, 30=month)
        """
        cutoff = datetime.utcnow() - timedelta(days=days)                        # earliest episode to include
    
        encodings = self.emc.msb.get_episode_encodings_since(cutoff)             # fetch encoded episodes within window
    
        if not episodes:                                                         # no episodes in window — anchors undefined
            return                                                               # leave self._static_anchors unchanged; caller handles absence
    
        vectors = []
        ages_days = []
    
        for blob, created_at in encodings:                                               # walk consolidated EM traces — (encoding blob, timestamp) tuples
            if blob:                                                                     # skip unencoded engrams
                vectors.append(np.frombuffer(blob, dtype=np.float32))                    # packed bytes → ndarray view, no copy
                age = (datetime.utcnow() - created_at).total_seconds() / 86400          # age in fractional days
                ages_days.append(age)                                                    # track age for recency weighting
    
        if not vectors:                                                          # all episodes had empty encodings
            return
    
        matrix = np.stack(vectors)                                               # (N, D) — one row per episode
        weights = np.exp(-0.1 * np.array(ages_days, dtype=np.float32))          # recency decay — half-weight at ~7 days

        try:
            import hdbscan
            clusterer = hdbscan.HDBSCAN(min_cluster_size=5)                     # discover topic clusters; no need to specify K
            clusterer.fit(matrix)                                                # fit on raw vectors, not weighted — HDBSCAN handles density
    
            anchors = []
            for label in set(clusterer.labels_):                                 # walk discovered clusters
                if label == -1:                                                  # noise points — not a coherent theme, skip
                    continue
                mask = clusterer.labels_ == label                                # boolean mask for this cluster
                cluster_weights = weights[mask]                                  # recency weights for members of this cluster
                centroid = np.average(matrix[mask], axis=0, weights=cluster_weights)  # weighted centroid — recent episodes pull harder
                anchors.append(centroid.tolist())                                # list[float] to match PMT.vector contract
    
            self._static_anchors = anchors if anchors else None                  # None if all points were noise

        except ImportError:                                                      # hdbscan not available — fall back to single weighted mean
            centroid = np.average(matrix, axis=0, weights=weights)              # loses topic separation but stays functional
            self._static_anchors = [centroid.tolist()]                          # wrap in list to keep caller interface uniform
            
    def _cosine_sim(self, a: list[float], b: list[float]) -> float:
        """
        Cosine similarity between two unit-normalized vectors.
        Vectors are pre-normalized at encoding — dot product equals cosine sim.
    
        Args:
            a (list[float]) : First unit-normalized vector
            b (list[float]) : Second unit-normalized vector
    
        Returns:
            float : Cosine similarity in range [0.0, 1.0]
        """
        return 0.0                                                        # stub — replaced when anchor vectors are initialized
