"""
EMC — Episodic Memory Cortex
==============================
AuRoRA · Semantic Cognitive System (SCS)

Episodic memory layer of the CNS — "I remember that specific moment."
Stores conversational PMTs as dated, semantically encoded engrams for future recall.
No expiry — 1TB NVMe means Grace remembers everything.

Responsibilities:
    - Receive evicted PMTs from MCC into crash-safe episodic buffer (binding)
    - Encode buffered turns into semantic vectors (encoding)
    - Consolidate encoded episodes into the episodic engram (synaptic consolidation)
    - Recall relevant past episodes via semantic and lexical search (recall)
    - Surface recalled episodes into MCC memory context (reinstatement)

Architecture:
    Episodic Buffer (Baddeley's model, 2-layer):
        _binding_stream  — private deque; evicted PMTs land here first (fast RAM staging)
                           invisible to MCC — MCC calls bind_pmt() and lets go,
                           mirroring how PFC does not monitor every hippocampal trace after handoff
        recall_stream    — shared stream; recalled episodes surface here before
                           being injected into MCC memory context

    SQLite engram (one table + two indexes):
        emc_storage      — permanent episodic memory store
        emc_vector       — sqlite-vec KNN index for semantic search
        emc_lexical      — FTS5 index for lexical search
        emc_staging      — crash-safe buffer for unencoded PMTs

    Encoding:
        Continuous during wake — online encoding, not sleep-only.
        sentence-transformers, CPU-only, zero GPU impact.
        Biological analogue: hippocampal initial encoding during wake.
        Deep consolidation (SMC distillation) deferred to M2 Dream Cycle.

    Encoding Cycle:
        Background thread drains _binding_stream → encodes → inscribes into emc_storage.
        Falls back to emc_staging on restart for crash recovery.
        Runs continuously, sleeps when buffer is empty via theta rhythm event.
        Never blocks active cognition.
        
    Retrieval:
        Dual-path retrieval fused via Reciprocal Rank Fusion (RRF):
            PATH 1 — Semantic (cf. CA3 pattern completion):
                sqlite-vec L2 KNN on unit-normalized vectors (cosine-equivalent)
            PATH 2 — Lexical (cf. dentate gyrus pattern separation):
                FTS5 porter-stemmed search on episode content
        Fallback chain:
            Both paths active  → RRF fusion (full memory convergence)
            Lexical only       → lexical results  (encoding engine unavailable)
            Neither            → []               (catastrophic — SQLite unavailable)

    Storage:
        SQLite WAL mode — Jetson-friendly, concurrent read/write

Terminology:
    episodic_buffer  — in memory buffer for evicted PMTs (crash-safe, temporary)
    encoding         — semantic encoding of a turn into a vector
    episode          — dated and encoded memory trace (a specific remembered moment)
    engram           — the memory store containing encoded episodes
    FTS5             — SQLite FTS5 full-text search extension for lexical search
    relevancy        — RRF-fused salience score of the episode (0.0–1.0) combining semantic and lexical rank
    recall stream    — shared stream for recalled episodes (cross-layer)
    RRF              — Reciprocal Rank Fusion for combining semantic and lexical search

Lifecycle:
    Binding → Encoding → Synaptic Consolidation → Recall → Reinstatement
    (System Consolidation — EMC → SMC distillation — deferred to M2 Dream Cycle)

Public interface:
    emc.bind_pmt(timestamp, content) → bool
    emc.recall_episodes(cue, recall_limit) → list[dict]
    emc.assess_emc() → dict
    emc.terminate() → None

TODO:
    M2 — date-range filtering on recall interface and buffer entries
    M2 — DiskANN ANN index when episodes exceed ~50k (currently exact KNN)
    M2 — SMC distillation trigger at 11pm reflection (Dream Cycle)
    M2 — heartbeat logging during long idle periods
    M2 — staging_id integrity check after Dream Cycle consolidation
    M2 — graceful drain + optional timeout fallback for SWR trigger during close
"""

# System libraries
import os                                   # for encoding thread priority via os.nice()
from pathlib import Path                    # for building engram complex storage path
import itertools                            # for islice — caps binding stream snapshot per theta rhythm cycle
import threading                            # for background thread, locks, and theta rhythm event
import time                                 # for CPU yield during theta rhythm cycle
from collections import deque               # for O(1) append/popleft in binding stream
from dataclasses import dataclass, field    # for EpisodicBuffer and episode dataclasses

# AGi libraries
from hrs.hrp import AGi                     # homeostatic regulation parameter namespace
CNS = AGi.CNS                               # CNS-level constants — e.g. UNITS_PER_CHUNK
EMC = AGi.CNS.EMC                           # EMC constants — encoding engine, limits, dims

from scs.msb import (                       # shared memory storage bank substrate
    EngramSchema,                           # blueprint for engram table structure
    EngramTrace,                            # single column definition within a blueprint
    EngramModality,                         # TEXT / INTEGER / REAL / BLOB type enum
    RecallCue,                              # encoded vector + raw text for dual-path recall
    EncodingEngine,                         # sentence-transformers wrapper with LRU prime
    pack_vector,                            # for packing float list into fp32 binary blob
    EngramComplex,                          # all SQL ops for a memory cortex
)

EMC_SCHEMA = EngramSchema(                  # define the engram schema for episodic memory
    storage=[
        EngramTrace(label="id",         modality=EngramModality.INTEGER),                           # auto-assigned primary key — rowid alias
        EngramTrace(label="timestamp",  modality=EngramModality.TEXT, essential=True),              # ISO-8601 datetime of the original PMT
        EngramTrace(label="date",       modality=EngramModality.TEXT, essential=True),              # YYYY-MM-DD slice of timestamp — B-tree indexed for date recall
        EngramTrace(label="content",    modality=EngramModality.TEXT, essential=True),              # raw turn content — fed into FTS5 lexical index
        EngramTrace(label="encoding",   modality=EngramModality.BLOB, essential=True),              # fp32 binary vector — linked to vec0 KNN index by rowid
        EngramTrace(label="created_at", modality=EngramModality.TEXT, baseline="(datetime('now'))"), # wall-clock inscription time — set by SQLite on INSERT
    ],
    staging=[
        EngramTrace(label="id",         modality=EngramModality.INTEGER),                           # auto-assigned staging_id — used for decay after consolidation
        EngramTrace(label="timestamp",  modality=EngramModality.TEXT, essential=True),              # preserved from original PMT
        EngramTrace(label="date",       modality=EngramModality.TEXT, essential=True),              # preserved from original PMT
        EngramTrace(label="content",    modality=EngramModality.TEXT, essential=True),              # raw content pending encoding
    ],
    semantic_traces="encoding",                                                                     # column linked to vec0 virtual table for KNN search
    lexical_traces=["content"],                                                                     # column fed into FTS5 virtual table for keyword search
    index_traces=["date"]                                                                           # B-tree index — speeds up date-filtered recall
)
        
@dataclass
class EpisodicBuffer:
    """
    Episodic Buffer — shared workspace between WMC and EMC.
    Modelled on Baddeley's episodic buffer — one buffer, two streams.

    _binding_stream — evicted PMTs pending encoding into episodic memory
    recall_stream   — recalled episodes pending reinstatement into active cognition
    """
    _binding_stream: deque[dict] = field(default_factory=deque)                # evicted PMTs queued for encoding — drained by encoding cycle
    recall_stream: list[dict] = field(default_factory=list)                    # recalled episodes queued for MCC context assembly

    def __post_init__(self) -> None:
        """Initialize locks not expressible as dataclass fields."""
        self._recall_lock = threading.Lock()                                   # serializes recall stream mutations across threads
       
    def clear_recall_stream(self) -> None:
        """Clear the recall stream before assembling a new memory context."""
        with self._recall_lock:                                                # hold lock for duration of mutation
            self.recall_stream.clear()                                         # discard all previously recalled episodes

    def stage_single_episode(self, item: dict) -> None:
        """Append a single recalled episode to the recall stream."""
        with self._recall_lock:                                                # hold lock for duration of mutation
            self.recall_stream.append(item)                                    # append single episode to recall stream

    def stage_episode_list(self, items: list[dict]) -> None:
        """Extend the recall stream with a list of recalled episodes."""
        with self._recall_lock:                                                # hold lock for duration of mutation
            self.recall_stream.extend(items)                                   # extend recall stream with episode batch

    def assess_recall_stream(self) -> list[dict]:
        """Return a snapshot of the recall stream for safe iteration.
        
        Returns:
            list[dict]: Stable copy of the current recall stream.
        """
        with self._recall_lock:                                                # hold lock while copying — prevents mutation mid-read
            return list(self.recall_stream)                                    # shallow copy — safe for iteration outside the lock

class EncodingCycle:
    """
    Encoding Cycle — theta-rhythm driven online encoding of episodic memories.
    Drains the binding stream, encodes PMTs into semantic vectors,
    and consolidates them into the episodic engram.

    Biological analogue: intra-hippocampal CA3/CA1 LTP-driven trace encoding during wake.
    """

    def __init__(self, logger, ecx: EngramComplex, encoding_engine: EncodingEngine,
                 episodic_buffer: EpisodicBuffer, episodic_buffer_lock: threading.Lock) -> None:
        """
        Initialize the episodic encoding cycle and start the background encoding thread.
        Recovers any unencoded PMTs from the staging table before the thread starts.

        Args:
            logger                                : Logger instance from EMC
            ecx (EngramComplex)                   : Engram complex for staging and inscription
            encoding_engine (EncodingEngine)      : Shared encoding engine for semantic vectors
            episodic_buffer (EpisodicBuffer)      : Episodic buffer — binding stream source
            episodic_buffer_lock (threading.Lock) : Lock for thread-safe binding stream access
        """
        self.logger                = logger                             # logger instance from EMC
        self._ecx                  = ecx                                # engram complex for staging and inscription
        self._encoding_engine      = encoding_engine                    # shared encoding engine for semantic vectors
        self._episodic_buffer      = episodic_buffer                    # episodic buffer — binding stream source
        self._episodic_buffer_lock = episodic_buffer_lock               # lock for thread-safe binding stream access
                     
        # Inscription lock — only encoding cycle inscribes into episodes
        self._inscription_lock     = threading.RLock()                  # serializes engram writes — RLock allows re-entry
                     
        # Set up encoding cycle
        self._encoder_running      = False                              # encoding cycle not yet started
        self._theta_rhythm         = threading.Event()                  # gates encoding cycle — set by trigger_theta_rhythm() or WMC
        self._encoder_thread: threading.Thread | None = None            # assigned in _ignite_cycle()
        self._ignite_cycle()                                            # kick start the encoding cycle

    def _ignite_cycle(self) -> None:
        """
        Recover unencoded PMTs from the staging table into the binding stream,
        then start the background encoding thread.
        Batch recovery avoids RAM spike after crash during a long session.
        """
        # Batch recovery of unencoded episodes — drain in chunks of EMC.RECOVERY_BATCH_SIZE
        # Avoids loading thousands of unencoded episodes into binding stream at once
        recovery_count = 0                                          # total recovered episode count
        recovery_offset = 0                                         # pagination offset for batch recovery

        while True:                                                 # keep recovering unencoded episodes in batches
            unencoded = self._ecx.retrieve_staged_batch(            # query staging table for unencoded episodes
                batch_size = EMC.RECOVERY_BATCH_SIZE,               # number of unencoded episodes to recover at once
                offset     = recovery_offset,                       # offset for pagination — skips already-recovered episodes
            )

            if not unencoded:                                       # if no more unencoded episodes to recover,
                break                                               # stop recovering unencoded episodes

            with self._episodic_buffer_lock:                        # hold lock for binding stream append
                for row in unencoded:                               # iterate through each unencoded episode
                    self._episodic_buffer._binding_stream.append({  # add the unencoded episode to the binding stream
                        "staging_id" : row["id"],                   # staging_id for decay after consolidation
                        "timestamp": row["timestamp"],              # preserved timestamp from original PMT
                        "date":      row["date"],                   # preserved date from original PMT
                        "content":   row["content"],                # raw content pending encoding
                    })

            recovery_count  += len(unencoded)                       # accumulate total recovered count
            recovery_offset += EMC.RECOVERY_BATCH_SIZE              # advance pagination offset

        if recovery_count:                                          # if any episodes were recovered,
            self.logger.info(                                       # Log the recovery of unencoded episodes
                f"⚡ EMC recovered {recovery_count} unencoded episode(s) from engram → binding stream"
            )                                                       
            self._theta_rhythm.set()                                # wake encoding cycle for recovered episodes

        self._encoder_running = True                                # mark cycle as active before thread starts
        self._encoder_thread  = threading.Thread(                   # assign a dormant thread for the encoding cycle
            target=self._run_cycle,                                 # encoding cycle main loop
            name="emc-encoding-cycle",                              # named for debugging
            daemon=True,                                            # dies with main process
        )
        self._encoder_thread.start()                                # start the neural thread of encoding cycle
        self.logger.info("🔄 EMC encoding cycle started")           # log the start of encoding cycle

    def _run_cycle(self) -> None:
        """
        Theta-rhythm driven encoding — drains binding stream into emc_storage.
        Wakes on PMT arrival or theta interval — whichever comes first.
        Processes a fixed snapshot per rhythm — new arrivals deferred to next cycle.
        """
        # Yield processing priority to active cognition threads
        os.nice(19)                                               # deprioritize — encoding is non-latency-sensitive

        encoder_conn = self._ecx.bifurcate_ecx()                  # bifurcated connection for parallel writes
        self.logger.info("⚙️ EMC encoding cycle running…")        # log the start of encoding cycle
    
        while self._encoder_running:                              # loop until stop signal
            # Rest state — wait for theta rhythm activation
            self._theta_rhythm.wait(timeout=EMC.THETA_INTERVAL)   # wake on PMT arrival or theta interval
            self._theta_rhythm.clear()                            # reset event for next cycle
    
            if not self._encoder_running:                         # if the encoder is not running,
                break                                             # clean exit if stopped while waiting
    
            # Snapshot IDs at this moment — one rhythm, one defined window
            with self._episodic_buffer_lock:                      # hold lock for snapshot and drain
                if not self._episodic_buffer._binding_stream:     # binding stream empty — nothing to encode
                    continue                                      # skip this encoding cycle
                rhythm: list[dict] = list(itertools.islice(
                    self._episodic_buffer._binding_stream, EMC.THETA_BATCH_LIMIT
                ))                                                # snapshot up to batch limit — remaining stays for next cycle
                
                for _ in range(len(rhythm)):                      # iterate through the length of snapshot
                    self._episodic_buffer._binding_stream.popleft() # drain only what was snapshotted
    
            self.logger.debug(f"EMC encoding cycle → {len(rhythm)} episode(s) in rhythm") # log the number of episodes in the rhythm
    
            # Replay each episode in the rhythm
            for episode in rhythm:                              # for each episode in the rhythm,
                if not self._encoder_running:                   # if the encoder is not running,
                    break                                       # respect stop signal mid-rhythm
                time.sleep(0.01)                                # 10ms yield — prevents starving active cognition

                # Inscribe to episodic_buffer (crash-safe record) before encoding
                # Skip if already recovered from episodic_buffer on restart
                if not episode.get("staging_id"):                           # if no staging_id, stage the episode
                    with self._inscription_lock:                            # hold inscription lock for staging write
                        episode["staging_id"] = self._ecx.stage_engram(     # insert the episode into the episodic buffer
                            engram = {                                      # create engram with timestamp, date, and content
                                "timestamp": episode["timestamp"],          # timestamp of episode
                                "date":      episode["date"],               # date of episode
                                "content":   episode["content"],            # content of episode
                            },
                            ecx_conn = encoder_conn,                        # connection to episodic buffer
                        )
      
                # Encode the episode content into a semantic vector
                encoded_episode: list[float] = self._encoding_engine.encode_engram(episode["content"]) # encode content into semantic vector
                if not encoded_episode:                                             # if the encoding failed,
                    # Encoding engine unavailable — skip for now, retry later
                    self.logger.warning(                                            # log the warning message of unavailability of the encoding engine
                        f"EMC encode skipped (encoding engine unavailable): "
                        f"{len(episode['content']) // CNS.UNITS_PER_CHUNK + 1} chunks"
                    )
                    with self._episodic_buffer_lock:                                # hold lock for appendleft
                        self._episodic_buffer._binding_stream.appendleft(episode)   # requeue for next theta cycle
                    continue                                                        # continue to the next episode
    
                # Pack encoded episode vector as fp32 binary for engram storage
                encoding_blob = pack_vector(encoded_episode)                        # pack float vector into fp32 binary blob
            
                self._synaptic_consolidate(encoder_conn, episode, encoding_blob)    # consolidate the episode from episodic buffer into engram
    
                self.logger.debug(
                    f"EMC coded and stored → episodes: {len(episode['content']) // CNS.UNITS_PER_CHUNK + 1} chunks" # log the number of chunks in the episode
                    f" (date={episode['date']})",                                   # log the date of the episode
                )
    
        encoder_conn.close()                                                        # close the encoder connection
        self.logger.info("EMC encoding cycle stopped")                              # log the stop of the EMC encoding cycle

    def trigger_theta_rhythm(self) -> None:    
        """
        Trigger the theta rhythm to wake the encoding cycle.
        """
        self._theta_rhythm.set()                            # wake the theta rhythm to start encoding

    def _synaptic_consolidate(self, encoder_conn, episode: dict, encoding_blob: bytes) -> None:
        """
        Stabilize one encoded episode into all three engram indexes.
        Biological analogue: LTP-driven trace stabilization in the hippocampus during wake.
        Distinct from systems consolidation (EMC → SMC) which occurs during the M2 Dream Cycle.

        Args:
            encoder_conn          : Bifurcated connection for encoding cycle writes
            episode (dict)        : Episode with timestamp, date, content, staging_id
            encoding_blob (bytes) : fp32 binary vector of the encoded episode
        """
        with self._inscription_lock:                                            # serializes all three inscriptions as one atomic operation
            # Primary episodic record
            episode_id = self._ecx.inscribe_engram(                             # inscribe primary episodic record into emc_storage
                engram={                                                        # episode dictionary with timestamp, date, and content
                    "timestamp": episode["timestamp"],                          # episode timestamp
                    "date":      episode["date"],                               # episode date
                    "content":   episode["content"],                            # episode content
                    "encoding":  encoding_blob,                                 # episode encoding
                }, 
                ecx_conn=encoder_conn,                                          # connection to use for the operation
            )

            # Engram vector index (vec0 KNN for semantic search)
            self._ecx.inscribe_vector_index(                                    # inscribe encoding into vec0 KNN index — rowid matches episode_id
                engram_id = episode_id,                                         # episode ID for which to insert the encoding
                blob = encoding_blob,                                           # binary encoding data of the episode
                ecx_conn = encoder_conn,                                        # connection to use for the operation
            )
     
            # Engram lexical index (FTS5 index for lexical search)
            self._ecx.inscribe_lexical_index(                                   # inscribe content into FTS5 index — rowid matches episode_id
                engram_id = episode_id,                                         # episode ID for which to insert the encoding
                content = episode["content"],                                   # raw text content of the episode
                ecx_conn = encoder_conn,                                        # connection to use for the operation
            )
     
            # Remove the entry in episodic buffer — synaptic consolidation is complete, staging row no longer needed
            if episode.get("staging_id") is not None:                           # staging_id present — episode was crash-recovered
                self._ecx.decay_staged_engram(                                  # decay staging row — synaptic consolidation complete
                    staging_id = episode["staging_id"],                         # episode ID for which to decay the encoding
                    ecx_conn  = encoder_conn,                                   # connection to use for the operation
                )
    
    def stop_cycle(self) -> None:
        """Signal the encoding cycle to stop and wait for clean exit."""
        self._encoder_running = False                                   # signal encoding thread to stop
        self._theta_rhythm.set()                                        # wake thread so it can exit cleanly
        if self._encoder_thread:                                        # if encoder cycle is still running,
            self._encoder_thread.join(timeout=3.0)                      # wait for clean exit — up to 3 seconds
            
class EpisodicMemoryCortex:
    """
    Episodic Memory Cortex (EMC) — hippocampal long-term episodic store.
    Receives evicted PMTs from WMC via MCC, encodes them as semantic vectors,
    and consolidates them into the episodic engram for future recall.

    SQLite engram (one table + two indexes):
        emc_storage  — permanent episodic memory store
        emc_vector   — sqlite-vec KNN index for semantic search
        emc_lexical  — FTS5 index for lexical search
        emc_staging  — crash-safe buffer for unencoded PMTs

    Thread-safety:
        _episodic_buffer_lock  — serializes binding stream access
        _recall_lock           — serializes recall stream access (on EpisodicBuffer)
        SQLite WAL mode allows concurrent reads during async writes
    """

    def __init__(self, logger, engram_gateway: Path) -> None:
        """
        Initialize the Episodic Memory Cortex, engram complex, encoding engine,
        and start the background encoding cycle.

        Recovers any unencoded PMTs from the crash-safe staging table back into
        the binding stream before the encoding cycle starts.
        
        Args:
            logger                : Logger instance for logging operations
            engram_gateway (Path) : Path to access the engram for storing episodic memories
        """
        self.logger                = logger                                 # logger from MCC — used throughout EMC
        self.episodic_buffer       = EpisodicBuffer()                       # two-stream buffer — binding and recall streams
        self._episodic_buffer_lock = threading.Lock()                       # serializes binding stream access
        self._encoding_engine      = EncodingEngine(                        # sentence-transformers wrapper with LRU prime
            logger          = logger,                                       # logger instance for logging operations
            encoding_engine = EMC.ENCODING_ENGINE,                          # model name — e.g. BAAI/bge-small-en-v1.5
            cue_prefix      = EMC.ENCODING_CUE_PREFIX,                      # query prefix for recall cues
            engram_prefix   = EMC.ENCODING_ENGRAM_PREFIX,                   # document prefix for engrams
            prime_limit     = EMC.PRIME_LIMIT,                              # max LRU prime entries before eviction
            prime_key_limit = EMC.PRIME_KEY_LIMIT,                          # max chars hashed per prime key
        )                                                                   

        self._ecx = EngramComplex(                                          # owns all SQL ops for EMC
            logger  = self.logger,                                          # logger instance for logging operations
            cortex  = "EMC",                                                # drives table name prefix — emc_storage, emc_vector etc.
            gateway = str(engram_gateway),                                  # cast to str — EngramComplex expects string path
            schema  = EMC_SCHEMA,                                           # blueprint defining EMC table structure
            dim     = EMC.ENCODING_DIM,                                     # vector dimension for vec0 KNN index
        )

        try:                                                                # attempt to start encoding cycle
            self._encoding_cycle = EncodingCycle(                           # theta-driven background encoding thread
                logger               = logger,
                ecx                  = self._ecx,                           # engram complex for staging and inscription
                encoding_engine      = self._encoding_engine,               # shared encoding engine
                episodic_buffer      = self.episodic_buffer,                # binding stream source
                episodic_buffer_lock = self._episodic_buffer_lock,          # lock for binding stream access
            )
        except RuntimeError as e:                                           # if failed to initialize the encoding cycle
            self.logger.error(f"❌ Encoding cycle initialization failed → {e}")  # log before re-raising
            raise                                                           # raise the anomaly to the caller

        self.logger.info(f"✅ EMC initialized → {engram_gateway}")          # log successful init with engram path
   
    def bind_pmt(self, timestamp: str, content: str) -> bool:
        """
        Receive an evicted PMT from MCC and bind it into the episodic buffer for encoding.
        Called at the WMC → EMC boundary — non-blocking, crash-safe via staging table.

        Args:
            timestamp (str): ISO-8601 timestamp of the original PMT
            content (str)  : Raw PMT content — truncated internally to engram content limit

        Returns:
            bool: True on success, False on failure
        """
        episode: dict = {                                                           # package the evicted PMT data into episode
            "timestamp": timestamp,                                                 # timestamp of PMT induced into WMC
            "date":      timestamp[:10],                                            # YYYY-MM-DD slice — B-tree indexed for date recall
            "content":   content[:EMC.EPISODE_CONTENT_LIMIT]                        # truncate to engram limit before binding
        }

        try:                                                                        # attempt to bind the evicted PMT into episodic buffer
            with self._episodic_buffer_lock:                                        # hold lock for binding stream append
                self.episodic_buffer._binding_stream.append(episode)                # queue episode for encoding cycle
            self._encoding_cycle.trigger_theta_rhythm()                             # wake encoding cycle — theta rhythm
            self.logger.debug(                                                      # log the binding of the evicted PMT into episodic buffer
                f"EMC buffer ← {len(content) // CNS.UNITS_PER_CHUNK + 1} chunks"
            )
            return True                                                             # indicate successful binding
        except Exception as e:
            self.logger.warning(f"EMC binding PMT failed: {e}")                     # log failure with reason
            return False                                                            # indicate failure during binding

    def recall_episodes(self, cue: str, recall_limit: int = EMC.RECALL_LIMIT) -> list[dict]:
        """
        Recall relevant episodes from episodic memory.
        Dual-path retrieval fused via RRF — handled internally by MSB.

        Args:
            cue (str)          : Recall cue string
            recall_limit (int) : Maximum number of episodes to return

        Returns:
            list[dict]: Recalled episodes sorted by descending relevancy.
                        Each dict: id, timestamp, date, content, relevancy
        """
        if not cue or not cue.strip():                                          # empty cue — nothing to recall
            return []                                                           # return empty list if no cue
    
        recall_cue: RecallCue = self._encoding_engine.encode_cue(cue)           # encode cue into vector + raw text for dual-path recall
        return self._ecx.recall_engram(recall_cue, recall_limit)                # semantic + lexical RRF fusion — handled by MSB

    def assess_emc(self) -> dict:
        """
        Return EMC health and storage stats.
    
        Returns:
            dict: EMC stats including encoding engine status, binding stream count, and engram complex stats.
                  Empty dict on failure.
        """
        try:
            with self._episodic_buffer_lock:                                    # hold lock for accurate binding stream count
                binding_pending = len(self.episodic_buffer._binding_stream)     # count episodes queued for encoding

            engram_stats = self._ecx.assess_engram_complex()                    # query engram complex for storage stats

            return {
                "engram_count"  : engram_stats.get("engram_count", 0),                    # total encoded episodes in engram storage
                "earliest_timestamp"  : engram_stats.get("earliest_timestamp", None),     # timestamp of the oldest stored episode
                "latest_timestamp"    : engram_stats.get("latest_timestamp", None),       # timestamp of the most recent stored episode
                "physical_volume"     : engram_stats.get("physical_volume", 0.0),         # engram complex size on disk (MB)
                "vector_index_active" : engram_stats.get("vector_index_active", False),   # True if KNN vector index is available
                "buffer_count"        : engram_stats.get("buffer_count", 0),              # unencoded episodes in staging buffer
                "binding_pending"       : binding_pending,                                # episodes queued in binding stream
                "encoding_engine_ready" : self._encoding_engine.is_available,             # True if sentence-transformers loaded
                "encoding_engine"       : EMC.ENCODING_ENGINE,                            # model name for reference
            }
        except Exception as e:                                                  # if assessment fails
            self.logger.error(f"EMC assessment failed: {e}")                    # log failure with reason
            return {}                                                           # empty dict — caller handles no results

    def terminate(self) -> None:
        """
        Gracefully close EMC and release the engram connection.
        Signals encoding cycle to stop and waits for clean exit
        before closing the engram complex.
        """
        self._encoding_cycle.stop_cycle()                                   # signal encoding cycle to stop and wait for clean exit
        self._ecx.terminate()                                               # release engram complex connection
        self.logger.info("🗄️ EMC deactivated")                              # log successful termination
