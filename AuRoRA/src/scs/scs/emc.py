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
    episodic_buffer  — raw binding table for evicted PMTs (crash-safe, temporary)
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
    emc.get_stats() → dict
    emc.close() → None

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
        _inscription_lock      — serializes all engram writes (RLock)
        SQLite WAL mode allows concurrent reads during async writes
    """

    def __init__(self, logger, engram_gateway: str) -> None:
        """
        Initialize the Episodic Memory Cortex, engram complex, encoding engine,
        and start the background encoding cycle.

        Recovers any unencoded PMTs from the crash-safe staging table back into
        the binding stream before the encoding cycle starts.
        
        Args:
            logger              : Logger instance for logging operations
            engram_gateway (str): Path to access the engram for storing episodic memories
        """
        self.logger                = logger                                 # logger from MCC — used throughout EMC
        self.episodic_buffer       = EpisodicBuffer()                       # two-stream buffer — binding and recall streams
        self._episodic_buffer_lock = threading.Lock()                       # serializes binding stream access
        self._encoding_engine      = EncodingEngine(                        # sentence-transformers wrapper with LRU prime
            logger          = logger,                                       # logger instance for logging operations
            encoding_engine = EMC.ENCODING_ENGINE,                          # model name — e.g. BAAI/bge-small-en-v1.5
            cue_prefix      = EMC.RECALL_CUE_PREFIX,                        # query prefix for recall cues
            engram_prefix   = EMC.RECALL_ENGRAM_PREFIX,                     # document prefix for engrams
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

        # Inscription lock — only encoding cycle inscribes into episodes
        self._inscription_lock     = threading.RLock()                      # serializes engram writes — RLock allows re-entry

        # Set up encoding cycle
        self._encoder_running      = False                                  # encoding cycle not yet started
        self._theta_rhythm         = threading.Event()                      # gates encoding cycle — set by bind_pmt()
        self._encoder_thread: threading.Thread | None = None                # assigned in _init_encoding_cycle()

        try:                                                                # attempt to start encoding cycle
            self._init_encoding_cycle()                                     # recover unencoded PMTs and start encoding thread
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
            "content":   content[:EMC.ENGRAM_CONTENT_LIMIT]                         # truncate to engram limit before binding
    }

        try:                                                                        # attempt to bind the evicted PMT into episodic buffer
            with self._episodic_buffer_lock:                                        # hold lock for binding stream append
                self.episodic_buffer._binding_stream.append(episode)                # queue episode for encoding cycle
            self._theta_rhythm.set()                                                # trigger encoding cycle — theta rhythm
            self.logger.debug(                                                      # Log the binding of the evicted PMT into episodic buffer
                f"EMC buffer ← {len(content) // CNS.UNITS_PER_CHUNK + 1} chunks"
            )
            return True                                                             # indicate successful binding
        except Exception as e:
            self.logger.warning(f"EMC binding PMT failed: {e}")                     # log failure with reason
            return False                                                            # indicate failure during binding

    def _init_encoding_cycle(self) -> None:
        """
        Initialize the dormant encoding cycle thread.
        Recovers unencoded PMTs from episodic buffer in batches to avoid
        a large RAM spike after a crash during a long session.
        """
        # Batch recovery of unencoded episodes — drain in chunks of EMC.RECOVERY_BATCH_SIZE
        # Avoids loading thousands of unencoded episodes into binding stream at once
        recovery_count = 0                                          # For tracking the number of recovered unencoded episodes
        recovery_offset = 0                                         # For tracking the offset for batch recovery of unencoded episodes

        while True:                                                 # Keep recovering unencoded episodes in batches
            unencoded = self._ecx.retrieve_staged_batch(            # Query the engram for unencoded episodes
                batch_size = EMC.RECOVERY_BATCH_SIZE,
                offset     = recovery_offset,
            )

            if not unencoded:                                       # If no more unencoded episodes to recover, break
                break                                               # Stop recovering unencoded episodes

            with self._episodic_buffer_lock:                        # Acquire the lock to prevent race conditions
                for row in unencoded:                               # Iterate through each unencoded episode
                    self.episodic_buffer._binding_stream.append({   # Add the unencoded episode to the binding stream
                        "staging_id" : row["id"],                   # Staging index of the episode
                        "timestamp": row["timestamp"],              # Timestamp of the episode
                        "date":      row["date"],                   # Date of the episode
                        "content":   row["content"],                # Content of the episode
                    })

            recovery_count  += len(unencoded)                       # Increment the total recovered count by the number of unencoded episodes
            recovery_offset += EMC.RECOVERY_BATCH_SIZE              # Increment the offset by the batch size

        if recovery_count:                                          # If any episodes were recovered,
            self.logger.info(                                       # Log the recovery of unencoded episodes
                f"⚡ EMC recovered {recovery_count} Unencoded episode(s) from engram → binding stream"
            )
            self._theta_rhythm.set()                                # Set the theta rhythm to trigger encoding

        self._encoder_running = True                                # Indicate the encoding cycle is running
        self._encoder_thread  = threading.Thread(                   # Assign a neural thread for the encoding cycle
            target=self._run_encoding_cycle,                        # Execute the encoding cycle
            name="emc-encoding-cycle",                              # Name the thread for future reference
            daemon=True,                                            # Set the thread to run in the background
        )
        self._encoder_thread.start()                                # Start the neural thread of encoding cycle
        self.logger.info("🔄 EMC encoding cycle started")           # Log the start of encoding cycle

    def _run_encoding_cycle(self) -> None:
        """
        Event-driven encoding — drains episodic_buffer into episodes.
        Trigger in 4-8Hz period or when buffer has episodes (theta rhythm pattern).
        Processes a snapshot of IDs per rhythm — new arrivals deferred to next cycle.
        """

        # Yield processing priority to active cognition threads
        os.nice(19)                                               # Encoding is non-latency-sensitive — defers to all active cognition threads
        encoder_conn = self._ecx.bifurcate_ecx()                  # Bifurcate a connection to the engram complex for parallel processing
        self.logger.info("⚙️ EMC encoding cycle running…")       # Log the start of encoding cycle
    
        while self._encoder_running:                              # While the encoder is running,
            # Rest state — wait for theta rhythm activation
            self._theta_rhythm.wait(timeout=EMC.THETA_INTERVAL)  # wake on PMT arrival or theta interval — whichever comes first
            self._theta_rhythm.clear()                            # Clear the theta rhythm activation flag
    
            if not self._encoder_running:                         # If the encoder is not running,
                break                                             # Clean exit if stopped while waiting
    
            # Snapshot IDs at this moment — one rhythm, one defined window
            with self._episodic_buffer_lock:                    # With lock on episodic buffer,
                if not self.episodic_buffer._binding_stream:    # If the binding stream is empty,
                    continue                                    # Skip this encoding cycle
                rhythm: list[dict] = list(itertools.islice(
                    self.episodic_buffer._binding_stream, EMC.THETA_BATCH_LIMIT
                ))                                                                # snapshot up to batch limit — remaining stays for next theta cycle
                
                for _ in range(len(rhythm)):                                      # iterate through the length of snapshot
                    self.episodic_buffer._binding_stream.popleft()                # drain only what was snapshotted
    
            self.logger.debug(f"EMC encoding cycle → {len(rhythm)} episode(s) in rhythm") # Log the number of episodes in the rhythm
    
            # Replay each episode in the rhythm
            for episode in rhythm:                              # For each episode in the rhythm,
                if not self._encoder_running:                   # If the encoder is not running,
                    break                                       # Respect stop signal mid-rhythm
                time.sleep(0.01)                                # 10ms yield — prevents encoding loop from starving active cognition mid-rhythm

                # Inscribe to episodic_buffer (crash-safe record) before encoding
                # Skip if already recovered from episodic_buffer on restart
                if not episode.get("staging_id"):
                    with self._inscription_lock:                                        # With inscription lock on episodic buffer,
                        episode["staging_id"] = self._ecx.stage_engram(                 # Insert the episode into the episodic buffer
                            engram = {
                                "timestamp": episode["timestamp"],
                                "date":      episode["date"],
                                "content":   episode["content"],
                            },
                            ecx_conn = encoder_conn,
                        )
      
                # Encode the episode content into a semantic vector
                encoded_episode: list[float] = self._encoding_engine.encode_engram(episode["content"])   # Encode the episode content into a semantic vector
                if not encoded_episode:                                             # If the encoding failed,
                    # Encoding engine unavailable — skip for now, retry later
                    self.logger.warning(                                            # Log the warning message of unavailability of the encoding engine
                        f"EMC encode skipped (encoding engine unavailable): "
                        f"{len(episode['content']) // CNS.UNITS_PER_CHUNK + 1} chunks"
                    )
                    with self._episodic_buffer_lock:                                # With lock on episodic buffer,
                        self.episodic_buffer._binding_stream.appendleft(episode)    # Append the episode to the binding stream
                    continue                                                        # Continue to the next episode
    
                # Pack encoded episode vector as fp32 binary for engram storage
                encoding_blob = pack_vector(encoded_episode)                        # Pack encoded episode vector via shared MSB utility
            
                self._synaptic_consolidate(encoder_conn, episode, encoding_blob)
    
                self.logger.debug(
                    f"EMC coded and stored → episodes: {len(episode['content']) // CNS.UNITS_PER_CHUNK + 1} chunks" # Log the number of chunks in the episode
                    f" (date={episode['date']})",                                   # Log the date of the episode
                )
    
        encoder_conn.close()                                                        # Close the encoder connection
        self.logger.info("EMC encoding cycle stopped")                              # Log the stop of the EMC encoding cycle
        
    def _synaptic_consolidate(self, encoder_conn, episode: dict, encoding_blob: bytes) -> None:
        """
        Synaptic consolidation — stabilizes one encoded episode into all three
        engram indexes. Biological analogue: LTP-driven trace stabilization 
        within the hippocampus during wake encoding.
        
        Distinct from systems consolidation (EMC → SMC), which occurs
        during the M2 Dream Cycle.
     
        Inscribes:
            - episodes          : Encoded episodic memory (intermediate, retrievable)
            - engram_vectors    : Semantic vectors for L2 distance semantic search
                                (Only created if engram vector index is activated)
            - engram_lexical    : Lexical index for fast retrieval of episodic memories

        Args:
            encoder_conn : Engine connection for writing
            episode      : Episode dictionary with timestamp, date, content
            encoding_blob: Binary encoding data of the episode
        """
        with self._inscription_lock:                                            # Ensure only one thread inscribes into the engram at a time
            # Primary episodic record
            episode_id = self._ecx.inscribe_engram(                             # Insert the episode into engram
                engram={                                                        # Episode dictionary with timestamp, date, and content
                    "timestamp": episode["timestamp"],                          # Episode timestamp
                    "date":      episode["date"],                               # Episode date
                    "content":   episode["content"],                            # Episode content
                    "encoding":  encoding_blob,                                 # Episode encoding
                }, 
                ecx_conn=encoder_conn,                                          # Connection to use for the operation
            )

            # Engram vector index (vec0 KNN for semantic search)
            self._ecx.insert_vector(                                            # Insert the episode encoding into engram vectors for fast retrieval
                engram_id = episode_id,                                         # Episode ID for which to insert the encoding
                blob = encoding_blob,                                           # Binary encoding data of the episode
                ecx_conn = encoder_conn,                                        # Connection to use for the operation
            )
     
            # Engram lexical index (FTS5 index for lexical search)
            self._ecx.insert_lexical(                                           # Insert the episode content into engram lexical for fast retrieval 
                engram_id = episode_id,                                         # Episode ID for which to insert the encoding
                text = episode["content"],                                      # Raw text content of the episode
                ecx_conn = encoder_conn,                                        # Connection to use for the operation
            )
     
            # Remove the entry in episodic buffer — synaptic consolidation is complete, staging row no longer needed
            if episode.get("staging_id") is not None:                           # If the episode still staging in episodic buffer,
                self._ecx.decay_engram(                                         # Remove the episode from episodic buffer after synaptic consolidation
                    engram_id = episode["staging_id"],                          # Episode ID for which to decay the encoding
                    ecx_conn  = encoder_conn,                                   # Connection to use for the operation
                )

    def recall_episodes(self, cue: str, recall_limit: int = EMC.RECALL_LIMIT) -> list[dict]:
        """
        Recall relevant episodes from episodic memory.
        Dual-path retrieval fused via RRF — handled internally by MSB.

        Args:
            cue (str): Recall cue string
            recall_limit (int): Number of episodes to return

        Returns:
            list[dict]: Recalled episodes sorted by descending relevancy.
                        Each dict: id, timestamp, date, content, relevancy
        """
        if not cue or not cue.strip():                                          # Empty cue — nothing to recall
            return []                                                           # Return empty list if no cue
    
        recall_cue: RecallCue = self._encoding_engine.encode_cue(cue)           # Encode cue into vector and text
        return self._ecx.recall_engram(recall_cue, recall_limit)                # Recall engram using the cue

    def get_stats(self) -> dict:
        """
        Return EMC health and storage stats.

        Returns:
            dict: Statistics about EMC, including encoding engine, binding stream, engram stats
                  Empty dict on failure.
        """
        try:
           
            with self._episodic_buffer_lock:                                    # Lock the episodic buffer to obtain accurate count
                binding_pending= len(self.episodic_buffer._binding_stream)      # Count episodes in binding stream of episodic buffer
                
            engram_stats = self._ecx.get_stats(self.engram_gateway)             # Obtain stats of engram from episodic storage bank

            return {                                                            # Retures the health and storage stats of EMC
                **engram_stats,
                "binding_pending":          binding_pending,
                "engram_index_active":      self._engram_index,
                "encoding_engine_ready":    self._encoding_engine.is_available,
                "encoding_engine":          EMC.ENCODING_ENGINE,
            }
        except Exception as e:                                                  # If any error occurs,
            self.logger.error(f"EMC get_stats failed: {e}")                     # Log the failure
            return {}                                                           # Return empty dict

    def close(self) -> None:
        """
        Gracefully close EMC and the engram gateway.
        Signals encoding cycle to stop, waits for clean exit,
        then releases the engram connection.
        """
        self._encoder_running = False                   # Signal the encoder cycle to stop
        self._theta_rhythm.set()                        # Wake up the encoder cycle so it can exit cleanly
        if self._encoder_thread:                        # If encoder cycle is still running,
            self._encoder_thread.join(timeout=3.0)      # Wait for it to finish
        if self.engram:                                 # If engram is still connected,
            self.engram.close()                         # Close the connection to the engram
        self.logger.info("🗄️  EMC deactivated")         # Log the deactivation of EMC
