"""
EMC — Episodic Memory Cortex
==============================
AuRoRA · Semantic Cognitive System (SCS)

Episodic memory layer of the CNS — "I remember that specific moment."
Stores memory episodes with semantic encodings into engrams for recall using semantic search.
No expiry — 1TB NVMe means the robot remembers everything.

Responsibilities:
    - Receive evicted PMTs from MCC into crash-safe buffer (binding)
    - Encode buffered turns into semantic encodings (encoding)
    - Store encoded episodes into SMC and PMC intermediately for future consolidation into LTM
    - Search episodes via semantical and lexical search for relevant past context (recall)
    - Inject recalled episodes into MCC memory context (reinstatement)

Architecture:
    Episodic Buffer (2-layer)
        _binding_stream  —  transient intake of evicted PMTs from WMC overflow
        episodic_buffer  — crash-safe pre-consolidation staging of episodes
    One Table + Two Indexes SQLite design:
        episodes         — embedded, searchable episodic memory (intermediate)
        engram_vectors   — semantic vectors for L2 distance semantic search
        engram_lexical   — FTS5 lexical index for pattern separation recall

     
    Episodic Buffer (Baddeley's model):
        Two streams, private vs shared, mirroring biological architecture:
 
        _binding_stream  — private deque inside EMC (cf. hippocampal binding)
                        Evicted PMTs land here first (fast RAM staging).
                        Invisible to MCC — MCC calls bind_pmt() and lets go,
                        just as the prefrontal cortex does not monitor every
                        hippocampal trace after handoff.
                        _run_encoding_cycle() drains _binding_stream, writes to
                        episodic_buffer (SQLite) before encoding, then stores
                        into engram as episodes. On restart, Unencoded episodes in
                        episodic_buffer are recovered back into _binding_stream
                        by _init_encoding_cycle() before the cycle starts.
 
        recall_stream   — shared on EpisodicBuffer (cross-layer)
                          Recalled episodes surface here before being injected
                          into MCC memory context — exactly as hippocampal recall
                          projects back into prefrontal awareness.
                          Visible to MCC for memory context assembly.
                          
    Encoding:
        Continuous during wake — online encoding, not sleep-only.
        sentence-transformers, CPU-only, zero GPU impact.
        Model configured via EMC.ENCODING_ENGINE constant.
        Biological analogue: hippocampal initial encoding during wake.
        Deep consolidation (SMC distillation) deferred to M2 Dream Cycle.

    Enccoding Cycle:
        Background thread drains episodic_buffer → encodes → episodes
        Falls back to SQLite episodic_buffer on restart for crash recovery
        Runs continuously, sleeps when buffer is empty
        Never blocks the robot's active cognition
        
    Relevancy:
        Dual-path retrieval fused via Reciprocal Rank Fusion (memory convergence):
            PATH 1 — Semantic (cf. CA3 pattern completion):
                SQLite-vec L2 distance KNN on unit-normalized vectors (cosine-equivalent)
                Falls back to Python cosine similarity if SQLite-vec not available
            PATH 2 — Lexical (cf. Dentate gyrus pattern separation):
                FTS5 porter-stemmed lexical search on episode content
        Both paths run in parallel and are fused via RRF scoring through CA1 convergence.
        Fallback chain:
            Both paths active  → RRF fusion (full memory convergence)
            Semantic only      → semantic results (encoding engine available, FTS5 unavailable)
            Lexical only       → lexical results  (encoding engine unavailable)
            Neither            → []               (full degraded state)

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
    Binding → Encoding → Synaptic Consolidation → System Consolidation → Recall → Reinstatement

Public interface:
    emc.bind_pmt(timestamp, content) → bool
    emc.recall_episode(query, recall_limit) → list[dict]
    emc.get_episodes_for_date(date_str) → list[dict]
    emc.get_stats() → dict
    emc.close() → None

TODO:
    M2 — add date-range filtering to buffer entries and recall interface
    M2 — migrate engram_vectors to sqlite-vec ANN index (DiskANN)
         when episodes exceed ~50k — currently exact KNN is sufficient
    M2 — date-range filtering exposed through MCC recall interface
    M2 — SMC distillation trigger at 11pm reflection
    M2 — add heartbeat logging during long idle periods (Dream Cycle sessions)
    M2 — add staging_id check (unencoded in episodic buffer) after consolidation (after Dream Cycle implementations)
"""

# System libraries
import os                                   # For process priority adjustment
import threading                            # For background encoding of episodes
import time                                 # For inter-encode yielding (sleep) and timing control within encoding cycle
from collections import deque               # For use in binding stream of episodic buffer — fast FIFO staging before encoding into engram
from dataclasses import dataclass, field    # For defining structures of episodes and engram
from datetime import datetime               # (TODO) Replace with hrs.blc when BioLogic Clock is built
from pathlib import Path                    # For handling gateway to the engram
from typing import Optional                 # For validating parameters

# AGi libraries
from hrs.hrp import AGi         # Import AGi homeostatic regulation parameters
CNS = AGi.CNS                   # Channel for interfacing with Central Nervous System (CNS)
EMC = AGi.CNS.EMC               # Channel for interfacing with Episodic Memory Cortex (EMC)

from scs.msb import (               # Aquire access to memory storage bank
    EngramSchema,               # Schema definition object for memory cortices
    EngramTrace,                # Trace definition object for individual engram fields
    EngramModality,             # Enumeration of valid engram field modalities
    RecallCue,                  # Recall cue definition object for memory cortices
    EncodingEngine,             # Shared encoding engine (sentence-transformers wrapper with cache)
    EngramComplex,          # Centralized engram memory storage interface
)

EMC_SCHEMA = EngramSchema(                                              # Define the engram schema for episodic memory
    storage=[
        EngramTrace(label="id", modality=EngramModality.INTEGER, essential=True),
        EngramTrace(label="timestamp", modality=EngramModality.TEXT, essential=True),
        EngramTrace(label="date", modality=EngramModality.TEXT, essential=True),
        EngramTrace(label="content", modality=EngramModality.TEXT, essential=True),
        EngramTrace(label="encoding", modality=EngramModality.BLOB, essential=True),
        EngramTrace(label="created_at", modality=EngramModality.TEXT, baseline="(datetime('now'))"),
    ],
    staging=[
        EngramTrace(label="id", modality=EngramModality.INTEGER, essential=True),
        EngramTrace(label="timestamp", modality=EngramModality.TEXT, essential=True),
        EngramTrace(label="date", modality=EngramModality.TEXT, essential=True),
        EngramTrace(label="content", modality=EngramModality.TEXT, essential=True),
    ],
    semantic_traces="encoding",
    lexical_traces=["content"],
    index_traces=["date"]
)
        
@dataclass
class EpisodicBuffer:
    """
    Episodic Buffer — shared workspace between WMC and EMC.
    Base on the concept of episodic buffer in Baddeley's Model — one buffer, two hippocampal processes.

    _binding_stream — evicted PMTs pending to be encoded into episodic memory
    recall_stream   — recalled episodes pending to surface into active cognition context
    """
    _binding_stream: deque[dict] = field(default_factory=deque)                # Evicted PMTs pending to be encoded into episodic memory
    recall_stream: list[dict] = field(default_factory=list)                    # Recalled episodes pending to surface into active cognition context

    def __post_init__(self) -> None:
        """Set up locks not defined in the episodic buffer schema."""
        self._recall_lock = threading.Lock()                                   # Guards recall stream mutations for safety in future implementation
        
    def clear_recall_stream(self) -> None:
        """Clear the recall stream before assembling a new memory context."""
        with self._recall_lock:                                                # Apply the lock to guard the process from mutations
            self.recall_stream.clear()                                         # Clear the content of recall stream

    def stage_single_episode(self, item: dict) -> None:
        """Stage a single recalled episode into the recall stream."""
        with self._recall_lock:                                                # Apply the lock to guard the process from mutations
            self.recall_stream.append(item)                                    # Stage a single recalled episode into recall stream

    def stage_episode_list(self, items: list[dict]) -> None:
        """Extend the recall stream with a list of recalled episodes."""
        with self._recall_lock:                                                # Apply the lock to guard the process from mutations
            self.recall_stream.extend(items)                                   # Stage a list of recalled episodes into recall stream

    def assess_recall_stream(self) -> list[dict]:
        """Return the current recall stream for memory context assembly."""
        with self._recall_lock:                                                # Apply the lock to guard the process from mutations
            return list(self.recall_stream)                                    # Assess the stable copy of the content in recall stream
        
class EpisodicMemoryCortex:
    """
    Episodic Memory Cortex.
    Receives evicted PMT schema from WMC via MCC, persisting them as
    retrievable long-term memory. Incoming PMT schemas land in episodic_buffer first
    (crash-safe binding), then are encoded and stored into engram as episodes.

    Episodic Buffer (2-layer)
        _binding_stream  —  transient intake of evicted PMTs from WMC overflow
        episodic_buffer  — crash-safe pre-consolidation staging of episodes
    One Table + Two Indexes SQLite design:
        episodes         — embedded, searchable episodic memory (intermediate)
        engram_vectors   — semantic vectors for L2 distance semantic search
        engram_lexical   — FTS5 lexical index for pattern separation recall

    The encoding cycle runs in a separate neural thread, draining binding stream of episodic buffer →
    encoding → episodes continuously during wake without blocking the main neural thread's responses.
    
    Thread-safety: 
        Binding stream uses a threading.Lock for safety purpose.
        Recall stream uses a threading.Lock — guarded via recall lock.
        Encoding cycle is isolated from the main neural thread.
        All inscription into engram are serialized through inscription lock.
        SQLite WAL mode allows concurrent reads during async writes.
    """

    def __init__(self, logger, engram_gateway: str) -> None:
        """
        Initialize the Episodic Memory Cortex with a logger and engram gateway.
        
        This method sets up the engram for storing episodic memories,
        initializes the encoding engine, and starts the encoding cycle in a
        background thread.
        
        Args:
            logger              : Logger instance for logging operations
            engram_gateway (str): Path to access the engram for storing episodic memories
        """
        self.logger                  = logger                               # Retrieve logger from CNC for logging EMC operations
        self.episodic_buffer         = EpisodicBuffer()                     # Initialize episodic buffer — binding and recall streams for active cognition
        self._episodic_buffer_lock   = threading.Lock()                     # Lock for thread-safe access to episodic buffer
        self._encoding_engine        = EncodingEngine(                      # Initialize shared encoding engine from MSB
            logger          = logger,                                       # Logger instance for logging operations
            encoding_engine = EMC.ENCODING_ENGINE,                          # Identifier for the encoding engine
            cue_prefix      = EMC.RECALL_CUE_PREFIX,                        # Prefix for recall cues
            engram_prefix   = EMC.RECALL_ENGRAM_PREFIX,                     # Prefix for engram identifiers
            prime_limit     = EMC.PRIME_LIMIT,                              # Limit for prime numbers
            prime_key_limit = EMC.PRIME_KEY_LIMIT                           # Limit for prime keys
        )                                                                      

        self._ecx = EngramComplex(                                          # Acquire access to the engram complex
            logger       = self.logger,                                     # Logger instance for logging operations
            cortex       = "EMC",                                           # Cortex identifier
            gateway      = engram_gateway,                                  # Path to the engram complex
            schema       = EMC_SCHEMA,                                      # Schema for episodic memories
            dim          = EMC.ENCODING_DIM,                                # Dimension for episodic memories
        )

        try:                                                                # Attempt to initialize the write lock
            # Inscription lock — only encoding cycle inscribes into episodes
            self._inscription_lock = threading.RLock()                      # Ensure only one thread inscribes into the engram at a time

            # Set up encoding cycle
            self._encoder_running = False                                   # Indicate that encoding cycle not yet running
            self._theta_rhythm = threading.Event()                          # Initialize the theta rhythm for encoding cycle
            self._encoder_thread: Optional[threading.Thread] = None         # Initialize background neural thread
            self._init_encoding_cycle()                                     # Start encoding cycle in the background
        except RuntimeError as e:                                           # If failed to initialize the encoding cycle
            self.logger.error(f"❌ Encoding cycle initialization failed → {e}")   # Log the failure to initialize the encoding cycle
            raise                                                           # Raise the anomaly to the caller

        self.logger.info(f"✅ EMC initialized → {engram_gateway}")          # Log the successful initialization of the engram
   
    def bind_pmt(self, timestamp: str, content: str) -> bool:
        """
        Entry point of the EMC lifecycle. Receives a PMT evicted from WMC
        and binds it into the episodic buffer as episode for encoding and storing into engram.
    
        Called by MCC asynchronously at the WMC → EMC boundary — crash-safe, non-blocking.

        Args:
            timestamp (str): Timestamp of interaction induced into WMC
            content (str): Content of interaction (truncated to engram content limit for safety)

        Returns:
            bool: True on success, False on failure
        """
        episode: dict = {                                                           # Package the evicted PMT data into episode
            "timestamp": timestamp,                                                 # Timestamp of PMT induced into WMC
            "date":      timestamp[:10],                                            # Date of PMT induced into WMC
            "content":   content[:EMC.ENGRAM_CONTENT_LIMIT]                         # Truncate content to the engram content limit
        }

        try:                                                                        # Attempt to bind the evicted PMT into episodic buffer
            with self._episodic_buffer_lock:                                        # Acquire the lock to prevent race conditions
                self.episodic_buffer._binding_stream.append(episode)                # Bind the episode to the binding stream
            self._theta_rhythm.set()                                                # Set the theta rhythm to trigger encoding cycle
            self.logger.debug(                                                      # Log the binding of the evicted PMT into episodic buffer
                f"EMC buffer ← {len(content) // CNS.UNITS_PER_CHUNK + 1} chunks"
            )
            return True                                                             # Indicate successful binding
        except Exception as e:
            self.logger.warning(f"EMC binding PMT failed: {e}")                     # Log the failure to bind the evicted PMT into episodic buffer
            return False                                                            # Report failure to bind the evicted PMT into episodic buffer

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
            unencoded = self._esb.retrieve_staged_batch(            # Query the engram for unencoded episodes
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
        Wakes only when buffer has episodes (sharp-wave ripple pattern).
        Processes a snapshot of IDs per ripple — new arrivals deferred to next cycle.
        """

        # Yield processing priority to active cognition threads
        os.nice(19)                                               # Encoding is non-latency-sensitive — defers to all active cognition threads
        encoder_conn = self._ecx.bifurcate_ecx()                  # Bifurcate a connection to the engram complex for parallel processing
        self.logger.info("⚙️ EMC encoding cycle running…")       # Log the start of encoding cycle
    
        while self._encoder_running:                              # While the encoder is running,
            # Rest state — wait for theta rhythm activation
            self._theta_rhythm.wait()                             # Wait for theta rhythm activation
            self._theta_rhythm.clear()                            # Clear the theta rhythm activation flag
    
            if not self._encoder_running:                         # If the encoder is not running,
                break                                             # Clean exit if stopped while waiting
    
            # Snapshot IDs at this moment — one ripple, one defined window
            with self._episodic_buffer_lock:                    # With lock on episodic buffer,
                if not self.episodic_buffer._binding_stream:    # If the binding stream is empty,
                    continue                                    # Skip this encoding cycle
                ripple: list[dict] = list(self.episodic_buffer._binding_stream)   # Capture point-in-time of the binding stream
                self.episodic_buffer._binding_stream.clear()    # Clear the binding stream
    
            self.logger.debug(f"EMC encoding cycle → {len(ripple)} episode(s) in ripple") # Log the number of episodes in the ripple
    
            # Replay each episode in the ripple
            for episode in ripple:                              # For each episode in the ripple,
                if not self._encoder_running:                   # If the encoder is not running,
                    break                                       # Respect stop signal mid-ripple
                time.sleep(0.01)                                # 10ms yield — prevents encoding loop from starving active cognition mid-ripple

                # Inscribe to episodic_buffer (crash-safe record) before encoding
                # Skip if already recovered from episodic_buffer on restart
                if not episode.get("staging_id"):
                    with self._inscription_lock:                                        # With inscription lock on episodic buffer,
                        episode["staging_id"] = self._ecx.stage_engram(                 # Insert the episode into the episodic buffer
                            conn=encoder_conn,
                            episode=episode,
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
                    # Retry next ripple
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
        with self._inscription_lock:                                                            # Ensure only one thread inscribes into the engram at a time
            # Primary episodic record
            episode_id = self._esb.insert_episode(encoder_conn, episode, encoding_blob)         # Insert the episode into engram

            # Engram vector index (vec0 KNN for semantic search)
            if self._engram_index:                                                              # Only create if engram vector index is activated
                self._esb.insert_vector(encoder_conn, episode_id, encoding_blob)                # Insert the episode encoding into engram vectors for fast retrieval
     
            # Engram lexical index (FTS5 index for lexical search)
            self._esb.insert_lexical(encoder_conn, episode_id, episode["content"])              # Insert the episode content into engram lexical for fast retrieval
     
            # Remove the entry in episodic buffer — synaptic consolidation is complete, staging row no longer needed
            if episode.get("staging_id") is not None:                                           # If the episode still staging in episodic buffer,
                self._esb.delete_staged(encoder_conn, episode["staging_id"])                    # Remove the episode from episodic buffer after synaptic consolidation
                
            encoder_conn.commit()                                                               # Commit the transaction
        
    def recall_episodes(self, query: str, recall_limit: int = 5) -> list[dict]:
        """
        Recall relevant episodes from episodic memory.
        Runs semantic and lexical retrieval, fusing results via RRF.

        Fallback chain:
            semantic + lexical  → RRF fusion          (both paths available)
            semantic only       → semantic results     (FTS5 unavailable)
            lexical only        → lexical results      (encoding engine unavailable)
            neither             → []                   (full degraded state)

        Args:
            query        : Recall cue string
            recall_limit : Number of episodes to return (default 5)

        Returns:
            list[dict]: Recalled episodes sorted by descending relevancy.
                        Each dict: id, timestamp, date, content, relevancy
        """
        if not query or not query.strip():                                                      # Empty cue — nothing to recall
            return []                                                                           
    
        semantic_results : list[dict] = []                                                      
        lexical_results  : list[dict] = []                                                      
    
        # PATH 1: Semantic
        if self._encoding_engine.is_available:                                                  # Encoding engine must be loaded for semantic path
            cue_vector: list[float] = self._encoding_engine.encode_cue(query)          # Encode cue — BGE instruction prefix applied internally
    
            if cue_vector:                                                                      # Empty vector means encoding failed — skip semantic path
                if self._engram_index:                                                          # sqlite-vec available — use KNN
                    try:                                                                        
                        cue_blob = pack_vector(cue_vector)                                      
                        semantic_results = self._esb.search_cosine(cue_vector, recall_limit, EMC.RECALL_POOL)
                        self.logger.debug(f"EMC semantic recall (KNN) → {len(semantic_results)} candidates")
                    except Exception as e:                                                      # KNN failed — fall through to cosine
                        self.logger.debug(f"EMC KNN failed, falling back to cosine: {e}")      
    
                if not semantic_results:                                                        # KNN unavailable or returned nothing — cosine fallback
                    try:                                                                        
                        semantic_results = self._esb.search_cosine(cue_vector, recall_limit)    
                        self.logger.debug(f"EMC semantic recall (cosine) → {len(semantic_results)} candidates")
                    except Exception as e:
                        self.logger.debug(f"EMC cosine fallback failed: {e}")   
    
        # PATH 2: Lexical
        try:
            lexical_results = self._esb.search_lexical_normalized(query, recall_limit)          
            self.logger.debug(f"EMC lexical recall → {len(lexical_results)} candidates")
        except Exception as e:
            self.logger.debug(f"EMC lexical path failed: {e}")
    
        # CONVERGENCE: RRF fusion
        if semantic_results and lexical_results:
            fused = memory_convergence(semantic_results, lexical_results, recall_limit)         
            self.logger.debug(
                f"EMC recall → semantic:{len(semantic_results)} "
                f"lexical:{len(lexical_results)} fused:{len(fused)}"
            )
            return fused
    
        if semantic_results:
            results = sorted(semantic_results, key=lambda x: x["relevancy"], reverse=True)[:recall_limit]
            for r in results:
                r.pop("_rank", None)
            self.logger.debug(f"EMC recall → semantic only ({len(results)})")
            return results
    
        if lexical_results:
            results = sorted(lexical_results, key=lambda x: x["relevancy"], reverse=True)[:recall_limit]
            for r in results:
                r.pop("_rank", None)
            self.logger.debug(f"EMC recall → lexical only ({len(results)})")
            return results
    
        self.logger.debug("EMC recall → no results from either path")
        return []

    def get_episodes_for_date(self, date_str: str) -> list[dict]:
        """
        Return all episodes for a given date in chronological order.
        Used by MCC for 11pm Dream Cycle reflection (M2).

        Args:
            date_str (str): ISO date string (e.g. "2026-04-05")

        Returns:
            List of dicts: {timestamp, content}
            Empty list if no episodes found or on failure.
        """
        return self._esb.get_episodes_for_date(date_str)                        # Obtain episodes for the given date from engram

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
                
            engram_stats = self._esb.get_stats(self.engram_gateway)             # Obtain stats of engram from episodic storage bank

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
