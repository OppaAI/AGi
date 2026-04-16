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
    emc.buffer_pending_count() → int
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
import math                                 # For relevance scoring (semantic relevancy) calculation (TODO: remove when implemented with sqlite-vec)
import os                                   # For process priority adjustment
import sqlite3                              # For storage of episodic memory and buffer
import struct                               # For packing semantic vectors (fp32) of episodes into engram
import threading                            # For background encoding of episodes
import time
from collections import deque               # For use in binding stream of episodic buffer — fast FIFO staging before encoding into engram
from dataclasses import dataclass, field    # For defining structures of episodes and engram
from datetime import datetime               # (TODO) Replace with hrs.blc when BioLogic Clock is built
from pathlib import Path                    # For handling gateway to the engram
from typing import Optional                 # For validating parameters

# AGi libraries
from hrs.hrp import AGi         # Import AGi homeostatic regulation parameters
CNS = AGi.CNS                   # Channel for interfacing with Central Nervous System (CNS)
EMC = AGi.CNS.EMC               # Channel for interfacing with Episodic Memory Cortex (EMC)

class _EncodingEngine:
    """
    Encoding engine for semantic encoding of episodic memories for storage and recall.
    Loads at EMC initialization - encoding engine ready for first recall.
    Primes recent encodings to avoid redundant encoding of identical or similar episodes
    and to speed up subsequent recall.
    """

    def __init__(self, logger) -> None:
        """
        Initialize the encoding engine with a logger.
        This method sets up the encoding engine core and cache for recent encodings.
        
        Args:
            logger: Logger instance for logging encoding engine operations
        """
        self.logger     = logger                    # Retrieve logger from CNC for logging EMC operations
        self._core      = None                      # For holding the encoding engine instance for semantic encoding
        self._cache: dict[str, list[float]] = {}    # For holding the cache of recent encodings to avoid redundant encoding

        try:                                                                        # Attempt to activate the encoding engine
            from sentence_transformers import SentenceTransformer                   # Load inferencing component of encoding engine
            self.logger.info("⏳ Activating Encoding Engine…")                      # Log the start of encoding engine activation
            self._core = SentenceTransformer(EMC.ENCODING_ENGINE)                   # Activate encoding engine defined in homeostatic Regulation parameters
            self.logger.info("✅ Encoding Engine activated")                        # Log the successful activation of encoding engine
        except ImportError:                                                         # If missing the inferencer component of encoding engine,
            self.logger.warning(                                                    # Log the warning about encoding engine being offline and falling back to lexical retrieval
                "⚠️ Encoding Engine offline - missing inferencing component.\n"
                "   EMC falling back to lexical recall.\n"
                "   Note to technician: pip3 install sentence-transformers --break-system-packages"
            )
        except Exception as e:                                                    # If other errors during activation,
            self.logger.warning(f"⚠️ Encoding Engine activation failed: {e}")     # Log the error during activation of encoding engine

    @property
    def is_available(self) -> bool:
        """
        Check if encoding engine is loaded successfully and ready for encoding.
        This is used by EMC to determine whether to perform semantic search is available,
        or to fall back to solely lexical search when the engine is unavailable.

        Returns:
            bool: True if ready for encoding, False if failed to load (e.g. missing inferencing component).
        """
        return self._core is not None            # Encoding engine is available if the core was successfully loaded during initialization

    def encode(self, trace: str, is_cue: bool = False) -> list[float]:
        """
        Encode the given memory trace into a semantic vector for storage or recall.
        Uses caching to avoid redundant encoding of identical or similar texts.
        Caches recent encodings to speed up subsequent recall.
        If encoding engine is unavailable, returns an empty list to signal that semantic encoding cannot be performed, 
        prompting EMC to fall back to lexical recall.
        
        Args:
            trace (str): The given memory trace to encode (e.g. episode content).
            is_cue (bool): Whether the trace is a recall cue (True) or an episode to be stored(False).
                             This allows for separate caching of cue and episode encodings, which may have different patterns of repetition.

        Returns:
            list[float]: The semantic encoding vector for the input trace, 
                         or an empty list if the encoding engine is unavailable.
                         Empty list signals EMC to fall back to lexical recall.
        """
        if not self.is_available:                                                       # If encoding engine is unavailable,
            self.logger.debug(                                                          # Log the debug message about encoding engine being unavailable
                "Encoding engine unavailable — semantic search inactive"
            )
            return []                                                                   # Return empty list to signal that semantic recall cannot be performed

        imprint = f"{'cue' if is_cue else 'episode'}:{hash(trace[:EMC.ENCODING_IMPRINT_LIMIT])}"  # Create a unique imprint hash and label the encoding type
        if imprint in self._cache:                                                      # If the imprint is already in the cache,
            return self._cache[imprint]                                                 # Return the encoded vector in the cache

        try:                                                                            # Attempt to encode the trace
            if is_cue:                                                                  # If the trace is a cue for memory recall,
                encoded_trace: list[float] = self._core.encode_query(trace).tolist()    # Encode the cue for memory recall
            else:                                                                       # If the trace is a memory trace to be stored,
                encoded_trace: list[float] = self._core.encode_document(trace).tolist() # Encode the memory trace for storage
            encoded_trace = _normalize(encoded_trace)                                   #

            # Keep cache small — evict oldest if over the encoding cache limit
            if len(self._cache) >= EMC.ENCODING_CACHE_LIMIT:                            # If the cache is over the limit,
                decayed_imprint: str = next(iter(self._cache))                          # Retrieve the decayed imprint (oldest entry)
                del self._cache[decayed_imprint]                                        # Remove the decayed entry from the cache
            self._cache[imprint]: list[float] = encoded_trace                           # Add the new entry to the cache
            return encoded_trace                                                        # Return the encoded vector
        except Exception as e:                                                          # If encoding fails,
            self.logger.debug(f"Encoding error: {e}")                                   # Log the debug message about encoding error
            return []                                                                   # Return empty list to signal that semantic recall cannot be performed

def _normalize(v: list[float]) -> list[float]:
    """Unit-normalize a vector for cosine-equivalent L2 search."""
    mag = math.sqrt(sum(x * x for x in v))
    if abs(mag - 1.0) < 1e-6:   # already unit-normalized
        return v
    return [x / mag for x in v] if mag > 0.0 else v
    
def _semantic_search(cue: list[float], episode: list[float]) -> float:
    """
    Search a recall cue against a stored episode semantically.
    TODO: To be replaced with sqlite-vec ANN search.

    Args:
        cue     (list[float]): Encoded recall cue.
        episode (list[float]): Encoded stored episode.
    
    Returns:
        float: Semantic relevancy score (0.0 – 1.0).
    """
    if not cue or not episode or len(cue) != len(episode):                      # If either vector is empty or they have different lengths,
        return 0.0                                                              # Return 0.0 as relevancy cannot be computed
    dot: float        = sum(c * e for c, e in zip(cue, episode))                # Compute the dot product of the two vectors
    cue_mag: float    = math.sqrt(sum(c * c for c in cue))                      # Compute the magnitude of the encoded recall cue
    episode_mag: float = math.sqrt(sum(e * e for e in episode))                 # Compute the magnitude of the encoded stored episode
    return dot / (cue_mag * episode_mag) if cue_mag and episode_mag else 0.0    # Return the semantic relevancy score, or 0.0 if either magnitude is 0
    
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
        self.logger                  = logger                                   # Retrieve logger from CNC for logging EMC operations
        self.engram_gateway: str     = str(engram_gateway)                      # Retrieve engram gateway passed down from MCC
        self.episodic_buffer         = EpisodicBuffer()                         # Initialize episodic buffer — binding and recall streams for active cognition
        self._episodic_buffer_lock   = threading.Lock()                         # Lock for thread-safe access to episodic buffer
        self._encoding_engine        = _EncodingEngine(logger=logger)           # Initialize encoding engine and provide the logger from MCC

        # SQLite — WAL mode for concurrent reads during async writes
        try:                                                                # Attempt to connect to the engram
            self.engram = sqlite3.connect(engram_gateway, check_same_thread=False) # Access the engram through the provided gateway
            self.engram.row_factory = sqlite3.Row                           # Define the structure of query results of the engram
            self.engram.execute("PRAGMA journal_mode=WAL;")                 # Set up engram to allow retrieval and storing simultaneously
            self.engram.execute("PRAGMA synchronous=NORMAL;")               # Balance episodes safety vs storing speed
            self.engram.commit()                                            # Apply the above parameters into the engram

            # Set up SQLite-vec for L2 distance semantic search
            # Graceful fallback to cosine similarity if SQLite-vec not available
            try:
                import sqlite_vec                                           # Initialize SQLite-vec for engram vector index semantic search
                sqlite_vec.load(self.engram)                                # Load SQLite-vec into the engram connection
                self._engram_index = True                                   # Activate engram vector index semantic search via SQLite-vec
                self.logger.info("✅ Activated semantic search via engram vectors") # Log the activation of engram vector index
            except Exception as e:
                self._engram_index = False                                  # Set engram vector index as unavailable and fallback to cosine similarity
                self.logger.warning(                                        # Log the fallback to cosine similarity due to engram vector index unavailability
                    f"⚠️ engram vector index not available, falling back to unindexed cosine similarity\n"
                    f"   Note to technician: pip3 install sqlite-vec --break-system-packages\n"
                    f"   Reason: {e}"
                ) 

            self._init_engram_schema()                                      # Initialize the schema of the engram

        except sqlite3.Error as e:                                          # If failed to connect to the engram
            self.logger.error(f"❌ Engram connection failed → {e}")         # Log the failure to connect to the engram
            if hasattr(self, 'engram'):                                     # If engram already exists,
                self.engram.close()                                         # Close the engram connection
            raise                                                           # Raise the anomaly to the caller

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

    def _init_engram_schema(self) -> None:
        """
        Initialize the schema of the engram.
        This method creates the necessary tables and indexes for storing episodic memories.

        Episodic Buffer (2-layer)
            _binding_stream  —  transient intake of evicted PMTs from WMC overflow
            episodic_buffer  — crash-safe pre-consolidation staging of episodes
        One Table + Two Indexes SQLite design:
            episodes         — embedded, searchable episodic memory (intermediate)
            engram_vectors   — semantic vectors for L2 distance semantic search
            engram_lexical   — FTS5 lexical index for pattern separation recall
        """
        self.engram.executescript("""                                       -- Create the tables and indexes of episodic memory
            -- Evicted PMT binding from WMC overflow (crash-safe, temporary)
            CREATE TABLE IF NOT EXISTS episodic_buffer (                    -- Temporary buffer for evicted PMT schemas
                id         INTEGER PRIMARY KEY AUTOINCREMENT,               -- Auto-incrementing buffer index
                timestamp  TEXT    NOT NULL,                                -- Induction timestamp inherited from WMC
                date       TEXT    NOT NULL,                                -- Pre-computed date (carries forward to episodes on encoding)
                content    TEXT    NOT NULL                                 -- Content of the episode (truncated to engram content limit)
            );

            -- Encoded episodic memory (intermediate, retrievable)
            CREATE TABLE IF NOT EXISTS episodes (                           -- Episodic memory storage
                id         INTEGER PRIMARY KEY AUTOINCREMENT,               -- Auto-incrementing engram id
                timestamp  TEXT    NOT NULL,                                -- Full datetime of induction into WMC
                date       TEXT    NOT NULL,                                -- Pre-computed date (temporal recall axis)
                content    TEXT    NOT NULL,                                -- Raw interaction content (user prompt + AI response)
                encoding   BLOB    NOT NULL,                                -- Encoded content of the episode (into vectors of the model dimension)
                created_at TEXT    DEFAULT (datetime('now'))                -- Consolidation timestamp
            );
            CREATE INDEX IF NOT EXISTS idx_episodes_date                    -- Temporal recall axis — get_episodes_for_date() and Dream Cycle (M2)
                ON episodes(date);
            """)
        self.engram.commit()                                                # Commit the changes to the engram

        # Create episode vector virtual schema for engram vector index semantic search
        # Created separately — only if engram vector index is activated
        if self._engram_index:                                              # If engram vector index is activated,
            self.engram.execute(f"""                                        -- Create a virtual schema for engram vector index semantic search
                CREATE VIRTUAL TABLE IF NOT EXISTS engram_vectors USING vec0(
                    encoding FLOAT[{EMC.ENCODING_DIM}]                      -- L2 distance semantic search on unit-normalized vectors (cosine-equivalent)
                )
            """)
            self.engram.commit()                                            # Commit the changes to the engram
            self.logger.debug("EMC engram vector index initialized")        # Log the initialization of the engram vector index
            
        # Create episode lexical virtual schema for FTS5 lexical search
        # Created separately — always created
        self.engram.execute("""                                             -- Create a virtual schema for the lexical search using FTS5 index
            CREATE VIRTUAL TABLE IF NOT EXISTS engram_lexical USING fts5(   -- FTS5 index for lexical search
               content,                                                     -- Content of the episode
               tokenize='porter unicode61'                                  -- Tokenize the content using porter unicode61
            )
        """)
        self.engram.commit()                                                # Commit the changes to the engram
        self.logger.debug("EMC FTS5 lexical index initialized")             # Log the initialization of the FTS5 lexical index
    
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
            unencoded = self.engram.execute(                        # Query the engram for unencoded episodes
                "SELECT id, timestamp, date, content "              # Collect the index, timestamp, date, and content
                "FROM episodic_buffer "                             # All episodes in the episodic buffer is unencoded by definition
                "ORDER BY id LIMIT ? OFFSET ?",                     # Sort by id and limit the results
                [EMC.RECOVERY_BATCH_SIZE, recovery_offset]          # Process episodes by batch size
            ).fetchall()                                            # Fetch all the unencoded episodes

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

        # Connect to engram gateway with WAL mode for concurrent access
        encoder_conn = sqlite3.connect(self.engram_gateway, check_same_thread=False)    # Connect to engram gateway without thread checking
        encoder_conn.row_factory = sqlite3.Row                                          # Set the row factory to return rows as dictionaries
        encoder_conn.execute("PRAGMA journal_mode=WAL;")                                # Set the journal mode to WAL for concurrent access

        # Yield processing priority to active cognition threads
        os.nice(19)                                             # Encoding is non-latency-sensitive — defers to all active cognition threads

        # Load sqlite-vec into encoder connection if engram vector index is active
        if self._engram_index:                                  # If engram vector index is active,
            try:                                                # Attempt to activate engram vector index into encoder connection
                import sqlite_vec                               # For vector similarity search of episodic memories
                sqlite_vec.load(encoder_conn)                   # Load engram vector index into encoder connection
            except Exception as e:                              # If engram vector index fails to load
                self.logger.warning(f"⚠️ engram vector index failure to load in encoder connection: {e}") # Log the failure to load engram vector index
                self._engram_index = False                      # Disable engram vector index — fall back to cosine similarity search

        self.logger.info("⚙️ EMC encoding cycle running…")      # Log the start of encoding cycle
    
        while self._encoder_running:                            # While the encoder is running,
            # Rest state — wait for theta rhythm activation
            self._theta_rhythm.wait()                           # Wait for theta rhythm activation
            self._theta_rhythm.clear()                          # Clear the theta rhythm activation flag
    
            if not self._encoder_running:                       # If the encoder is not running,
                break                                           # Clean exit if stopped while waiting
    
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
                time.sleep(0.01)                                # 10ms yield — defers to active cognition between encodes

                # Inscribe to episodic_buffer (crash-safe record) before encoding
                # Skip if already recovered from episodic_buffer on restart
                if not episode.get("staging_id"):
                    with self._inscription_lock:                                        # With inscription lock on episodic buffer,
                        staging_id = encoder_conn.execute(                              # Insert the episode into the episodic buffer
                            "INSERT INTO episodic_buffer (timestamp, date, content) "
                            "VALUES (?, ?, ?)",
                            [episode["timestamp"], episode["date"], episode["content"]],
                        )
                        encoder_conn.commit()                                           # Commit the transaction
                        episode["staging_id"] = staging_id.lastrowid                    # Capture staging index for deletion after encoding
        
                # Encode the episode content into a semantic vector
                encoded_episode: list[float] = self._encoding_engine.encode(episode["content"], is_cue=False)   # Encode the episode content into a semantic vector
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
                encoding_blob = struct.pack(f"{len(encoded_episode)}f", *encoded_episode)   # Pack encoded episode vector as fp32 binary for engram storage
            
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
            engram_id = encoder_conn.execute(                                                   # Insert the episode into engram
                "INSERT INTO episodes (timestamp, date, content, encoding) VALUES (?,?,?,?)",   # Insert the episode into engram
                [episode["timestamp"], episode["date"], episode["content"], encoding_blob],     # With the episode timestamp, date, content, and encoding
            )
            episode_id = engram_id.lastrowid                                                    # Get the ID of the inscribed episode
     
            # Engram vector index (vec0 KNN for semantic search)
            if self._engram_index:                                                              # Only create if engram vector index is activated
                encoder_conn.execute(                                                           # Insert the episode encoding into engram vectors for fast retrieval
                    "INSERT INTO engram_vectors (rowid, encoding) VALUES (?,?)",                # Insert the episode encoding into engram vectors
                    [episode_id, encoding_blob],                                                # With the episode ID and encoding
                )
     
            # Engram lexical index (FTS5 index for lexical search)
            # rowid must match episodes.id so JOIN works during recall
            encoder_conn.execute(                                                               # Insert the episode content into engram lexical for fast retrieval
                "INSERT INTO engram_lexical (rowid, content) VALUES (?,?)",                     # Insert the episode content into engram lexical
                [episode_id, episode["content"]],                                               # With the episode ID and content
            )
     
            # Remove the entry in episodic buffer — synaptic consolidation is complete, staging row no longer needed
            if episode.get("staging_id") is not None:                                           # If the episode still staging in episodic buffer,
                encoder_conn.execute(                                                           # Remove the episode from episodic buffer after synaptic consolidation
                    "DELETE FROM episodic_buffer WHERE id=?",                                   # Remove the episode from episodic buffer
                    [episode["staging_id"]],                                                    # Using the episode ID
                )
            encoder_conn.commit()                                                               # Commit the transaction
        
    def recall_episodes(self, query: str, recall_limit: int = 5) -> list[dict]:
        """
        Recall relevant episodes from episodic memory.
        Runs semantic (engram vector similarity) and lexical (full-text pattern matching)
        retrieval in parallel, fusing results via memory convergence scoring (RRF).
     
        Fallback chain:
            semantic + lexical  → RRF fusion          (both paths available)
            semantic only       → semantic results     (FTS5 unavailable)
            lexical only        → lexical results      (encoding engine unavailable)
            neither             → []                   (full degraded state)
     
        Args:
            query  : Recall cue string
            recall_limit: Number of episodes to return (default 5)
     
        Returns:
            list[dict]: list of recalled episodes, sorted by descending relevancy.
                        Each dict: id, timestamp, date, content, relevancy
        """
        if not query or not query.strip():                                                      # If the query is empty or without meaningful content,
            return []                                                                           # Do not recall any episodes
     
        semantic_results : list[dict] = []                                                      # For storing episodes matching semantic search
        lexical_results  : list[dict] = []                                                      # For storing episodes matching lexical search
     
        # PATH 1: Semantic search via engram vector similarity
        if self._encoding_engine.is_available:                                                  # If the encoding engine is available,
            cue_vector: list[float] = self._encoding_engine.encode(query, is_cue=True)          # Encode the query as a vector
     
            if cue_vector:                                                                      # If the query was encoded successfully,
                if self._engram_index:                                                          # And if the engram vector index is available,
                    # Primary: sqlite-vec L2 KNN (cosine-equivalent on unit-normalized vectors)
                    try:                                                                        # Attempt to use the engram vector index
                        cue_blob = struct.pack(f"{len(cue_vector)}f", *cue_vector)              # Pack the cue vector as a binary blob
                        semantic_candidates = self.engram.execute(                              # Query the engram vector index and retrieve the top recall_limit * 2 episodes
                            """
                            SELECT e.id, e.timestamp, e.date, e.content,                        -- Retrieve the episode ID, timestamp, date, and content
                                   ev.distance                                                  -- Retrieve the semantic similarity between the cue and the episode
                            FROM engram_vectors ev                                              -- From the engram vector index virtual table
                            JOIN episodes e ON e.id = ev.rowid                                  -- Join with the episodes table using the row ID
                            WHERE ev.encoding MATCH ?                                           -- Match the cue vector
                            ORDER BY ev.distance                                                -- Order by ascending order of semantic similarity (closest first)
                            LIMIT ?                                                             -- Limit to recall_limit * 2 episodes
                            """,
                            [cue_blob, recall_limit * 2],                                       # Retrieve the top recall_limit * 2 episodes
                        ).fetchall()                                                            # Fetch all matching episode candidates
                        if semantic_candidates:                                                 # If there are matching episodes,
                            for i, candidate in enumerate(semantic_candidates):                 # Iterate through each matching episode
                                semantic_results.append({                                       # Add the matching episode to the semantic results
                                    "id"        : candidate["id"],                              # Add the episode ID
                                    "timestamp" : candidate["timestamp"],                       # Add the episode timestamp (timestamp memory was induced)
                                    "date"      : candidate["date"],                            # Add the episode date (date memory was induced)
                                    "content"   : candidate["content"],                         # Add the episode content
                                    "relevancy" : 1.0 / (1.0 + candidate["distance"]),          # Add the semantic similarity score
                                    "_rank"     : i,                                            # Add the zero-indexed rank for memory convergence (0=closest, 1=second closest, etc.)
                                })
                    except Exception as e:                                                      # If the engram vector index query fails,
                        self.logger.debug(f"EMC engram vector index query failed, falling back to cosine: {e}") # Log the error
     
                if not semantic_results:                                                        # When engram vector index is unavailable or semantic search returned no candidates
                    # Fallback: Cosine similarity
                    try:                                                                        # Attempt to use cosine similarity
                        # Stratified sampling — half recent, half oldest
                        # Bounds memory usage and ensures old memories score alongside recent ones
                        half_pool = (recall_limit * EMC.RECALL_POOL) // 2
                        rows = self.engram.execute(
                            "SELECT id, timestamp, date, content, encoding FROM episodes "
                            "ORDER BY rowid DESC LIMIT ? "
                            "UNION ALL "
                            "SELECT id, timestamp, date, content, encoding FROM episodes "
                            "ORDER BY rowid ASC LIMIT ?",
                            [half_pool, half_pool],     
                        ).fetchall()

                        scored = []

                        for row in rows:            
                            expected_bytes = len(cue_vector) * 4
                            if len(row["encoding"]) != expected_bytes:
                                continue      # skip mismatched dimension — stale encoding from old model
                            engram_vec = list(struct.unpack(
                                f"{len(cue_vector)}f",
                                row["encoding"]
                            ))
                            score = _semantic_search(cue_vector, engram_vec)
                            if score > 0.0:
                                scored.append((score, row))
                        scored.sort(key=lambda x: x[0], reverse=True)
                        for i, (score, row) in enumerate(scored[:recall_limit * 2]):
                            semantic_results.append({
                                "id"        : row["id"],
                                "timestamp" : row["timestamp"],
                                "date"      : row["date"],
                                "content"   : row["content"],
                                "relevancy" : score,
                                "_rank"     : i,
                            })
                    except Exception as e:
                        self.logger.debug(f"EMC cosine fallback failed: {e}")   
     
        # ── PATH 2: Lexical — dentate gyrus pattern separation ────────────────
        try:
            lexical_results = self._lexical_search(query, recall_limit)
        except Exception as e:
            self.logger.debug(f"EMC lexical path failed: {e}")
     
        # ── CONVERGENCE: RRF fusion through CA1 ───────────────────────────────
        if semantic_results and lexical_results:
            # Both paths active — full memory convergence
            fused = self._memory_convergence(semantic_results, lexical_results, recall_limit)
            self.logger.debug(
                f"EMC recall → semantic:{len(semantic_results)} "
                f"lexical:{len(lexical_results)} fused:{len(fused)}"
            )
            return fused
     
        if semantic_results:
            # Lexical unavailable — return semantic only, capped to recall_limit
            results = sorted(semantic_results, key=lambda x: x["relevancy"], reverse=True)
            for r in results:
                r.pop("_rank", None)
            return results[:recall_limit]
     
        if lexical_results:
            # Encoding engine unavailable — return lexical only, capped to recall_limit
            results = sorted(lexical_results, key=lambda x: x["relevancy"], reverse=True)
            for r in results:
                r.pop("_rank", None)
            return results[:recall_limit]
     
        self.logger.debug("EMC recall → no results from either path")
        return []

    def _sanitize_fts_query(self, query: str) -> str:
        """
        Sanitize raw query string for safe FTS5 MATCH usage.
        Quotes each token to treat them as literal terms, neutralizing
        FTS5 operators (*, -, ", parentheses, AND, OR, NOT).
        """
        tokens = query.strip().split()
        return " ".join(f'"{t}"' for t in tokens if t)
    
    def _lexical_search(self, query: str, recall_limit: int) -> list[dict]:
        """
        Lexical pattern separation search via FTS5.
        Dentate gyrus analogue — precise, keyword-driven engram retrieval.
    
        FTS5 rank() is negative (more negative = better). We normalise to
        a 0.0–1.0 relevancy score so RRF can treat both paths uniformly.
    
        Args:
            query   : Raw recall cue string (not encoded — FTS5 works on text)
            recall_limit   : Maximum candidates to return before RRF fusion
    
        Returns:
            list[dict] with keys: id, timestamp, date, content, relevancy
                       sorted best-first (highest relevancy first)
        """
        safe_query = self._sanitize_fts_query(query)
        if not safe_query:
            return []
    
        try:
            # Fetch 2× recall_limit candidates — RRF will cull to recall_limit after fusion   
            rows = self.engram.execute(
                """
                SELECT e.id, e.timestamp, e.date, e.content,
                       fts.rank AS raw_rank
                FROM engram_lexical fts
                JOIN episodes e ON e.id = fts.rowid
                WHERE engram_lexical MATCH ?
                ORDER BY fts.rank          -- most negative = best
                LIMIT ?
                """,
                [safe_query, recall_limit * 2],
            ).fetchall()
    
            if not rows:
                return []
    
            # Normalise raw FTS5 rank to 0.0–1.0 relevancy
            # raw_rank is negative; least negative = worst; most negative = best
            raw_scores = [abs(row["raw_rank"]) for row in rows]
            max_score  = max(raw_scores) if raw_scores else 1.0
    
            results = []
            for i, row in enumerate(rows):
                results.append({
                    "id"        : row["id"],
                    "timestamp" : row["timestamp"],
                    "date"      : row["date"],
                    "content"   : row["content"],
                    "relevancy" : abs(row["raw_rank"]) / max_score if max_score else 0.0,
                    "_rank"     : i,   # 0-based rank for RRF (best = 0)
                })
            return results
    
        except Exception as e:
            self.logger.debug(f"EMC lexical search failed: {e}")
            return []
        
    def _memory_convergence(
        self,
        semantic_results : list[dict],
        lexical_results  : list[dict],
        recall_limit     : int,
        k                : int = 60,
    ) -> list[dict]:
        """
        Memory convergence via Reciprocal Rank Fusion (RRF).
        Semantic search (vec KNN / cosine) + lexical search (FTS5)
        → unified engram salience ranking through memory convergence (RRF).
     
        RRF formula per engram:
            rrf_score = 1/(k + rank_semantic) + 1/(k + rank_lexical)
     
        Engrams appearing in only one list get a rank of (list_length + 1)
        in the missing list — a mild penalty, not a full exclusion.
        This mirrors biology: a memory with strong semantic match but zero
        lexical match is still a valid recall candidate.
     
        Args:
            semantic_results : Ranked list from semantic search (vec KNN / cosine)
            lexical_results  : Ranked list from lexical search (FTS5)
            recall_limit     : Final number of engrams to return
            k                : RRF constant (default 60, standard in literature)
     
        Returns:
            list[dict]: list of recall_limit engrams sorted by descending RRF score,
                        with 'relevancy' set to normalised RRF score (0.0–1.0)
        """
        # Build rank lookup by episode id — 0-based, best = 0
        sem_rank: dict[int, int] = {}
        episode_lookup: dict[int, dict] = {}
        for i, r in enumerate(semantic_results):
            sem_rank[r["id"]] = i
            episode_lookup[r["id"]] = r
        
        lex_rank: dict[int, int] = {}
        for i, r in enumerate(lexical_results):
            lex_rank[r["id"]] = i
            episode_lookup.setdefault(r["id"], r)
        
        sem_miss = len(semantic_results)
        lex_miss = len(lexical_results)
        
        # Union of all candidate episode ids
        all_ids = set(sem_rank) | set(lex_rank)
     
        # Compute RRF score for each candidate
        scored: list[tuple[float, int]] = []  # (rrf_score, episode_id)
        for eid in all_ids:
            sr = sem_rank.get(eid, sem_miss)
            lr = lex_rank.get(eid, lex_miss)
            rrf = 1.0 / (k + sr) + 1.0 / (k + lr)
            scored.append((rrf, eid))
     
        # Sort descending by RRF score
        scored.sort(key=lambda x: x[0], reverse=True)
     
        # Normalise RRF scores to 0.0–1.0 for the 'relevancy' field
        max_rrf = scored[0][0] if scored else 1.0
     
        fused = []
        for rrf_score, eid in scored[:recall_limit]:
            ep = dict(episode_lookup[eid])  # copy — don't mutate cached result
            ep["relevancy"] = rrf_score / max_rrf
            ep.pop("_rank", None)           # remove internal rank field
            fused.append(ep)
     
        return fused
    
     # ── Episode retrieval for a specific date ─────────────────────────────────

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
        try:
            rows = self.engram.execute(
                "SELECT timestamp, content FROM episodes "
                "WHERE date = ? ORDER BY timestamp",
                [date_str],
            ).fetchall()
            return [
                {"timestamp": r["timestamp"], "content": r["content"]}
                for r in rows
            ]
        except Exception as e:
            self.logger.error(f"EMC get_episodes_for_date failed: {e}")
            return []

    # ── Buffer status ─────────────────────────────────────────────────────────

    def buffer_pending_count(self) -> int:
        """
        Return number of episodes pending encoding.
        Combines epsodes in binding stream and episodic buffer (unencoded)
        for a complete picture of pending encoding work.
 
        Returns:
            int: Total number of PMTs pending encoding, 0 on failure.
        """
        try:
            with self._episodic_buffer_lock:
                ram_pending = len(self.episodic_buffer._binding_stream)
            sql_pending = self.engram.execute(
                 "SELECT COUNT(*) FROM episodic_buffer"
            ).fetchone()[0]
            return ram_pending + sql_pending
        except Exception:
            return 0

    # ── Stats ─────────────────────────────────────────────────────────────────

    def get_stats(self) -> dict:
        """
        Return EMC health and storage stats.

        Returns:
            dict: {
                episodes, oldest_episode, newest_episode,
                buffer_total, binding_pending,
                db_size_mb, encoding_engine_ready,
                engram_vector_active, encoding_engine
            }
            Empty dict on failure.
        """
        try:
            ep_row = self.engram.execute(
                "SELECT COUNT(*) as total, "
                "MIN(date) as oldest, MAX(date) as newest "
                "FROM episodes"
            ).fetchone()

            buf_row = self.engram.execute(
                "SELECT COUNT(*) as total FROM episodic_buffer"
            ).fetchone()
            
            with self._episodic_buffer_lock:
                binding_pending= len(self.episodic_buffer._binding_stream)
                
            db_size_mb = round(Path(self.engram_gateway).stat().st_size / 1_048_576, 2) \
                if Path(self.engram_gateway).exists() else 0.0

            return {
                "episodes":                 ep_row["total"]  if ep_row  else 0,
                "oldest_episode":           ep_row["oldest"] if ep_row  else None,
                "newest_episode":           ep_row["newest"] if ep_row  else None,
                "buffer_total":             buf_row["total"]   if buf_row else 0,
                "binding_pending":          binding_pending,
                "db_size_mb":               db_size_mb,
                "encoding_engine_ready":    self._encoding_engine.is_available,
                "engram_index_active":      self._engram_index,
                "encoding_engine":          EMC.ENCODING_ENGINE,
            }
        except Exception as e:
            self.logger.error(f"EMC get_stats failed: {e}")
            return {}

    def close(self) -> None:
        """
        Gracefully close EMC and the engram gateway.
        Signals encoding cycle to stop, waits for clean exit,
        then releases the engram connection.
        """
        self._encoder_running = False
        self._theta_rhythm.set()                        # Wake up the encoder thread so it can exit cleanly
        if self._encoder_thread:
            self._encoder_thread.join(timeout=3.0)
        if self.engram:
            self.engram.close()
        self.logger.info("🗄️  EMC closed")
