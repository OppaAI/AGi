"""
EMC — Episodic Memory Cortex
==============================
AuRoRA · Semantic Cognitive System (SCS)

Episodic memory layer of the CNS — "I remember that specific moment."
Stores PMTs with semantic encodings into engrams for recall using semantic search.
No expiry — 1TB NVMe means the robot remembers everything.

Responsibilities:
    - Receive evicted PMTs from MCC into crash-safe buffer (binding)
    - Encode buffered turns into semantic encodings (encoding)
    - Consolidate encoded episodes into SMC and PMC permanently (consolidation)
    - Search episodes semantically for relevant past context (recall)
    - Inject recalled episodes into MCC memory context (reinstatement)

Architecture:
    Two-table SQLite design:
        episodic_buffer  — crash-safe raw binding from WMC overflow
        episodes   — embedded, searchable episodic memory (permanent)

     
    Episodic Buffer (Baddeley's model):
        Two streams, private vs shared, mirroring biological architecture:
 
        _binding_stream  — private deque inside EMC (hippocampal binding)
                          Evicted PMTs land here first (fast RAM staging).
                          Invisible to MCC — MCC calls bind_pmt() and lets go,
                          just as the prefrontal cortex does not monitor every
                          hippocampal trace after handoff.
                          Dual-write: also written to SQLite episodic_buffer
                          simultaneously as crash-safe backstop.
                          _encoding_cycle drains _binding_stream as primary path.
                          On restart, _encoding_cycle falls back to SQLite to
                          recover any orphaned unprocessed PMTs.
 
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
        SQLite-vec L2 distance KNN search on unit-normalized vectors (cosine-equivalent)
        Falls back to Python cosine if SQLite-vec not available
        Falls back to lexical search if encoding engine unavailable

    Storage:
        SQLite WAL mode — Jetson-friendly, concurrent read/write

Terminology:
    episodic_buffer  — raw PMT binding table (crash-safe, temporary)
    encoding         — semantic encoding of a turn into a vector
    episodes         — embedded episodic memory table (permanent)
    engram           — one embedded episode (a specific remembered moment)
    relevancy        — cosine distance between query and episode encodings

Lifecycle:
    Binding → Encoding → Consolidation → Storing → Recall → Reinstatement

Public interface:
    emc.bind_pmt(speaker, content, timestamp) → bool
    emc.recall(query, top_k) → list[dict]
    emc.get_episodes_for_date(date_str) → list[dict]
    emc.buffer_pending_count() → int
    emc.get_stats() → dict
    emc.cleanup_processed_buffer(keep_days) → None
    emc.close() → None

TODO:
    M2 — consolidate turn pairs (user+assistant) into single engrams
        current: 1 buffer row → 1 episode (turn-level)
        target:  2 buffer rows → 1 episode (interaction-level)    
    M2 — put user_id instead of role in speaker column
    M2 — add date-range filtering to buffer entries and recall interface
    M2 — migrate episode_vectors to sqlite-vec ANN index (DiskANN)
         when episodes exceed ~50k — currently exact KNN is sufficient
    M2 — date-range filtering exposed through MCC recall interface
    M2 — SMC distillation trigger at 11pm reflection
"""

# System libraries
import json                                 # For serializing engram encodings to JSON (TODO: remove when implmented with sqlite-vec)
import math                                 # For relevance scoring (semantic relevancy) calculation (TODO: remove when implmented with sqlite-vec)
import sqlite3                              # For storage of episodic memory and buffer
import struct                               # For packing semantic vectors (fp32) into engram storage
import threading                            # For background encoding of engrams
from collections import deque               # For use in binding stream of episodic buffer — fast FIFO staging before encoding into engram
from dataclasses import dataclass, field    # For defining structures of engrams and episodes
from datetime import datetime               # (TODO) Replace with hrs.blc when BioLogic Clock is built
from pathlib import Path                    # For handling gateway to the engrams
from typing import Optional                 # For validating parameters

# AGi libraries
from hrs.hrp import AGi         # Import AGi homeostatic regulation parameters
EMC = AGi.CNS.EMC               # Channel for interfacing with Episodic Memory Cortex (EMC)

class _EncodingEngine:
    """
    Encoding engine for semantic encoding of episodic memories for storage and recall.
    Loads at EMC initialization - encoding engine ready for first recall.
    Primes recent encodings to avoid redundant encoding of identical or similar engrams
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
        self._core      = None                      # Initialize encoding engine core to hold the engine instance
        self._cache: dict[str, list[float]] = {}    # Initialize cache for recent encodings to avoid redundant encoding

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
        This is used by EMC to determine whether to perform semantic encoding and search,
        or to fall back to keyword search when the engine is unavailable.

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
            trace (str): The given memory trace to encode (e.g. PMT content).
            is_cue (bool): Whether the trace is a recall cue (True) or an episode to be stored(False).
                             This allows for separate caching of cue and episode encodings, which may have different patterns of repetition.

        Returns:
            list[float]: The semantic encoding vector for the input trace, 
                         or an empty list if the encoding engine is unavailable.
                         Empty list signals EMC to fall back to lexical recall.
        """
        if not self.is_available:                                                       # If encoding engine is unavailable,
            self.logger.debug(                                                          # Log the debug message about encoding engine being unavailable
                "Encoding engine unavailable — falling back to lexical recall"
            )
            return []                                                                   # Return empty list to signal that semantic recall cannot be performed

        imprint = f"{'cue' if is_cue else 'pmt'}:{hash(trace[:EMC.ENCODING_IMPRINT_LIMIT])}"  # Create a unique imprint hash and label the encoding type
        if imprint in self._cache:                                                      # If the imprint is already in the cache,
            return self._cache[imprint]                                                 # Return the encoded vector in the cache

        try:                                                                            # Attempt to encode the trace
            if is_cue:                                                                  # If the trace is a cue for memory recall,
                encoded_trace: list[float] = self._core.encode_query(trace).tolist()    # Encode the cue for memory recall
            else:                                                                       # If the trace is a memory trace to be stored,
                encoded_trace: list[float] = self._core.encode_document(trace).tolist() # Encode the memory trace for storage
            # Keep cache small — evict oldest if over the encoding cache limit
            if len(self._cache) >= EMC.ENCODING_CACHE_LIMIT:                            # If the cache is over the limit,
                decayed_imprint: str = next(iter(self._cache))                          # Retrieve the decayed imprint (oldest entry)
                del self._cache[decayed_imprint]                                        # Remove the decayed entry from the cache
            self._cache[imprint]: list[float] = encoded_trace                           # Add the new entry to the cache
            return encoded_trace                                                        # Return the encoded vector
        except Exception as e:                                                          # If encoding fails,
            self.logger.debug(f"Encoding error: {e}")                                   # Log the debug message about encoding error
            return []                                                                   # Return empty list to signal that semantic recall cannot be performed

def _semantic_match(cue: list[float], engram: list[float]) -> float:
    """
    Match a recall cue against a stored engram semantically.
    TODO: To be replaced with sqlite-vec ANN search.

    Args:
        cue    (list[float]): Encoded recall cue.
        engram (list[float]): Encoded stored engram.
    
    Returns:
        float: Semantic relevancy score (0.0 – 1.0).
    """
    if not cue or not engram or len(cue) != len(engram):                       # If either vector is empty or they have different lengths,
        return 0.0                                                             # Return 0.0 as relevancy cannot be computed
    dot: float        = sum(c * e for c, e in zip(cue, engram))                # Compute the dot product of the two vectors
    cue_mag: float    = math.sqrt(sum(c * c for c in cue))                     # Compute the magnitude of the encoded recall cue
    engram_mag: float = math.sqrt(sum(e * e for e in engram))                  # Compute the magnitude of the encoded stored engram
    return dot / (cue_mag * engram_mag) if cue_mag and engram_mag else 0.0     # Return the semantic relevancy score, or 0.0 if either magnitude is 0
    
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

    def clear_recall_stream(self) -> None:
        """Clear the recall stream before assembling a new memory context."""
        self.recall_stream.clear()                                             # Clear the content of recall stream

    def stage_single_episode(self, item: dict) -> None:
        """Stage a single recalled episode into the recall stream."""
        self.recall_stream.append(item)                                        # Stage a single recalled episode into recall stream

    def stage_episode_list(self, items: list[dict]) -> None:
        """Extend the recall stream with a list of recalled episodes."""
        self.recall_stream.extend(items)                                       # Stage a list of recalled episodes into recall stream

    def assess_recall_stream(self) -> list[dict]:
        """Return the current recall stream for memory context assembly."""
        return self.recall_stream                                              # Assess the content in recall stream
        
class EpisodicMemoryCortex:
    """
    Episodic Memory Cortex.
    Receives evicted PMT schema from WMC via MCC, persisting them as
    retrievable long-term memory. Incoming PMT schemas land in episodic_buffer first
    (crash-safe binding), then are encoded and consolidated into episodes.

    Two-table SQLite design:
        episodic_buffer  — raw PMT schemas from WMC overflow (crash-safe binding)
        episodes         — encoded, retrievable episodic memory (permanent)

    The consolidation worker runs in a separate neural thread, draining binding stream of episodic buffer →
    encoding → episodes continuously during wake without blocking the main neural thread's responses.
    
    Thread-safety: 
        Binding stream uses a threading.Lock for safety purpose.
        Consolidation cycle is isolated from the main neural thread.
        All engram writes are serialized through _write_lock.
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
                import sqlite_vec                                           # Initialize SQLite-vec for semantic search
                sqlite_vec.load(self.engram)                                # Load SQLite-vec into the engram connection
                self._engram_vector = True                                  # Activate engram vector search via SQLite-vec
                self.logger.info("✅ Activated semantic search via engram vectors") # Log the activation of engram vector search
            except Exception as e:
                self._engram_vector = False                                 # Set engram vector search as unavailable and fallback to cosine similarity
                self.logger.warning(                                        # Log the fallback to cosine similarity due to engram vector search unavailability
                    f"⚠️ Engram vector search not available, falling back to semantic search via cosine similarity\n"
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
            # Write lock — only encoding cycle writes to episodes
            self._write_lock = threading.Lock()                             # Ensure only one thread inscribes into the engram at a time

            # Set up encoding cycle
            self._encoder_running = False                                   # Indicate that encoding cycle not yet running
            self._theta_rhythm = threading.Event()                          # Initialize the theta rhythm for encoding cycle
            self._encoder_thread: Optional[threading.Thread] = None         # Initialize background neural thread
            self._start_encoder()                                           # Start encoding cycle in the background
        except RuntimeError as e:                                           # If failed to initialize the encoding cycle
            self.logger.error(f"❌ Encoding cycle initialization failed → {e}")   # Log the failure to initialize the encoding cycle
            raise                                                           # Raise the anomaly to the caller

        self.logger.info(f"✅ EMC initialized → {engram_gateway}")          # Log the successful initialization of the engram

    def _init_engram_schema(self) -> None:
        """
        Initialize the schema of the engram.
        This method creates the necessary tables and indexes for storing episodic memories.

        Tables:
        - episodic_buffer   : Raw PMT binding from WMC overflow (crash-safe)
        - episodes          : Encoded episodic memory (permanent, retrievable)
        - episode_vectors   : Semantic vectors for L2 distance semantic search
                              (Only created if engram vector search is activated)
        """
        self.engram.executescript("""                                       -- Execute SQL script to create the tables and indexes
            -- Raw PMT binding from WMC overflow (crash-safe, temporary)
            CREATE TABLE IF NOT EXISTS episodic_buffer (                    -- Create the episodic buffer schema
                id         INTEGER PRIMARY KEY AUTOINCREMENT,               -- Auto-incrementing index
                timestamp  TEXT    NOT NULL,                                -- Full datetime of the PMT
                date       TEXT    NOT NULL,                                -- Pre-computed date of the PMT (carries forward to episodes)
                speaker    TEXT    NOT NULL,                                -- Speaker of the PMT (TODO: M2 - user_id)
                content    TEXT    NOT NULL,                                -- Content of the PMT (truncated to fit within context window)
                processed  BOOLEAN DEFAULT FALSE                            -- Indicator if PMT has been encoded into episodic memory
            );
            CREATE INDEX IF NOT EXISTS idx_episodic_buffer_processed        -- Create index to retrieve PMTs by encoding status
                ON episodic_buffer(processed);

            -- Encoded episodic memory (permanent, retrievable)
            CREATE TABLE IF NOT EXISTS episodes (                           -- Create the episodes schema
                id         INTEGER PRIMARY KEY AUTOINCREMENT,               -- Auto-incrementing index
                timestamp  TEXT    NOT NULL,                                -- Full datetime of the episode
                date       TEXT    NOT NULL,                                -- Date of the episode
                speaker    TEXT    NOT NULL,                                -- Speaker of the episode (TODO: M2 - user_id)
                content    TEXT    NOT NULL,                                -- Content of the episode
                encoding   BLOB    NOT NULL,                                -- Encoded content of the episode (into 768 dim vectors)
                created_at TEXT    DEFAULT (datetime('now'))                -- Timestamp of when the episode was created
            );
            CREATE INDEX IF NOT EXISTS idx_episodes_date                    -- Temporal recall axis — get_episodes_for_date() and Dream Cycle (M2)
                ON episodes(date);
            """)
        self.engram.commit()                                                # Commit the changes to the engram

        # Create episode vector virtual schema for L2 distance semantic search
        # Created separately — only if engram vector search is activated
        if self._engram_vector:                                             # If engram vector search is activated,
            self.engram.execute("""                                         # Create a virtual schema for the L2 distance semantic search
                CREATE VIRTUAL TABLE IF NOT EXISTS episode_vectors USING vec0(
                    encoding FLOAT[768]                                     -- L2 distance semantic search on unit-normalized vectors (cosine-equivalent)
                )
            """)
            self.engram.commit()                                            # Commit the changes to the engram
            self.logger.debug("EMC engram vector index initialized")        # Log the initialization of the engram vector index

    def bind_pmt(self, speaker: str, content: str, timestamp: str) -> bool:
        """
        Entry point of the EMC lifecycle. Receives a PMT evicted from WMC
        and binds it into the episodic buffer for consolidation.
    
        Called by MCC asynchronously at the WMC → EMC boundary — crash-safe, non-blocking.

        Args:
            speaker (str): User ID or assistant ID
            content (str): Content of PMT (truncated to 2000 chars for safety)
            timestamp (str): Timestamp of PMT induced into WMC

        Returns:
            bool: True on success, False on failure
        """
        pmt_date = timestamp[:10]
        pmt: dict = {"timestamp": timestamp, "date": pmt_date,
             "speaker": speaker, "content": content[:2000]}

        try:
            with self._episodic_buffer_lock:
                self.episodic_buffer._binding_stream.append(pmt)
            self.engram.execute(
                "INSERT INTO episodic_buffer (timestamp, date, speaker, content) "
                "VALUES (?, ?, ?, ?)",
                [timestamp, pmt_date, speaker, content[:2000]],
            )
            self.engram.commit()
            self._theta_rhythm.set()
            self.logger.debug(
                f"EMC buffer ← [{speaker}] {content[:40]}…"
            )
            return True
        except Exception as e:
            self.logger.warning(f"EMC binding PMT failed: {e}")
            return False

    def _start_encoder(self) -> None:
        """
        Start background encoding cycle thread.
        """
        # Prepend orphan recovery before thread start
        orphaned = self.engram.execute(
            "SELECT timestamp, date, speaker, content "
            "FROM episodic_buffer WHERE processed = FALSE ORDER BY id"
        ).fetchall()
        if orphaned:
            with self._episodic_buffer_lock:
                for row in orphaned:
                    self.episodic_buffer._binding_stream.append({
                        "timestamp": row["timestamp"],
                        "date":      row["date"],
                        "speaker":   row["speaker"],
                        "content":   row["content"],
                        "recovered": True,                              # Flag as recovered — already in SQLite, no re-insert needed
                    })
            self.logger.info(
                f"⚡ EMC recovered {len(orphaned)} orphaned PMT(s) from engram → _binding_stream"
            )
            self._theta_rhythm.set()
           
        self._encoder_running = True
        self._encoder_thread  = threading.Thread(
            target=self._encoding_cycle,
            name="emc-encoding-cycle",
            daemon=True,
        )
        self._encoder_thread.start()
        self.logger.info("🔄 EMC encoding cycle started")

    def _encoding_cycle(self):
        """
        Event-driven consolidation — drains episodic_buffer into episodes.
        Wakes only when buffer has episodes (sharp-wave ripple pattern).
        Processes a snapshot of IDs per ripple — new arrivals deferred to next cycle.
        """

        # Worker-local SQLite connection (WAL allows concurrent access)
        worker_conn = sqlite3.connect(self.engram_gateway, check_same_thread=False)
        worker_conn.row_factory = sqlite3.Row
        worker_conn.execute("PRAGMA journal_mode=WAL;")

        # Load sqlite-vec into worker connection if engram vector search is active
        if self._engram_vector:
            try:
                import sqlite_vec
                sqlite_vec.load(worker_conn)                    # sqlite-vec must be loaded per connection
            except Exception as e:
                self.logger.warning(f"⚠️ sqlite-vec load failed in encode worker: {e}")
                self._engram_vector = False                     # Disable vector search — fall back to Python cosine

        self.logger.info("⚙️ EMC consolidation cycle running…")
    
        while self._encoder_running:
            # Rest state — wait for hippocampal activation
            self._theta_rhythm.wait()
            self._theta_rhythm.clear()
    
            if not self._encoder_running:
                break  # clean exit if stopped while waiting
    
            # Snapshot IDs at this moment — one ripple, one defined window
            with self._episodic_buffer_lock:
                if not self.episodic_buffer._binding_stream:
                    continue
                snapshot: list[dict] = list(self.episodic_buffer._binding_stream)
                self.episodic_buffer._binding_stream.clear()
    
            self.logger.debug(f"EMC encoding cycle → {len(snapshot)} episode(s) in snapshot")
    
            # Replay each episode in the snapshot
            for pmt in snapshot:
                if not self._encoder_running:
                    break  # respect stop signal mid-ripple
    
                # Encode the PMT content into a semantic vector
                vec = self._encoding_engine.encode(pmt["content"], is_cue=False)
                if not vec:
                    # Encoding engine unavailable — skip for now, retry later
                    self.logger.warning(
                        f"EMC encode skipped (encoding engine unavailable): "
                        f"[{pmt['speaker']}] {pmt['content'][:40]}…"
                    )
                    with self._episodic_buffer_lock:
                        self.episodic_buffer._binding_stream.appendleft(pmt)                    
                    # Don't mark processed — retry next ripple
                    continue
    
                # Pack vector as binary float32 for BLOB storage
                encoding_blob = struct.pack(f"{len(vec)}f", *vec)
            
                with self._write_lock:
                    # Insert into episodes (engram)
                    worker_conn.execute(
                        "INSERT INTO episodes "
                        "(timestamp, date, speaker, content, encoding) "
                        "VALUES (?, ?, ?, ?, ?)",
                        [
                            pmt["timestamp"],
                            pmt["date"],
                            pmt["speaker"],
                            pmt["content"],
                            encoding_blob,
                        ],
                    )
                    # Insert into episode_vectors if engram vector search is active
                    if self._engram_vector:
                        episode_id_new = worker_conn.execute("SELECT last_insert_rowid()").fetchone()[0]
                        worker_conn.execute(
                            "INSERT INTO episode_vectors (rowid, encoding) VALUES (?, ?)",
                            [episode_id_new, encoding_blob],
                        )
                    # Mark buffer episode as processed
                    worker_conn.execute(
                        "UPDATE episodic_buffer SET processed = TRUE "
                        "WHERE timestamp = ? AND speaker = ? AND processed = FALSE",
                        [pmt["timestamp"], pmt["speaker"]],
                    )
                    worker_conn.commit()
    
                self.logger.debug(
                    f"EMC consolidated → episodes: [{pmt['speaker']}] "
                    f"{pmt['content'][:40]}… (date={pmt['date']})"
                )
    
        worker_conn.close()
        self.logger.info("EMC consolidation cycle stopped")

    def recall(
        self,
        query: str,
        top_k: int = EMC.RECALL_DEPTH,
        date_from: Optional[str] = None,
        date_to: Optional[str] = None,
    ) -> list[dict]:
        """
        Semantic search over all embedded episodes.
        Returns top-K most relevant episodes sorted by semantic relevancy.
        Falls back to cosine similarity search if engram vector search is unavailable.
        Falls back to lexical search if encoding engine is unavailable.

        Args:
            query:     Natural language recall cue
            top_k:     Number of recalled episodes to return
            date_from: Optional ISO date string lower bound (inclusive)
            date_to:   Optional ISO date string upper bound (inclusive)

        Returns:
            List of dicts: {timestamp, date, speaker, content, relevancy}
        """
        query_vec = self._encoding_engine.encode(query, is_cue=True)
        if not query_vec:
            return self._keyword_search(query, top_k, date_from, date_to)

        try:
            if self._engram_vector:
                # L2 distance KNN search via engram vector (SQLite-vec)
                query_blob = struct.pack(f"{len(query_vec)}f", *query_vec)          # Pack query vector as binary float32

                sql = """
                    SELECT e.timestamp, e.date, e.speaker, e.content, v.distance
                    FROM episode_vectors v
                    JOIN episodes e ON e.id = v.rowid
                    WHERE v.encoding MATCH ?
                      AND v.k = ?
                    ORDER BY v.distance
                """
                
                params = [query_blob, top_k]
                where = []

                if date_from:
                    where.append("e.date >= ?")
                    params.append(date_from)
                if date_to:
                    where.append("e.date <= ?")
                    params.append(date_to)
                if where:
                    sql += " AND " + " AND ".join(where)

                sql += " ORDER BY v.distance LIMIT ?"
                params.append(top_k)

                rows = self.engram.execute(sql, params).fetchall()
                if not rows:
                    return []

                self.logger.debug(
                    f"EMC vector search '{query[:30]}…' → "
                    f"{len(rows)} results "
                    f"(top dist={rows[0]['distance'] if rows else 0})"
                )

                return [
                    {
                        "timestamp": row["timestamp"],
                        "date":      row["date"],
                        "speaker":   row["speaker"],
                        "content":   row["content"][:EMC.ENGRAM_CHUNK_LIMIT],
                        "relevancy": round(1 / (1 + row["distance"]), 4),       # Inverse of distance (higher is more relevant)
                    }
                    for row in rows
                ]

            else:
                sql    = "SELECT timestamp, date, speaker, content, encoding FROM episodes"
                params = []
                where  = []

                if date_from:
                    where.append("date >= ?")
                    params.append(date_from)
                if date_to:
                    where.append("date <= ?")
                    params.append(date_to)
                if where:
                    sql += " WHERE " + " AND ".join(where)

                rows = self.engram.execute(sql, params).fetchall()
                if not rows:
                    return []

                scored = []
                for row in rows:
                    raw = row["encoding"]
                    stored_vec = list(struct.unpack(f"{len(raw) // 4}f", raw)) if raw else []
                    sim = _semantic_match(query_vec, stored_vec)
                    scored.append({
                        "timestamp":  row["timestamp"],
                        "date":       row["date"],
                        "speaker":    row["speaker"],
                        "content":    row["content"][:EMC.ENGRAM_CHUNK_LIMIT],
                        "relevancy": round(sim, 4),
                    })

                scored.sort(key=lambda x: x["relevancy"], reverse=True)
                results = scored[:top_k]

                self.logger.debug(
                    f"EMC search '{query[:30]}…' → "
                    f"{len(results)} results "
                    f"(top sim={results[0]['relevancy'] if results else 0})"
                )
                return results

        except Exception as e:
            self.logger.error(f"EMC recall failed: {e}")
            return []

    def _keyword_search(
        self,
        query: str,
        top_k: int,
        date_from: Optional[str],
        date_to: Optional[str],
    ) -> list[dict]:
        """
        Lexical recall as fallback when encoding engine is unavailable.
        Matches lexicons against episode content using LIKE queries.

        Args:
            query:     Natural language recall cue
            top_k:     Number of episodes to return
            date_from: Optional ISO date string lower bound (inclusive)
            date_to:   Optional ISO date string upper bound (inclusive)

        Returns:
            List of dicts: {"timestamp": int, "date": str, "speaker": str, "content": str, "relevancy": float}
            relevancy is always 0.0 - no semantic scoring in lexical search
        """
        words = [w for w in query.lower().split() if len(w) > 3]
        if not words:
            return []

        where  = ["(" + " OR ".join(["LOWER(content) LIKE ?" for _ in words]) + ")"]
        params = [f"%{w}%" for w in words]

        if date_from:
            where.append("date >= ?")
            params.append(date_from)
        if date_to:
            where.append("date <= ?")
            params.append(date_to)

        sql = (
            f"SELECT timestamp, date, speaker, content FROM episodes "
            f"WHERE {' AND '.join(where)} "
            f"ORDER BY timestamp DESC LIMIT {top_k}"
        )
        try:
            rows = self.engram.execute(sql, params).fetchall()
            return [
                {
                    "timestamp":  r["timestamp"],
                    "date":       r["date"],
                    "speaker":    r["speaker"],
                    "content":    r["content"][:EMC.ENGRAM_CHUNK_LIMIT],
                    "relevancy": 0.0,
                }
                for r in rows
            ]
        except Exception as e:
            self.logger.error(f"EMC keyword search failed: {e}")
            return []

    # ── Episode retrieval for a specific date ─────────────────────────────────

    def get_episodes_for_date(self, date_str: str) -> list[dict]:
        """
        Return all episodes for a given date in chronological order.
        Used by MCC for 11pm Dream Cycle reflection (M2).

        Args:
            date_str (str): ISO date string (e.g. "2026-04-05")

        Returns:
            List of dicts: {timestamp, speaker, content}
            Empty list if no episodes found or on failure.
        """
        try:
            rows = self.engram.execute(
                "SELECT timestamp, speaker, content FROM episodes "
                "WHERE date = ? ORDER BY timestamp",
                [date_str],
            ).fetchall()
            return [
                {"timestamp": r["timestamp"], "speaker": r["speaker"], "content": r["content"]}
                for r in rows
            ]
        except Exception as e:
            self.logger.error(f"EMC get_episodes_for_date failed: {e}")
            return []

    # ── Buffer status ─────────────────────────────────────────────────────────

    def buffer_pending_count(self) -> int:
        """
        Return number of PMTs pending consolidation.
        Combines binding stream (RAM) and episodic buffer (unprocessed)
        for a complete picture of pending encoding work.
 
        Returns:
            int: Total number of PMTs pending encoding, 0 on failure.
        """
        try:
            with self._episodic_buffer_lock:
                ram_pending = len(self.episodic_buffer._binding_stream)
            sql_pending = self.engram.execute(
                 "SELECT COUNT(*) FROM episodic_buffer WHERE processed = FALSE"
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
                buffer_total, buffer_pending, intake_pending,
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
                "SELECT COUNT(*) as total, "
                "SUM(CASE WHEN processed=FALSE THEN 1 ELSE 0 END) as pending "
                "FROM episodic_buffer"
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
                "buffer_pending":           buf_row["pending"]  if buf_row else 0,
                "binding_pending":          binding_pending,
                "db_size_mb":               db_size_mb,
                "encoding_engine_ready":    self._encoding_engine.is_available,
                "engram_vector_active":     self._engram_vector,
                "encoding_engine":          EMC.ENCODING_ENGINE,
            }
        except Exception as e:
            self.logger.error(f"EMC get_stats failed: {e}")
            return {}

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def cleanup_processed_buffer(self, keep_days: int = 1) -> None:
        """
        Remove processed episodic_buffer rows older than keep_days.
        Safe to call periodically — processed rows are already consolidated into episodes.

        Args:
            keep_days (int): Number of days to retain processed rows (default: 1)
        """
        try:
            self.engram.execute(
                "DELETE FROM episodic_buffer "
                "WHERE processed = TRUE "
                "AND date < date('now', ?)",
                [f"-{keep_days} days"],
            )
            self.engram.commit()
            self.logger.debug("EMC buffer cleanup done")
        except Exception as e:
            self.logger.warning(f"EMC buffer cleanup failed: {e}")

    def close(self) -> None:
        """
        Gracefully close EMC and the engram gateway.
        Signals encoding cycle to stop, waits for clean exit,
        then releases the engram connection.
        """
        self._encoder_running = False
        self._theta_rhythm.set()                    # Wake up the encoder thread so it can exit cleanly
        if self._encoder_thread:
            self._encoder_thread.join(timeout=3.0)
        if self.engram:
            self.engram.close()
        self.logger.info("🗄️  EMC closed")
