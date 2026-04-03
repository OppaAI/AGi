"""
EMC — Episodic Memory Cortex
==============================
AuRoRA · Semantic Cognitive System (SCS)

Episodic memory layer of the CNS — "I remember that specific moment."
Stores conversation turns with semantic embeddings, permanently.
No expiry — 1TB NVMe means the robot remembers everything.

Responsibilities:
    - Receive evicted PMTs from MCC into crash-safe buffer (binding)
    - Encode buffered turns into semantic embeddings (encoding)
    - Store embedded episodes in SQLite permanently (consolidation)
    - Search episodes semantically for relevant past context (recall)
    - Inject recalled episodes into MCC memory context (reinstatement)

Architecture:
    Two-table SQLite design:
        em_buffer  — crash-safe raw intake from WMC overflow
        episodes   — embedded, searchable episodic memory (permanent)

    Encoding:
        sentence-transformers, CPU-only, zero GPU impact
        Model configured via EMC.ENCODING_ENGINE constant

    Relevancy:
        Pure-Python cosine — no FAISS needed until millions of rows
        Falls back to keyword search if encoding engine unavailable

    Storage:
        SQLite WAL mode — Jetson-friendly, concurrent read/write

    Worker:
        Background thread drains em_buffer → encodes → episodes
        Runs continuously, sleeps when buffer is empty
        Never blocks the robot's active cognition

Terminology:
    em_buffer  — raw turn intake table (crash-safe, temporary)
    encoding   — semantic embedding of a turn into a vector
    episodes   — embedded episodic memory table (permanent)
    engram     — one embedded episode (a specific remembered moment)
    relevancy  — cosine distance between query and episode embeddings

Lifecycle:
    Binding → Encoding → Consolidation → Storing → Recall → Reinstatement

Public interface:
    emc.buffer_append(role, content, timestamp) → bool
    emc.recall(query, top_k) → list[dict]
    emc.get_episodes_for_date(date_str) → list[dict]
    emc.buffer_pending_count() → int
    emc.get_stats() → dict
    emc.cleanup_processed_buffer(keep_days) → None
    emc.close() → None

TODO:
    M2 — FAISS index for sub-millisecond search at millions of episodes
    M2 — date-range filtering exposed through MCC recall interface
    M2 — SMC distillation trigger at 11pm reflection
"""

# System libraries
import json                     # For serializing engram encodings to JSON
import math                     # For relevance scoring (semantic relevancy) calculation
import sqlite3                  # For storage of episodic memory and buffer
import threading                # For background encoding of engrams
from datetime import datetime   # (TODO) Replace with hrs.blc when BioLogic Clock is built
from pathlib import Path        # For handling gateway to the engrams
from typing import Optional     # For validating parameters

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

    def __init__(self, logger):
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
        except Exception as exc:                                                    # If other errors during activation,
            self.logger.warning(f"⚠️ Encoding Engine activation failed: {exc}")     # Log the error during activation of encoding engine

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
            list[float]: The semantic embedding vector for the input trace, 
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
                decayed_imprint = next(iter(self._cache))                               # Retrieve the decayed imprint (oldest entry)
                del self._cache[decayed_imprint]                                        # Remove the decayed entry from the cache
            self._cache[imprint] = encoded_trace                                        # Add the new entry to the cache
            return encoded_trace                                                        # Return the encoded vector
        except Exception as exc:                                                        # If encoding fails,
            self.logger.debug(f"Encoding error: {exc}")                                 # Log the debug message about encoding error
            return []                                                                   # Return empty list to signal that semantic recall cannot be performed

def _semantic_match(cue: list[float], engram: list[float]) -> float:
    """
    Match a recall cue against a stored engram semantically.

    Args:
        cue    (list[float]): Encoded recall cue.
        engram (list[float]): Encoded stored engram.
    
    Returns:
        float: Semantic relevancy score (0.0 – 1.0).
    """
    # TODO: migrate to FAISS when recall set exceeds ~10k vectors
    if not cue or not engram or len(cue) != len(engram)                        # If either vector is empty or they have different lengths,
        return 0.0                                                             # Return 0.0 as relevancy cannot be computed
    dot: float        = sum(c * e for c, e in zip(cue, engram))                # Compute the dot product of the two vectors
    cue_mag: float    = math.sqrt(sum(c * c for c in cue))                     # Compute the magnitude of the encoded recall cue
    engram_mag: float = math.sqrt(sum(e * e for e in engram))                  # Compute the magnitude of the encoded stored engram
    return dot / (cue_mag * engram_mag) if cue_mag and engram_mag else 0.0     # Return the semantic relevancy score, or 0.0 if either magnitude is 0

class EpisodicMemoryCortex:
    """
    Episodic Memory Cortex.
    Receives evicted PMT schema from WMC via MCC, persisting them as
    retrievable long-term memory. Incoming PMT schemas land in em_buffer first
    (crash-safe intake), then are encoded and consolidated into episodes.

    Two-table SQLite design:
        em_buffer  — raw PMT schemas from WMC overflow (crash-safe intake)
        episodes   — encoded, retrievable episodic memory (permanent)

    The consolidation worker runs in a separate neural thread, draining em_buffer →
    encoding → episodes continuously without blocking the main neural thread's responses.
    Thread-safety: Consolidation worker is isolated from the main neural thread.
    All em_buffer writes are serialized through SQLite's WAL mode.
    """

    def __init__(self, logger, engram_gateway: str):
        self.logger              = logger                                # Retrieve logger from CNC for logging WMC operations
        self.engram_gateway: str = engram_gateway                        # Retrieve engram gateway passed down from MCC

        # Encoding engine
        self._encoding_engine = _EncodingEngine(logger=logger)           # Initialize encoding engine and provide the logger from MCC

        # SQLite — WAL mode for concurrent reads during async writes
        self.engram = sqlite3.connect(engram_gateway, check_same_thread=False) # Access the engram through the provided gateway
        self.engram.row_factory = sqlite3.Row                            # Define the structure of query results of the engram
        self.engram.execute("PRAGMA journal_mode=WAL;")                  # Set up engram to aloow retrieval and storing simultaneously
        self.engram.execute("PRAGMA synchronous=NORMAL;")                # Balance episodes safety vs storing speed
        self.engram.commit()                                             # Apply the above parameters into the engram
        self._init_engram_schema()                                       # Initialize the schema of the engram

        # Write lock — only encode cycle writes to episodes
        self._write_lock = threading.Lock()                              # Ensure only one thread inscribes into the engram at a time

        # Set up encode cycle
        self._encoder_running = False                                    # Indicate that encode cycle not yet running
        self._encoder_thread: Optional[threading.Thread] = None          # Initialize background neural thread
        self._start_encoder()                                            # Start encode cycle in the background

        self.logger.info(f"✅ EMC initialized → {engram_gateway}")      # Log the successful initialization of the engram

    def _init_engram_schema(self):
        self.engram.executescript("""
            -- Raw turn intake from WMC overflow (crash-safe)
            CREATE TABLE IF NOT EXISTS em_buffer (
                id         INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp  TEXT    NOT NULL,
                role       TEXT    NOT NULL,
                content    TEXT    NOT NULL,
                date       TEXT    NOT NULL,
                processed  BOOLEAN DEFAULT FALSE
            );
            CREATE INDEX IF NOT EXISTS idx_em_buffer_processed
                ON em_buffer(processed);
            CREATE INDEX IF NOT EXISTS idx_em_buffer_date
                ON em_buffer(date);

            -- Encoded episodic memory (permanent, retrievable)
            CREATE TABLE IF NOT EXISTS episodes (
                id         INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp  TEXT    NOT NULL,
                date       TEXT    NOT NULL,
                role       TEXT    NOT NULL,
                content    TEXT    NOT NULL,
                embedding  TEXT    NOT NULL,   -- JSON float array (768 dims)
                created_at TEXT    DEFAULT (datetime('now'))
            );
            CREATE INDEX IF NOT EXISTS idx_episodes_date
                ON episodes(date);
            CREATE INDEX IF NOT EXISTS idx_episodes_role
                ON episodes(role);
        """)
        self.engram.commit()

    # ── Buffer intake (called by MCC from asyncio loop) ───────────────────────

    def buffer_append(self, role: str, content: str, timestamp: str) -> bool:
        """
        Atomically write an evicted WMC turn to em_buffer.
        Called by MCC — survives crashes, never blocks the robot.

        Args:
            role:    "user" or "assistant"
            content: Turn text (truncated to 2000 chars for safety)

        Returns:
            True on success, False on failure
        """
        date_str = timestamp[:10]

        try:
            self.engram.execute(
                "INSERT INTO em_buffer (timestamp, role, content, date) "
                "VALUES (?, ?, ?, ?)",
                [timestamp, role, content[:2000], date_str],
            )
            self.engram.commit()
            self.logger.debug(
                f"EMC buffer ← [{role}] {content[:40]}…"
            )
            return True
        except Exception as exc:
            self.logger.warning(f"EMC buffer_append failed: {exc}")
            return False

    # ── Async worker — buffer → encode → episodes ─────────────────────────────

    def _start_worker(self):
        """Start background encoding worker thread."""
        self._worker_running = True
        self._worker_thread  = threading.Thread(
            target=self._encode_worker,
            name="emc-encode-worker",
            daemon=True,
        )
        self._worker_thread.start()
        self.logger.info("🔄 EMC encoding worker started")

    def _encode_cycle(self):
        """
        Event-driven consolidation — drains em_buffer into episodes.
        Wakes only when buffer has episodes (sharp-wave ripple pattern).
        Processes a snapshot of IDs per ripple — new arrivals deferred to next cycle.
        """

        # Worker-local SQLite connection (WAL allows concurrent access)
        worker_conn = sqlite3.connect(self.db_path, check_same_thread=False)
        worker_conn.row_factory = sqlite3.Row
        worker_conn.execute("PRAGMA journal_mode=WAL;")
        self.logger.info("⚙️  EMC consolidation cycle running…")
    
        while self._consolidation_running:
            # Rest state — wait for hippocampal activation
            self._consolidation_event.wait()
            self._consolidation_event.clear()
    
            if not self._consolidation_running:
                break  # clean exit if stopped while waiting
    
            # Snapshot IDs at this moment — one ripple, one defined window
            snapshot = worker_conn.execute(
                "SELECT id FROM em_buffer WHERE processed = FALSE ORDER BY id"
            ).fetchall()
            snapshot_ids = [row["id"] for row in snapshot]
    
            if not snapshot_ids:
                continue  # spurious wake — go back to rest
    
            self.logger.debug(f"EMC ripple → {len(snapshot_ids)} episode(s) in snapshot")
    
            # Replay each episode in the snapshot
            for episode_id in snapshot_ids:
                if not self._consolidation_running:
                    break  # respect stop signal mid-ripple
    
                row = worker_conn.execute(
                    "SELECT id, timestamp, date, role, content "
                    "FROM em_buffer WHERE id = ?",
                    [episode_id]
                ).fetchone()
    
                if row is None:
                    continue  # already processed by another path
    
                # Encode the episode content
                vec = self._encoding_engine.encode(row["content"], is_query=False)
                if not vec:
                    # Encoding engine unavailable — skip for now, retry later
                    self.logger.warning(
                        f"EMC encode skipped (encoding engine unavailable): "
                        f"buffer id={row['id']}"
                    )
                    # Don't mark processed — retry next ripple
                    continue
    
                embedding_json = json.dumps(vec)
    
                with self._write_lock:
                    # Insert into episodes (engram)
                    worker_conn.execute(
                        "INSERT INTO episodes "
                        "(timestamp, date, role, content, embedding) "
                        "VALUES (?, ?, ?, ?, ?)",
                        [
                            row["timestamp"],
                            row["date"],
                            row["role"],
                            row["content"],
                            embedding_json,
                        ],
                    )
                    # Mark buffer episode as processed
                    worker_conn.execute(
                        "UPDATE em_buffer SET processed = TRUE WHERE id = ?",
                        [episode_id]
                    )
                    worker_conn.commit()
    
                self.logger.debug(
                    f"EMC consolidated → episodes: [{row['role']}] "
                    f"{row['content'][:40]}… (date={row['date']})"
                )
    
        worker_conn.close()
        self.logger.info("EMC consolidation cycle stopped")

    # ── Semantic search (called by MCC) ───────────────────────────────────────

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

        Falls back to keyword search if encoding engine is unavailable.

        Args:
            query:     Natural language query
            top_k:     Number of results to return
            date_from: Optional ISO date string lower bound (inclusive)
            date_to:   Optional ISO date string upper bound (inclusive)

        Returns:
            List of dicts: {timestamp, date, role, content, relevancy}
        """
        query_vec = self._encoding_engine.encode(query, is_query=True)
        if not query_vec:
            return self._keyword_search(query, top_k, date_from, date_to)

        try:
            sql    = "SELECT timestamp, date, role, content, embedding FROM episodes"
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
                stored_vec = json.loads(row["embedding"] or "[]")
                sim = _semantic_match(query_vec, stored_vec)
                scored.append({
                    "timestamp":  row["timestamp"],
                    "date":       row["date"],
                    "role":       row["role"],
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

        except Exception as exc:
            self.logger.error(f"EMC search failed: {exc}")
            return []

    def _keyword_search(
        self,
        query: str,
        top_k: int,
        date_from: Optional[str],
        date_to: Optional[str],
    ) -> list[dict]:
        """Fallback keyword search when encoding engine is unavailable."""
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
            f"SELECT timestamp, date, role, content FROM episodes "
            f"WHERE {' AND '.join(where)} "
            f"ORDER BY timestamp DESC LIMIT {top_k}"
        )
        try:
            rows = self.engram.execute(sql, params).fetchall()
            return [
                {
                    "timestamp":  r["timestamp"],
                    "date":       r["date"],
                    "role":       r["role"],
                    "content":    r["content"][:EMC.ENGRAM_CHUNK_LIMIT],
                    "relevancy": 0.0,
                }
                for r in rows
            ]
        except Exception as exc:
            self.logger.error(f"EMC keyword search failed: {exc}")
            return []

    # ── Episode retrieval for a specific date ─────────────────────────────────

    def get_episodes_for_date(self, date_str: str) -> list[dict]:
        """
        Return all episodes for a given date in chronological order.
        Used by MCC for 11pm reflection (future M2).
        """
        try:
            rows = self.engram.execute(
                "SELECT timestamp, role, content FROM episodes "
                "WHERE date = ? ORDER BY timestamp",
                [date_str],
            ).fetchall()
            return [
                {"timestamp": r["timestamp"], "role": r["role"], "content": r["content"]}
                for r in rows
            ]
        except Exception as exc:
            self.logger.error(f"EMC get_episodes_for_date failed: {exc}")
            return []

    # ── Buffer status ─────────────────────────────────────────────────────────

    def buffer_pending_count(self) -> int:
        """Return number of turns waiting to be embedded."""
        try:
            row = self.engram.execute(
                "SELECT COUNT(*) FROM em_buffer WHERE processed = FALSE"
            ).fetchone()
            return row[0] if row else 0
        except Exception:
            return 0

    # ── Stats ─────────────────────────────────────────────────────────────────

    def get_stats(self) -> dict:
        """Return EMC health and storage stats."""
        try:
            ep_row = self.engram.execute(
                "SELECT COUNT(*) as total, "
                "MIN(date) as oldest, MAX(date) as newest "
                "FROM episodes"
            ).fetchone()

            buf_row = self.engram.execute(
                "SELECT COUNT(*) as total, "
                "SUM(CASE WHEN processed=FALSE THEN 1 ELSE 0 END) as pending "
                "FROM em_buffer"
            ).fetchone()

            db_size_mb = round(Path(self.db_path).stat().st_size / 1_048_576, 2) \
                if Path(self.db_path).exists() else 0.0

            return {
                "episodes":         ep_row["total"]  if ep_row  else 0,
                "oldest_episode":   ep_row["oldest"] if ep_row  else None,
                "newest_episode":   ep_row["newest"] if ep_row  else None,
                "buffer_total":     buf_row["total"]   if buf_row else 0,
                "buffer_pending":   buf_row["pending"]  if buf_row else 0,
                "db_size_mb":       db_size_mb,
                "encoding_engine_ready":   self._encoding_engine.is_available(),
                "ENCODING_ENGINE":  EMC.ENCODING_ENGINE,
            }
        except Exception as exc:
            self.logger.error(f"EMC get_stats failed: {exc}")
            return {}

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def cleanup_processed_buffer(self, keep_days: int = 1) -> None:
        """
        Remove processed em_buffer rows older than keep_days.
        Safe to call periodically — processed rows are already in episodes.
        """
        try:
            self.engram.execute(
                "DELETE FROM em_buffer "
                "WHERE processed = TRUE "
                "AND date < date('now', ?)",
                [f"-{keep_days} days"],
            )
            self.engram.commit()
            self.logger.debug("EMC buffer cleanup done")
        except Exception as exc:
            self.logger.warning(f"EMC buffer cleanup failed: {exc}")

    def close(self) -> None:
        """
        Gracefully close EMC and the engram gateway.
        This releases any open engram gateway resources and ensures proper shutdown of the engram complex.
        """
        self._worker_running = False
        if self._worker_thread:
            self._worker_thread.join(timeout=3.0)
        if self.engram:
            self.engram.close()
        self.logger.info("🗄️  EMC closed")
