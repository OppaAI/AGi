"""
MSB — Memory Storage Bank
==========================
AuRoRA · Semantic Cognitive System (SCS)

Shared neural substrate for all memory cortices — EMC, SMC, PMC.

MSB carries no memory domain knowledge of its own. It does not know what
an episode is, what a skill is, or what a semantic concept means. It only
knows how to encode thought into vectors, persist traces into engrams, and
retrieve them by meaning or pattern. The cortices supply domain knowledge —
MSB supplies the tissue they run on.

Separated into its own layer so encoding models, vector math, SQLite
connection strategy, and retrieval fusion logic are defined once and
shared — not duplicated across every cortex that needs them.

Terminology:
    encoding    — semantic vector representation of a memory trace
    engram      — the persistent memory store for a cortex
    FTS5        — SQLite full-text search for lexical pattern retrieval
    RRF         — Reciprocal Rank Fusion — fuses semantic and lexical ranked
                  results into a single relevancy-ordered list
    unit vector — L2-normalized vector; cosine similarity becomes equivalent
                  to L2 distance, enabling sqlite-vec KNN search

TODO: migrate pack_vector, unpack_vector, unit_normalize to hrs.py if
      vector math is needed outside memory cortices
"""

# System libraries
from dataclasses import dataclass, field    # Dataclass for EngramTrace/EngramSchema, field for default_factory
from enum import Enum                       # Enum base for EngramModality type definitions
import numpy as np                          # For fast vector math — normalization and cosine similarity
import sqlite3                              # For engram connection factory
import struct                               # For packing/unpacking semantic vectors (fp32)
import re                                   # For lexical cue sanitization
from pathlib import Path                    # For calculating database size — owned here, never passed back up

class EngramModality(Enum):
    """
    Defines the modality of engram memory traces (ie. the type of data stored).
    """
    TEXT    = 'TEXT'        # Maps to SQLite TEXT type — stores strings (content, timestamps, dates)
    INTEGER = 'INTEGER'     # Maps to SQLite INTEGER type — stores whole numbers (primary keys, counts)
    REAL    = 'REAL'        # Maps to SQLite REAL type — stores floating point numbers (scores, floats)
    BLOB    = 'BLOB'        # Maps to SQLite BLOB type — stores raw bytes (packed encoding vectors)

@dataclass
class EngramTrace:
    """
    Define the schema for a single engram trace.
    """
    label: str                       # The SQL column name — e.g. "content", "timestamp", "encoding"
    modality: EngramModality         # The SQL column type — calls .value to get the raw string e.g. 'TEXT'
    essential: bool = False          # If True, adds NOT NULL constraint — column must have a value on insert
    baseline: str | None = None      # If set, adds DEFAULT — raw SQL expression if starts with '(', quoted string otherwise
 
@dataclass
class EngramSchema:
    """
    Define the structure, storage schema, and search capabilities for an engram.
    """
    storage: list[EngramTrace]                                  # Defines columns for the main episodes table — every cortex needs this
    staging: list[EngramTrace] | None = None                    # Defines columns for the crash-safe buffer table — optional, EMC uses it, SMC may not
    semantic_traces: str | None = None                          # Names the BLOB column that holds the encoding vector — tells MSB which column to KNN search against
    lexical_traces: list[str] | None = None                     # Names the text columns to feed into FTS5 — tells MSB what to keyword-search
    index_traces: list[str] | None = None                       # Names columns to put a B-tree index on — speeds up WHERE/ORDER BY on those columns

class EncodingEngine:
    """
    Encoding engine for semantic encoding of memory traces for storage and recall.
    Shared across EMC, SMC, and PMC — loaded once per cortex initialization.
    Primes recent encodings to avoid redundant encoding of identical or similar episodes
    and to speed up subsequent recall.
    """

    def __init__(self, logger, encoding_engine: str, cache_limit: int = 256, imprint_limit: int = 300) -> None:
        """
        This method sets up the encoding engine core and cache for recent encodings.
        
        Args:
            logger               : Logger instance for logging encoding engine operations
            encoding_engine (str): The specific embedding model to load (e.g. from HRP)
            cache_limit     (int): Maximum number of entries in the LRU encoding cache
            imprint_limit   (int): Maximum number of characters to hash for the imprint
        """
        self.logger                         = logger                # Stores the logger passed in from the cortex — used in encode() and activation logs
        self.encoding_engine: str           = encoding_engine       # Stores the model name string — passed to SentenceTransformer() below
        self.cache_limit: int               = cache_limit           # Stores max cache size — checked in encode() before every insert
        self.imprint_limit: int             = imprint_limit         # Stores max chars to hash — longer text is truncated before hashing to make the cache key
        self._core                          = None                  # The live SentenceTransformer model — None until load succeeds, encode() checks this before every inference call
        self._cache: dict[str, list[float]] = {}                    # LRU encoding cache — maps imprint hash → float vector, avoids re-encoding identical or similar text
        
        try:                                                                        # Attempt to activate SentenceTransformer
            from sentence_transformers import SentenceTransformer                   # Deferred import — optional dependency, avoids hard crash if package is missing
            self.logger.info(f"⏳ Activating Encoding Engine ({encoding_engine})…") # Logs before the blocking load — model can take seconds on Jetson
            self._core = SentenceTransformer(self.encoding_engine)                  # Loads model weights into RAM — blocks until complete, sets _core to the live model
            self.logger.info("✅ Encoding Engine activated")                        # Only reached if load succeeded — _core is now usable
        except ImportError:                                                         # Triggers if sentence_transformers package is not installed
            self.logger.warning(                                                    # Warns but does not crash — cortex continues with lexical-only recall
                "⚠️ Encoding Engine offline - missing inferencing component.\n"
                "   Memory cortices falling back to lexical recall.\n"
                "   Note to technician: pip3 install sentence-transformers --break-system-packages"
            )
        except Exception as e:                                                      # Catches everything else — bad model path, corrupted weights, out of memory
            self.logger.warning(f"⚠️ Encoding Engine activation failed: {e}")       # Logs the specific failure reason — _core stays None, same fallback behavior
            
    @property
    def is_available(self) -> bool:
        """
        Returns True if encoding engine loaded successfully and ready for encoding.

        Returns:
            bool: True if ready for encoding, False if failed to load (e.g. missing inferencing component).
        """
        return self._core is not None            # Checks if SentenceTransformer loaded — None means load failed or was never attempted, any other value means model is live

    def encode(self, trace: str, is_cue: bool = False) -> list[float]:
        """
        Encode the given memory trace into a semantic vector for storage or recall.
        Uses caching to avoid redundant encoding of identical or similar texts.
        Caches recent encodings to speed up subsequent recall.
        If encoding engine is unavailable, returns an empty list to signal that semantic encoding cannot be performed, 
        prompting the caller to fall back to lexical search.
        
        Args:
            trace (str): The given memory trace to encode (e.g. episode content).
            is_cue (bool): Whether the trace is a recall cue (True) or an episode to be stored (False).
                             This allows for separate caching of cue and episode encodings, which may have different patterns of repetition.

        Returns:
            list[float]: The semantic encoding vector for the input trace
        """
        if not self.is_available:                                                       # Calls the property above — if _core is None, skip everything and return empty
            self.logger.debug(                                                          # Log the SentenceTransformer being unavailable
                "Encoding engine unavailable — semantic search inactive"
            )
            return []                                                                   # Empty list is the signal to callers — means "fall back to lexical, semantic is offline"

        imprint = f"{'cue' if is_cue else 'episode'}:{hash(trace[:self.imprint_limit])}"# Builds cache key — prefixed with 'cue' or 'episode' so the same text encoded for storage vs recall gets separate cache entries
                                                                                        # trace[:self.imprint_limit] truncates before hashing — only first 300 chars checked, long texts still get cache hits if they share the same opening
        if imprint in self._cache:                                                      # Looks up the key in the dict — O(1) hash lookup
            return self._cache[imprint]                                                 # Cache hit — returns the stored vector directly, skips model inference entirely

        try:                                                                            # Attempt to embed the query or engram vector
            cue_prefix = f"Represent this sentence for searching relevant passages: "   # BGE instruction prefix — only applied to recall cues, tells the model this text is a query not a passage
            encoded_trace: list[float] = self._core.encode(cue_prefix + trace if is_cue else trace).tolist() # Runs model inference — prepends prefix if cue, encodes raw if episode, .tolist() converts numpy array → Python float list
            encoded_trace = unit_normalize(encoded_trace)                              # Normalizes the vector to unit length — required so L2 distance equals cosine similarity in sqlite-vec KNN search
            
            # Keep cache small — evict oldest if over the encoding cache limit
            if len(self._cache) >= self.cache_limit:                                    # Cache is full — must evict before inserting or it grows forever
                decayed_imprint: str = next(iter(self._cache))                          # Gets the first key in the dict — dicts preserve insertion order, so first = oldest entry
                del self._cache[decayed_imprint]                                        # Removes the oldest entry — makes room for the new one
            self._cache[imprint] = encoded_trace                                        # Stores the new vector under its imprint key — available for future cache hits
            return encoded_trace                                                        # Returns the freshly encoded and normalized vector to the caller
        except Exception as e:                                                          # If embedding fails,
            self.logger.debug(f"Encoding error: {e}")                                   # Logs what went wrong — debug level, not warning, inference errors are transient
            return []                                                                   # Same empty list signal as the unavailable guard above — caller falls back to lexical
            
def unit_normalize(vector: list[float]) -> list[float]:
    """
    Unit-normalize a vector so L2 distance search is cosine-equivalent.
    Already-normalized vectors and zero vectors are returned unchanged.

    Args:
        vector (list[float]): Vector to normalize

    Returns:
        list[float]: A unit-normalized copy of vector, or vector itself if already normalized or zero
    """
    vector_array = np.array(vector)                                                # list[float] → ndarray for vectorized math
    vector_mag = np.linalg.norm(vector_array)                                      # L2-norm — Euclidean length of the vector
    if abs(vector_mag - 1.0) < 1e-6:                                               # Already unit — dtype conversion rarely erodes past 1e-6
        return vector                                                              # Return original — no copy needed
    return (vector_array / vector_mag).tolist() if vector_mag > 0.0 else vector    # Zero vector guard — division by zero would corrupt the engram

def pack_vector(vector: list[float]) -> bytes:
    """
    Pack a float vector into fp32 binary blob for engram storage.
    Used before INSERT into episodes and engram_vectors.

    Args:
        vector (list[float]): Semantic encoding vector.

    Returns:
        bytes: Binary blob of fp32 values.
    """
    return struct.pack(f"{len(vector)}f", *vector)              # fp32 little-endian binary — matches sqlite-vec storage format

def unpack_vector(blob: bytes, dim: int) -> list[float]:
    """
    Unpack a fp32 binary blob back into a float vector.
    Used when reading encodings from the engram for cosine similarity fallback.

    Args:
        blob (bytes): Binary blob stored in the engram.
        dim  (int)  : Expected vector dimension.

    Returns:
        list[float]: Unpacked float vector.
    """
    return list(struct.unpack(f"{dim}f", blob))                 # Dim must match original vector length — mismatched dim silently corrupts

def semantic_match(cue: list[float], episode: list[float]) -> float:
    """
    Match a recall cue against a stored episode semantically.
    Cosine similarity fallback — used when sqlite-vec ANN index is unavailable.
    TODO: To be replaced with sqlite-vec ANN search.

    Args:
        cue     (list[float]): Encoded recall cue.
        episode (list[float]): Encoded stored episode.
        * Both cue and episode vectors have already been unit-normalized — only dot product is needed to equal cosine similarity
    
    Returns:
        float: Semantic relevancy score (0.0 – 1.0).
    """
    if not cue or not episode or len(cue) != len(episode):                      # Dimension guard — to avoid dimension mismatch or empty
        return 0.0                                                              # Return 0.0 rather than exception
    return float(np.dot(np.array(cue), np.array(episode)))                      # Dot product == cosine sim — valid only on unit-normalized vectors
    
def connect_engram(gateway: str, logger=None) -> sqlite3.Connection:
    """
    Connect to engram (SQLite) with WAL mode and row factory.
    Shared connection factory — all memory cortices (EMC, SMC, PMC) use
    the same pragma configuration for consistent concurrent access behavior.

    WAL mode allows concurrent reads during async writes — Jetson-friendly.
    NORMAL synchronous mode balances episode safety vs storing speed.

    Args:
        gateway (str): Filesystem path to the SQLite engram file.
        logger       : Optional logger for error reporting.

    Returns:
        sqlite3.Connection: Configured connection with WAL mode and row factory.

    Raises:
        sqlite3.Error: If connection cannot be established.
    """
    engram_conn = sqlite3.connect(gateway, check_same_thread=False)    # check_same_thread=False — allows concurrent access from different threads
    engram_conn.row_factory = sqlite3.Row                              # Row objects allow column access by name
    engram_conn.execute("PRAGMA journal_mode=WAL;")                    # WAL — concrrent reads during encoding cycle writes
    engram_conn.execute("PRAGMA synchronous=NORMAL;")                  # NORMAL — fsync on checkpoint only, not every writes
    engram_conn.commit()                                               # Commit pragma changes before returning
    return engram_conn                                                 # Return the configured connection

def activate_engram_index(engram_conn: sqlite3.Connection, logger=None) -> bool:
    """
    Activate the engram vector index for KNN semantic search.
    Returns True if successfully activated, False on failure.
    
    Vec0 virtual tables provide L2 distance KNN search over encoded episodes.
    Graceful fallback to Python cosine similarity if index unavailable.
    
    Args:
        engram_conn: An open engram connection.
        logger     : Optional logger for warning on failure.
    
    Returns:
        bool: True if engram vector index activated, False otherwise.
    """
    try:                                                        # Attempt to activate engram vector index
        import sqlite_vec                                       # Deferred import — optional dependency
        sqlite_vec.load(engram_conn)                            # Loads vec0 virtual table support into this connection
        return True                                             # Signal successful activation of sqlite-vec
    except Exception as e:                                      # If sqlite-vec fails to load,
        if logger:                                              # If a logger is provided,
            logger.warning(                                     # Log the fallback to cosine similarity due to engram vector index unavailability
                f"⚠️ engram vector index not available, falling back to unindexed cosine similarity\n"
                f"   Note to technician: pip3 install sqlite-vec --break-system-packages\n"
                f"   Reason: {e}"
            )
        return False                                            # Signal failed activation — caller falls back to cosine similarity

LEXICAL_FILTER_WORDS = {                                        # Filtered before FTS5 query — common words that match everything and hurt precision
    "a", "an", "the", "is", "are", "was", "were", "what", "how", "why", "where", 
    "when", "who", "which", "in", "on", "at", "to", "for", "of", "with", "by", 
    "from", "as", "and", "or", "but", "if", "then", "else", "my", "your", "our", 
    "their", "his", "her", "its", "me", "you", "him", "them", "us", "i", "can", 
    "could", "would", "should", "will", "shall", "do", "does", "did", "have", 
    "has", "had", "it", "this", "that", "those", "these"
}

def sanitize_lexical_cue(cue: str) -> str:
    """
    Sanitize and tokenize a recall cue for FTS5 lexical matching.
    Strips punctuation, filters stop-words, and joins keywords with OR
    to improve pattern-separation recall while maintaining precision.

    Args:
        cue (str): Raw recall cue string from the caller.

    Returns:
        str: FTS5-safe quoted token string, or empty string if query is blank.
    """
    clean_cue = re.sub(r'[^\w\s]', ' ', cue.lower())                 # Punctuation → space, lowercase — FTS5 MATCH rejects special chars
    lexemes = clean_cue.strip().split()                              # Whitespace tokenization — FTS5 handles stemming internally
    
    # Extract keywords (non-stop-words longer than 1 character)
    terms = [kw for kw in lexemes if kw not in LEXICAL_FILTER_WORDS and len(kw) > 1]    # Filter stop words and single chars — improves FTS5 precision
    
    # Fallback to original lexemes if no terms remain
    if not terms:                                                    # All words were stop words — fall back to full token list
        terms = lexemes
        
    if not terms:                                                    # Empty cue — caller skips lexical path
        return ""

    return " OR ".join(f'"{kw}"' for kw in keywords if kw)           # OR join — any keyword match surfaces the episode

def memory_convergence(
    semantic_results : list[dict],
    lexical_results  : list[dict],
    recall_limit     : int,
    rrf_k            : int = 60,
) -> list[dict]:
    """
    Fuse semantic and lexical recall results into a unified engram salience ranking.
    Semantic search (vec KNN / cosine) + lexical search (FTS5)
    → unified engram salience ranking through memory convergence (RRF).

    Shared across EMC, SMC, and PMC — all memory cortices use the same
    dual-path retrieval and convergence pattern.

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
        rrf_k            : RRF constant (default 60, standard in literature)

    Returns:
        list[dict]: list of episodes up to recall limit sorted by descending RRF score,
                    with 'relevancy' set to normalised RRF score (0.0–1.0)
    """
    # Build rank lookup schemas and episode pool — 0-based, best = 0
    episode_pool: dict[int, dict] = {}                                        # Union of all candidates from both paths — keyed by episode id
    semantic_rank: dict[int, int] = {}                                        # episode_id → 0-based rank in semantic results
    lexical_rank: dict[int, int] = {}                                         # episode_id → 0-based rank in lexical results

    for rank, episode in enumerate(semantic_results):                         # Iterate through each semantic match results
        semantic_rank[episode["id"]] = rank                                   # Store semantic rank — 0-based, best = 0
        episode_pool[episode["id"]] = episode                                 # Add episode to pool for later retrieval

    for rank, episode in enumerate(lexical_results):                          # Iterate through each lexical match results
        lexical_rank[episode["id"]] = rank                                    # Store lexical rank — 0-based, best = 0
        episode_pool.setdefault(episode["id"], episode)                       # Add episode to pool if not already, semantic path takes precedence as default

    semantic_miss = len(semantic_results)                                     # Penalty rank for episodes absent from semantic results
    lexical_miss = len(lexical_results)                                       # Penalty rank for episodes absent from lexical results
    
    # Compute RRF score for each candidate
    for episode in list(episode_pool.values()):                               # Iterate through a clone of episode pool — # list() — snapshot prevents mutation during iteration
        episode = dict(episode)                                               # dict() — clone prevents mutating the original pool entry
        episode["rrf_score"] = (                                              # Calculate rrf score — RRF formula: 1/(k+rank_semantic) + 1/(k+rank_lexical)
            1.0 / (rrf_k + semantic_rank.get(episode["id"], semantic_miss)) +
            1.0 / (rrf_k + lexical_rank.get(episode["id"], lexical_miss))
        )
        episode_pool[episode["id"]] = episode                                 # Replace original episode entry with rrf score added

    # Sort descending by RRF score
    sorted_episodes = sorted(episode_pool.values(),                           # Sort descending — highest RRF score first
                             key=lambda episode: episode["rrf_score"], 
                             reverse=True)

    # Normalise RRF scores to 0.0–1.0 for the 'relevancy' field
    max_rrf = sorted_episodes[0]["rrf_score"] if sorted_episodes else 1.0     # Computing best RRF score for normalization — guard against empty results
 
    for episode in sorted_episodes[:recall_limit]:                            # Surface top episodes up to recall limit
        episode["relevancy"] = episode["rrf_score"] / max_rrf                 # Normalize RRF score to 0.0–1.0 relevancy
        episode.pop("_rank", None)                                            # Strip internal rank field before surfacing
        episode.pop("rrf_score", None)                                        # Strip internal RRF score before surfacing
    
    return sorted_episodes[:recall_limit]                                     # Return top episodes normalized and cleaned

class EngramStorageBank:
    """
    Memory Bank (MB) — memory storage bank abstraction layer.
    Owns all SQL operations for memory cortex — schema creation, staging, consolidation,
    search, and retrieval. The memory cortex never touches SQL directly.

    Shared engram file (memory_bank.db) — EMC tables only at this stage.
    SMC and PMC will extend with their own tables in M2.

    Backend today: SQLite + sqlite-vec
    Backend future: Qdrant, pgvector, or other vector DB — swap here only,
                    EMC cognitive logic untouched.

    Args:
        engram_conn      : Open SQLite connection (WAL + row_factory configured)
        engram_dim       : Vector dimension of the encoding engine (e.g. 384 for BGE-small)
        engram_index     : Whether sqlite-vec vector index is active
        logger           : Logger instance for logging operations
    """

    def __init__(
        self,
        memory_type: str,
        engram_conn: sqlite3.Connection,
        engram_schema: EngramSchema,
        engram_dim: int,
        engram_index: bool,
        logger,
    ) -> None:

        self._storage_schema = f"{memory_type}_storage"                 # Schema of engram storage
        self._staging_schema = f"{memory_type}_staging"                 # Schema of staging buffer
        self._vector_schema  = f"{memory_type}_vector"                  # Schema of semantic search index
        self._lexical_schema = f"{memory_type}_lexical"                 # Schema of lexical search index

        self._conn         = engram_conn
        self._schema       = engram_schema                              # Store the engram schema for dynamic table generation
        self._engram_dim   = engram_dim
        self._engram_index = engram_index
        self.logger        = logger

    # ── Schema ────────────────────────────────────────────────────────────────

    def init_schema(self) -> None:
        """
        Initialize the schema of the engram.
        This method dynamically creates tables and indexes based on the injected EngramSchema.
        """
        try:
            # 1. Build Staging Table (if defined)
            if self._schema.staging:
                staging_cols = self._build_columns_sql(self._schema.staging)
                self._conn.execute(f"""
                    CREATE TABLE IF NOT EXISTS {self._staging_schema} (
                        {staging_cols}
                    )
                """)

            # 2. Build Storage Table
            storage_cols = self._build_columns_sql(self._schema.storage)
            self._conn.execute(f"""
                CREATE TABLE IF NOT EXISTS {self._storage_schema} (
                    {storage_cols}
                )
            """)

            # 3. Build Indexes
            if self._schema.index_traces:
                for trace in self._schema.index_traces:
                    self._conn.execute(f"""
                        CREATE INDEX IF NOT EXISTS idx_{self._storage_schema}_{trace}
                        ON {self._storage_schema}({trace})
                    """)
            
            self._conn.commit()

            # 4. Build Vector Search Virtual Table
            if self._engram_index and self._schema.semantic_traces:
                semantic_col = self._schema.semantic_traces
                self._conn.execute(f"""
                    CREATE VIRTUAL TABLE IF NOT EXISTS {self._vector_schema} USING vec0(
                        {semantic_col} FLOAT[{self._engram_dim}]
                    )
                """)
                self._conn.commit()
                self.logger.debug(f"EMC engram vector index initialized for {self._vector_schema}")

            # 5. Build Lexical Search Virtual Table (FTS5)
            if self._schema.lexical_traces:
                lexical_cols = ", ".join(self._schema.lexical_traces)
                self._conn.execute(f"""
                    CREATE VIRTUAL TABLE IF NOT EXISTS {self._lexical_schema} USING fts5(
                       {lexical_cols},
                       tokenize='porter unicode61'
                    )
                """)
                self._conn.commit()
                
        except sqlite3.Error as e:
            self.logger.error(f"Engram schema initialization failed: {e}")
            raise

    def _build_columns_sql(self, traces: list[EngramTrace]) -> str:
        """Helper to generate SQL column definitions from a list of EngramTraces."""
        columns = []
        for trace in traces:
            col_def = f"{trace.label} {trace.modality.value}"
            if trace.label == "id":
                # Ensure SQLite correctly auto-increments INTEGER PRIMARY KEY
                col_def += " PRIMARY KEY AUTOINCREMENT" if trace.modality == EngramModality.INTEGER else " PRIMARY KEY"
            elif trace.essential:
                col_def += " NOT NULL"
                
            if trace.baseline is not None:
                if trace.modality == EngramModality.TEXT and not trace.baseline.startswith('('):
                    col_def += f" DEFAULT '{trace.baseline}'"
                else:
                    col_def += f" DEFAULT {trace.baseline}"
            columns.append(col_def)
        self.logger.debug(f"MSB schema columns built — {len(columns)} fields")             # Log the initialization of the FTS5 lexical index
        return ",\n                        ".join(columns)

    # ── Staging ───────────────────────────────────────────────────────────────

    def insert_staged(self, conn: sqlite3.Connection, episode: dict) -> int:
        """
        Insert an episode into the episodic buffer (crash-safe staging).
        Returns the staging_id for deletion after synaptic consolidation.

        Args:
            conn    : Encoder connection (separate from main engram connection)
            episode : Episode dict with timestamp, date, content

        Returns:
            int: staging_id of the inserted episode
        """
        staging_id = conn.execute(                                              # Insert the episode into the episodic buffer
            f"INSERT INTO {self._staging_schema} (timestamp, date, content) "
            "VALUES (?, ?, ?)",
            [episode["timestamp"], episode["date"], episode["content"]],
        )
        conn.commit()                                                           # Commit the transaction
        return staging_id.lastrowid                                             # Capture staging index for deletion after encoding

    def get_unencoded(self, batch_size: int, offset: int) -> list:
        """
        Fetch a batch of unencoded episodes from the episodic buffer.
        Used during recovery to drain episodic_buffer back into binding stream.

        Args:
            batch_size : Maximum number of episodes to fetch per batch
            offset     : Batch offset for pagination

        Returns:
            list: Rows of unencoded episodes
        """
        return self._conn.execute(                                          # Query the engram for unencoded episodes
            f"SELECT id, timestamp, date, content "                         # Collect the index, timestamp, date, and content
            f"FROM {self._staging_schema} "                                 # All episodes in the episodic buffer is unencoded by definition
            "ORDER BY id LIMIT ? OFFSET ?",                                 # Sort by id and limit the results
            [batch_size, offset]                                            # Process episodes by batch size
        ).fetchall()                                                        # Fetch all the unencoded episodes

    def get_buffer_count(self) -> int:
        """
        Return count of episodes currently staged in episodic buffer.

        Returns:
            int: Number of unencoded episodes in episodic buffer
        """
        return self._conn.execute(
            f"SELECT COUNT(*) FROM {self._staging_schema}"
        ).fetchone()[0]

    def delete_staged(self, conn: sqlite3.Connection, staging_id: int) -> None:
        """
        Remove a staged episode from the episodic buffer after synaptic consolidation.

        Args:
            conn       : Encoder connection (separate from main engram connection)
            staging_id : ID of the staged episode to remove
        """
        conn.execute(                                                           # Remove the episode from episodic buffer after synaptic consolidation
            f"DELETE FROM {self._staging_schema} WHERE id=?",                       # Remove the episode from episodic buffer
            [staging_id],                                                       # Using the episode ID
        )

    # ── Consolidation ─────────────────────────────────────────────────────────

    def insert_episode(self, conn: sqlite3.Connection, episode: dict, encoding_blob: bytes) -> int:
        """
        Insert an encoded episode into the episodes table.

        Args:
            conn          : Encoder connection for writing
            episode       : Episode dict with timestamp, date, content
            encoding_blob : Binary encoding blob of the episode

        Returns:
            int: episode_id of the inserted episode
        """
        engram_id = conn.execute(                                               # Insert the episode into engram
            f"INSERT INTO {self._storage_schema} (timestamp, date, content, encoding) VALUES (?,?,?,?)",   # Insert the episode into engram
            [episode["timestamp"], episode["date"], episode["content"], encoding_blob],     # With the episode timestamp, date, content, and encoding
        )
        return engram_id.lastrowid                                              # Get the ID of the inscribed episode

    def insert_vector(self, conn: sqlite3.Connection, episode_id: int, encoding_blob: bytes) -> None:
        """
        Insert an episode encoding into the engram vector index.
        Only called if engram vector index is activated.

        Args:
            conn          : Encoder connection for writing
            episode_id    : ID of the episode to index
            encoding_blob : Binary encoding blob of the episode
        """
        conn.execute(                                                           # Insert the episode encoding into engram vectors for fast retrieval
            f"INSERT INTO {self._vector_schema} (rowid, {self._schema.semantic_traces}) VALUES (?,?)",        # Insert the episode encoding into engram vectors
            [episode_id, encoding_blob],                                        # With the episode ID and encoding
        )

    def insert_lexical(self, conn: sqlite3.Connection, episode_id: int, content: str) -> None:
        """
        Insert episode content into the FTS5 lexical index.
        rowid must match episodes.id so JOIN works during recall.

        Args:
            conn       : Encoder connection for writing
            episode_id : ID of the episode to index
            content    : Raw content of the episode
        """
        conn.execute(                                                           # Insert the episode content into engram lexical for fast retrieval
            f"INSERT INTO {self._lexical_schema} (rowid, {', '.join(self._schema.lexical_traces)}) VALUES (?,?)",         # Insert the episode content into engram lexical
            [episode_id, content],                                              # With the episode ID and content
        )

    # ── Search ────────────────────────────────────────────────────────────────

    def search_knn(self, cue_blob: bytes, limit: int) -> list[dict]:
        """
        KNN semantic search via sqlite-vec L2 distance.
        Returns shaped results ready for RRF fusion — no post-processing in caller.

        Args:
            cue_blob : fp32 binary blob of the encoded recall cue
            limit    : Maximum candidates to return

        Returns:
            list[dict]: id, timestamp, date, content, relevancy, _rank
        """
        try:
            rows = self._conn.execute(
                f"""
                SELECT e.id, e.timestamp, e.date, e.content,
                    ev.distance
                FROM {self._vector_schema} ev
                JOIN {self._storage_schema} e ON e.id = ev.rowid
                WHERE ev.{self._schema.semantic_traces} MATCH ?
                AND k = ?
                ORDER BY ev.distance
                """,
                [cue_blob, limit],
            ).fetchall()

            if not rows:
                return []

            self.logger.debug(f"ESB KNN → {len(rows)} candidates | best_dist:{rows[0]['distance']:.4f}")

            return [
                {
                    "id"        : row["id"],
                    "timestamp" : row["timestamp"],
                    "date"      : row["date"],
                    "content"   : row["content"],
                    "relevancy" : max(0.0, 1.0 - (row["distance"]**2 / 2.0)),  # L2 distance → cosine similarity on unit vectors
                    "_rank"     : i,                                             # 0-based rank for RRF
                }
                for i, row in enumerate(rows)
            ]
        except Exception as e:
            self.logger.debug(f"ESB KNN search failed: {e}")
            return []

    def search_cosine(self, cue_vector: list[float], recall_limit: int, recall_pool: int) -> list[dict]:
        """
        Brute-force cosine similarity fallback when sqlite-vec is unavailable.
        Stratified sampling — half recent, half oldest — bounds RAM usage while
        ensuring old memories score alongside recent ones.

        Args:
            cue_vector   : Encoded recall cue as float list
            recall_limit : Maximum candidates to return
            recall_pool  : Candidate pool multiplier — controls stratified sample size

        Returns:
            list[dict]: id, timestamp, date, content, relevancy, _rank
        """
        try:
            half_pool = (recall_limit * recall_pool) // 2                       # Stratified pool — half recent, half oldest
            rows = self._conn.execute(
                f"SELECT id, timestamp, date, content, encoding FROM {self._storage_schema} "
                "ORDER BY rowid DESC LIMIT ? "
                "UNION ALL "
                f"SELECT id, timestamp, date, content, encoding FROM {self._storage_schema} "
                "ORDER BY rowid ASC LIMIT ?",
                [half_pool, half_pool],
            ).fetchall()

            scored = []
            expected_bytes = len(cue_vector) * 4                                    # fp32 — 4 bytes per float
            for row in rows:
                if len(row["encoding"]) != expected_bytes:                          # Dimension mismatch — stale encoding from old model
                    continue
                engram_vec = unpack_vector(row["encoding"], len(cue_vector))        
                score = semantic_match(cue_vector, engram_vec)                      # Dot product == cosine sim on unit vectors
                if score > 0.0:
                    scored.append((score, row))

            scored.sort(key=lambda x: x[0], reverse=True)

            return [
                {
                    "id"        : row["id"],
                    "timestamp" : row["timestamp"],
                    "date"      : row["date"],
                    "content"   : row["content"],
                    "relevancy" : score,
                    "_rank"     : i,                                                # 0-based rank for RRF
                }
                for i, (score, row) in enumerate(scored[:recall_limit * 2])
            ]
        except Exception as e:
            self.logger.debug(f"ESB cosine search failed: {e}")
            return []

    def search_lexical_normalized(self, query: str, recall_limit: int) -> list[dict]:
        """
        FTS5 lexical search with normalized relevancy scores.
        Sanitizes raw query, executes FTS5 match, normalizes raw rank to 0.0-1.0
        so results are shaped identically to semantic path for RRF fusion.

        Args:
            query        : Raw recall cue string — sanitized internally
            recall_limit : Maximum candidates to return

        Returns:
            list[dict]: id, timestamp, date, content, relevancy, _rank
        """
        safe_query = sanitize_lexical_cue(query)                                    # Strip punctuation, filter stop words, join with OR
        if not safe_query:
            return []

        try:
            rows = self._conn.execute(
                f"""
                SELECT e.id, e.timestamp, e.date, e.content,
                    fts.rank AS raw_rank
                FROM {self._lexical_schema} fts
                JOIN {self._storage_schema} e ON e.id = fts.rowid
                WHERE {self._lexical_schema} MATCH ?
                ORDER BY fts.rank
                LIMIT ?
                """,
                [safe_query, recall_limit * 2],                                     # 2× limit — RRF culls to recall_limit after fusion
            ).fetchall()

            if not rows:
                return []

            raw_scores = [abs(row["raw_rank"]) for row in rows]
            max_score  = max(raw_scores) if raw_scores else 1.0                     # Guard against empty — should never reach here

            return [
                {
                    "id"        : row["id"],
                    "timestamp" : row["timestamp"],
                    "date"      : row["date"],
                    "content"   : row["content"],
                    "relevancy" : abs(row["raw_rank"]) / max_score if max_score else 0.0,  # FTS5 rank is negative — abs() then normalize to 0.0-1.0
                    "_rank"     : i,                                                        # 0-based rank for RRF
                }
                for i, row in enumerate(rows)
            ]
        except Exception as e:
            self.logger.debug(f"ESB lexical search failed: {e}")
            return []

    # ── Retrieval ─────────────────────────────────────────────────────────────

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
            rows = self._conn.execute(
                f"SELECT timestamp, content FROM {self._storage_schema} "
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

    def get_stats(self, engram_gateway: str) -> dict:
        """
        Return EMC health and storage stats.

        Returns:
            dict: {
                episodes, oldest_episode, newest_episode,
                buffer_total, db_size_mb
            }
            Empty dict on failure.
        """
        try:
            ep_row = self._conn.execute(
                "SELECT COUNT(*) as total, "
                "MIN(date) as oldest, MAX(date) as newest "
                f"FROM {self._storage_schema}"
            ).fetchone()

            buf_row = self._conn.execute(
                f"SELECT COUNT(*) as total FROM {self._staging_schema}"
            ).fetchone()

            db_size_mb = round(Path(engram_gateway).stat().st_size / 1_048_576, 2) \
                if Path(engram_gateway).exists() else 0.0

            return {
                "episodes":       ep_row["total"]  if ep_row  else 0,
                "oldest_episode": ep_row["oldest"] if ep_row  else None,
                "newest_episode": ep_row["newest"] if ep_row  else None,
                "buffer_total":   buf_row["total"] if buf_row else 0,
                "db_size_mb":     db_size_mb,
            }
        except Exception as e:
            self.logger.error(f"EMC get_stats failed: {e}")
            return {}


    # ── Diagnostic stats —————————————————————————————————————————————————————
    def count_stored_engrams(self) -> int:
        """
        Count number of engrams stored in the memory bank.
        """
        return self._conn.execute(                              # Return count of engrams in storage schema
            f"SELECT COUNT(*) FROM {self._storage_schema}"      # Query engram storage schema for count
        ).fetchone()[0]                                         # Return first element of result tuple

    def count_staged_engrams(self) -> int:
        """
        Count number of engrams staged in the buffer pending for storage.
        """
        return self._conn.execute(                              # Return count of engrams in staging schema
            f"SELECT COUNT(*) FROM {self._staging_schema}"      # Query engram staging schema for count
        ).fetchone()[0]                                         # Return first element of result tuple
