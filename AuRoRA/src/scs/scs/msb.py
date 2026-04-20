"""
MSB — Memory Storage Bank
==========================
AuRoRA · Semantic Cognitive System (SCS)
 
Shared memory infrastructure layer — encoding, vector math, lexical search,
storage utilities, and convergence fusion used across all memory cortices.
 
Responsibilities:
    - Provide a shared encoding engine (semantic vector encoding) for EMC, SMC, PMC
    - Provide shared vector math utilities (normalization, cosine similarity)
    - Provide shared SQLite connection factory (WAL mode, row factory)
    - Provide shared lexical search utilities (FTS5 query sanitization)
    - Provide shared RRF memory convergence fusion for dual-path retrieval
 
Architecture:
    Encoding:
        EncodingEngine         — Sentence-transformers model wrapper with caching
        encode()               — Encode a trace or cue into a semantic vector
 
    Vector Math:
        unit_normalize()       — L2-normalize a vector for cosine-equivalent L2 search
        semantic_match()       — Cosine similarity fallback (when sqlite-vec unavailable)
 
    Storage:
        connect_engram()       — Connect to engram (SQLite) with WAL mode and row factory
        activate_engram_index()— Load sqlite-vec extension into a connection
 
    Lexical:
        sanitize_lexical_cue() — Sanitize a raw cue string for safe FTS5 MATCH usage
 
    Convergence:
        memory_convergence()   — RRF fusion of semantic + lexical ranked result lists
 
    EngramStorageBank:
        Storage primitive interface — speaks rows, blobs, counts.
        Zero memory domain vocabulary. EMC, SMC, PMC supply meaning.
        Each cortex gets its own ESB instance pointing at its own table prefix.
 
Used by:
    EMC — Episodic Memory Cortex
    SMC — Semantic Memory Cortex      (M2)
    PMC — Procedural Memory Cortex    (M2)
 
Terminology:
    encoding    — semantic vector representation of a memory trace
    engram      — the memory store containing encoded episodes
    FTS5        — SQLite FTS5 full-text search extension for lexical search
    RRF         — Reciprocal Rank Fusion for combining semantic and lexical search
    unit vector — vector with L2-norm = 1.0 (cosine sim ≡ L2 distance on unit vectors)
 
TODO: migrate pack_vector, unpack_vector, unit_normalize to hrs.py if vector math needed outside memory cortices
"""

# System libraries
from dataclasses import dataclass, field    # For defining memory schema types
from enum import Enum                       # For defining memory schema types
import numpy as np                          # For fast vector math — normalization and cosine similarity
import sqlite3                              # For engram connection factory
import struct                               # For packing/unpacking semantic vectors (fp32)
from pathlib import Path                    # For calculating database size — owned here, never passed back up

# AGi libraries
from hrs.hrp import AGi                     # Import AGi homeostatic regulation parameters
EMC = AGi.CNS.EMC                           # Channel for interfacing with Episodic Memory Cortex (EMC)

# ===== ENGRAM PRIMITIVE STORAGE CONCEPTS =====
# Note: This section defines the fundamental building blocks of engram schemas.
# Each memory cortex (EMC, SMC, PMC) instantiates these concepts to define its own specific schemas.
class EngramModality(Enum):
    """
    Defines the modality of engram memory traces (ie. the type of data stored).
    """
    TEXT    = 'TEXT'    # String storage for lexical recall and encoding
    INTEGER = 'INTEGER' # Integer storage for primary key and other integer values
    REAL    = 'REAL'    # Real-number storage for storing encoding vector values (floats)
    BLOB    = 'BLOB'    # Blob storage for storing packed semantic vectors

@dataclass
class EngramTrace:
    """
    Define the schema for a single engram trace.
    """
    label: str                       # Label of the engram field (ie. name of the data stored)
    modality: EngramModality         # Modality of the engram field (ie. type of data stored)
    essential: bool = False          # Whether the field is essential to the engram (default: False)
    baseline: str | None = None      # Baseline value for the engram field (default: None)

@dataclass
class EngramSchema:
    """
    Define the structure, storage schema, and search capabilities for an engram.
    """
    storage: list[EngramTrace]                                  # The storage schema for the engram
    staging: list[EngramTrace] | None = None                    # The staging schema for the engram (used for temporary storage)
    semantic_traces: str | None = None                          # The semantic trace for the engram used for semantic search
    lexical_traces: list[str] | None = None                     # The lexical fields for the engram used for lexical search
    index_traces: list[str] | None = None                       # The index fields for the engram used for fast retrieval

# ===== ENCODING ENGINE =====
class EncodingEngine:
    """
    Encoding engine for semantic encoding of memory traces for storage and recall.
    Shared across EMC, SMC, and PMC — loaded once per cortex initialization.
    Primes recent encodings to avoid redundant encoding of identical or similar episodes
    and to speed up subsequent recall.
    """

    def __init__(self, logger, encoding_engine: str, cache_limit: int = 256, imprint_limit: int = 300) -> None:
        """
        Initialize the encoding engine with a logger.
        This method sets up the encoding engine core and cache for recent encodings.
        
        Args:
            logger               : Logger instance for logging encoding engine operations
            encoding_engine (str): The specific embedding model to load (e.g. from HRP)
            cache_limit     (int): Maximum number of entries in the LRU encoding cache
            imprint_limit   (int): Maximum number of characters to hash for the imprint
        """
        self.logger                         = logger                # Retrieve logger from CNC for logging operations
        self.encoding_engine: str           = encoding_engine       # Store the model name for dynamic formatting during encoding
        self.cache_limit: int               = cache_limit           # Maximum number of imprints to hold in cache to control memory usage
        self.imprint_limit: int             = imprint_limit         # Maximum length of the imprint to control cache hit rate vs false positive risk
        self._core                          = None                  # For holding the encoding engine instance for semantic encoding
        self._cache: dict[str, list[float]] = {}                    # For holding the cache of recent encodings to avoid redundant encoding

        try:                                                                        # Attempt to activate the encoding engine
            from sentence_transformers import SentenceTransformer                   # Load inferencing component of encoding engine
            self.logger.info(f"⏳ Activating Encoding Engine ({encoding_engine})…") # Log the start of encoding engine activation
            self._core = SentenceTransformer(self.encoding_engine)                  # Activate the requested model
            self.logger.info("✅ Encoding Engine activated")                        # Log the successful activation of encoding engine
        except ImportError:                                                         # If missing the inferencer component of encoding engine,
            self.logger.warning(                                                    # Log the warning about encoding engine being offline and falling back to lexical retrieval
                "⚠️ Encoding Engine offline - missing inferencing component.\n"
                "   Memory cortices falling back to lexical recall.\n"
                "   Note to technician: pip3 install sentence-transformers --break-system-packages"
            )
        except Exception as e:                                                      # If other errors during activation,
            self.logger.warning(f"⚠️ Encoding Engine activation failed: {e}")       # Log the error during activation of encoding engine

    @property
    def is_available(self) -> bool:
        """
        Check if encoding engine is loaded successfully and ready for encoding.
        This is used by memory cortices to determine whether semantic search is available,
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
        prompting the caller to fall back to lexical recall.
        
        Args:
            trace (str): The given memory trace to encode (e.g. episode content).
            is_cue (bool): Whether the trace is a recall cue (True) or an episode to be stored (False).
                             This allows for separate caching of cue and episode encodings, which may have different patterns of repetition.

        Returns:
            list[float]: The semantic encoding vector for the input trace, 
                         or an empty list if the encoding engine is unavailable.
                         Empty list signals the caller to fall back to lexical recall.
        """
        if not self.is_available:                                                       # If encoding engine is unavailable,
            self.logger.debug(                                                          # Log the debug message about encoding engine being unavailable
                "Encoding engine unavailable — semantic search inactive"
            )
            return []                                                                   # Return empty list to signal that semantic recall cannot be performed

        imprint = f"{'cue' if is_cue else 'episode'}:{hash(trace[:self.imprint_limit])}"# Create a unique imprint hash and label the encoding type
        if imprint in self._cache:                                                      # If the imprint is already in the cache,
            return self._cache[imprint]                                                 # Return the encoded vector in the cache

        try:                                                                            # Attempt to encode the trace
            prompt_name = "query" if is_cue else "document"                             # Assign the appropriate prompt name based on whether the trace is a cue or an episode to be encoded
            encoded_trace: list[float] = self._core.encode(trace, prompt_name=prompt_name).tolist() # Encode the trace using the encoding engine
            encoded_trace = unit_normalize(encoded_trace)                               # Unit-normalize for cosine-equivalent L2 search

            # Keep cache small — evict oldest if over the encoding cache limit
            if len(self._cache) >= self.cache_limit:                                    # If the cache is over the limit,
                decayed_imprint: str = next(iter(self._cache))                          # Retrieve the decayed imprint (oldest entry)
                del self._cache[decayed_imprint]                                        # Remove the decayed entry from the cache
            self._cache[imprint] = encoded_trace                                        # Add the new entry to the cache
            return encoded_trace                                                        # Return the encoded vector
        except Exception as e:                                                          # If encoding fails,
            self.logger.debug(f"Encoding error: {e}")                                   # Log the debug message about encoding error
            return []                                                                   # Return empty list to signal that semantic recall cannot be performed

# ===== VECTOR OPERATIONS =====
def unit_normalize(vector: list[float]) -> list[float]:
    """
    Ensure encoding engine conduct semantic search properly by
    unit-normalizing a vector so that L2 distance search is equivalent to cosine similarity.

    sqlite-vec uses Euclidean (L2) distance, not cosine similarity. For unit vectors,
    cosine_sim(a, b) == 1 - L2(a, b)² / 2, so normalizing before insert/query makes
    L2 nearest-neighbor search semantically equivalent to cosine nearest-neighbor search.

    BGE-base outputs near-unit vectors, but normalization may be eroded by dtype
    conversion or serialization. This function is a zero-cost safety net: already
    unit-normalized vectors (‖v‖ ≈ 1.0 within 1e-6) are returned as-is. Zero vectors
    are returned unchanged to avoid division by zero.

    Args:
        vector: A float vector, e.g. from an embedding model.

    Returns:
        list[float]: A unit-normalized copy of vector, or vector itself if already normalized or zero.
    """
    vector_array = np.array(vector)                                                # Convert to NumPy array for fast vector math
    vector_mag = np.linalg.norm(vector_array)                                      # Compute L2-norm - Euclidean magnitude of the vector
    if abs(vector_mag - 1.0) < 1e-6:                                               # If the vector is already unit-normalized,
        return vector                                                              # Return the original vector as-is
    return (vector_array / vector_mag).tolist() if vector_mag > 0.0 else vector    # Return unit-normalized vector if magnitude larger than 0, otherwise return original vector

def pack_vector(vector: list[float]) -> bytes:
    """
    Pack a float vector into fp32 binary blob for engram storage.
    Used before INSERT into episodes and engram_vectors.

    Args:
        vector (list[float]): Semantic encoding vector.

    Returns:
        bytes: Binary blob of fp32 values.
    """
    return struct.pack(f"{len(vector)}f", *vector)                              # Pack encoded vector as fp32 binary for engram storage

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
    return list(struct.unpack(f"{dim}f", blob))                                 # Unpack fp32 binary blob back into a float vector

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
    if not cue or not episode or len(cue) != len(episode):                      # If either vector is empty or they have different lengths,
        return 0.0                                                              # Return 0.0 as relevancy cannot be computed
    return float(np.dot(np.array(cue), np.array(episode)))                      # Dot product == cosine similarity on unit vectors
    
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
    engram_conn = sqlite3.connect(gateway, check_same_thread=False)    # Allow cross-thread engram access for concurrent cortex connections
    engram_conn.row_factory = sqlite3.Row                              # Define the structure of query results of the engram
    engram_conn.execute("PRAGMA journal_mode=WAL;")                    # Set up engram to allow retrieval and storing simultaneously
    engram_conn.execute("PRAGMA synchronous=NORMAL;")                  # Balance episode safety vs storing speed
    engram_conn.commit()                                               # Apply the above settings into the engram
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
        import sqlite_vec                                       # Load sqlite-vec extension for engram vector index
        sqlite_vec.load(engram_conn)                            # Activate vector index on the engram connection
        return True                                             # Signal successful activation
    except Exception as e:                                      # If sqlite-vec fails to load,
        if logger:                                              # If a logger is provided,
            logger.warning(                                     # Log the fallback to cosine similarity due to engram vector index unavailability
                f"⚠️ engram vector index not available, falling back to unindexed cosine similarity\n"
                f"   Note to technician: pip3 install sqlite-vec --break-system-packages\n"
                f"   Reason: {e}"
            )
        return False                                            # Signal failed activation — caller falls back to cosine similarity

def sanitize_lexical_cue(cue: str) -> str:
    """
    Sanitize raw cue string for safe FTS5 MATCH usage.
    Quotes each token to treat them as literal terms, neutralizing
    FTS5 operators (*, -, ", parentheses, AND, OR, NOT).

    Args:
        cue (str): Raw recall cue string from the caller.

    Returns:
        str: FTS5-safe quoted token string, or empty string if query is blank.
    """
    lexemes = cue.strip().split()                                    # Split cue into individual lexemes for FTS5 matching
    return " ".join(f'"{lexeme}"' for lexeme in lexemes if lexeme)   # Wrap each lexeme in quotes — neutralizes FTS5 operators

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
    episode_pool: dict[int, dict] = {}                                        # Union pool of all candidate episodes from both paths
    semantic_rank: dict[int, int] = {}                                        # Semantic rank lookup — episode_id → 0-based rank
    lexical_rank: dict[int, int] = {}                                         # Lexical rank lookup  — episode_id → 0-based rank

    for rank, episode in enumerate(semantic_results):                         # Iterate through each semantic match results
        semantic_rank[episode["id"]] = rank                                   # Store semantic rank — 0-based, best = 0
        episode_pool[episode["id"]] = episode                                 # Add episode to pool for later retrieval

    for rank, episode in enumerate(lexical_results):                          # Iterate through each lexical match results
        lexical_rank[episode["id"]] = rank                                    # Store lexical rank — 0-based, best = 0
        episode_pool.setdefault(episode["id"], episode)                       # Add episode to pool if not already from semantic path

    semantic_miss = len(semantic_results)                                     # Penalty rank for episodes missed by semantic path
    lexical_miss = len(lexical_results)                                       # Penalty rank for episodes missed by lexical path

    # Compute RRF score for each candidate
    for episode in list(episode_pool.values()):                               # Iterate through a clone of episode pool — safe to prevent mutation to original pool
        episode = dict(episode)                                               # Obtain a clone of the episode — prevent mutation of original episode
        episode["rrf_score"] = (                                              # Calculate rrf score of each episode from the ranks of both paths
            1.0 / (rrf_k + semantic_rank.get(episode["id"], semantic_miss)) +
            1.0 / (rrf_k + lexical_rank.get(episode["id"], lexical_miss))
        )
        episode_pool[episode["id"]] = episode                                 # Replace original episode entry with rrf score added

    # Sort descending by RRF score
    sorted_episodes = sorted(episode_pool.values(),                           # Sort episodes by RRF score — best first
                             key=lambda episode: episode["rrf_score"], 
                             reverse=True)

    # Normalise RRF scores to 0.0–1.0 for the 'relevancy' field
    max_rrf = sorted_episodes[0]["rrf_score"] if sorted_episodes else 1.0     # Computing best RRF score for normalization — guard against empty results

    for episode in sorted_episodes[:recall_limit]:                            # Surface top episodes up to recall limit
        episode["relevancy"] = episode["rrf_score"] / max_rrf                 # Normalize RRF score to 0.0–1.0 relevancy
        episode.pop("_rank", None)                                            # Remove internal rank field before surfacing
        episode.pop("rrf_score", None)                                        # Remove internal RRF score before surfacing
    
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
        This method creates the necessary tables and indexes for storing episodic memories.

        Episodic Buffer (2-layer)
            _binding_stream  —  transient intake of evicted PMTs from WMC overflow
            episodic_buffer  — crash-safe pre-consolidation staging of episodes
        One Table + Two Indexes SQLite design:
            episodes         — embedded, searchable episodic memory (intermediate)
            engram_vectors   — semantic vectors for L2 distance semantic search
            engram_lexical   — FTS5 lexical index for pattern separation recall
        """
        self._conn.executescript("""                                        -- Create the tables and indexes of episodic memory
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
        self._conn.commit()                                                 # Commit the changes to the engram

        if self._engram_index:                                              # If engram vector index is activated,
            self._conn.execute(f"""                                         -- Create a virtual schema for engram vector index semantic search
                CREATE VIRTUAL TABLE IF NOT EXISTS {self._storage_schema}_vectors USING vec0(
                    encoding FLOAT[{self._engram_dim}]                      -- L2 distance semantic search on unit-normalized vectors (cosine-equivalent)
                )
            """)
            self._conn.commit()                                             # Commit the changes to the engram
            self.logger.debug("EMC engram vector index initialized")        # Log the initialization of the engram vector index

        self._conn.execute(f"""                                             -- Create a virtual schema for the lexical search using FTS5 index
            CREATE VIRTUAL TABLE IF NOT EXISTS {self._storage_schema}_lexical USING fts5(   -- FTS5 index for lexical search
               content,                                                     -- Content of the episode
               tokenize='porter unicode61'                                  -- Tokenize the content using porter unicode61
            )
        """)
        self._conn.commit()                                                 # Commit the changes to the engram
        self.logger.debug("EMC FTS5 lexical index initialized")             # Log the initialization of the FTS5 lexical index

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
            "INSERT INTO episodic_buffer (timestamp, date, content) "
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
            "SELECT id, timestamp, date, content "                          # Collect the index, timestamp, date, and content
            "FROM episodic_buffer "                                         # All episodes in the episodic buffer is unencoded by definition
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
            "SELECT COUNT(*) FROM episodic_buffer"
        ).fetchone()[0]

    def delete_staged(self, conn: sqlite3.Connection, staging_id: int) -> None:
        """
        Remove a staged episode from the episodic buffer after synaptic consolidation.

        Args:
            conn       : Encoder connection (separate from main engram connection)
            staging_id : ID of the staged episode to remove
        """
        conn.execute(                                                           # Remove the episode from episodic buffer after synaptic consolidation
            "DELETE FROM episodic_buffer WHERE id=?",                           # Remove the episode from episodic buffer
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
            "INSERT INTO episodes (timestamp, date, content, encoding) VALUES (?,?,?,?)",   # Insert the episode into engram
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
            "INSERT INTO engram_vectors (rowid, encoding) VALUES (?,?)",        # Insert the episode encoding into engram vectors
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
            "INSERT INTO engram_lexical (rowid, content) VALUES (?,?)",         # Insert the episode content into engram lexical
            [episode_id, content],                                              # With the episode ID and content
        )

    # ── Search ────────────────────────────────────────────────────────────────

    def search_knn(self, cue_blob: bytes, limit: int) -> list:
        """
        Search the engram vector index for the top KNN episodes.
        Primary semantic search path — sqlite-vec L2 distance KNN
        (cosine-equivalent on unit-normalized vectors).

        Args:
            cue_blob : Binary blob of the encoded recall cue
            limit    : Maximum number of candidates to return

        Returns:
            list: Rows of matching episodes with distance scores
        """
        return self._conn.execute(                                              # Query the engram vector index and retrieve the top recall_limit * 2 episodes
            """
            SELECT e.id, e.timestamp, e.date, e.content,                        -- Retrieve the episode ID, timestamp, date, and content
                   ev.distance                                                  -- Retrieve the semantic similarity between the cue and the episode
            FROM engram_vectors ev                                              -- From the engram vector index virtual table
            JOIN episodes e ON e.id = ev.rowid                                  -- Join with the episodes table using the row ID
            WHERE ev.encoding MATCH ?                                           -- Match the cue vector
            ORDER BY ev.distance                                                -- Order by ascending order of semantic similarity (closest first)
            LIMIT ?                                                             -- Limit to recall_limit * 2 episodes
            """,
            [cue_blob, limit],                                                  # Retrieve the top limit episodes
        ).fetchall()                                                            # Fetch all matching episode candidates

    def search_cosine_pool(self, half_pool: int) -> list:
        """
        Fetch a stratified pool of episodes for cosine similarity fallback.
        Stratified sampling — half recent, half oldest.
        Bounds memory usage and ensures old memories score alongside recent ones.

        Args:
            half_pool : Half the total pool size (recent + oldest)

        Returns:
            list: Rows of episodes with encoding blobs for cosine scoring
        """
        return self._conn.execute(                                          # Attempt to use cosine similarity
            "SELECT id, timestamp, date, content, encoding FROM episodes "
            "ORDER BY rowid DESC LIMIT ? "
            "UNION ALL "
            "SELECT id, timestamp, date, content, encoding FROM episodes "
            "ORDER BY rowid ASC LIMIT ?",
            [half_pool, half_pool],
        ).fetchall()

    def search_lexical(self, safe_query: str, limit: int) -> list:
        """
        Lexical pattern separation search via FTS5.
        Dentate gyrus analogue — precise, keyword-driven engram retrieval.

        FTS5 rank() is negative (more negative = better). We normalise to
        a 0.0–1.0 relevancy score so RRF can treat both paths uniformly.

        Args:
            safe_query : Sanitized FTS5 query string
            limit      : Maximum candidates to return before RRF fusion

        Returns:
            list: Rows of matching episodes with raw FTS5 rank scores
        """
        return self._conn.execute(                                          # Fetch 2× recall_limit candidates — RRF will cull to recall_limit after fusion
            """
            SELECT e.id, e.timestamp, e.date, e.content,
                   fts.rank AS raw_rank
            FROM engram_lexical fts
            JOIN episodes e ON e.id = fts.rowid
            WHERE engram_lexical MATCH ?
            ORDER BY fts.rank          -- most negative = best
            LIMIT ?
            """,
            [safe_query, limit],
        ).fetchall()

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
                "FROM episodes"
            ).fetchone()

            buf_row = self._conn.execute(
                "SELECT COUNT(*) as total FROM episodic_buffer"
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

 # ── Schema ————————————————————————————————————————————————————————————————
    def build_schema(self) -> None:
        """
        Build the schema for the memory bank.
        Generates the SQL table based on the injected EngramSchema.
        """
        try:
            columns = []
            for trace in self._schema.storage:                                  # Iterate over all defined traces in the schema
                col_def = f"{trace.label} {trace.modality.value}"               # Define the column name and SQLite datatype (e.g., 'content TEXT')
                
                if trace.label == "id":                                         # If the field is the id, mark it as primary key
                    col_def += " PRIMARY KEY"
                elif trace.essential:                                           # If the field is essential, ensure it cannot be null
                    col_def += " NOT NULL"
                    
                if trace.baseline is not None:                                  # If there is a baseline value, assign the SQLite default
                    if trace.modality == EngramModality.TEXT:
                        col_def += f" DEFAULT '{trace.baseline}'"               # Text defaults need quotes
                    else:
                        col_def += f" DEFAULT {trace.baseline}"                 # Numeric/boolean defaults do not
                        
                columns.append(col_def)                                         # Append the column definition to the list
                
            columns_sql = ",\n                    ".join(columns)               # Join all columns into a single string for the SQL statement

            self._conn.execute(
                f"""CREATE TABLE IF NOT EXISTS {self._storage_schema} (
                    {columns_sql}
                )"""
            )
            self._conn.commit()
        except Exception as e:
            self.logger.error(f"EMC build_schema failed: {e}")
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