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
        _EncodingEngine      — sentence-transformers model wrapper with caching
        encode()             — encode a trace or cue into a semantic vector

    Vector Math:
        unit_normalize()     — L2-normalize a vector for cosine-equivalent L2 search
        semantic_search()    — cosine similarity fallback (when sqlite-vec unavailable)

    Storage:
        open_engram()        — open a SQLite connection with WAL mode and row factory

    Lexical:
        sanitize_fts_query() — sanitize a raw query string for safe FTS5 MATCH usage

    Convergence:
        memory_convergence() — RRF fusion of semantic + lexical ranked result lists

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
import math                     # For vector magnitude and relevance scoring
import numpy as np              # For fast vector math — normalization and cosine similarity
import sqlite3                  # For engram connection factory
import struct                   # For packing/unpacking semantic vectors (fp32)

# AGi libraries
from hrs.hrp import AGi         # Import AGi homeostatic regulation parameters
EMC = AGi.CNS.EMC               # Channel for interfacing with Episodic Memory Cortex (EMC)

class EncodingEngine:
    """
    Encoding engine for semantic encoding of memory traces for storage and recall.
    Shared across EMC, SMC, and PMC — loaded once per cortex initialization.
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
        self.logger     = logger                    # Retrieve logger from CNC for logging operations
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
                "   Memory cortices falling back to lexical recall.\n"
                "   Note to technician: pip3 install sentence-transformers --break-system-packages"
            )
        except Exception as e:                                                    # If other errors during activation,
            self.logger.warning(f"⚠️ Encoding Engine activation failed: {e}")     # Log the error during activation of encoding engine

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

        imprint = f"{'cue' if is_cue else 'episode'}:{hash(trace[:EMC.ENCODING_IMPRINT_LIMIT])}"  # Create a unique imprint hash and label the encoding type
        if imprint in self._cache:                                                      # If the imprint is already in the cache,
            return self._cache[imprint]                                                 # Return the encoded vector in the cache

        try:                                                                            # Attempt to encode the trace
            if is_cue:                                                                  # If the trace is a cue for memory recall,
                encoded_trace: list[float] = self._core.encode_query(trace).tolist()    # Encode the cue for memory recall
            else:                                                                       # If the trace is a memory trace to be stored,
                encoded_trace: list[float] = self._core.encode_document(trace).tolist() # Encode the memory trace for storage
            encoded_trace = unit_normalize(encoded_trace)                               # Unit-normalize for cosine-equivalent L2 search

            # Keep cache small — evict oldest if over the encoding cache limit
            if len(self._cache) >= EMC.ENCODING_CACHE_LIMIT:                            # If the cache is over the limit,
                decayed_imprint: str = next(iter(self._cache))                          # Retrieve the decayed imprint (oldest entry)
                del self._cache[decayed_imprint]                                        # Remove the decayed entry from the cache
            self._cache[imprint]: list[float] = encoded_trace                           # Add the new entry to the cache
            return encoded_trace                                                        # Return the encoded vector
        except Exception as e:                                                          # If encoding fails,
            self.logger.debug(f"Encoding error: {e}")                                   # Log the debug message about encoding error
            return []                                                                   # Return empty list to signal that semantic recall cannot be performed

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

def semantic_search(cue: list[float], episode: list[float]) -> float:
    """
    Search a recall cue against a stored episode semantically.
    Cosine similarity fallback — used when sqlite-vec ANN index is unavailable.
    TODO: To be replaced with sqlite-vec ANN search.

    Args:
        cue     (list[float]): Encoded recall cue.
        episode (list[float]): Encoded stored episode.
        * Both cue and episode vectors have alreaddy been unit-normalized — only dot product is needed to equal cosine similarity
    
    Returns:
        float: Semantic relevancy score (0.0 – 1.0).
    """
    if not cue or not episode or len(cue) != len(episode):                      # If either vector is empty or they have different lengths,
        return 0.0                                                              # Return 0.0 as relevancy cannot be computed
    return float(np.dot(np.array(cue), np.array(episode)))                      # Dot product == Compute cosine similarity on the unit vectors

def pack_memory_vector(vector: list[float]) -> bytes:
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


# ── Storage ───────────────────────────────────────────────────────────────────

def open_engram(gateway: str, logger=None) -> sqlite3.Connection:
    """
    Open a SQLite engram connection with WAL mode and row factory.
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
    conn = sqlite3.connect(gateway, check_same_thread=False)    # Connect to engram gateway without thread checking
    conn.row_factory = sqlite3.Row                              # Define the structure of query results of the engram
    conn.execute("PRAGMA journal_mode=WAL;")                    # Set up engram to allow retrieval and storing simultaneously
    conn.execute("PRAGMA synchronous=NORMAL;")                  # Balance episode safety vs storing speed
    conn.commit()                                               # Apply the above parameters into the engram
    return conn                                                 # Return the configured connection


def try_load_sqlite_vec(conn: sqlite3.Connection, logger=None) -> bool:
    """
    Attempt to load the sqlite-vec extension into an engram connection.
    Returns True if successfully loaded, False on failure.

    sqlite-vec provides vec0 virtual tables for L2 distance KNN search.
    Graceful fallback to Python cosine similarity if unavailable.

    Args:
        conn        : An open SQLite connection.
        logger      : Optional logger for warning on failure.

    Returns:
        bool: True if sqlite-vec loaded successfully, False otherwise.
    """
    try:                                                        # Attempt to activate engram vector index
        import sqlite_vec                                       # Initialize SQLite-vec for engram vector index semantic search
        sqlite_vec.load(conn)                                   # Load SQLite-vec into the provided connection
        return True                                             # Signal successful load
    except Exception as e:                                      # If sqlite-vec fails to load,
        if logger:                                              # If a logger is provided,
            logger.warning(                                     # Log the fallback to cosine similarity due to engram vector index unavailability
                f"⚠️ engram vector index not available, falling back to unindexed cosine similarity\n"
                f"   Note to technician: pip3 install sqlite-vec --break-system-packages\n"
                f"   Reason: {e}"
            )
        return False                                            # Signal failed load


# ── Lexical ───────────────────────────────────────────────────────────────────

def sanitize_fts_query(query: str) -> str:
    """
    Sanitize raw query string for safe FTS5 MATCH usage.
    Quotes each token to treat them as literal terms, neutralizing
    FTS5 operators (*, -, ", parentheses, AND, OR, NOT).

    Args:
        query (str): Raw recall cue string from the caller.

    Returns:
        str: FTS5-safe quoted token string, or empty string if query is blank.
    """
    tokens = query.strip().split()
    return " ".join(f'"{t}"' for t in tokens if t)


# ── Memory Convergence (RRF) ──────────────────────────────────────────────────

def memory_convergence(
    semantic_results : list[dict],
    lexical_results  : list[dict],
    recall_limit     : int,
    k                : int = 60,
) -> list[dict]:
    """
    Memory convergence via Reciprocal Rank Fusion (RRF).
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
