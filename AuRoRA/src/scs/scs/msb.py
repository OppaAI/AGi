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
    blueprint       — EngramSchema instance defining the structure of a memory bank
    encoding        — semantic vector representation of a memory trace
    engram          — one physical stored memory record
    engram complex  — the SQLite database holding all engrams across cortices
    FTS5            — SQLite full-text search for lexical pattern retrieval
    memory          — the contextual meaning of an engram
    memory bank     — the abstraction layer managing the engram complex
    prime           — LRU encoding cache mapping prime keys to encoding vectors
    prime key       — truncated hash of a memory trace used for prime lookup
    RRF             — Reciprocal Rank Fusion — fuses semantic and lexical ranked
                      results into a single relevancy-ordered list
    transcript      — SQL column definition string generated from a blueprint
    unit vector     — L2-normalized vector; cosine similarity becomes equivalent
                      to L2 distance, enabling sqlite-vec KNN search
                  
TODO: migrate pack_vector, unpack_vector, normalize_vector to hrs.py if
      vector math is needed outside memory cortices
"""

# System libraries
from dataclasses import dataclass, field    # dataclass for EngramTrace/EngramSchema, field for default_factory
from enum import Enum                       # enum base for EngramModality type definitions
import numpy as np                          # for fast vector math — normalization and cosine similarity
import sqlite3                              # for engram connection factory
import struct                               # for packing/unpacking semantic vectors (fp32)
import re                                   # for lexical cue sanitization
from pathlib import Path                    # for calculating database size — owned here, never passed back up

class EngramModality(Enum):
    """
    Defines the modality of a single trace (i.e., the type of data stored).
    """
    TEXT    = 'TEXT'        # SQLite TEXT type — stores strings
    INTEGER = 'INTEGER'     # SQLite INTEGER type — stores whole numbers
    REAL    = 'REAL'        # SQLite REAL type — stores floating point numbers
    BLOB    = 'BLOB'        # SQLite BLOB type — stores raw bytes (packed vectors)

@dataclass
class EngramTrace:
    """
    Defines a single trace in an engram blueprint.
    """
    label: str                       # SQL column name — e.g. "content", "encoding"
    modality: EngramModality         # SQL column type — .value drops into CREATE TABLE as raw SQL
    essential: bool = False          # adds NOT NULL constraint if True
    baseline: str | None = None      # set adds DEFAULT — raw SQL if starts with '(', quoted string otherwise
 
@dataclass
class EngramSchema:
    """
    Defines the full structure of an engram complex's trace storage and search indexes.
    """
    storage: list[EngramTrace]                      # column defintions for the main storage table
    staging: list[EngramTrace] | None = None        # column defintions for the crash-safe staging table — optional
    semantic_traces: str | None = None              # column name holding the encoding BLOB — used for KNN vector search
    lexical_traces: list[str] | None = None         # column names fed into FTS5 — used for keyword search
    index_traces: list[str] | None = None           # column names to B-tree index — speeds up WHERE/ORDER BY

class EncodingEngine:
    """
    Encoding engine for semantic encoding of memory traces for storage and recall.
    Shared across EMC, SMC, and PMC — loaded once per cortex initialization.
    Primes recent encodings to avoid redundant encoding of identical or similar memory traces
    and to speed up subsequent recall.
    """

    def __init__(self, logger, encoding_engine: str, prime_limit: int = 256, prime_key_limit: int = 300) -> None:
        """
        Initializes the encoding engine and encoding prime for recent memory traces.
    
        Args:
            logger                   : Logger instance passed from the cortex
            encoding_engine (str)    : Embedding model to load (e.g. from HRP)
            prime_limit     (int)    : Maximum number of entries in the encoding prime
            prime_key_limit (int)    : Maximum characters hashed for the prime key
        """
        self.logger                         = logger                # logger from cortex — used throughout this class
        self.encoding_engine: str           = encoding_engine       # model name string — passed to SentenceTransformer()
        self.prime_limit: int               = prime_limit           # max prime entries before LRU eviction
        self.prime_key_limit: int           = prime_key_limit       # max chars hashed for prime key
        self._core                          = None                  # live SentenceTransformer model — None until load succeeds
        self._prime: dict[str, list[float]] = {}                    # encodig prime — maps prime key → float vector
        
        try:                                                                        # attempt to activate SentenceTransformer
            from sentence_transformers import SentenceTransformer                   # deferred import — avoids hard crash if package missing
            self.logger.info(f"⏳ Activating Encoding Engine ({encoding_engine})…") # logs before the blocking load
            self._core = SentenceTransformer(self.encoding_engine)                  # loads model weights into RAM — blocks until complete
            self.logger.info("✅ Encoding Engine activated")                        # only reached if load succeeded
        except ImportError:                                                         # triggers if sentence_transformers package not installed
            self.logger.warning(                                                    # package not installed — _core stays None
                "⚠️ Encoding Engine offline - missing inferencing component.\n"
                "   Memory cortices falling back to lexical recall.\n"
                "   Note to technician: pip3 install sentence-transformers --break-system-packages"
            )
        except Exception as e:                                                      # bad model path, corrupted weights, OOM — _core stays None
            self.logger.warning(f"⚠️ Encoding Engine activation failed: {e}")       # logs specific failure with reason
            
    @property
    def is_available(self) -> bool:
        """
        Returns True if the encoding engine is loaded and ready.
    
        Returns:
            bool: True if ready for encoding, False if failed to load.
        """
        return self._core is not None            # True only if SentenceTransformer loaded successfully

    def encode(self, trace: str, is_cue: bool = False) -> list[float]:
        """
        Encodes a memory trace into a semantic vector for storage or recall.
        Primes recent encodings to speed up subsequent recall.
        Returns empty list if encoding engine is unavailable — signals caller to fall back to lexical search.
        
        Args:
        trace  (str) : Memory trace to encode — episode content or recall cue
        is_cue (bool): If True, prepends BGE instruction prefix before encoding — cues and episodes
                       encode differently so identical text gets separate prime entries

        Returns:
            list[float]:  Semantic encoding vector, or empty list if engine unavailable
        """
        if not self.is_available:                                                       # _core is None, skip encoding entirely
            self.logger.debug(                                                          # log the SentenceTransformer being unavailable
                "Encoding engine unavailable — semantic search inactive"
            )
            return []                                                                   # empty list signals fallback to lexical search

        prime_key = f"{'cue' if is_cue else 'episode'}:{hash(trace[:self.prime_key_limit])}"# prime key — prefixed by types so same text encodes separately as cue vs episode
        if prime_key in self._prime:                                                      # O(1) dict lookup
            return self._prime[prime_key]                                                 # prime hit — skips model inference

        try:                                                                            # attempt to embed the query or engram vector
            cue_prefix = "Represent this sentence for searching relevant passages: "    # BGE instruction prefix — applied to recall cues only , not stored episodes
            encoded_trace: list[float] = self._core.encode(cue_prefix + trace if is_cue else trace).tolist() # model inference — .tolist() converts ndarray → float list
            encoded_trace = normalize_vector(encoded_trace)                               # normalizes to unit length — required so L2 == cosine sim in sqlite-vec
            
            # Keep prime small — evict oldest if over the encoding prime limit
            if len(self._prime) >= self.prime_limit:                                    # prime full — must evict before inserting
                decayed_prime_key: str = next(iter(self._prime))                          # first key = oldest — dicts preserve insertion order
                del self._prime[decayed_prime_key]                                        # evicts oldest entry
            self._prime[prime_key] = encoded_trace                                        # stores new vector under prime key
            return encoded_trace                                                        # returns encoded and normalized vector
        except Exception as e:                                                          # if embedding fails,
            self.logger.debug(f"Encoding error: {e}")                                   # logs encoding errors with reason
            return []                                                                   # same empty list fallback as unavailable guard
            

class EngramComplex:
    """
    Engram Complex (ECX) — memory storage bank abstraction layer.
    Owns all SQL operations for memory cortex — schema creation, staging, consolidation,
    search, and retrieval. The memory cortex never touches SQL directly.

    TODO: extend engram complex with SMC and PMC tables in M2.
    TODO: backend swap — Qdrant, pgvector, or other vector DB — swap here only,
          cortex cognitive logic untouched.
    """
    
    LEXICAL_FILTER_WORDS = {                                                            # set literal — O(1) membership test in sanitize_lexical_cue
            "a", "an", "the", "is", "are", "was", "were", "what", "how", "why", "where", 
            "when", "who", "which", "in", "on", "at", "to", "for", "of", "with", "by", 
            "from", "as", "and", "or", "but", "if", "then", "else", "my", "your", "our", 
            "their", "his", "her", "its", "me", "you", "him", "them", "us", "i", "can", 
            "could", "would", "should", "will", "shall", "do", "does", "did", "have", 
            "has", "had", "it", "this", "that", "those", "these"
        }

    def __init__(self, logger, cortex: str, gateway: str, schema: EngramSchema, dim: int) -> None:
        """
        Initializes the memory bank, connects to the engram complex, and builds the schema.

        Args:
            logger                  : Logger instance passed from the cortex
            cortex (str)            : The cortex which this engrma complex belongs to (e.g. "emc", "smc", "pmc")
            gateway (str)           : Gateway to access the engram complex
            schema (EngramSchema)   : Blueprint defining the engram complex structure
            dim (int)               : Vector dimension of the engram complex
        """
        self.logger                     = logger                  # logger instance passed from caller
        self._storage_schema: str       = f"{cortex}_storage"     # table name for main storage
        self._staging_schema: str       = f"{cortex}_staging"     # table name for crash-safe buffer
        self._vector_schema: str        = f"{cortex}_vector"      # virtual table name for KNN search
        self._lexical_schema: str       = f"{cortex}_lexical"     # virtual table name for FTS5 search
        self._gateway: str              = gateway                 # gateway to access the engram complex
        self._blueprint: EngramSchema   = schema                  # blueprint driving dynamic table generation
        self._conn: sqlite3.Connection  = self._connect()         # opens and configures SQLite connection
        self._vector_dim: int           = dim                     # vector dimension — used in vec0 CREATE VIRTUAL TABLE
        self._vector_index: bool        = self._activate_index()  # True if sqlite-vec loaded successfully
        self.build_schema()                                       # creates all tables and indexes from blueprint

    def _connect_ecx(self) -> sqlite3.Connection:
        """
        Establishes connection to the engram complex for concurrent cortex access.
        WAL mode allows concurrent reads during writes.
        NORMAL synchronous mode balances safety vs write speed.

        Returns:
            sqlite3.Connection: Configured connection with WAL mode and row factory.

        Raises:
            sqlite3.Error: If connection cannot be established.
        """
        try:
            ecx_conn = sqlite3.connect(self._gateway, check_same_thread=False)  # opens SQLite file at given path — check_same_thread=False allows access from multiple threads
            ecx_conn.row_factory = sqlite3.Row                                  # rows returned as dict-like objects — columns accessible by name instead of index
            ecx_conn.execute("PRAGMA journal_mode=WAL;")                        # WAL mode — concurrent reads allowed during writes
            ecx_conn.execute("PRAGMA synchronous=NORMAL;")                      # fsync on checkpoint only — faster writes, safe enough for embedded
            ecx_conn.commit()                                                   # commits pragma changes before returning
            return ecx_conn                                                     # return the configured connection
        except sqlite3.Error as e:                                              # if error during connection to database,
            self.logger.error(f"Engram complex connection failed: {e}")         # logs failure before re-raising
            raise                                                               # re-raise to let caller handle the error

    def _activate_vector_index(self) -> bool:
        """
        Activates the vector index for semantic search.
        Graceful fallback to cosine similarity if index unavailable.
        
        Returns:
            bool: True if vector index activated, False otherwise.
        """
        try:                                                        # attempt to activate engram vector index
            import sqlite_vec                                       # deferred import — avoids hard crash if package missing
            sqlite_vec.load(self._conn)                             # loads vec0 virtual table support into this connection
            return True                                             # signals KNN search is available
        except Exception as e:                                      # if sqlite-vec fails to load,
            if self.logger:                                         # if a logger is provided,
                self.logger.warning(                                # log the fallback to cosine similarity due to engram vector index unavailability
                    f"⚠️ engram vector index not available, falling back to unindexed cosine similarity\n"
                    f"   Note to technician: pip3 install sqlite-vec --break-system-packages\n"
                    f"   Reason: {e}"
                )
            return False                                            # signals caller to fall back to cosine similarity

    def build_schema(self) -> None:
        """
        Initialize the schema of the engram.
        This method dynamically creates tables and indexes based on the injected EngramSchema.
        """
        try:

            # 1. Build Storage Table
            storage_transcript = self._transcribe_traces(self._blueprint.storage)            # converts main storage EngramTrace list → SQL column definition string
            self._conn.execute(f"""                                                          # create table according to the given blueprint
                CREATE TABLE IF NOT EXISTS {self._storage_schema} (
                    {storage_transcript}
                )
            """)
   
            # 2. Build Staging Table (if defined)
            if self._blueprint.staging:                                                       # staging is optional — not all cortices need a buffer
                staging_transcript = self._transcribe_traces(self._blueprint.staging)         # converts staging EngramTrace list → SQL column definition string
                self._conn.execute(f"""                                                       # create table according to the given blueprint
                    CREATE TABLE IF NOT EXISTS {self._staging_schema} (
                        {staging_transcript}
                    )
                """)

            # 3. Build Indexes
            if self._blueprint.index_traces:                                                   # optional — only if cortex defined index columns
                for trace in self._blueprint.index_traces:                                     # one B-tree index per column
                    self._conn.execute(f"""                                                    # create index for fast retrieval
                        CREATE INDEX IF NOT EXISTS idx_{self._storage_schema}_{trace}
                        ON {self._storage_schema}({trace})
                    """)
            self._conn.commit()                                                                # commits tables and indexes together

            # 4. Build Vector Search Virtual Table
            if self._engram_index and self._blueprint.semantic_traces:                         # only if sqlite-vec loaded and cortex defined a vector column
                semantic_col = self._blueprint.semantic_traces                                 # column name holding the encoding BLOB
                self._conn.execute(f"""                                                        # creates vec0 virtual table for KNN search
                    CREATE VIRTUAL TABLE IF NOT EXISTS {self._vector_schema} USING vec0(
                        {semantic_col} FLOAT[{self._engram_dim}]
                    )
                """)
                self._conn.commit()                                                            # commits the virtual table to the memory bank
                self.logger.debug(f"Engram vector index initialized for {self._vector_schema}")    # log the successful initialization of engram vector index

            # 5. Build Lexical Search Virtual Table (FTS5)
            if self._blueprint.lexical_traces:                                                 # only if cortex defined lexical columns
                lexical_transcript = ", ".join(self._blueprint.lexical_traces)                 # joins column names into comma-separated string for FTS5 definition
                self._conn.execute(f"""                                                        # creates FTS5 virtual table for keyword search
                    CREATE VIRTUAL TABLE IF NOT EXISTS {self._lexical_schema} USING fts5(
                       {lexical_transcript},
                       tokenize='porter unicode61'
                    )
                """)
                self._conn.commit()                                                            # commits the virtual table to the memory bank
                
        except sqlite3.Error as e:                                                             # if error occurs during building schema
            self.logger.error(f"Engram schema initialization failed: {e}")                     # log the failed initialization with reason
            raise                                                                              # re-raises — schema failure is unrecoverable, cortex cannot function

    def normalize_vector(vector: list[float]) -> list[float]:
        """
        Normalizes an encoding vector to unit length for cosine-equivalent L2 distance search.
        Already-normalized vectors and empty vectors are returned unchanged.

        Args:
            vector (list[float]): Vector to normalize

        Returns:
            list[float]: A unit-normalized copy of vector, or vector itself if already normalized or empty
        """
        vector_array = np.array(vector)                                                # list → ndarray for vectorized math
        vector_mag = np.linalg.norm(vector_array)                                      # L2 norm — Euclidean length of the vector
        if vector_mag > 0 and abs(vector_mag - 1.0) > 1e-6:                            # tolerance check — exact == 1.0 unreliable with floats
            return (vector_array / vector_mag).tolist()                                # already unit length — skip normalization
        return vector                                                                  # divide each element by magnitude → unit vector; zero vector guard

    def pack_vector(vector: list[float]) -> bytes:
        """
        Packs an encoding vector into binary blob for engram storage.

        Args:
            vector (list[float]): Semantic encoding vector.

        Returns:
            bytes: Binary blob of fp32 values.
        """
        return struct.pack(f"{len(vector)}f", *vector)              # packs float list into fp32 binary blob — e.g. "384f" for 384 floats

    def unpack_vector(blob: bytes, dim: int) -> list[float]:
        """
        Unpacks a binary blob back into an encoding vector.

        Args:
            blob (bytes): Binary blob stored in the engram.
            dim  (int)  : Expected vector dimension.

        Returns:
            list[float]: Unpacked float vector.
        """
        return list(struct.unpack(f"{dim}f", blob))                 # reverses pack_vector — dim must match original or values corrupt silently

    def semantic_match(cue: list[float], episode: list[float]) -> float:
        """
        Measures semantic relevancy between a recall cue and a stored episode.
        Fallback when sqlite-vec is unavailable.
        TODO: To be replaced with sqlite-vec ANN search.

        Args:
            cue     (list[float]): Encoded recall cue.
            episode (list[float]): Encoded stored episode.
        
        Returns:
            float: Semantic relevancy score (0.0 – 1.0).
        """
        if not cue or not episode or len(cue) != len(episode):      # guards empty vectors and dimension mismatch
            return 0.0                                              # no match rather than exception
        return float(np.dot(np.array(cue), np.array(episode)))      # dot product == cosine sim on already unit-normalized vectors
        
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
        clean_cue = re.sub(r'[^\w\s]', ' ', cue.lower())                 # strip punctuation → space, lowercases — FTS5 MATCH rejects special chars
        lexemes = clean_cue.strip().split()                              # splits on whitespace → list of tokens
        
        # Extract keywords (non-stop-words longer than 1 character)
        terms = [kw for kw in lexemes if kw not in LEXICAL_FILTER_WORDS and len(kw) > 1]    # filter stop words and single chars
        
        # Fallback to original lexemes if no terms remain
        if not terms:                                                    # all words were stop words — fall back to full token list
            terms = lexemes
            
        if not terms:                                                    # cue was empty or whitespace only — caller skips lexical path
            return ""

        return " OR ".join(f'"{term}"' for term in terms if term)        # wraps each token in quotes, joins with OR — any keyword match surfaces the episode

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
        episode_pool: dict[int, dict] = {}                                        # union of all candidates from both paths — keyed by episode id
        semantic_rank: dict[int, int] = {}                                        # episode_id → 0-based rank in semantic results
        lexical_rank: dict[int, int] = {}                                         # episode_id → 0-based rank in lexical results

        for rank, episode in enumerate(semantic_results):                         # enumerate gives 0-based rank alongside each episode
            semantic_rank[episode["id"]] = rank                                   # stores rank by id — look up during RRF scoring
            episode_pool[episode["id"]] = episode                                 # add episode to union pool

        for rank, episode in enumerate(lexical_results):                          # iterate through each lexical match results
            lexical_rank[episode["id"]] = rank                                    # stores rank by id — looked up during RRF scoring
            episode_pool.setdefault(episode["id"], episode)                       # setdefault — only add episode if id not already, semantic path takes precedence

        semantic_miss = len(semantic_results)                                     # penalty rank for episodes absent from semantic results
        lexical_miss = len(lexical_results)                                       # penalty rank for episodes absent from lexical results
        
        # Compute RRF score for each candidate
        for episode in list(episode_pool.values()):                               # list() snapshots pool — prevents mutation during iteration
            episode = episode.copy()                                              # clone entry — prevents mutating the original pool entry
            episode["rrf_score"] = (                                              # RRF formula — higher score = stronger combined rank
                1.0 / (rrf_k + semantic_rank.get(episode["id"], semantic_miss)) + # .get() returns penalty rank if episode missing from semantic
                1.0 / (rrf_k + lexical_rank.get(episode["id"], lexical_miss))     # .get() returns penalty rank if episode missing from lexical
            )
            episode_pool[episode["id"]] = episode                                 # writes cloned episode with rrf_score back into pool

        # Sort descending by RRF score
        sorted_episodes = sorted(episode_pool.values(),                           # sorts all candidates by rrf_score descending
                                key=lambda episode: episode["rrf_score"],        # uses rrf_score as the sorting key
                                reverse=True)                                    # sorts in descending order

        # Normalise RRF scores to 0.0–1.0 for the 'relevancy' field
        max_rrf = sorted_episodes[0]["rrf_score"] if sorted_episodes else 1.0     # best score for normalization — guard against empty list
    
        for episode in sorted_episodes[:recall_limit]:                            # only processes top episodes up to recall limit
            episode["relevancy"] = episode["rrf_score"] / max_rrf                 # normalize RRF score to 0.0–1.0
            episode.pop("_rank", None)                                            # strip internal rank field before surfacing to caller
            episode.pop("rrf_score", None)                                        # strip internal RRF score before surfacing to caller
        
        return sorted_episodes[:recall_limit]                                     # Return top episodes normalized and cleaned

    def _transcribe_traces(self, traces: list[EngramTrace]) -> str:
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
            f"INSERT INTO {self._vector_schema} (rowid, {self._blueprint.semantic_traces}) VALUES (?,?)",        # Insert the episode encoding into engram vectors
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
            f"INSERT INTO {self._lexical_schema} (rowid, {', '.join(self._blueprint.lexical_traces)}) VALUES (?,?)",         # Insert the episode content into engram lexical
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
                WHERE ev.{self._blueprint.semantic_traces} MATCH ?
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
