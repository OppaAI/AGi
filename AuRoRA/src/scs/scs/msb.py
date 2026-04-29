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
    memory bank     — the abstraction layer managing the engram complex
    prime           — LRU encoding cache mapping prime keys to encoding vectors
    prime key       — MD5 hash of a truncated trace used for prime lookup
    RRF             — Reciprocal Rank Fusion — fuses semantic and lexical ranked
                      results into a single relevancy-ordered list
    transcript      — SQL column definition string generated from a blueprint
    unit vector     — L2-normalized vector; cosine similarity becomes equivalent
                      to L2 distance, enabling sqlite-vec KNN search

Public interface:
    EncodingEngine:
        encode_engram(trace: str) → list[float]
        encode_cue(cue: str) → RecallCue

    EngramComplex:
        bifurcate_ecx() → sqlite3.Connection
        stage_engram(engram: dict, ecx_conn?) → int
        inscribe_engram(engram: dict, schema?, ecx_conn?) → int
        retrieve_staged_batch(batch_size: int, offset: int) → list[dict]
        decay_staged_engram(staging_id: int, ecx_conn?) → None
        inscribe_vector_index(engram_id: int, blob: bytes, ecx_conn?) → None
        inscribe_lexical_index(engram_id: int, content: str, ecx_conn?) → None
        recall_engram(cue: RecallCue, depth: int, date_range?) → list[dict]
        assess_engram_complex() → dict
        terminate() → None

TODO: migrate pack_vector, normalize_vector to hrs.py if
      vector math is needed outside memory cortices
"""

# System libraries
from dataclasses import dataclass           # dataclass for EngramTrace/EngramSchema
from enum import Enum                       # enum base for EngramModality type definitions
import hashlib                              # for prime key generation
import numpy as np                          # for fast vector math — normalization
from pathlib import Path                    # for calculating database size — owned here, never passed back up
import re                                   # for lexical cue sanitization
import sqlite3                              # for engram connection factory
import struct                               # for packing semantic vectors (fp32)

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
    essential: bool = False          # add NOT NULL constraint if True
    baseline: str | None = None      # set adds DEFAULT — raw SQL if starts with '(', quoted string otherwise
 
@dataclass
class EngramSchema:
    """
    Defines the full structure of an engram complex's trace storage and search indexes.
    """
    storage: list[EngramTrace]                      # column definitions for the main storage table
    staging: list[EngramTrace] | None = None        # column definitions for the crash-safe staging table — optional
    semantic_traces: str | None = None              # column name holding the encoding BLOB — used for KNN vector search
    lexical_traces: list[str] | None = None         # column names fed into FTS5 — used for keyword search
    index_traces: list[str] | None = None           # column names to B-tree index — speeds up WHERE/ORDER BY

@dataclass
class RecallCue:
    """
    Defines a recall cue for engram retrieval.
    Bundles the encoded vector and raw text for dual-path recall.
    """
    vector: list[float]     # encoded semantic vector — passed to semantic recall
    text: str               # raw cue text — passed to lexical recall

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
    return struct.pack(f"{len(vector)}f", *vector)              # pack float list into fp32 binary blob — e.g. "384f" for 384 floats

class EncodingEngine:
    """
    Encoding engine for semantic encoding of memory traces for storage and recall.
    Shared across EMC, SMC, and PMC — loaded once per cortex initialization.
    Primes recent encodings to avoid redundant encoding of identical or similar memory traces
    and to speed up subsequent recall.
    """

    def __init__(self, logger, encoding_engine: str, cue_prefix: str, engram_prefix: str, prime_limit: int = 256, prime_key_limit: int = 300) -> None:
        """
        Initializes the encoding engine and encoding prime for recent memory traces.
    
        Args:
            logger                      : Logger instance passed from the cortex
            encoding_engine (str)       : Embedding model to load (e.g. from HRP)
            encoding_cue_prefix (str)   : Prompt prefix for encoding cues
            encoding_engram_prefix (str): Prompt prefix for engrams
            prime_limit (int)           : Maximum number of entries in the encoding prime
            prime_key_limit (int)       : Maximum characters hashed for the prime key
        """
        self.logger                         = logger                # logger from cortex — used throughout this class
        self.encoding_engine: str           = encoding_engine       # model name string — passed to SentenceTransformer()
        self._cue_prefix: str               = cue_prefix            # prompt prefix for encoding cues
        self._engram_prefix: str            = engram_prefix         # prompt prefix for engrams
        self.prime_limit: int               = prime_limit           # max prime entries before LRU eviction
        self.prime_key_limit: int           = prime_key_limit       # max chars hashed for prime key
        self._core                          = None                  # live SentenceTransformer model — None until load succeeds
        self._prime: dict[str, list[float]] = {}                    # encoding prime — maps prime key → float vector
        
        try:                                                                        # attempt to activate SentenceTransformer
            from sentence_transformers import SentenceTransformer                   # deferred import — avoids hard crash if package missing
            self.logger.info(f"⏳ Activating Encoding Engine ({self.encoding_engine})…") # log before the blocking load
            self._core = SentenceTransformer(self.encoding_engine)                  # load model weights into RAM — blocks until complete
            self.logger.info("✅ Encoding Engine activated")                        # only reached if load succeeded
        except ImportError:                                                         # trigger if sentence_transformers package not installed
            self.logger.warning(                                                    # package not installed — _core stays None
                "⚠️ Encoding Engine offline - missing inferencing component.\n"
                "   Memory cortices falling back to lexical recall.\n"
                "   Note to technician: pip3 install sentence-transformers --break-system-packages"
            )
        except Exception as e:                                                      # bad model path, corrupted weights, OOM — _core stays None
            self.logger.warning(f"⚠️ Encoding Engine activation failed: {e}")       # log specific failure with reason
            
    @property
    def is_available(self) -> bool:
        """
        Returns True if the encoding engine is loaded and ready.
    
        Returns:
            bool: True if ready for encoding, False if failed to load.
        """
        return self._core is not None            # True only if SentenceTransformer loaded successfully

    def _encode(self, trace: str, prefix: str) -> list[float]:
        """
        Core encoding logic — primes recent encodings to speed up subsequent recall.
        Returns empty list if encoding engine is unavailable — signals caller to fall back to lexical recall.

        Args:
            trace (str): Memory trace or recall cue to encode.
            prefix (str): Instruction prefix to prepend before encoding — empty for engrams, BGE prefix for cues.

        Returns:
            list[float]: Semantic encoding vector, or empty list if engine unavailable.
        """
        if not self.is_available:                                                           # _core is None, skip encoding entirely
            self.logger.debug(                                                              # log the SentenceTransformer being unavailable
                "Encoding engine unavailable — semantic recall inactive"
            )
            return []                                                                       # empty list signals fallback to lexical recall

        prime_key: str = f"engram:{hashlib.md5((prefix + trace)[:self.prime_key_limit].encode()).hexdigest()}"  # prefix included — cue and engram encode separately
        if prime_key in self._prime:                                                        # O(1) dict lookup
            return self._prime[prime_key]                                                   # prime hit — skips model inference

        try:                                                                                # attempt to embed the query or engram vector
            encoded_trace: list[float] = self._core.encode(prefix + trace).tolist()         # model inference — prefix prepended; .tolist() converts ndarray → float list
            encoded_trace = normalize_vector(encoded_trace)                                 # normalize to unit length — required so L2 == cosine sim in sqlite-vec
            
            # Keep prime small — evict oldest if over the encoding prime limit
            if len(self._prime) >= self.prime_limit:                                        # prime full — must evict before inserting
                decayed_prime_key: str = next(iter(self._prime))                            # first key = oldest — dicts preserve insertion order
                del self._prime[decayed_prime_key]                                          # evict oldest entry
            self._prime[prime_key] = encoded_trace                                          # store new vector under prime key
            return encoded_trace                                                            # return encoded and normalized vector
        except Exception as e:                                                              # if embedding fails,
            self.logger.debug(f"Encoding error: {e}")                                       # log encoding errors with reason
            return []                                                                       # same empty list fallback as unavailable guard

    def encode_engram(self, trace: str) -> list[float]:
        """
        Encode a memory trace into a semantic vector for storage.

        Args:
            trace (str): Memory trace to encode.

        Returns:
            list[float]: Semantic encoding vector, or empty list if encoding engine unavailable.
        """
        return self._encode(trace, self._engram_prefix)                                     # wrap _encode with engram prefix for encoding engine

    def encode_cue(self, cue: str) -> RecallCue:
        """
        Encodes a recall cue and store with raw cue for dual-path retrieval.

        Args:
            cue (str) : Recall cue to encode

        Returns:
            RecallCue : Encoded vector and raw cue for dual-path retrieval
        """
        return RecallCue(                                                                   # return a RecallCue for dual-path retrieval
            vector=self._encode(cue, self._cue_prefix),                                     # wrap _encode with cue prefix for encoding engine
            text=cue                                                                        # raw cue text preserved for lexical fallback
        )

class EngramComplex:
    """
    Engram Complex (ECX) — memory storage bank abstraction layer.
    Owns all SQL operations for memory cortex — schema creation, staging, consolidation,
    recall, and retrieval. The memory cortex never touches SQL directly.

    TODO: extend engram complex with SMC and PMC tables in M2.
    TODO: backend swap — Qdrant, pgvector, or other vector DB — swap here only,
          cortex cognitive logic untouched.
    """
    
    LEXICAL_FILTER_WORDS = {                                        # set literal — O(1) membership test in clean_cue
            "a", "an", "the", "is", "are", "was", "were", "what", "how", "why", "where", 
            "when", "who", "which", "in", "on", "at", "to", "for", "of", "with", "by", 
            "from", "as", "and", "or", "but", "if", "then", "else", "my", "your", "our", 
            "their", "his", "her", "its", "me", "you", "him", "them", "us", "i", "can", 
            "could", "would", "should", "will", "shall", "do", "does", "did", "have", 
            "hello", "hi", "hey", "bye", "goodbye", "thanks", "please", "yes", "no", "maybe",
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
        self.logger                     = logger                        # logger instance passed from caller
        cortex: str                     = cortex.lower()                # sanitize cortex name for consistent table naming
        self._storage_schema: str       = f"{cortex}_storage"           # table name for main storage
        self._staging_schema: str       = f"{cortex}_staging"           # table name for crash-safe buffer
        self._vector_schema: str        = f"{cortex}_vector"            # virtual table name for KNN search
        self._lexical_schema: str       = f"{cortex}_lexical"           # virtual table name for FTS5 search
        self._gateway: str              = gateway                       # gateway to access the engram complex
        self._blueprint: EngramSchema   = schema                        # blueprint driving dynamic table generation
        self._ecx_conn: sqlite3.Connection  = self._connect_ecx()       # open and configure SQLite connection
        self._vector_index: bool        = self._activate_vector_index() # True if sqlite-vec loaded successfully
        self._vector_dim: int           = dim                           # vector dimension — used in vec0 CREATE VIRTUAL TABLE
        self._build_schema()                                            # create all tables and indexes from blueprint

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
            ecx_conn = sqlite3.connect(self._gateway, check_same_thread=False)  # open SQLite file at given path — check_same_thread=False allows access from multiple threads
            ecx_conn.row_factory = sqlite3.Row                                  # rows returned as dict-like objects — columns accessible by name instead of index
            ecx_conn.execute("PRAGMA journal_mode=WAL;")                        # WAL mode — concurrent reads allowed during writes
            ecx_conn.execute("PRAGMA synchronous=NORMAL;")                      # fsync on checkpoint only — faster writes, safe enough for embedded
            ecx_conn.commit()                                                   # commit pragma changes before returning
            return ecx_conn                                                     # return the configured connection
        except sqlite3.Error as e:                                              # if error during connection to database,
            self.logger.error(f"Engram complex connection failed: {e}")         # log failure before re-raising
            raise                                                               # re-raise to let caller handle the error

    def _activate_vector_index(self, ecx_conn: sqlite3.Connection | None = None) -> bool:
        """
        Activates the vector index for semantic search.
        Graceful fallback to lexical search if index unavailable.
        
        Args:
            ecx_conn (sqlite3.Connection | None): Connection to the engram complex
            
        Returns:
            bool: True if vector index activated, False otherwise.
        """
        ecx_conn = ecx_conn or self._ecx_conn                       # use separate connection or fallback to main connection
        try:                                                        # attempt to activate engram vector index
            import sqlite_vec                                       # deferred import — avoids hard crash if package missing
            sqlite_vec.load(ecx_conn)                               # load vec0 virtual table support into this connection
            return True                                             # signal KNN search is available
        except Exception as e:                                      # if sqlite-vec fails to load,
            if self.logger:                                         # if a logger is provided,
                self.logger.warning(                                # log the fallback to lexical recall due to engram vector index unavailability
                    f"⚠️ engram vector index not available, falling back to lexical recall\n"
                    f"   Note to technician: pip3 install sqlite-vec --break-system-packages\n"
                    f"   Reason: {e}"
                )
            return False                                            # signals caller to fall back to lexical recall

    def bifurcate_ecx(self) -> sqlite3.Connection:
        """
        Open a parallel connection to the engram complex for parallel access.
        Biological analogue: axonal bifurcation — a parallel pathway off the 
        same substrate serving synaptic consolidation independently of the main 
        retrieval pathway.

        Returns:
            sqlite3.Connection: Separate connection for parallel access to the engram complex
        """
        ecx_conn = self._connect_ecx()                              # create separate connection to engram complex
        if self._vector_index:                                      # if vector index is available,
            self._activate_vector_index(ecx_conn)                   # activate vector index for parallel retrieval
        return ecx_conn                                             # return separate connection for parallel access

    def _build_schema(self) -> None:
        """
        Builds all tables and indexes for the engram complex from the blueprint.
        """
        try:

            # 1. Build Storage Table
            storage_transcript: str = self._transcribe_traces(self._blueprint.storage)       # convert main storage trace → SQL column definition string
            self._ecx_conn.execute(f"""
                CREATE TABLE IF NOT EXISTS {self._storage_schema} (
                    {storage_transcript}
                )
            """)                                                                              # create storage table from blueprint
   
            # 2. Build Staging Table (if defined)
            if self._blueprint.staging:                                                       # staging is optional — not all cortices need a buffer
                staging_transcript: str = self._transcribe_traces(self._blueprint.staging)    # convert staging traces → SQL column definition string
                self._ecx_conn.execute(f"""
                    CREATE TABLE IF NOT EXISTS {self._staging_schema} (
                        {staging_transcript}
                    )
                """)                                                                           # create staging table from blueprint

            # 3. Build Indexes
            if self._blueprint.index_traces:                                                   # optional — only if cortex defined index columns
                for trace in self._blueprint.index_traces:                                     # one B-tree index per column
                    self._ecx_conn.execute(f"""
                        CREATE INDEX IF NOT EXISTS idx_{self._storage_schema}_{trace}
                        ON {self._storage_schema}({trace})
                    """)                                                                       # create index for fast retrieval
            self._ecx_conn.commit()                                                                # commit tables and indexes together

            # 4. Build Vector Search Virtual Table
            if self._vector_index and self._blueprint.semantic_traces:                         # only if sqlite-vec loaded and cortex defined a vector column
                self._ecx_conn.execute(f"""
                    CREATE VIRTUAL TABLE IF NOT EXISTS {self._vector_schema} USING vec0(
                        {self._blueprint.semantic_traces} FLOAT[{self._vector_dim}]
                    )
                """)                                                                           # create vec0 virtual table for KNN search
                self._ecx_conn.commit()                                                            # commit the virtual table to the memory bank
                self.logger.debug(f"Engram vector index initialized for {self._vector_schema}")# log the successful initialization of engram vector index

            # 5. Build Lexical Search Virtual Table (FTS5)
            if self._blueprint.lexical_traces:                                                 # only if cortex defined lexical columns
                lexical_transcript = ", ".join(self._blueprint.lexical_traces)                 # join column names into comma-separated string for FTS5 definition
                self._ecx_conn.execute(f"""
                    CREATE VIRTUAL TABLE IF NOT EXISTS {self._lexical_schema} USING fts5(
                       {lexical_transcript},
                       tokenize='porter unicode61'
                    )
                """)                                                                           # create FTS5 virtual table for keyword search
                self._ecx_conn.commit()                                                            # commit the virtual table to the memory bank
                
        except sqlite3.Error as e:                                                             # if error occurs during building schema
            self.logger.error(f"Engram schema initialization failed: {e}")                     # log the failed initialization with reason
            raise                                                                              # re-raise — schema failure is unrecoverable

    def _transcribe_traces(self, traces: list[EngramTrace]) -> str:
        """
        Transcribes a list of EngramTraces into a SQL column definition string.
    
        Args:
            traces (list[EngramTrace]): List of engram traces to transcribe.
    
        Returns:
            str: SQL column transcript — e.g. "id INTEGER PRIMARY KEY, content TEXT NOT NULL"
        """
        transcripts: list[str] = []                                                             # collect completed column definitions
        for trace in traces:                                                                    # one SQL column definition per trace
            transcript = f"{trace.label} {trace.modality.value}"                                # base definition — .value pulls raw string from Enum e.g. "content TEXT"
            if trace.label == "id":                                                             # naming convention — "id" always becomes primary key
                # Ensure SQLite correctly auto-increments INTEGER PRIMARY KEY
                transcript += " PRIMARY KEY"                                                    # uniquely identifies every row
                if trace.modality == EngramModality.INTEGER:                                    # only INTEGER can auto-assign rowid in SQLite
                    transcript += " AUTOINCREMENT"                                              # auto-increments on INSERT, never reuses deleted IDs
            elif trace.essential:                                                               # skipped if label is "id"
                transcript  += " NOT NULL"                                                      # SQLite rejects INSERT if column is empty
                
            if trace.baseline is not None:                                                      # runs for any trace with a default value set
                if trace.modality == EngramModality.TEXT and not trace.baseline.startswith('('):# TEXT type and not a raw SQL expression — needs quotes
                    transcript += f" DEFAULT '{trace.baseline}'"                                # quoted string literal — e.g. DEFAULT 'active'
                else:                                                                           # raw SQL or non-TEXT — e.g. DEFAULT (datetime('now'))
                    transcript += f" DEFAULT {trace.baseline}"                                  # add without quotes
                    
            transcripts.append(transcript)                                                      # add completed column definition to list
            
        self.logger.debug(f"Blueprint transcribed — {len(transcripts)} traces")                 # log column count for debug verification
        return ",\n                        ".join(transcripts)                                  # join all into one SQL string for CREATE TABLE

    def stage_engram(self, engram: dict, ecx_conn: sqlite3.Connection | None = None) -> int:
        """
        Stage an engram into the buffer pending processing.
        Crash-safe — engram persists until explicitly deleted after processing.

        Args:
            engram (dict): Engram to stage — keys map to transcript labels, values to trace content
            ecx_conn (sqlite3.Connection | None) : External connection to use for the operation. Defaults to self._ecx_conn if None
    
        Returns:
            int: staging_id for deletion after processing
        """
        return self.inscribe_engram(engram, self._staging_schema, ecx_conn)         # wrap inscribe_engram with staging schema

    def inscribe_engram(self, engram: dict, schema: str | None = None, ecx_conn: sqlite3.Connection | None = None) -> int:
        """
        Inscribe an engram into the target schema.

        Args:
            engram (dict): Engram to inscribe — keys map to transcript labels, values to trace content
            schema (str | None) : Target schema to inscribe the engram into. Defaults to self._storage_schema if None
            ecx_conn (sqlite3.Connection | None) : External connection to use for the operation. Defaults to self._ecx_conn if None

        Returns:
            int: engram_id of the inscribed engram
        """
        schema                    = schema or self._storage_schema          # default to permanent storage if no schema provided
        ecx_conn                  = ecx_conn or self._ecx_conn              # default to internal connection if no external connection provided
        labels: str               = ", ".join(engram.keys())                # transcript label list from engram
        slots: str                = ", ".join([ "?"] * len(engram))         # one ? per trace
        engram_id: sqlite3.Cursor = ecx_conn.execute(                       # INSERT dynamically built from transcript labels
            f"INSERT INTO {schema} ({labels}) VALUES ({slots})",            # trace content in same order as labels
            list(engram.values()),
        )
        ecx_conn.commit()                                                   # commit before returning — permanent
        return engram_id.lastrowid                                          # return engram_id for vector and lexical indexing

    def retrieve_staged_batch(self, batch_size: int, offset: int) -> list:
        """
        Retrieve a batch of staged engrams from the buffer pending processing.

        Args:
            batch_size (int): Maximum number of engrams to retrieve per batch
            offset (int)    : Batch offset for pagination

        Returns:
            list: Staged engrams in the buffer
        """
        return self._ecx_conn.execute(                                      # query staging table for pending engrams
            f"SELECT * FROM {self._staging_schema} "                        # all traces returned — cortex knows its own schema
            "ORDER BY id LIMIT ? OFFSET ?",                                 # oldest first — paginated by batch_size and offset
            [batch_size, offset]                                            # bind batch_size and offset to placeholders
        ).fetchall()                                                        # return all rows in the batch

    def decay_staged_engram(self, staging_id: int, ecx_conn: sqlite3.Connection | None = None) -> None:
        """
        Decay a staged engram from the buffer after processing.

        Args:
            staging_id (int): staging_id of the engram to decay.
            ecx_conn (sqlite3.Connection | None) : External connection to use for the operation. Defaults to self._ecx_conn if None
        """
        ecx_conn                  = ecx_conn or self._ecx_conn              # default to internal connection if no external connection provided
        ecx_conn.execute(                                                   # remove staged engram from buffer by staging_id
            f"DELETE FROM {self._staging_schema} WHERE id=?",               # match by staging_id — only decays the target engram
            [staging_id],                                                   # placeholder for staging_id
        )
        ecx_conn.commit()                                                   # commit decay before returning

    def inscribe_vector_index(self, engram_id: int, blob: bytes, ecx_conn: sqlite3.Connection | None = None) -> None:
        """
        Inscribe an engram encoding into the vector index for semantic recall.
        Only called if vector index is activated.

        Args:
            engram_id (int) : ID of the inscribed engram
            blob (bytes)    : Binary encoding blob of the engram
            ecx_conn (sqlite3.Connection | None) : External connection to use for the operation. Defaults to self._ecx_conn if None
        """
        if not self._vector_index:                                          # If the vector index is not available, return
            return                                                          # No vector index = no semantic recall

        ecx_conn                  = ecx_conn or self._ecx_conn              # default to internal connection if no external connection provided
        ecx_conn.execute(                                                   # insert encoding into vector index for KNN semantic recall
            f"INSERT INTO {self._vector_schema} (rowid, {self._blueprint.semantic_traces}) VALUES (?,?)",
            [engram_id, blob],                                              # rowid must match engram_id for JOIN during recall
        )
        ecx_conn.commit()                                                   # commit before returning

    def inscribe_lexical_index(self, engram_id: int, content: str, ecx_conn: sqlite3.Connection | None = None) -> None:
        """
        Inscribe an engram into the lexical index for lexical recall.

        Args:
            engram_id (int) : ID of the inscribed engram
            content (str)   : Trace content to index for lexical recall
            ecx_conn (sqlite3.Connection | None) : External connection to use for the operation. Defaults to self._ecx_conn if None
        """
        ecx_conn                  = ecx_conn or self._ecx_conn              # default to internal connection if no external connection provided
        ecx_conn.execute(                                                   # insert content into FTS5 index for lexical recall
            f"INSERT INTO {self._lexical_schema} (rowid, {', '.join(self._blueprint.lexical_traces)}) VALUES (?,?)",
            [engram_id, content],                                           # rowid must match engram_id for JOIN during recall
        )
        ecx_conn.commit()                                                   # commit before returning

    def recall_engram(self, cue: RecallCue, depth: int, date_range: tuple[str, str] | None = None) -> list[dict]:
        """
        Recall engrams by fusing semantic and lexical matches.
        Cortex encodes the cue before calling — MSB owns retrieval only.

        Args:
            cue (RecallCue)                     : Encoded recall cue as float vector and raw cue text.
            depth (int)                         : Number of engrams to recall.
            date_range (tuple[str, str] | None) : Optional, ISO date range (start, end) inclusive — filters recall to that period.

        Returns:
            list[dict]: Engram traces with relevancy field, sorted by descending RRF score.
        """
        semantic_matches = self._semantic_recall(cue.vector, depth, date_range)     # recall by meaning
        lexical_matches  = self._lexical_recall(cue.text, depth, date_range)        # recall by keyword
        return self._converge_memories(semantic_matches, lexical_matches, depth)    # fuse into unified ranking

    def _semantic_recall(self, cue: list[float], depth: int, date_range: tuple[str, str] | None = None) -> list[dict]:
        """
        Recall engrams from the vector index by semantic similarity to the cue.

        Args:
            cue (list[float])                   : Encoded recall cue as float vector.
            depth (int)                         : Number of engrams to recall.
            date_range (tuple[str, str] | None) : Optional, ISO date range (start, end) inclusive — filters recall to that period.
        
        Returns:
            list[dict]: Engram traces with relevancy and _rank fields appended.
        """
        try:                                                                    # attempt semantic recall with given cue
            date_from, date_to = date_range if(date_range and self._is_temporal) else (None, None) # unpack date range — None if no filter; skip date filter if schema has no date column
            temporal_filter: str = """
                AND (store.date >= ? OR ? IS NULL)
                AND (store.date <= ? OR ? IS NULL)
            """ if self._is_temporal else ""                                    # only filter by date if schema has a date column
            temporal_params: list = [date_from, date_from, date_to, date_to] if self._is_temporal else [] # only pass date params if schema has a date column

            matches: list[sqlite3.Row] = self._ecx_conn.execute(                # semantic recall for nearest engrams by L2 distance
                f"""
                SELECT store.*, vec.distance
                FROM {self._vector_schema} vec
                JOIN {self._storage_schema} store ON store.id = vec.rowid
                WHERE vec.{self._blueprint.semantic_traces} MATCH ?
                {temporal_filter}
                AND k = ?
                ORDER BY vec.distance, store.id DESC
                """,
                [pack_vector(cue), *temporal_params, depth * 2],                # only pass date params if schema has a date column
            ).fetchall()                                                        # return all matching engrams by semantic similarity

            if not matches:                                                     # if no matches found,
                return []                                                       # return empty list

            self.logger.debug(f"MSB semantic recall → {len(matches)} candidates | best_dist:{matches[0]['distance']:.4f}")  # log number of matches and best distance

            return [                                                            # return matches with relevancy and _rank fields appended
                {
                    **dict(match),                                              # unpack all trace columns — cortex knows its own schema
                    "relevancy" : max(0.0, 1.0 - (match["distance"]**2 / 2.0)), # L2 → cosine sim on unit-normalized vectors
                    "_rank"     : i,                                            # 0-based rank — 0 = closest match, used by RRF
                }
                for i, match in enumerate(matches)                              # enumerate matches to capture rank
            ]
        except Exception as e:                                                  # if semantic recall fails
            self.logger.debug(f"MSB semantic recall failed: {e}")               # log failure with reason
            return []                                                           # empty list — caller handles no results

    def _lexical_recall(self, cue: str, depth: int, date_range: tuple[str, str] | None = None) -> list[dict]:
        """
        Recall engrams from the lexical index by keyword matching.

        Args:
            cue (str)                           : Raw recall cue string — sanitized internally.
            depth (int)                         : Number of engrams to recall.
            date_range (tuple[str, str] | None) : ISO date range (start, end) inclusive — filters recall to that period.

        Returns:
            list[dict]: Engram traces with relevancy and _rank fields appended.
        """
        clean_cue: str = self._clean_cue(cue)                               # strip punctuation, filter stop words, join with OR
        if not clean_cue:                                                   # if clean_cue is empty or pure whitespace — nothing to recall
            return []                                                       # return empty list

        try:                                                                # attempt lexical recall with sanitized cue
            date_from, date_to = date_range if(date_range and self._is_temporal) else (None, None) # unpack date range — None if no filter; skip date filter if schema has no date column
            temporal_filter: str = """
                AND (store.date >= ? OR ? IS NULL)
                AND (store.date <= ? OR ? IS NULL)
            """ if self._is_temporal else ""                                # only filter by date if schema has a date column
            temporal_params: list = [date_from, date_from, date_to, date_to] if self._is_temporal else [] # only pass date params if schema has a date column

            matches: list[sqlite3.Row] = self._ecx_conn.execute(            # FTS5 lexical recall for matching engrams
                f"""
                SELECT store.*, lexeme.rank AS raw_rank
                FROM {self._lexical_schema} lexeme
                JOIN {self._storage_schema} store on store.id = lexeme.rowid
                WHERE {self._lexical_schema} MATCH ? 
                {temporal_filter}
                ORDER BY lexeme.rank, store.id DESC
                LIMIT ?
                """,
                [clean_cue, *temporal_params, depth * 2],                   # only pass date params if schema has a date column
            ).fetchall()                                                    # fetch all matching engrams

            if not matches:                                                 # if no matches found,
                return []                                                   # return empty list

            raw_scores: list[float] = [abs(match["raw_rank"]) for match in matches] # FTS5 rank is negative — abs() to normalize
            max_score:  float       = max(raw_scores) if raw_scores else 1.0        # best score for normalization — guard against empty

            return [                                                        # return matches with relevancy and _rank fields appended
                {
                    **dict(match),                                          # unpack all trace columns — cortex knows its own schema
                    "relevancy" : abs(match["raw_rank"]) / max_score if max_score else 0.0,  # normalize to 0.0–1.0
                    "_rank"     : i,                                        # 0-based rank — 0 = best match, used by RRF
                }
                for i, match in enumerate(matches)                          # enumerate matches to capture rank
            ]
        except Exception as e:                                              # if lexical recall fails
            self.logger.debug(f"MSB lexical recall failed: {e}")            # log failure with reason
            return []                                                       # empty list — caller handles no results

    def _clean_cue(self, cue: str) -> str:
        """
        Clean and prepare a recall cue for keyword search for lexical recall.

        Args:
            cue (str): Raw recall cue string from the caller.

        Returns:
            str: FTS5-safe quoted token string, or empty string if query is blank.
        """
        clean_cue: str     = re.sub(r'[^\w\s]', ' ', cue.lower())        # replace punctuation with space — preserves word boundaries
        lexemes: list[str] = clean_cue.strip().split()                   # strip whitespace then split into tokens
        
        # Extract keywords (non-stop-words longer than 1 character)
        terms: list[str] = [                                             # filter stop words and single characters
            kw for kw in lexemes
            if kw not in self.LEXICAL_FILTER_WORDS and len(kw) > 1
        ]
        
        # Fallback to original lexemes if no terms remain
        if not terms:                                                    # if all words filtered,
            terms = lexemes                                              # fallback to original tokens
            
        if not terms:                                                    # if cue was empty or pure whitespace,
            return ""                                                    # return empty string

        return " OR ".join(f'"{term}"' for term in terms if term)        # wraps each token in quotes, joins with OR — any keyword match surfaces the engram

    @property
    def _is_temporal(self) -> bool:
        """
        Returns True if this memory cortex is temporal.

        Returns:
            bool: True if storage schema has a date column, False otherwise
        """
        return any(t.label == "date" for t in self._blueprint.storage)      # True if storage schema has a date column

    @staticmethod
    def _converge_memories(semantic_matches : list[dict], lexical_matches  : list[dict], depth: int, rrf_k: int = 60,) -> list[dict]:
        """
        Fuse semantic and lexical recall matches into a unified engram salience ranking.
        Semantic recall (vec KNN) + lexical recall (FTS5)
        → unified engram salience ranking via memory convergence (RRF).

        Shared across EMC, SMC, and PMC — all memory cortices use the same
        dual-path retrieval and convergence pattern.

        RRF formula per engram:
            rrf_score = 1/(k + rank_semantic) + 1/(k + rank_lexical)

        Engrams appearing in only one list get a rank of (list_length + 1)
        in the missing list — a mild penalty, not a full exclusion.
        This mirrors biology: a memory with strong semantic match but zero
        lexical match is still a valid recall candidate.

        Args:
            semantic_matches : Ranked list from semantic recall (vec KNN)
            lexical_matches  : Ranked list from lexical recall (FTS5)
            depth            : Final number of engrams to return
            rrf_k            : RRF constant (default 60, standard in literature)

        Returns:
            list[dict]: list of engrams up to recall depth sorted by descending RRF score,
                        with 'relevancy' set to normalised RRF score (0.0–1.0)
        """
        # Build rank lookup schemas and engram pool — 0-based, best = 0
        engram_pool: dict[int, dict] = {}                                       # union of all candidates from both paths — keyed by engram_id
        semantic_rank: dict[int, int] = {}                                      # engram_id → 0-based rank in semantic results
        lexical_rank: dict[int, int] = {}                                       # engram_id → 0-based rank in lexical results

        for rank, engram in enumerate(semantic_matches):                        # enumerate gives 0-based rank alongside each engram
            semantic_rank[engram["id"]] = rank                                  # stores rank by id — look up during RRF scoring
            engram_pool[engram["id"]] = engram                                  # add engram to union pool

        for rank, engram in enumerate(lexical_matches):                         # iterate through each lexical match results
            lexical_rank[engram["id"]] = rank                                   # stores rank by id — looked up during RRF scoring
            engram_pool.setdefault(engram["id"], engram)                        # setdefault — only add engram if id not already, semantic path takes precedence

        semantic_miss: int = len(semantic_matches)                              # penalty rank for engrams absent from semantic results
        lexical_miss:  int = len(lexical_matches)                               # penalty rank for engrams absent from lexical results
        
        # Compute RRF score for each candidate
        for engram in list(engram_pool.values()):                               # list() snapshots pool — prevents mutation during iteration
            engram["rrf_score"] = (                                             # RRF formula — higher score = stronger combined rank
                1.0 / (rrf_k + semantic_rank.get(engram["id"], semantic_miss)) + # .get() returns penalty rank if engram missing from semantic
                1.0 / (rrf_k + lexical_rank.get(engram["id"], lexical_miss))    # .get() returns penalty rank if engram missing from lexical
            )

        # Sort descending by RRF score
        sorted_engrams: list[dict] = sorted(engram_pool.values(),                   # sorts all candidates by rrf_score descending
                                            key=lambda engram: engram["rrf_score"], # uses rrf_score as the sorting key
                                            reverse=True)                           # sorts in descending order

        # Normalise RRF scores to 0.0–1.0 for the 'relevancy' field
        max_rrf: float = sorted_engrams[0]["rrf_score"] if sorted_engrams else 1.0  # best score for normalization — guard against empty list
    
        for engram in sorted_engrams[:depth]:                                   # only processes top engrams up to depth
            engram["relevancy"] = engram["rrf_score"] / max_rrf                 # normalize RRF score to 0.0–1.0
            engram.pop("_rank", None)                                           # strip internal rank field before surfacing to caller
            engram.pop("rrf_score", None)                                       # strip internal RRF score before surfacing to caller
        
        return sorted_engrams[:depth]                                           # return top engrams normalized and cleaned

    def assess_engram_complex(self) -> dict:
        """
        Assess the current status of the engram complex for logging and monitoring.

        Returns:
            dict: Current engram complex stats including engram count, earliest and latest timestamp, buffer count, and database size.
        """
        try:                                                                    # attempt retrieve stats of engram complex database
            storage_stats: sqlite3.Row = self._ecx_conn.execute(                # query storage schema for engram count and timestamp range
                "SELECT COUNT(*) as total, "
                "MIN(timestamp) as earliest, MAX(timestamp) as latest "
                f"FROM {self._storage_schema}"
            ).fetchone()                                                        # fetch engram count and timestamp range

            if self._blueprint.staging:                                         # check if blueprint has staging table
                staging_stats: sqlite3.Row | None = self._ecx_conn.execute(     # query staging schema for buffer count
                    f"SELECT COUNT(*) as total FROM {self._staging_schema}"
                ).fetchone()                                                    # fetch buffer count
            else:                                                               # no staging table defined in blueprint
                staging_stats: sqlite3.Row | None = None                        # query returns None

            ecx_volume: float = round(Path(self._gateway).stat().st_size / 1_048_576, 2) \
                if Path(self._gateway).exists() else 0.0                        # calculate engram complex physical volume size (in MB)

            return {                                                            # return engram complex stats for logging and monitoring
                "engram_count"        : storage_stats["total"] if storage_stats else 0,         # total encoded episodes in engram storage
                "buffer_count"        : staging_stats["total"] if staging_stats else 0,         # unencoded episodes in staging buffer
                "vector_index_active" : self._vector_index,                                     # True if KNN vector index is available
                "earliest_timestamp"  : storage_stats["earliest"] if storage_stats else None,   # timestamp of the earliest engram complex
                "latest_timestamp"    : storage_stats["latest"] if storage_stats else None,     # timestamp of the latest engram complex
                "physical_volume"     : ecx_volume,                                             # engram complex size on disk (MB)
            }
        except Exception as e:                                                  # catch any database access errors
            self.logger.error(f"MSB assess engram complex failed: {e}")         # log failure with reason
            return {}                                                           # empty dict — caller handles no results

    def terminate(self) -> None:
        """
        Close the engram complex connection and release all resources.
        """
        self._ecx_conn.close()                                              # release SQLite connection — no further ops possible after this
