"""
STC — Semantic Translation Cortex
==================================
AuRoRA · Semantic Cognitive System (SCS)

Translation layer between raw cognitive content and hippocampal memory space.
Shared across all memory cortices — EMC, SMC, PMC.

STC carries no memory domain knowledge of its own. It does not know what
an episode is, what a skill is, or what a semantic concept means. It only
knows how to transform raw thought into vector representations that the
engram complex can index and retrieve. The cortices supply domain knowledge —
STC supplies the translation tissue they run on.

Biological analogue: entorhinal cortex (EC) — the gateway structure between
high-level cortical association areas and the hippocampal memory system.
Like the EC, STC neither stores nor reasons — it translates, in both directions.

Separated into its own layer so encoding models, vector math, and
serialization logic are defined once and shared — not duplicated across
every cortex that needs them.

Terminology:
    encoding        — semantic vector representation of a memory trace
    normalization   — L2 normalization to unit length so cosine similarity
                      is equivalent to L2 distance in sqlite-vec KNN search
    packing         — serialization of a float vector into fp32 binary blob
                      for engram storage
    prime           — LRU encoding cache mapping prime keys to encoding vectors
    prime key       — MD5 hash of a truncated trace used for prime lookup

Public interface:
    RecallCue -> dataclass
    normalize_vector(vector: list[float]) -> list[float]
    pack_vector(vector: list[float]) -> bytes
    EncodingEngine:
        encode_engram(trace: str) -> bytes
        encode_cue(cue: str) -> RecallCue
"""
# System libraries
from dataclasses import dataclass                       # for defining data structures
import hashlib                                          # for prime key generation
import numpy as np                                      # for vector normalization — normalize_vector
import struct                                           # for vector packing — pack_vector
import threading                                        # for background thread hosting the encoding engine

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
    Normalize an encoding vector to unit length for cosine-equivalent L2 distance search.".

    Args:
        vector (list[float]): Vector to normalize

    Returns:
        list[float]: unit-normalized vector, or the original if already normalized or empty
    """
    vector_array = np.array(vector)                                                # list → ndarray for vectorized math
    vector_mag = np.linalg.norm(vector_array)                                      # L2 norm — Euclidean length of the vector
    if vector_mag > 0 and abs(vector_mag - 1.0) > 1e-6:                            # tolerance check — exact == 1.0 unreliable with floats
        return (vector_array / vector_mag).tolist()                                # divide each element by magnitude → unit vector; zero vector guard
    return vector                                                                  # already unit length — skip normalization
    
def pack_vector(vector: list[float]) -> bytes:
    """
    Pack an encoding vector into a binary blob for engram storage.

    Args:
        vector (list[float]): encoding vector to pack

    Returns:
        bytes: fp32 binary blob — e.g. 768 floats → 3072 bytes
    """
    return struct.pack(f"{len(vector)}f", *vector)                                  # pack float list into fp32 binary blob — e.g. "768f" for 768 floats
        
class EncodingEngine:
    """
    Encoding engine for semantic encoding of memory traces for storage and recall.
    Shared across EMC, SMC, and PMC — loaded once per cortex initialization.
    Primes recent encodings to avoid redundant encoding of identical or similar memory traces
    and to speed up subsequent recall.
    """

    def __init__(self, logger, encoding_engine: str, cue_prefix: str, engram_prefix: str, prime_capacity: int, prime_key_len: int) -> None:
        """
        Initializes the encoding engine and encoding prime for recent memory traces.
        Model load is deferred to a background thread — init returns immediately.
        Lexical-only recall is active from the first turn; semantic recall activates
        once the background load completes.
    
        Args:
            logger                  : Logger instance passed from the cortex
            encoding_engine (str)   : Embedding model to load (e.g. from HRP)
            cue_prefix (str)        : Prompt prefix for encoding cues
            engram_prefix (str)     : Prompt prefix for engrams
            prime_capacity (int)    : Maximum number of entries in the encoding prime
            prime_key_len (int)     : Maximum characters hashed for the prime key
        """
        self.logger                         = logger                # logger from cortex — used throughout this class
        self.encoding_engine: str           = encoding_engine       # model name string — passed to SentenceTransformer()
        self._cue_prefix: str               = cue_prefix            # prompt prefix for encoding cues
        self._engram_prefix: str            = engram_prefix         # prompt prefix for engrams
        self._prime_capacity: int           = prime_capacity        # max prime entries before LRU eviction
        self._prime_key_len: int            = prime_key_len         # max chars hashed for prime key
        self._core                          = None                  # live SentenceTransformer model — None until background load succeeds
        self._prime: dict[str, list[float]] = {}                    # encoding prime — maps prime key → float vector
        
        # Load encoding engine on separate neural thread, not to block main thread
        threading.Thread(                                           # load model on background thread — init returns immediately
            target=self._load_engine,                               # deferred load — never blocks EMC or MCC initialization
            name="encoding-engine-loader",                          # named for thread dump debugging
            daemon=True,                                            # dies with main process — no clean shutdown needed
        ).start()                                                   # fire and forget — _core flips to live model when ready.

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

        prime_key: str = f"engram:{hashlib.md5((prefix + trace)[:self._prime_key_len].encode()).hexdigest()}"  # prefix included — cue and engram encode separately
        if prime_key in self._prime:                                                        # O(1) dict lookup
            return self._prime[prime_key]                                                   # prime hit — skips model inference

        try:                                                                                # attempt to embed the query or engram vector
            encoded_trace: list[float] = self._core.encode(prefix + trace).tolist()         # model inference — prefix prepended; .tolist() converts ndarray → float list
            encoded_trace = normalize_vector(encoded_trace)                                 # normalize to unit length — required so L2 == cosine sim in sqlite-vec
            
            # Keep prime small — evict oldest if over the encoding prime limit
            if len(self._prime) >= self._prime_capacity:                                    # prime full — must evict before inserting
                decayed_prime_key: str = next(iter(self._prime))                            # first key = oldest — dicts preserve insertion order
                del self._prime[decayed_prime_key]                                          # evict oldest entry
            self._prime[prime_key] = encoded_trace                                          # store new vector under prime key
            return encoded_trace                                                            # return encoded and normalized vector
        except Exception as e:                                                              # if embedding fails,
            self.logger.debug(f"Encoding error: {e}")                                       # log encoding errors with reason
            return []                                                                       # same empty list fallback as unavailable guard

    def _load_engine(self) -> None:
        """
        Load the encoding engine on a background thread.
        Called once during initialization — never blocks EMC initialization or active cognition.
        Sets self._core atomically on success — is_available flips True at that moment.
        Lexical-only recall remains active throughout; semantic recall activates once load completes.
        CPython GIL guarantees atomic attribute assignment — no lock needed.
        """
        try:                                                                                # attempt to load SentenceTransformer on a background thread
            import huggingface_hub.utils                                                    # silence lint warnings about unused imports
            from sentence_transformers import SentenceTransformer                           # deferred import — avoids hard crash if package missing
            self.logger.info(f"⏳ Activating Encoding Engine ({self.encoding_engine})…")    # log before the blocking load
            self._core = SentenceTransformer(self.encoding_engine)                          # blocking model load — safe here, isolated from init path
            self.logger.info("✅ Encoding Engine activated")                                # only reached if load succeeded — _core is now live
        except ImportError:                                                                 # package not installed — _core stays None
            self.logger.warning(                                                            # package not installed — _core stays None
                "⚠️ Encoding Engine offline - missing inferencing component.\n"
                "   Memory cortices falling back to lexical recall.\n"
                "   Note to technician: pip3 install sentence-transformers --break-system-packages"
            )
        except Exception as e:    
            self.logger.warning(f"⚠️ Encoding Engine activation failed: {e}")               # log specific failure with reason
