"""
EMC — Episodic Memory Cortex
==============================
AuRoRA · Semantic Cognitive System (SCS)

Episodic memory layer of the SCS — "I remember that specific moment."
Stores conversational PMTs as dated, semantically encoded engrams for future recall.
No expiry — 1TB NVMe means Grace remembers everything.

Responsibilities:
    - Receive evicted PMTs from MCC into crash-safe episodic buffer (binding)
    - Encode buffered turns into semantic vectors (encoding)
    - Consolidate encoded episodes into the episodic engram (synaptic consolidation)
    - Recall relevant past episodes via semantic and lexical search (recall)
    - Surface recalled episodes into MCC memory context (reinstatement)

Architecture:
    Episodic Buffer (Baddeley's model, 2-layer):
        _binding_stream  — private deque; evicted PMTs land here first (fast RAM staging)
                           invisible to MCC — MCC calls bind_pmt() and lets go,
                           mirroring how PFC does not monitor every hippocampal trace after handoff
        recall_stream    — shared stream; recalled episodes surface here before
                           being injected into MCC memory context

    SQLite engram (one table + two indexes):
        emc_storage      — permanent episodic memory store
        emc_vector       — sqlite-vec KNN index for semantic search
        emc_lexical      — FTS5 index for lexical search
        emc_staging      — crash-safe buffer for unencoded PMTs

    Encoding:
        Continuous during wake — online encoding, not sleep-only.
        sentence-transformers, CPU-only, zero GPU impact.
        Biological analogue: hippocampal initial encoding during wake.
        Deep consolidation (SMC distillation) deferred to M2 Dream Cycle.

    Encoding Cycle:
        Background thread drains _binding_stream → encodes → inscribes into emc_storage.
        Falls back to emc_staging on restart for crash recovery.
        Runs continuously, sleeps when buffer is empty via theta rhythm event.
        Never blocks active cognition.
        
    Retrieval:
        Dual-path retrieval fused via Reciprocal Rank Fusion (RRF):
            PATH 1 — Semantic (cf. CA3 pattern completion):
                sqlite-vec L2 KNN on unit-normalized vectors (cosine-equivalent)
            PATH 2 — Lexical (cf. dentate gyrus pattern separation):
                FTS5 porter-stemmed search on episode content
        Fallback chain:
            Both paths active  → RRF fusion (full memory convergence)
            Lexical only       → lexical results  (encoding engine unavailable)
            Neither            → []               (catastrophic — SQLite unavailable)

    Storage:
        SQLite WAL mode — Jetson-friendly, concurrent read/write

Terminology:
    Episodic Buffer    — two-stream in-memory workspace bridging WMC eviction and EMC
                         encoding. Binding stream receives evicted PMTs; recall stream
                         stages reinstated episodes for MCC context assembly.
    Encoding           — transformation of a raw PMT into a unit-normalized semantic
                         vector for storage and similarity search in the engram complex.
    Engram             — one physical stored memory record in the engram complex,
                         carrying a timestamp, date, raw content, and semantic
                         encoding vector.
    Episode            — a dated, encoded memory trace representing a specific
                         remembered moment — one complete PMT bound to the scaffold
                         and consolidated into the engram complex.
    Lexical Recall     — keyword search over episode content via SQLite FTS5 with
                         porter stemming, surfacing episodes by pattern match rather
                         than meaning. Biological analogue: dentate gyrus pattern
                         separation — discriminates episodes by surface features.
    Memory Convergence — fusion of semantic and lexical recall rankings into a single
                         relevancy-ordered list via Reciprocal Rank Fusion (RRF).
                         Neither path alone is canonical — convergence is the result.
    Recall Stream      — shared output stream of the episodic buffer where reinstated
                         episodes are staged before injection into MCC memory context.
    Relevancy          — RRF-fused salience score (0.0–1.0) assigned to each recalled
                         episode, combining semantic similarity rank and lexical match
                         rank into a single measure of contextual fitness.
    Scaffold           — the pre-structured temporal-semantic coordinate space that
                         anchors every engram at bind time and sequences recalled
                         episodes before reinstatement. The temporal axis (timestamp),
                         semantic axis (vec0), and lexical axis (FTS5) together
                         constitute the scaffold. Reinstatement traverses it;
                         the Dream Cycle reorganizes attachments within it.
    Semantic Recall    — vector similarity search over unit-normalized engram encodings
                         via sqlite-vec KNN, surfacing episodes by meaning rather than
                         keyword match. Biological analogue: CA3 pattern completion —
                         a partial cue reinstates the full stored trace.

Lifecycle:
    Binding → Encoding → Synaptic Consolidation → Recall → Reinstatement
    (System Consolidation — EMC → SMC distillation — deferred to M2 Dream Cycle)

Public interface:
    emc.bind_pmt(timestamp, content) → bool
    emc.recall_episodes(cue) → → list[Episode]
    emc.assess_emc() → dict
    emc.terminate() → None
"""

# System components
import itertools                            # for islice — caps binding stream snapshot per theta rhythm cycle
import os                                   # for encoding thread priority via os.nice()
from pathlib import Path                    # for building engram complex storage path
import threading                            # for background thread, locks, and theta rhythm event
import time                                 # for CPU yield during theta rhythm cycle
from collections import deque               # for O(1) append/popleft in binding stream
from dataclasses import dataclass, field    # for EpisodicBuffer and episode dataclasses

# AGi components
from hrs.hrm import AGi                     # homeostatic regulation manifest namespace — system-wide constants
EMC = AGi.SCS.EMC                           # EMC parameter namespace alias — keeps WMC constant references concise
from hrs.hru import(                        # homeostatic regulation helper functions
    pack_vector,                            # for packing float list into fp32 binary blob   
    ChunkSampler,                           # probes and truncates cognitive context for budget management
)
from scs.msb import (                       # shared memory storage bank substrate
    EngramSchema,                           # blueprint for engram table structure
    EngramTrace,                            # single column definition within a blueprint
    EngramModality,                         # TEXT / INTEGER / REAL / BLOB type enum
    RecallCue,                              # encoded vector + raw text for dual-path recall
    EncodingEngine,                         # sentence-transformers wrapper with LRU prime
    EngramComplex,                          # all SQL ops for a memory cortex
)

EMC_SCHEMA = EngramSchema(                  # define the engram schema for episodic memory
    storage=[
        EngramTrace(label="id", modality=EngramModality.INTEGER),                                   # auto-assigned primary key — rowid alias
        EngramTrace(label="user_id",    modality=EngramModality.TEXT, essential=True),              # ID of the user who created the engram
        EngramTrace(label="timestamp",  modality=EngramModality.TEXT, essential=True),              # ISO-8601 datetime of the original PMT
        EngramTrace(label="date",       modality=EngramModality.TEXT, essential=True),              # YYYY-MM-DD slice of timestamp — reserved for Dream Cycle
        EngramTrace(label="trace",      modality=EngramModality.TEXT, essential=True),              # formatted interaction text — used for both embedding and reinstatement display
        EngramTrace(label="encoding",   modality=EngramModality.BLOB, essential=True),              # fp32 binary vector — linked to vec0 KNN index by rowid
        EngramTrace(label="created_at", modality=EngramModality.TEXT, baseline="(datetime('now'))"), # wall-clock inscription time — set by SQLite on INSERT
        # (TODO) M2a — importance scoring
        EngramTrace(label="last_recalled_at", modality=EngramModality.TEXT),                        # when the memory was last recalled
        EngramTrace(label="recall_count",     modality=EngramModality.INTEGER, baseline="0"),       # how many times the memory has been recalled
        # (TODO) M2b — versioning
        EngramTrace(label="conflict",         modality=EngramModality.INTEGER, baseline="0"),       # if the memory is in conflict with another memory
        EngramTrace(label="superseded_by",    modality=EngramModality.INTEGER),                     # the ID of the memory that superseded this memory
        EngramTrace(label="valid_from",       modality=EngramModality.TEXT),                        # when the memory is valid from
        EngramTrace(label="valid_until",      modality=EngramModality.TEXT),                        # when the memory is valid until
    ],
    staging=[
        EngramTrace(label="id", modality=EngramModality.INTEGER),                                   # auto-assigned staging_id — used for decay after consolidation
        EngramTrace(label="user_id",    modality=EngramModality.TEXT, essential=True),              # ID of the user who created the engram
        EngramTrace(label="timestamp",  modality=EngramModality.TEXT, essential=True),              # preserved from original PMT
        EngramTrace(label="date",       modality=EngramModality.TEXT, essential=True),              # preserved from original PMT
        EngramTrace(label="trace",      modality=EngramModality.TEXT, essential=True),              # plain-text — consistent with storage
    ],
    semantic_traces="encoding",                                                                     # column linked to vec0 virtual table for KNN search
    lexical_traces=["trace"],                                                                       # FTS5 now indexes clean natural language
    index_traces=["user_id", "timestamp"],                                                          # B-tree index — speeds up temporal-filtered recall
    temporal_trace="timestamp",                                                                     # temporal filter column — represents when the memory occurred
)

@dataclass
class Episode:
    """
    Episode — one consolidated episodic memory record in the engram complex.
    Represents a complete interaction (user prompt + AI response) that has been
    encoded into a semantic vector and inscribed into permanent storage.   
    """
    storage_id     : int | None = None      # SQLite rowid — auto-assigned on inscription, None before storage
    staging_id     : int | None = None      # transient — staging rowid for decay after consolidation, None if not yet staged
    user_id        : str | None = None      # Speaker identity — preserved from original PMT
    timestamp      : str = ""               # ISO-8601 induction time — temporal anchor for chronological recall
    date           : str = ""               # YYYY-MM-DD slice of timestamp — indexed for date-filtered recall
    trace          : str = ""               # Formatted interaction text — used for both embedding and reinstatement display
    encoding       : bytes = b""            # fp32 binary vector blob — inscribed at consolidation, pre-fetched at recall for M2 active vector memory
    created_at     : str = ""               # Wall-clock inscription time — set by SQLite on INSERT
    relevancy   : float = 0.0               # transient — RRF-fused relevancy score, populated at recall time, never stored
    
@dataclass
class EpisodicBuffer:
    """
    Episodic Buffer — shared workspace between WMC and EMC.
    Modelled on Baddeley's episodic buffer — one buffer, two streams.

    _binding_stream — evicted PMTs pending encoding into episodic memory
    recall_stream   — recalled episodes pending reinstatement into active cognition
    """
    _binding_stream: deque[Episode] = field(                                   # evicted PMTs queued for encoding — drained by encoding cycle
        default_factory=lambda: deque(maxlen=EMC.BINDING_STREAM_LIMIT)         # maxlen cap — oldest silently dropped if encoding engine offline too long
    )
    recall_stream: list[dict] = field(default_factory=list)                    # recalled episodes queued for MCC context assembly@dataclass

    def __post_init__(self) -> None:
        """Initialize locks not expressible as dataclass fields."""
        self._recall_lock = threading.Lock()                                   # serializes recall stream mutations across threads
       
    def clear_recall_stream(self) -> None:
        """Clear the recall stream before assembling a new memory context."""
        with self._recall_lock:                                                # hold lock for duration of mutation
            self.recall_stream.clear()                                         # discard all previously recalled episodes

    def stage_single_episode(self, item: dict) -> None:
        """Append a single recalled episode to the recall stream."""
        with self._recall_lock:                                                # hold lock for duration of mutation
            self.recall_stream.append(item)                                    # append single episode to recall stream

    def stage_episode_list(self, items: list[dict]) -> None:
        """Extend the recall stream with a list of recalled episodes."""
        with self._recall_lock:                                                # hold lock for duration of mutation
            self.recall_stream.extend(items)                                   # extend recall stream with episode batch

    def assess_recall_stream(self) -> list[dict]:
        """Return a snapshot of the recall stream for safe iteration.
        
        Returns:
            list[dict]: Stable copy of the current recall stream.
        """
        with self._recall_lock:                                                # hold lock while copying — prevents mutation mid-read
            return list(self.recall_stream)                                    # shallow copy — safe for iteration outside the lock

def _translate_engram(episode: Episode, encoding: bytes | None = None) -> dict:
    """
    Translate an Episode into an engram dict for inscription into the engram complex.
    Owns the Episode → MSB boundary mapping — single point of change if Episode schema evolves.
    Encoding is optional — omitted for staging, included for synaptic consolidation.
    """
    engram = {                           # fill the episodes field into engram
        "user_id"  : episode.user_id,    # speaker identity — preserved from original PMT
        "timestamp": episode.timestamp,  # ISO-8601 temporal anchor
        "date"     : episode.date,       # YYYY-MM-DD slice — B-tree indexed for date recall
        "trace"    : episode.trace,      # formatted interaction text — embedding source and reinstatement display
    }
    if encoding is not None:             # if encoding present,
        engram["encoding"] = encoding    # fp32 binary blob — only present after encoding cycle completes
    return engram                        # return the translated engram

class EpisodicScaffold:
    """
    Episodic Scaffold — temporal-semantic coordinate space governing reinstatement.
    Anchors recalled episodes within the chunk budget and restores chronological order
    before injection into the cognitive context window.

    M1.5: Operates purely on list[Episode] — no SQL, no encoding.
    M2:   Extend with sequence_index, session_id, salience, consolidation_state
          for Dream Cycle tracking and importance-weighted reinstatement.

    Owned by EpisodicMemoryCortex. Called once per turn in reinstate_episodes().
    """

    def __init__(self, chunk_sampler: ChunkSampler) -> None:
        """
        Args:
            chunk_sampler (ChunkSampler) : Probes and truncates episode content for budget management
        """
        self._chunk_sampler  = chunk_sampler           # for probing and truncating episode content
        self._chunk_limit    = EMC.RECALL_RESERVE      # chunk budget ceiling for reinstatement

    def fragment(self, episodes: list[Episode]) -> list[Episode]:
        """
        Fragment recalled episodes to the RECALL_RESERVE chunk limit before reinstatement.
        Consumes episodes in RRF-ranked order — highest relevance preserved first.
        Last episode exceeding limit is truncated to a fragment rather than dropped —
        mirrors human partial recall under working memory capacity pressure.

        Biological analogue: CA1 output gating — limits the volume of reinstated
        traces before prefrontal delivery to avoid overwriting active cognition.

        Args:
            episodes (list[Episode]): Relevancy-filtered episodes, RRF-ranked descending.

        Returns:
            list[Episode]: Budget-clipped episode list. Final entry may carry a
                        truncated trace field if it exceeded the remaining reserve.
        """
        chunk_limit: int = self._chunk_limit                                                # remaining chunk budget for reinstatement
        fragment: list[Episode] = []                                                        # budget-trimmed episode list

        for episode in episodes:                                                            # consume in RRF rank order — highest relevance first
            trace       = episode.trace                                                     # trace is the pre-formatted text field
            chunk_count = self._chunk_sampler.probe(trace  )                                # estimate chunk count of the episode trace
    
            if chunk_count > chunk_limit:                                                   # episode exceeds remaining budget — truncate to fragment
                if chunk_limit > 0:                                                         # remaining budget can still surface a fragment
                    episode       = Episode(**vars(episode))                                # shallow copy — never mutate source list
                    episode.trace = self._chunk_sampler.truncate(trace, chunk_limit)        # truncate trace to remaining budget
                    fragment.append(episode)                                                # reinstate memory fragment into memory context
                    break                                                                   # budget exhausted after fragment — stop regardless
            
            chunk_limit -= chunk_count                                                      # deduct the token cost from remaining budget
            fragment.append(episode)                                                        # episode fits — reinstate in full
        return fragment                                                                     # return budget-trimmed episode list

    def sequence(self, episodes: list[Episode]) -> list[Episode]:
        """
        Re-organize fragmented episodes into ascending chronological order before reinstatement.
        RRF returns episodes ranked by relevancy — sequencing restores the temporal
        narrative so the cognitive engine reads reinstated engrams in the order
        they were originally consolidated into the engram complex.

        Biological analogue: hippocampal temporal context signal — reinstated traces
        are ordered by their original encoding time before surfacing to prefrontal cortex.

        Args:
            episodes (list[Episode]): episode fragmented to fit the chunk reserve in any order.

        Returns:
            list[Episode]: Same episodes sorted ascending by timestamp.
        """
        return sorted(                                                          # stable sort — preserves RRF rank within timestamp ties
            episodes,                                                           # episode fragmented to fit the chunk reserve in any order
            key=lambda episode: episode.timestamp                               # ISO-8601 lexicographic order == chronological order
        )
        
class EncodingCycle:
    """
    Encoding Cycle — theta-rhythm driven online encoding of episodic memories.
    Drains the binding stream, encodes PMTs into semantic vectors,
    and consolidates them into the episodic engram.

    Biological analogue: intra-hippocampal CA3/CA1 LTP-driven trace encoding during wake.
    """

    def __init__(self, logger, ecx: EngramComplex, encoding_engine: EncodingEngine,
                 episodic_buffer: EpisodicBuffer, episodic_buffer_lock: threading.Lock) -> None:
        """
        Initialize the episodic encoding cycle and start the background encoding thread.
        Recovers any unencoded PMTs from the staging table before the thread starts.

        Args:
            logger                                : Logger instance from EMC
            ecx (EngramComplex)                   : Engram complex for staging and inscription
            encoding_engine (EncodingEngine)      : Shared encoding engine for semantic vectors
            episodic_buffer (EpisodicBuffer)      : Episodic buffer — binding stream source
            episodic_buffer_lock (threading.Lock) : Lock for thread-safe binding stream access
        """
        self.logger                = logger                               # logger instance from EMC
        self._ecx                  = ecx                                  # engram complex for staging and inscription
        self._encoding_engine      = encoding_engine                      # shared encoding engine for semantic vectors
        self._episodic_buffer      = episodic_buffer                      # episodic buffer — binding stream source
        self._episodic_buffer_lock = episodic_buffer_lock                 # lock for thread-safe binding stream access
                     
        # Inscription lock — only encoding cycle inscribes into episodes
        self._inscription_lock     = threading.RLock()                    # serializes engram writes — RLock allows re-entry
                     
        # Set up encoding cycle
        self._encoder_running: bool         = False                       # encoding cycle not yet started
        self._theta_rhythm                  = threading.Event()           # gates encoding cycle — set by trigger_theta_rhythm() or WMC
        self._encoder_thread                = None                        # assigned in _ignite_cycle()
        self._ignite_cycle()                                              # kick start the encoding cycle

    def _ignite_cycle(self) -> None:
        """
        Recover unencoded PMTs from the staging table into the binding stream,
        then start the background encoding thread.
        Batch recovery avoids RAM spike after crash during a long session.
        """
        # Batch recovery of unencoded episodes — drain in chunks of batch size
        # Avoids loading thousands of unencoded episodes into binding stream at once
        recovery_count = 0                                              # total recovered episode count
        recovery_offset = 0                                             # pagination offset for batch recovery

        while True:                                                     # keep recovering unencoded episodes in batches
            unencoded = self._ecx.retrieve_staged_batch(                # query staging table for unencoded episodes
                batch_size = EMC.RECOVERY_BATCH_SIZE,                   # number of unencoded episodes to recover at once
                offset     = recovery_offset,                           # offset for pagination — skips already-recovered episodes
            )

            if not unencoded:                                           # if no more unencoded episodes to recover,
                break                                                   # stop recovering unencoded episodes

            with self._episodic_buffer_lock:                            # hold lock for binding stream append
                for row in unencoded:                                   # iterate through each unencoded episode
                    self._episodic_buffer._binding_stream.append(Episode(  # add the unencoded episode to the binding stream
                        staging_id = row["id"],                         # staging_id for decay after consolidation
                        user_id    = row["user_id"],                    # user id of episode
                        timestamp  = row["timestamp"],                  # preserved timestamp from original PMT
                        date       = row["date"],                       # preserved date from original PMT
                        trace      = row["trace"],                      # formatted interaction text pending encoding
                    ))

            recovery_count  += len(unencoded)                           # accumulate total recovered count
            recovery_offset += EMC.RECOVERY_BATCH_SIZE                  # advance pagination offset

        if recovery_count:                                              # if any episodes were recovered,
            self.logger.info(                                           # Log the recovery of unencoded episodes
                f"⚡ EMC recovered {recovery_count} unencoded episode(s) from engram → binding stream"
            )                                                       
            self._theta_rhythm.set()                                    # wake encoding cycle for recovered episodes

        self._encoder_running = True                                    # mark cycle as active before thread starts
        self._encoder_thread  = threading.Thread(                       # assign a dormant thread for the encoding cycle
            target=self._run_cycle,                                     # encoding cycle main loop
            name="emc-encoding-cycle",                                  # named for debugging
            daemon=True,                                                # dies with main process
        )
        self._encoder_thread.start()                                    # start the neural thread of encoding cycle
        self.logger.info("🔄 EMC encoding cycle started")               # log the start of encoding cycle

    def _run_cycle(self) -> None:
        """
        Theta-rhythm driven encoding — drains binding stream into emc_storage.
        Wakes on PMT arrival or theta interval — whichever comes first.
        Processes a fixed snapshot per rhythm — new arrivals deferred to next cycle.
        """
        # Yield processing priority to active cognition threads
        os.nice(19)                                               # deprioritize — encoding is non-latency-sensitive

        encoder_conn = self._ecx.bifurcate_ecx()                  # bifurcated connection for parallel writes
        self.logger.info("⚙️ EMC encoding cycle running…")        # log the start of encoding cycle
    
        while self._encoder_running:                              # loop until stop signal
            # Rest state — wait for theta rhythm activation
            self._theta_rhythm.wait(timeout=EMC.THETA_INTERVAL)   # wake on PMT arrival or theta interval
            self._theta_rhythm.clear()                            # reset event for next cycle
    
            if not self._encoder_running:                         # if the encoder is not running,
                break                                             # clean exit if stopped while waiting
    
            # Snapshot IDs at this moment — one rhythm, one defined window
            with self._episodic_buffer_lock:                      # hold lock for snapshot and drain
                if not self._episodic_buffer._binding_stream:     # binding stream empty — nothing to encode
                    continue                                      # skip this encoding cycle
                rhythm: list[Episode] = list(itertools.islice(    # snapshot up to batch limit — remaining stays for next cycle
                    self._episodic_buffer._binding_stream, EMC.THETA_BATCH_LIMIT
                ))

                # Drain the snapshotted episodes from the binding stream
                for _ in range(len(rhythm)):                      # iterate through the length of snapshot
                    self._episodic_buffer._binding_stream.popleft() # drain only what was snapshotted
    
            self.logger.debug(f"EMC encoding cycle → {len(rhythm)} episode(s) in rhythm") # log the number of episodes in the rhythm
    
            # Replay each episode in the rhythm
            for episode in rhythm:                                 # for each episode in the rhythm,
                if not self._encoder_running:                      # if the encoder is not running,
                    break                                          # respect stop signal mid-rhythm
                time.sleep(0.01)                                   # 10ms yield — prevents starving active cognition

                # Inscribe to episodic_buffer (crash-safe record) before encoding
                # Skip if already recovered from episodic_buffer on restart
                if episode.staging_id is None:                     # if no staging_id, stage the episode
                    with self._inscription_lock:                            # hold inscription lock for staging write
                        episode.staging_id = self._ecx.stage_engram(        # insert the episode into the episodic buffer
                            engram   = translate_engram(episode),           # translate Episode → engram dict for staging — encoding excluded until consolidation
                            ecx_conn = encoder_conn,                        # connection to episodic buffer
                        )
      
                # Encode the episode trace into a semantic vector
                encoded_episode: list[float] = self._encoding_engine.encode_engram(episode.trace)  # encode trace into semantic vector
                if not encoded_episode:                                             # if the encoding failed,
                    # Encoding engine unavailable — skip for now, retry later
                    self.logger.warning(                                            # log the warning message of unavailability of the encoding engine
                        f"EMC encode skipped — encoding engine unavailable (date={episode.date})"
                    )
                    with self._episodic_buffer_lock:                                # hold lock for appendleft
                        self._episodic_buffer._binding_stream.appendleft(episode)   # requeue for next theta cycle
                    continue                                                        # continue to the next episode
    
                # Pack encoded episode vector as fp32 binary for engram storage
                encoding_blob = pack_vector(encoded_episode)                        # pack float vector into fp32 binary blob
            
                self._synaptic_consolidate(encoder_conn, episode, encoding_blob)    # consolidate the episode from episodic buffer into engram
    
                self.logger.debug(                                                  # log successful synaptic consolidation of the episode
                    f"EMC encoded → consolidated to engram (date={episode.date})"
                )
    
        encoder_conn.close()                                                        # close the encoder connection
        self.logger.info("EMC encoding cycle stopped")                              # log the stop of the EMC encoding cycle

    def trigger_theta_rhythm(self) -> None:    
        """
        Trigger the theta rhythm to wake the encoding cycle.
        """
        self._theta_rhythm.set()                            # wake the theta rhythm to start encoding

    def _synaptic_consolidate(self, encoder_conn, episode: Episode, encoding_blob: bytes) -> None:
        """
        Stabilize one encoded episode into all three engram indexes.
        Biological analogue: LTP-driven trace stabilization in the hippocampus during wake.
        Distinct from systems consolidation (EMC → SMC) which occurs during the M2 Dream Cycle.

        Args:
            encoder_conn          : Bifurcated connection for encoding cycle writes
            episode (Episode)     : Episode with timestamp, date, trace, staging_id
            encoding_blob (bytes) : fp32 binary vector of the encoded episode
        """
        with self._inscription_lock:                                            # serializes all three inscriptions as one atomic operation
            # Primary episodic record
            episode_id = self._ecx.inscribe_engram(                             # inscribe primary episodic record into emc_storage
                engram=translate_engram(episode, encoding=encoding_blob),       # translate Episode → engram dict for inscription — encoding included
                ecx_conn=encoder_conn,                                          # connection to use for the operation
            )

            # Engram vector index (vec0 KNN for semantic search)
            self._ecx.inscribe_vector_index(                                    # inscribe encoding into vec0 KNN index — rowid matches episode_id
                engram_id = episode_id,                                         # episode ID for which to insert the encoding
                blob = encoding_blob,                                           # binary encoding data of the episode
                ecx_conn = encoder_conn,                                        # connection to use for the operation
            )
     
            # Engram lexical index (FTS5 index for lexical search)
            self._ecx.inscribe_lexical_index(                                   # inscribe trace into FTS5 index — rowid matches episode_id
                engram_id = episode_id,                                         # episode ID for which to insert the encoding
                trace = episode.trace,                                          # formattted interactive text of the episode
                ecx_conn = encoder_conn,                                        # connection to use for the operation
            )
     
            # Remove the entry in episodic buffer — synaptic consolidation is complete, staging row no longer needed
            if episode.staging_id is not None:                                  # staging_id present — episode was crash-recovered
                self._ecx.decay_staged_engram(                                  # decay staging row — synaptic consolidation complete
                    staging_id = episode.staging_id,                            # episode ID for which to decay the encoding
                    ecx_conn  = encoder_conn,                                   # connection to use for the operation
                )
    
    def stop_cycle(self) -> None:
        """Signal the encoding cycle to stop and wait for clean exit."""
        self._encoder_running = False                                       # signal encoding thread to stop
        self._theta_rhythm.set()                                            # wake thread so it can exit cleanly
        if self._encoder_thread:                                            # if encoder cycle is still running,
            self._encoder_thread.join(timeout=EMC.ENCODING_CYCLE_TIMEOUT)   # wait for clean exit — up to 3 seconds
            
class EpisodicMemoryCortex:
    """
    Episodic Memory Cortex (EMC) — hippocampal long-term episodic store.
    Receives evicted PMTs from WMC via MCC, encodes them as semantic vectors,
    and consolidates them into the episodic engram for future recall.

    SQLite engram (one table + two indexes):
        emc_storage  — permanent episodic memory store
        emc_vector   — sqlite-vec KNN index for semantic search
        emc_lexical  — FTS5 index for lexical search
        emc_staging  — crash-safe buffer for unencoded PMTs

    Thread-safety:
        _episodic_buffer_lock  — serializes binding stream access
        _recall_lock           — serializes recall stream access (on EpisodicBuffer)
        SQLite WAL mode allows concurrent reads during async writes
    """

    def __init__(self, logger, engram_gateway: Path, chunk_sampler: ChunkSampler, encoding_engine: EncodingEngine) -> None:
        """
        Initialize the Episodic Memory Cortex, engram complex, encoding engine,
        and start the background encoding cycle.

        Recovers any unencoded PMTs from the crash-safe staging table back into
        the binding stream before the encoding cycle starts.
        
        Args:
            logger                           : Logger instance for logging operations
            engram_gateway (Path)            : Path to access the engram for storing episodic memories
            chunk_sampler (ChunkSampler)     : Shared chunk sampler for reinstatement budgeting
            encoding_engine (EncodingEngine) : Shared encoding engine — owned by MCC, passed in at init
        """
        self.logger                = logger                                 # logger from MCC — used throughout EMC
        self._chunk_sampler        = chunk_sampler                          # for reinstatement budgeting
        self.episodic_buffer       = EpisodicBuffer()                       # two-stream buffer — binding and recall streams
        self._episodic_buffer_lock = threading.Lock()                       # serializes binding stream access
        self._episodic_scaffold    = EpisodicScaffold(                      # temporal-semantic reinstatement coordinator
            chunk_sampler=chunk_sampler
        )
        self._encoding_engine = encoding_engine                             # shared encoding engine — owned by MCC
        self._ecx = EngramComplex(                                          # owns all SQL ops for EMC
            logger  = self.logger,                                          # logger instance for logging operations
            cortex  = "EMC",                                                # drives table name prefix — emc_storage, emc_vector etc.
            gateway = str(engram_gateway),                                  # cast to str — EngramComplex expects string path
            schema  = EMC_SCHEMA,                                           # blueprint defining EMC table structure
            dim     = EMC.ENCODING_DIM,                                     # vector dimension for vec0 KNN index
        )

        try:                                                                # attempt to start encoding cycle
            self._encoding_cycle = EncodingCycle(                           # theta-driven background encoding thread
                logger               = logger,
                ecx                  = self._ecx,                           # engram complex for staging and inscription
                encoding_engine      = self._encoding_engine,               # shared encoding engine
                episodic_buffer      = self.episodic_buffer,                # binding stream source
                episodic_buffer_lock = self._episodic_buffer_lock,          # lock for binding stream access
            )
        except RuntimeError as e:                                           # if failed to initialize the encoding cycle
            self.logger.error(f"❌ Encoding cycle initialization failed → {e}")  # log before re-raising
            raise                                                           # raise the anomaly to the caller

        self.logger.info(f"✅ EMC initialized → {engram_gateway}")          # log successful init with engram path
   
    def bind_pmt(self, user_id: str, timestamp: str, content: str) -> bool:
        """
        Receive an evicted PMT from MCC and bind it into the episodic buffer for encoding.
        Called at the WMC → EMC boundary — non-blocking, crash-safe via staging table.

    Args:
        user_id (str)   : ID of the user who originated the PMT
        timestamp (str) : ISO-8601 timestamp of the original PMT
        content (str)   : Raw PMT content — truncated internally to engram content limit

        Returns:
            bool: True on success, False on failure
        """
        episode = Episode(
            user_id   = user_id,                                                                # user ID — should be retrieved from MCC
            timestamp = timestamp,                                                              # timestamp of PMT induced into WMC
            date      = timestamp[:10],                                                         # YYYY-MM-DD slice — B-tree indexed for date recall
            trace     = self._chunk_sampler.truncate(content, EMC.EPISODE_CONTENT_LIMIT),       # truncate to engram limit before binding
        )

        try:                                                                                    # attempt to bind the evicted PMT into episodic buffer
            with self._episodic_buffer_lock:                                                    # hold lock for binding stream append
                _binding_stream = self.episodic_buffer._binding_stream                          # reference for capacity check
                if len(_binding_stream) >= EMC.BINDING_STREAM_LIMIT:                            # at capacity — oldest will be silently dropped by deque maxlen
                    self.logger.warning(                                                        # warn — encoding engine may be offline or falling behind
                        f"⚠️  EMC binding stream at capacity ({EMC.BINDING_STREAM_LIMIT}) — "
                        f"oldest episode dropped. Encoding engine may be offline."
                    )
                _binding_stream.append(episode)                                                 # queue episode — oldest dropped automatically if at maxlen
            self._encoding_cycle.trigger_theta_rhythm()                                         # wake encoding cycle — theta rhythm
            self.logger.debug(                                                                  # log the binding of the evicted PMT into episodic buffer
                f"EMC buffer ← {self._chunk_sampler.probe(content)} chunks"
            )
            return True                                                                         # indicate successful binding
        except Exception as e:
            self.logger.warning(f"EMC binding PMT failed: {e}")                                 # log failure with reason
            return False                                                                        # indicate failure during binding

    def recall_episodes(self, user_id: str, cue: str) -> list[Episode]:
        """
        Recall relevant episodes from episodic memory.
        Dual-path retrieval fused via RRF — handled internally by MSB.

        Args:
            cue (str)          : Recall cue string

        Returns:
            list[Episode]: Recalled episodes sorted by descending relevancy.
                        Each dict: id, timestamp, date, trace, relevancy
        """
        if not cue or not cue.strip():                                                # empty cue — nothing to recall
            return []                                                                 # return empty list if no cue
    
        recall_cue: RecallCue = self._encoding_engine.encode_cue(cue)                 # encode cue into vector + raw text for dual-path recall
        rows: list[Episode] = self._ecx.recall_engram(                                # semantic + lexical RRF fusion — handled by MSB
            user_id, recall_cue, EMC.RECALL_SURFACE_LIMIT, EMC.RECALL_DEPTH
        )
    
        return [                                                                      # map SQL rows to Episode dataclasses at the EMC boundary
            Episode(
                storage_id = row.get("id"),                                           # SQLite rowid of the stored engram
                user_id    = row.get("user_id"),                                      # speaker identity preserved from original PMT
                timestamp  = row.get("timestamp", ""),                                # ISO-8601 temporal anchor
                date       = row.get("date", ""),                                     # YYYY-MM-DD slice for date-filtered recall
                trace      = row.get("trace", ""),                                    # formatted interaction text for reinstatement display
                created_at = row.get("created_at", ""),                               # wall-clock inscription time
                relevancy  = row.get("relevancy", 0.0),                               # transient RRF-fused score — populated here, consumed by reinstate_episodes
                encoding   = bytes(row["encoding"]) if row["encoding"] else b"",      # pre-fetch blob — active vector memory for M2 dual-plane reinstatement
            )
            for row in rows                                                           # iterate through each row
        ]

    def reinstate_episodes(self, user_id: str, cue: str) -> list[Episode]:
        """
        Recall, fragment, sequence, format, and reinstate relevant episodes into the recall stream.
        Full episodic reinstatement pipeline — MCC calls this once per turn.
        Primary output is injection into the recall stream for MCC context assembly.

        Pipeline:
            recall_episodes() → relevancy gate → fragment → sequence → format → recall stream

        Lifecycle stage: Recall → Reinstatement
            Recall         — dual-path RRF retrieval from the engram complex (semantic + lexical)
            Reinstatement  — fragmented, sequenced episodes injected into recall stream as a system message

        Args:
            user_id (str): ID of the user whose engrams to recall.
            cue (str)    : Current user prompt used as recall cue.

        Returns:
            list[Episode]: Reinstated episodes — informational only, primary output is recall stream injection.
        """
        # Recall candidates
        raw_episodes: list[Episode] = self.recall_episodes(user_id, cue)                    # recall candidates via RRF dual-path retrieval

        # Relevancy gate — EMC owns this threshold
        episode_scaffold: list[Episode] = [                                                 # filter out irrelevant episodes
            episode for episode in raw_episodes                                             # iterate over raw episodes 
            if episode.relevancy >= EMC.RELEVANCE_THRESHOLD                                 # episode must meet relevance threshold
        ]

        # Memory fragmenting — surface fragments of memory when recall reserve is exceeded
        episode_scaffold: list[Episode] = self._episodic_scaffold.fragment(episode_scaffold) # trim the recalled episodes to fit chunk reserve

        # Reorder episodes chronologically (oldest → newest)
        episode_scaffold: list[Episode] = self._episodic_scaffold.sequence(episode_scaffold) # sequence episodes in chronological order

        if not episode_scaffold:                                                            # nothing to stage — skip
            return []                                                                       # return if no filtered episodes

        lines = [                                                                           # system message header
            "Past memories (for context only — these are not the current conversation):",
            ""
        ]    
        
        for episode in episode_scaffold:                                                    # iterate through each episode in the scaffold
            date = episode.date or "unknown date"                                           # date attribute — fallback if empty
            lines.append(f"[{date}] {episode.trace}")                                       # trace is pre-formatted at bind time — no deserialization needed
            lines.append("")                                                                # blank line for readability
    
        self.episodic_buffer.stage_single_episode({                                         # inject as single system message to the recallstream
            "role":    "system",                                                            # set role as system
            "content": "\n".join(lines)                                                     # join all lines into a single string
        })

        return episode_scaffold                                                             # return filtered episodes
    
    def assess_emc(self) -> dict:
        """
        Return EMC health and storage stats.
    
        Returns:
            dict: EMC stats including encoding engine status, binding stream count, and engram complex stats
                  Empty dict on failure
        """
        try:
            with self._episodic_buffer_lock:                                    # hold lock for accurate binding stream count
                binding_pending = len(self.episodic_buffer._binding_stream)     # count episodes queued for encoding

            engram_stats = self._ecx.assess_engram_complex()                    # query engram complex for storage stats

            return {
                "engram_count"  : engram_stats.get("engram_count", 0),                    # total encoded episodes in engram storage
                "earliest_timestamp"  : engram_stats.get("earliest_timestamp", None),     # timestamp of the oldest stored episode
                "latest_timestamp"    : engram_stats.get("latest_timestamp", None),       # timestamp of the most recent stored episode
                "physical_volume"     : engram_stats.get("physical_volume", 0.0),         # engram complex size on disk (MB)
                "vector_index_active" : engram_stats.get("vector_index_active", False),   # True if KNN vector index is available
                "buffer_count"        : engram_stats.get("buffer_count", 0),              # unencoded episodes in staging buffer
                "binding_pending"       : binding_pending,                                # episodes queued in binding stream
                "encoding_engine_ready" : self._encoding_engine.is_available,             # True if sentence-transformers loaded
                "encoding_engine"       : self._encoding_engine.encoding_engine           # model name for reference
            }
        except Exception as e:                                                  # if assessment fails
            self.logger.error(f"EMC assessment failed: {e}")                    # log failure with reason
            return {}                                                           # empty dict — caller handles no results

    def drain_encoding_cycle(self, timeout: float = 10.0) -> None:
        """
        Wait for the binding stream to empty before shutdown.
        Gives the encoding cycle time to consolidate flushed PMTs.
        Called by MCC after flushing remaining WMC PMTs on close.
    
        Args:
            timeout (float): Max seconds to wait — prevents hanging on shutdown
        """
        expiry_time = time.time() + timeout                                 # calculate expiry time from now
        while time.time() < expiry_time:                                    # keep checking until expiry time
            with self._episodic_buffer_lock:                                # hold lock for accurate count
                if not self.episodic_buffer._binding_stream:                # binding stream empty — safe to terminate
                    self.logger.info("✅ EMC binding stream drained")       # log successful drain
                    return
            time.sleep(0.1)                                                 # yield — encoding cycle runs on its own thread
        self.logger.warning(                                                # expiry time reached — some PMTs may be lost
            "⚠️  EMC drain timeout — some PMTs may not have been consolidated"
        )

    def terminate(self) -> None:
        """
        Gracefully close EMC and release the engram connection.
        Signals encoding cycle to stop and waits for clean exit
        before closing the engram complex.
        """
        self._encoding_cycle.stop_cycle()                                   # signal encoding cycle to stop and wait for clean exit
        self._ecx.terminate()                                               # release engram complex connection
        self.logger.info("🗄️ EMC deactivated")                              # log successful termination
