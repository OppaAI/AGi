"""
MCC — Memory Coordination Core
================================
AuRoRA · Semantic Cognitive System (SCS)

Single memory interface for CNC — coordinates WMC and EMC.
CNC never touches WMC or EMC directly; all memory operations route through MCC.

Responsibilities:
    - Receive induced PMTs from CNC and fill them into WMC
    - Bind evicted PMTs to episodic buffer for EMC encoding (async, non-blocking)
    - Assemble unified memory context from sustained WMC PMTs and recalled EMC episodes
    - Manage cortical capacity across the full memory core

Architecture:
    MCC mirrors the human prefrontal cortex — the brain's active workspace
    where working memory and episodic recall converge into a single conscious field.

    WMC and EMC operate independently but are unified in assemble_memory_context()
    into one memory context passed to the cognitive engine — exactly as fresh thoughts
    and recalled memories surface together into the same prefrontal awareness.
    The cognitive engine cannot distinguish a sustained WMC PMT from a recalled EMC
    engram; to it, they are all active cognition.

    Cortical capacity budget:
        Identity and cognition          →  CNS.COGNITIVE_RESERVE
        EMC recalled engrams            →  CNS.EMC.RECALL_RESERVE
        WMC sustained PMTs              →  CNS.WMC.GLOBAL_CHUNK_LIMIT
        ─────────────────────────────────────────────────────────────────────
        Total active cognitive core     →  CNS.CORTICAL_CAPACITY

Terminology:
    Buffer      — temporary staging area for memory traces in transition
                  (evicted PMTs from WMC awaiting EMC encoding, or recalled EMC 
                  episodes awaiting reinstatment into the memory context)
    Context     — the full active memory presented to the cognitive engine
                  (WMC PMTs + relevant EMC episodes, assembled each turn)
    Engram      — one physical memory record stored in the engram complex
    PMT         — phonological memory trace; one complete interaction
                  (user prompt + AI response). WMC pairs turns internally —
                  MCC forwards complete evicted PMTs to EMC.
    Reserve     — cortical capacity allocated to a specific memory function
    Threshold   — minimum relevancy score required for an EMC episode
                  to enter the active memory context

Public interface:
    MemoryCoordinationCore:
        await register_memory(user_id: str, content: str) -> None
        context = await assemble_memory_context(user_prompt: str) -> list[dict]
        assess_memory_schema() -> dict
        report_memory_stats() -> None
        forget_memory() -> None
        close() -> None

TODO:
    M2 — implement session-end consolidation: flush WMC PMTs to EMC on shutdown,
         gated on novelty/importance scoring — low-salience turns should be
         truly forgotten, not blindly bound
    M2 — salience gate at eviction boundary in _bind_to_episodic_buffer():
         score evicted PMTs for novelty and importance before binding;
         discard low-salience turns, bind high-salience turns to EMC.
         WMC and EMC remain salience-agnostic — MCC owns this gate.
    M2 — dynamic EMC capacity adjustment: if recalled engrams exceed
         EMC_RECALL_RESERVE, trim to fit rather than silently overrunning
         WMC's chunk limit
    M2 — WMC/EMC health check with capacity breach warnings
    M2 — add SMC, 11pm reflection trigger
    M3 — add PMC, procedural skill retrieval
"""

# System libraries
import asyncio                              # for fire-and-forget episodic binding and EMC recall timeout racing
from pathlib import Path                    # for constructing and creating the engram gateway on disk

# AGi libraries
from scs.wmc import WorkingMemoryCortex     # Working Memory Cortex — sustains active PMTs in hot short-term memory
from scs.emc import EpisodicMemoryCortex    # Episodic Memory Cortex — encodes evicted PMTs and recalls past episodes
from hrs.hrp import AGi                     # homeostatic regulation parameter registry — system-wide constants
CNS = AGi.CNS                               # CNS parameter namespace alias — keeps constant references concise

class MemoryCoordinationCore:
    """
    Memory Coordination Core — the memory manager of the CNS.

    Central relay between CNC and the memory cortex layers (WMC, EMC).
    Instantiated once at startup. All memory operations go through MCC
    — CNC never touches WMC or EMC directly.
    """

    def __init__(self, logger) -> None:
        """
        Initiatiate the Memory Coordination Core and prepare its memory layers for operation.

        Args:
            logger: Logger instance forwarded from CNC
        """
        self.logger = logger             # logger forwarded from CNC — all MCC methods emit through this handle

        # Ensure engram gateway exists
        # TODO: HRS milestone — move path construction to hrs.py entity gateway
        self.engram_gateway = (         # construct absolute path to the engram complex on disk
            Path.home() /               # anchor at OS home (~)
            AGi.ENTITY_GATEWAY /        # descend into the entity gateway directory
            CNS.NEURAL_GATEWAY /        # descend into the neural gateway subdirectory
            CNS.ENGRAM_COMPLEX          # land at the engram complex — where encoded episodes live
        )
        self.engram_gateway.parent.mkdir(parents=True, exist_ok=True)      # create all missing parent dirs — no-op if already exists

        # Initialize memory cortex layers
        self.logger.info("🔄 Activating Memory Coordination Core…")                         # log entry on MCC activation
        self.wmc = WorkingMemoryCortex(logger=logger)                                       # boot WMC — owns the active PMT slot
        self.emc = EpisodicMemoryCortex(logger=logger, engram_gateway=self.engram_gateway)  # boot EMC — owns the engram complex on disk

        self._recall_timeout = CNS.EMC.RECALL_TIMEOUT                                        # cache recall timeout — EMC runs on a thread and must not stall inference

        self.logger.info("✅ Memory Coordination Core Activated")                           # log entry on successful MCC activation

    async def register_memory(self, user_id: str, content: str) -> None:
        """
        Receive a new conversation turn and commit it to working memory.
        Any PMTs evicted by WMC are handed off to the episodic buffer — non-blocking.
        Eviction only fires at interaction boundary — never mid-exchange.

        1. Pair to complete interaction and fill induced PMT into WMC
        2. Bind any evicted PMT to episodic buffer (non-blocking)

        Args:
            user_id (str) : User ID of the speaker interacting with GRACE
            content (str) : Content of the conversation turn
        """
        # Fill induced PMT to WMC — returns evicted PMTs synchronously (fast, in-memory)
        evicted_pmts = self.wmc.fill_pmt(speaker=user_id, content=content)  # induce turn into WMC — returns any PMTs displaced by the new arrival

        # Bind evicted PMTs to episodic buffer
        # Run and forget — never blocks active cognition
        if evicted_pmts:                                                    # evicted PMTs present — hand off to episodic buffer
            _ = asyncio.get_running_loop().run_in_executor(                 # recruit a dormant thread — binding never blocks active cognition
                None, self._bind_to_episodic_buffer, evicted_pmts
            )
            self.logger.debug(                                              # log the binding transition of evicted PMTs to episodic buffer
                f"MCC bound {len(evicted_pmts)} evicted PMT(s) → episodic buffer"
            )

    def _bind_to_episodic_buffer(self, evicted_pmts: list[dict]) -> None:
        """
        Bind evicted PMTs from WMC into the episodic buffer for encoding and consolidation.
        Runs on a dormant thread — never blocks active cognition.
        Trivial PMT filter (M1): discards turns under 50 chars — filler turns not worth encoding.
        Anchor vector filter (M1.5): semantic filtering via embeddinggemma replaces this.

        Args:
            evicted_pmts (list[dict]) : List of evicted PMTs [{content, timestamp}]
        """
        try:                                                         # attempt binding evicted PMTs to episodic buffer
            for evicted_pmt in evicted_pmts:                         # iterate through each evicted PMT
                # M1 trivial filter — discard filler turns under 50 chars
                if len(evicted_pmt.get("content", "")) < 50:         # trivial filter — discard filler turns under 50 chars
                    self.logger.debug(                               # log the discarded trivial PMT
                        "MCC discarded trivial PMT — below length threshold"
                    )
                    continue                                         # skip — not worth encoding into episodic memory
                self.emc.bind_pmt(                                   # bind evicted pmt into episodic buffer
                    timestamp=evicted_pmt["timestamp"],              # timestamp of the original PMT
                    content=evicted_pmt["content"],                  # raw content of the evicted PMT
                )
        except Exception as e:                                       # if binding lapse occurs, log and continue
            self.logger.error(                                       # Log the binding lapse
                f"MCC binding lapse — {len(evicted_pmts)} PMT(s) unbound: {e}",
            )
            
    async def assemble_memory_context(self, user_prompt: str) -> list[dict]:
        """
        Assemble the full memory context for the cognitive engine.
        EMC reinstatement runs on a dormant thread — awaited before returning.
        Awaits both before returning — inference requires full memory context.

        Structure:
            [EMC reinstated episodes (system block, if relevant)]
            + [WMC PMTs (chronological, chat turns)]

        Args:
            user_prompt (str) : Current user message — used as EMC recall cue

        Returns:
            list[dict] : List of message dicts [{role, content}] ready for inference
        """
        # Recall WMC PMTs directly in main neural pathway
        wmc_pmts: list[dict[str, str]] = self.wmc.recall_pmt_schema()        # recall sustained PMTs from working memory
        
        # EMC reinstatement on isolated neural pathway — recall, filter, format, inject
        self.emc.episodic_buffer.clear_recall_stream()                       # clear recall stream before reinstating fresh episodes
        reinstated_episodes: list[dict] = []                                 # holds reinstated episodes from EMC
        try:                                                                 # attempt to recall EMC episodes
            future = asyncio.get_running_loop().run_in_executor(             # recruit a dormant thread — EMC recall blocks on encoding engine
                None, self.emc.reinstate_episodes, user_prompt               # reinstate relevant episodes using current prompt as cue
            )
            reinstated_episodes = await asyncio.wait_for(                    # await with timeout — recall must not stall inference
                future, timeout=self._recall_timeout                         # set a time limit for recall of episodic memory traces
            )
        except asyncio.TimeoutError:                                         # recall exceeded time limit — cancel dormant thread
            self.logger.warning("⚠️  EMC recall timed out — proceeding without episodic context")    # log the timeout error while recalling EMC episodes

        # Reinstate WMC PMTs after EMC episodes — for chronological order in memory context
        self.emc.episodic_buffer.stage_episode_list(wmc_pmts)                # inject WMC PMTs after EMC episodes — preserves chronological order
        
        self.logger.debug(                                                   # log the memory context assembled
            f"MCC context assembled: "
            f"{len(wmc_pmts)} WMC PMTs + {len(reinstated_episodes)} EMC episodes reinstated"
        )

        return self.emc.episodic_buffer.assess_recall_stream()               # return full memory context for inference

    def assess_memory_schema(self) -> dict:
        """
        Assess the current state of all memory cortex layers for logging and health checks.

        Returns:
            dict : Combined schema from WMC and EMC cortex layers
        """
        wmc_schema: dict = self.wmc.assess_pmt_schema()      # assess current PMT state of working memory
        emc_schema: dict = self.emc.assess_emc()             # assess current engram state of episodic memory
        return {                                             # return the current stats of all memory cortex layers
            "wmc": wmc_schema,                               # current stats of working memory
            "emc": emc_schema,                               # current stats of engram complex
        }

    def report_memory_stats(self) -> None:
        """
        Report current memory cortex stats to the log.
        Called by CNC after every turn for health monitoring.

        TODO:
        - expand into detailed health check with warnings on capacity breaches, anomalous eviction rates, etc.
        - GUI — expose via ROS2 topic for real-time memory visualisation
        """
        stats: dict = self.assess_memory_schema()           # assess current state of all memory cortex layers
        wmc_stats: dict = stats["wmc"]                      # extract working memory stats
        emc_stats: dict = stats["emc"]                      # extract episodic memory stats
        self.logger.info(                                   # log the current stats of all memory cortex layers 
            f"🧠 Memory stats:\n"
            f"   WMC: {wmc_stats['pmt_count']} PMTs ({wmc_stats['slot_occupancy']}%) | "
            f"{wmc_stats['sustained_chunks']}/{wmc_stats['global_chunk_limit']} chunks ({wmc_stats['chunk_occupancy']}%)\n"
            f"   EMC: {emc_stats.get('engram_count', 0)} episodes | "
            f"{emc_stats.get('binding_pending', 0)} binding | "
            f"{emc_stats.get('buffer_count', 0)} staged | "
            f"{emc_stats.get('physical_volume', 0)} MB"
        )

    def forget_memory(self) -> None:
        """
        Forget active memory at session end or conversation reset.

        Clears:
            - Working memory (WMC pmt_slot)
        Preserves:
            - Episodic memory (EMC) — permanent
            - Semantic memory (SMC) — permanent (future)
            - Procedural memory (PMC) — permanent (future)
        """
        self.wmc.forget_pmt_schema()                        # clear all sustained PMTs from working memory
        self.logger.info("🧹 Working memory forgotten")     # log entry on successful working memory forgetting

    def close(self) -> None:
        """
        Gracefully close MCC and its memory cortex layers.
        WMC has no persistent resources to close — EMC releases its
        open handles to the engram gateway on termination.
        """
        self.emc.terminate()                                        # release EMC engram gateway file handles
        self.logger.info("🗄️  MCC shutdown sequence complete")      # log entry on successful MCC closure
