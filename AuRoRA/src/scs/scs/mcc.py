"""
MCC — Memory Coordination Core
================================
AuRoRA · Semantic Cognitive System (SCS)

Single memory interface for CNC — coordinates WMC and EMC.
CNC never touches WMC or EMC directly, only calls MCC.

Responsibilities:
    - Fill induced PMTs to WMC
    - Bind evicted PMTs → episodic buffer (async, non-blocking)
    - Assemble memory context into episodic buffer for cognitive engine (sustained WMC PMTs + recalled EMC episodes)
    - Manage cortical capacity across the full memory context

Architecture:
    MCC mirrors the human prefrontal cortex — the brain's active workspace
    where working memory and episodic recall share the same conscious space.

    WMC and EMC operate independently but converge in assemble_memory_context() into
    a single unified memory context sent to cognitive engine — exactly as fresh thoughts
    and recalled memories both surface into the same prefrontal awareness.
    Cognitive engine cannot distinguish a recent sustained  WMC PMTs from a recalled EMC engram —
    they are all just active cognition context.

    Cortical capacity budget:
        Identity and cognition          →  CNS_COGNITIVE_RESERVE
        EMC recalled engrams            →  EMC_RECALL_RESERVE
        WMC sustained PMTs              →  WMC_GLOBAL_CHUNK_LIMIT
        ─────────────────────────────────────────────────────────────────────
        Total active cognitive core     →  CNS_CORTICAL_CAPACITY

Terminology:
    Buffer      — temporary staging area for memory traces in transition 
                (e.g. evicted PMTs from WMC waiting for encoding and consolidation in EMC, or
                recalled EMC episodes waiting to be injected into memory context)
    Context     — active memory for cognition (WMC PMTs + relevant EMC episodes)
    Engram      — episodic memory trace (one past interaction, user prompt + AI response)
    PMT         — phonological memory trace (one interaction, user prompt + AI response)
                WMC pairs the interaction internally; MCC forwards evicted PMTs to EMC for encoding and consolidation
    Scaffold    — temporary staging area for relevant EMC episodes to be injected into memory context
    Reserve     — cortical capacity reserve for a specific memory function (e.g. EMC_RECALL_RESERVE for recalling relevant EMC episodes)
    Threshold   — minimum relevancy score for an EMC episode to be injected into memory context

Public interface:
    await mcc.register_memory(role, content)
    context = await mcc.assemble_memory_context(user_prompt)
    mcc.report_memory_stats()
    mcc.forget_memory()
    mcc.close()

TODO:
    M2 — implement session-end consolidation: flush WMC PMTs to EMC on shutdown
        gate on novelty/importance scoring — low-salience turns should be
        truly forgotten, not blindly bound
    M2 — salience gate at eviction boundary in _bind_to_episodic_buffer():
         score evicted PMTs for novelty and importance before binding;
         discard low-salience turns, bind high-salience turns to EMC.
         WMC and EMC remain salience-agnostic — MCC owns this decision.
    M2 — dynamic EMC capacity adjustment — if recalled engrams exceed
         EMC_RECALL_RESERVE, trim to fit rather than silently overrunning
         WMC's chunk limit
    M2 — WMC/EMC health check
    M2 — add SMC, 11pm reflection trigger
    M3 — add PMC, procedural skill retrieval
"""

# System libraries
import asyncio                              # For concurrent WMC and EMC recall
import json                                 # For structured PMT storage — crash-safe serialization and recall
from pathlib import Path                    # For handling gateway to the engrams

# AGi libraries
from scs.wmc import WorkingMemoryCortex     # Working Memory Cortex layer of the CNS, responsible for sustaining PMTs in working memory
from scs.emc import EpisodicMemoryCortex    # Episodic Memory Cortex layer of the CNS, responsible for recalling relevant episodes from the engram complex
from hrs.hrp import AGi                     # Import AGi homeostatic regulation parameters
CNS = AGi.CNS                               # Channel for interfacing with Central Nervous System (CNS)

class MemoryCoordinationCore:
    """
    Memory Coordination Core — the memory manager of the CNS.

    Central relay between CNC and the memory cortex layers (WMC, EMC).
    Instantiated once at startup. All memory operations go through MCC
    — CNC never touches WMC or EMC directly.
    """

    def __init__(self, logger) -> None:
        """
        Initialize the Memory Coordination Core with a logger and set up the engram gateway.
        Also initializes the WMC and EMC layers of the CNS.

        Args:
            logger: Logger instance from CNC for logging MCC operations
        """
        self.logger = logger            # Retrieve logger from CNC for logging MCC operations

        # Ensure engram gateway exists
        # TODO: HRS milestone — move path construction to hrs.py entity gateway
        self.engram_gateway = (         # Construct the gateway towards engram complex
            Path.home() / 
            AGi.ENTITY_GATEWAY / 
            CNS.NEURAL_GATEWAY / 
            CNS.ENGRAM_COMPLEX
        )
        self.engram_gateway.parent.mkdir(parents=True, exist_ok=True)      # Generate the gateway if not already exists

        # Initialize memory cortex layers
        self.logger.info("🔄 Activating Memory Coordination Core…")                         # Log entry on MCC activation
        self.wmc = WorkingMemoryCortex(logger=logger)                                       # For invoking WMC with provided logger
        self.emc = EpisodicMemoryCortex(logger=logger, engram_gateway=self.engram_gateway)  # For invoking EMC with provided logger and gateway to engram complex
        self.logger.info("✅ Memory Coordination Core Activated")                           # Log entry on successful MCC activation

    async def register_memory(self, user_id: str, content: str) -> None:
        """
        Register new PMT into working memory and bind any evicted interactions to episodic buffer.
        WMC accumulates individual turns and pairs them into complete interactions internally.
        Eviction only fires at interaction boundary — never mid-exchange.

        1. Pair to complete interaction and fill induced PMT into WMC
        2. Bind any evicted PMT to episodic buffer (non-blocking)

        Args:
            speaker (str): User ID of the speaker interacting with the AI
            content (str): Content of the conversation turn (the message text)
        """
        # Fill induced PMT to WMC — returns evicted PMTs synchronously (fast, in-memory)
        evicted_pmts = self.wmc.fill_pmt(speaker=user_id, content=content)  # Induce conversation turn to WMC, and collect any evicted PMTs
            
        # Bind evicted PMTs to episodic buffer
        # Run and forget — never blocks active cognition
        if evicted_pmts:                                                    # If WMC evicted any PMTs, bind them to episodic buffer
            asyncio.get_running_loop().run_in_executor(                     # Recruit a dormant neural thread — run binding on isolated neural pathway
                None, self._bind_to_episodic_buffer, evicted_pmts
            )
            self.logger.debug(                                              # Log the binding transition of evicted PMTs to episodic buffer
                f"MCC bound {len(evicted_pmts)} evicted PMT(s) → episodic buffer"
            )

    def _bind_to_episodic_buffer(self, evicted_pmts: list[dict]) -> None:
        """
        Bind evicted PMTs from WMC into episodic buffer for pending encoding and consolidation.
        Takes place in isolated neural pathway — never blocks active cognition neural pathway.

        Args:
            evicted_pmts (list[dict]): List of evicted PMTs [{content, timestamp}]
        """
        try:                                                         # Attempt binding evicted PMTs to episodic buffer
            for evicted_pmt in evicted_pmts:                         # Process each evicted PMT                
                self.emc.bind_pmt(                                   # Bind evicted pmt into episodic buffer
                    timestamp=evicted_pmt["timestamp"],              # Bind the timestamp of the interaction
                    content=evicted_pmt["content"],                  # Bind the content of the interaction
                )       
        except Exception as e:                                       # If binding lapse occurs, log and continue
            self.logger.error(                                       # Log the binding lapse
                f"MCC binding lapse — {len(evicted_pmts)} PMT(s) unbound: {e}",
            )
            
    async def assemble_memory_context(self, user_prompt: str) -> list[dict]:
        """
        Assemble full memory context for the cognitive engine.
        Structure:
            [WMC PMTs (chronological)]
            + [EMC episodes injected into memory context (if relevant)]
        WMC PMTs are recalled directly in the main neural pathway, while 
        EMC episodes are recalled on an isolated neural pathway — awaited before returning.
        Awaits both before returning — inference requires full memory context.
        
        Args:
            user_prompt (str): Current user message (used as EMC recall query)
        Returns:
            list[dict]: List of message dicts [{role, content}] ready for inference
        """
        
        # Recall WMC PMTs directly in main neural pathway, then EMC episodes on isolated neural pathway
        wmc_pmts = self.wmc.recall_pmt_schema()                              # Recall WMC PMTs in main neural pathway
        
        try:                                                                 # Attempt to recall EMC episodes
            emc_episodes = await asyncio.wait_for(                           # Await recalling of EMC episodes that is on isolated neural pathway
                asyncio.get_running_loop().run_in_executor(
                    None, self.emc.recall_episodes, user_prompt, CNS.EMC.RECALL_DEPTH
                ),
                timeout=CNS.EMC.RECALL_TIMEOUT                               # Set a time limit for recalling EMC episodes
            )
        except asyncio.TimeoutError:                                         # If timeout error happens while recalling EMC episodes
            self.logger.warning("⚠️  EMC recall timed out — proceeding without episodic context")    # Log the timeout error while recalling EMC episodes
            emc_episodes = []                                                # Proceed without episodic context
        
        self.logger.info(                                                    # Log the EMC recall results with relevancy scores
            f"EMC raw recall: {len(emc_episodes)} episodes, "
            f"scores: {[e['relevancy'] for e in emc_episodes]}"
        )

        # Pass through memory gate — suppress episodes below relevancy threshold
        episodic_scaffold = [                                                # Set up EMC episodic scaffold to stage the relevant episodes
            episode for episode in emc_episodes                              # Process each of the recalled EMC episodes
            if episode["relevancy"] >= CNS.EMC.RELEVANCE_THRESHOLD           # Stage the EMC episode if above relevancy threshold
        ]

        # Assemble memory context
        self.emc.episodic_buffer.clear_recall_stream()                       # Reset recall stream of episodic buffer before assembling memory context

        # Inject relevant EMC episodes as a system message
        if episodic_scaffold:                                                # If EMC episodic scaffold is not empty,
            for episode in episodic_scaffold:                                # Access each EMC episode in the EMC episodic scaffold
                content     = episode.get("content", "")                     # Retrieve the content of the EMC episode

                # Deserialize JSON content into user prompt/AI response pairs for context assembly
                try:
                    content = json.loads(content)
                    self.emc.episodic_buffer.stage_single_episode({"role": "user",      "content": content["user"]})
                    self.emc.episodic_buffer.stage_single_episode({"role": "assistant", "content": content["assistant"]})
                except (json.JSONDecodeError, KeyError):
                    # Malformed — surface as-is rather than silent drop
                    self.emc.episodic_buffer.stage_single_episode({"role": "user", "content": content})
                 
            self.logger.debug(                                               # Log the number of EMC episodes bound into episodic buffer
                f"MCC injected {len(episodic_scaffold)} EMC episode(s) into episodic buffer"
            )

        # Bind sustained WMC PMTs in chronological order
        self.emc.episodic_buffer.stage_episode_list(wmc_pmts)                # Stage the sustained WMC PMTS into episodic buffer

        self.logger.debug(                                                   # Log the recalled memory context (WMC PMTs + recalled EMC episodes)
            f"MCC memory context recalled: "
            f"{len(wmc_pmts)} WMC PMTs + "
            f"{len(episodic_scaffold)} EMC episodes"
        )

        return self.emc.episodic_buffer.assess_recall_stream()               # Return the memory context staged in episodic buffer

    def assess_memory_schema(self) -> dict:
        """
        Assess the current stats of all memory cortex layers for logging and health checks.
    
        Returns:
            dict: Combined schema from WMC and EMC cortex layers
        """
        wmc_schema: dict = self.wmc.assess_pmt_schema()      # Assess the PMT schema of WMC
        emc_schema: dict = self.emc.get_stats()              # Assess the engram complex of EMC
        return {                                             # Return the current stats of all memory cortex layers
            "wmc": wmc_schema,
            "emc": emc_schema,
        }

    def report_memory_stats(self) -> None:
        """
        Report current memory cortex stats.
        Called by CNC after every turn for health monitoring.

        TODO:
        - expand into detailed health check with warnings on capacity breaches, anomalous eviction rates, etc.
        - GUI — expose via ROS2 topic for real-time memory visualisation
        """
        stats: dict = self.assess_memory_schema()           # Assess the current stats of all memory cortex layers
        wmc_stats: dict = stats["wmc"]                      # Retrieve current stats of working memory
        emc_stats: dict = stats["emc"]                      # Retrieve current stats of episodic memory
        self.logger.info(                                   # Log the current stats of all memory cortex layers 
            f"🧠 Memory stats:\n"
            f"   WMC: {wmc_stats['pmt_count']} PMTs ({wmc_stats['slot_occupancy']}%) | "
            f"{wmc_stats['sustained_chunks']}/{wmc_stats['global_chunk_limit']} chunks ({wmc_stats['chunk_occupancy']}%)\n"
            f"   EMC: {emc_stats.get('episodes', 0)} episodes | "
            f"{emc_stats.get('buffer_pending', 0)} pending embed | "
            f"{emc_stats.get('db_size_mb', 0)} MB"
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
        self.wmc.forget_pmt_schema()                        # Forget all sustaining PMT schema in working memory
        self.logger.info("🧹 Working memory forgotten")     # Log entry on successful working memory forgetting

    def close(self) -> None:
        """
        Gracefully close MCC and its memory cortex layers.
        WMC has no persistent resources to close,
        but EMC may have open file handles to the engram gateway that need to be released.
        """
        self.emc.close()                                            # Close the engram gateway
        self.logger.info("🗄️  MCC shutdown sequence complete")      # Log entry on successful MCC closure
