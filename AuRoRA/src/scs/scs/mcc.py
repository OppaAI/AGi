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

TODO:
    M2 — dynamic EMC capacity adjustment — if recalled engrams exceed
         EMC_RECALL_RESERVE, trim to fit rather than silently overrunning
         WMC's chunk limit
    M2 — WMC/EMC health check
    M2 — add SMC, 11pm reflection trigger
    M3 — add PMC, procedural skill retrieval
"""

# System libraries
import asyncio                              # For concurrent WMC and EMC recall
from pathlib import Path                    # For handling gateway to the engrams

# AGi libraries
from scs.wmc import WorkingMemoryCortex     # Working Memory Cortex layer of the CNS, responsible for sustaining PMTs in working memory
from scs.emc import EpisodicMemoryCortex    # Episodic Memory Cortex layer of the CNS, responsible for recalling relevant episodes from the engram complex
from hrs.hrp import AGi                     # Import AGi homeostatic regulation parameters
CNS = AGi.CNS                               # Channel AGi CNS parameters for direct access in MCC

class MemoryCoordinationCore:
    """
    Memory Coordination Core — the thalamus of the CNS.

    Central relay between CNC and the memory organs (WMC, EMC).
    Instantiated once by CNC at startup. All memory operations go
    through MCC — CNC never touches WMC or EMC directly.

    Access:
        MCC reads AGi.CNS.* parameters from hrp.py directly.
        MCC accesses WMC and EMC parameters through their instances only —
        never via hrp.py directly.

    CNC interface:
        await mcc.add_turn(role, content)
        context = await mcc.build_context(user_input)
        mcc.log_stats()
        mcc.close()
    """

    def __init__(self, logger) -> None:
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
        self.logger.info("🔄 Activating Memory Coordination Core...")                       # Log entry on MCC activation
        self.wmc = WorkingMemoryCortex(logger=logger)                                       # Initialize WMC with provided logger
        self.emc = EpisodicMemoryCortex(engram_gateway=self.engram_gateway, logger=logger)  # Initialize EMC with provided gateway to engram complex and logger
        self.logger.info("✅ Memory Coordination Core Activated")                           # Log entry on successful MCC activation

    async def relay_pmt(self, role: str, content: str) -> None:
        """
        Relay new PMT to WMC and bind any evicted PMTs to episodic buffer.

        1. Fill induced PMT to WMC
        2. Bind any evicted PMTs to episodic buffer (non-blocking)

        Args:
            role:    "user" or "assistant"
            content: Message text
        """
        # Fill induced PMT to WMC — returns evicted PMTs synchronously (fast, in-memory)
        evicted_pmts = self.wmc.fill_pmt(role, content)                     # Fill the induced PMT into WMC, collect any evicted PMTs
            
        # Bind evicted PMTs to episodic buffer
        # Run and forget — never blocks active cognition
        if evicted_pmts:                                                    # If WMC evicted any PMTs, bind them to episodic buffer
            loop = asyncio.get_running_loop()                               # Access the main neural pathway
            loop.run_in_executor(                                           # Recruit a dormant neural thread — run binding on isolated neural pathway
                None, self._bind_to_episodic_buffer, evicted_pmts
            )
            self.logger.debug(                                              # Log the binding transition of evicted PMTs to episodic buffer
                f"MCC bound {len(evicted_pmts)} evicted PMT(s) → episodic buffer"
            )

    def _bind_to_episodic_buffer(self, evicted_pmts: list[dict]) -> None:
        """
        Bind evicted PMTs from WMC into episodic buffer for pending encoding and consolidation.
        Runs in isolated neural pathway — never blocks active cognition.

        Args:
            evicted_pmts: List of evicted PMTs [{role, content, timestamp}]
        """
        try:                                                # Attempt binding evicted PMTs to episodic buffer
            for evicted_pmt in evicted_pmts:                # Process each evicted PMT
                self.emc.buffer_append(                     # Bind each evicted PMT into episodic buffer
                    role    = evicted_pmt["role"],
                    content = evicted_pmt["content"],
                    timestamp = evicted_pmt["timestamp"],
                )
        except Exception as e:                             # If binding lapse occurs, log and continue
            self.logger.error(                             # Log the binding lapse
                f"EMC binding lapse — {len(evicted_pmts)} PMT(s) unbound: {e}",
                exc_info=True
            )
            
    async def assemble_memory_context(self, user_prompt: str) -> list[dict]:
        """
        Assemble full memory context for the cognitive engine.
        Structure:
            [WMC PMTs (chronological)]
            + [EMC episodes injected into memory context (if relevant)]
        EMC recall runs concurrently with WMC PMT recall — no added latency.
        Awaits both before returning — inference requires full memory context.
        
        Args:
            user_prompt: Current user message (used as EMC recall query)
        Returns:
            List of message dicts [{role, content}] ready for inference
        """
        
        # Recall WMC PMTs and EMC episodes concurrently
        loop = asyncio.get_running_loop()                                    # Access the main neural pathway

        wmc_pmts_pending = loop.run_in_executor(                             # Recruit a dormant neural thread — run WMC PMT recall on isolated neural pathway
            None, self.wmc.recall_pmt_schema
        )
        emc_episodes_pending = loop.run_in_executor(                         # Recruit another dormant neural thread — run EMC episode recall on isolated neural pathway
            None, self.emc.recall, user_prompt, CNS.EMC.RECALL_DEPTH
        )

        wmc_pmts, emc_episodes = await asyncio.gather(                       # Await both pending recalls — synchronize into active cognition
            wmc_pmts_pending, emc_episodes_pending
        )

        # Pass through memory gate — suppress episodes below relevancy threshold
        episodic_scaffold = [                                                # Set up EMC episodic scaffold to stage the relevant episodes
            episode for episode in emc_episodes                              # Process each of the recalled EMC episodes
            if episode["relevancy"] >= CNS.EMC.RECALL_THRESHOLD              # Stage the EMC episode if above relevancy threshold
        ]

        # Assemble memory context
        episodic_buffer: list[dict] = []                                     # Set up episodic buffer for holding memory context

        # Inject relevant EMC episodes as a system message
        if episodic_scaffold:                                                # If EMC episodic scaffold is not empty,
            episode_content = ["Relevant memories from past conversations:"]
            for episode in episodic_scaffold:                                # Access each EMC episode in the EMC episodic scaffold
                date        = episode.get("date", "unknown date")            # Retrieve the date of the EMC episode
                role        = episode.get("role", "unknown")                 # Retrieve the role of the EMC episode
                content     = episode.get("content", "")                     # Retrieve the content of the EMC episode
                relevancy   = episode.get("relevancy", 0.0)                  # Retrieve the relevancy score of the EMC episode
                episode_content.append(                                      # Stage the content of EMC episode into the episodic buffer
                    f"[{date}] {role}: {content} (relevance: {relevancy:.2f})"
                )

            episodic_buffer.append({                                          # Bind the recalled EMC episodes into episodic buffer
                "role":    "system",
                "content": "\n".join(episode_content),
            })

            self.logger.debug(                                                # Log the number of EMC episodes bound into episodic buffer
                f"MCC injected {len(episodic_scaffold)} EMC episode(s) into Recall Episodic Buffer"
            )

        # Bind sustained WMC PMTs in chronological order
        episodic_buffer.extend(wmc_pmts)                                       # Bind the sustained WMC PMTS into episodic buffer

        self.logger.debug(                                                     # Log the recalled memory context (WMC PMTs + recalled EMC episodes)
            f"MCC memory context recalled: "
            f"{len(wmc_pmts)} WMC PMTs + "
            f"{len(episodic_scaffold)} EMC episodes"
        )

        return episodic_buffer                                                 # Return the memory context staged in episodic buffer

    def assess_memory_schema(self) -> dict:
        """
        Assess the current stats of all memory cortex layers for logging and health checks.
    
        Returns:
            dict: Combined schema from WMC and EMC cortex layers
        """
        wmc_schema = self.wmc.assess_pmt_schema()            # Assess the PMT schema of WMC
        emc_schema = self.emc.get_stats()                    # Assess the engram complex of EMC
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
        stats = self.assess_memory_schema()                 # Assess the current stats of all memory cortex layers
        wmc_stats = stats["wmc"]                            # Retrieve current stats of working memory
        emc_stats = stats["emc"]                            # Retrieve current stats of episodic memory
        self.logger.info(                                   # Log the current stats of all memory cortex layers 
            f"🧠 Memory stats:\n"
            f"   WMC: {wmc_stats['pmt_count']} PMTs | "
            f"{wmc_stats['sustained_chunks']}/{wmc_stats['global_chunk_limit']} chunks ({wmc_stats['chunk_occupancy']}%)\n"
            f"   EMC: {emc_stats.get('episodes', 0)} episodes | "
            f"{emc_stats.get('buffer_pending', 0)} pending embed | "
            f"{emc_stats.get('db_size_mb', 0)} MB"
        )

    def forget_wm(self) -> None:
        """
        Forget working memory — called at conversation reset or session end.
        Does NOT clear EMC — episodic memory is permanent.
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
