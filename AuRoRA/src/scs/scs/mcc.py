"""
MCC — Memory Coordination Core
================================
AuRoRA · Semantic Cognitive System (SCS)

Single memory interface for CNC — coordinates WMC and EMC.
CNC never touches WMC or EMC directly, only calls MCC.

Responsibilities:
    - Route new turns to WMC
    - Forward WMC overflow → EMC buffer (async, non-blocking)
    - Build context window for Cosmos (WMC turns + EMC recalled engrams)
    - Manage cortical capacity across the full context

Architecture:
    MCC mirrors the human prefrontal cortex — the brain's active workspace
    where working memory and episodic recall share the same conscious space.

    WMC and EMC operate independently but converge in build_context() into
    a single unified context window sent to Cosmos — exactly as fresh thoughts
    and recalled memories both surface into the same prefrontal awareness.
    Cosmos cannot distinguish a recent WMC turn from a recalled EMC engram —
    they are all just active cognition.

    Cortical capacity budget:
        System prompt + robot identity  →  CNS_COGNITIVE_RESERVE
        EMC recalled engrams            →  EMC_RECALL_RESERVE
        WMC active turns                →  WMC_GLOBAL_CHUNK_LIMIT
        ─────────────────────────────────────────────────────────────────────
        Total                           →  CNS_CORTICAL_CAPACITY

Todo:
    M2 — dynamic EMC capacity adjustment — if recalled engrams exceed
         EMC_RECALL_RESERVE, trim to fit rather than silently overrunning
         WMC's chunk limit
    M2 — WMC/EMC health check
    M2 — add SMC, 11pm reflection trigger
    M3 — add PMC, procedural skill retrieval
"""

# System libraries
import asyncio                          # For concurrent WMC retrieval and EMC search
from pathlib import Path                # For handling gateway to the engrams

# AGi libraries
from scs.wmc import WorkingMemoryCortex
from scs.emc import EpisodicMemoryCortex
from hrs.hrp import AGi
CNS = AGi.CNS                            # Alias CNS parameter class for concise access

class MemoryCoordinationCore:
    """
    Memory Coordination Core — the thalamus of the CNS.

    Central relay between CNC and the memory organs (WMC, EMC).
    Instantiated once by CNC at startup. All memory operations go
    through MCC — CNC never touches WMC or EMC directly.

    Access:
        MCC reads AGi.CNS.* constants from hrp.py directly.
        MCC accesses WMC and EMC constants through their instances only —
        never via hrp.py directly.

    CNC interface:
        await mcc.add_turn(role, content)
        context = await mcc.build_context(user_input)
        mcc.log_stats()
        mcc.close()
    """

    def __init__(self, logger):
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

    async def relay_pmt(self, role: str, content: str):
        """
        Relay new PMT to WMC and bind any evicted PMTs to EMC buffer.

        1. Fill induced PMT to WMC
        2. Bind any evicted PMTs to EMC buffer (non-blocking)

        Args:
            role:    "user" or "assistant"
            content: Message text
        """
        # Fill induced PMT to WMC — returns evicted PMTs synchronously (fast, in-memory)
        evicted_pmts = self.wmc.fill_pmt(role, content)                     # Fill the induced PMT into WMC, collect any evicted PMTs
            
        # Bind evicted PMTs to EMC buffer
        # Run and forget — never blocks active cognition
        if evicted_pmts:                                                    # If WMC evicted any PMTs, bind them to EMC buffer
            loop = asyncio.get_running_loop()                               # Access the main neural pathway
            loop.run_in_executor(                                           # Recruit a dormant neural thead — run binding on isolated neural pathway
                None, self._bind_to_emc_buffer, evicted_pmts
            )
            self.logger.debug(                                              # Log the binding of evicted PMTs to EMC buffer
                f"MCC bound {len(evicted_pmts)} evicted PMT(s) → EMC buffer"
            )

    def _bind_to_emc_buffer(self, evicted_pmts: list[dict]) -> None:
        """
        Bind evicted PMTs from WMC into EMC buffer for pending encoding and consolidation.
        Runs in isolated neural pathway — never blocks active cognition.
    
        Args:
            evicted_pmts: List of evicted PMTs [{role, content, timestamp}]
    
        Returns:
            None
        """
        try:                                                # Attempt binding evicted PMTs to EMC buffer
            for evicted_pmt in evicted_pmts:                # Process each evicted PMT
                self.emc.buffer_append(                     # Bind each evicted PMT into EMC buffer
                    role    = evicted_pmt["role"],
                    content = evicted_pmt["content"],
                    timestamp = evicted_pmt["timestamp"],
                )
        except Exception as e:                             # If binding lapse occurs, log and continue
            self.logger.error(                             # Log the binding lapse
                f"EMC binding lapse — {len(evicted_pmts)} PMT(s) unbound: {e}",
                exc_info=True
            )
            
    async def retrieve_full_memory(self, user_input: str) -> list[dict]:
        """
        Build the full context window to send to Cosmos.

        Structure:
            [WMC turns (chronological)]
            + [EMC episodes injected as system context (if relevant)]

        EMC search runs concurrently with WMC retrieval — no added latency.

        Args:
            user_input: Current user message (used as EMC search query)

        Returns:
            List of message dicts [{role, content}] ready for Cosmos
        """
        # Run WMC retrieval and EMC search concurrently
        loop = asyncio.get_event_loop()

        wmc_turns_future = loop.run_in_executor(None, self.wmc.get_turns)
        emc_results_future = loop.run_in_executor(
            None, self.emc.search, user_input, EMC_TOP_K
        )

        wmc_turns, emc_results = await asyncio.gather(
            wmc_turns_future, emc_results_future
        )

        # Filter EMC results by minimum similarity threshold
        relevant_episodes = [
            ep for ep in emc_results
            if ep["similarity"] >= EMC_MIN_SIMILARITY
        ]

        # Build context list
        context: list[dict] = []

        # Inject relevant EMC episodes as a system message
        if relevant_episodes:
            episode_lines = ["Relevant memories from past conversations:"]
            for ep in relevant_episodes:
                date_str = ep.get("date", "unknown date")
                role_str = ep.get("role", "unknown")
                content  = ep.get("content", "")
                sim      = ep.get("similarity", 0.0)
                episode_lines.append(
                    f"[{date_str}] {role_str}: {content} (relevance: {sim:.2f})"
                )

            context.append({
                "role":    "system",
                "content": "\n".join(episode_lines),
            })

            self.logger.debug(
                f"MCC injected {len(relevant_episodes)} EMC episode(s) into context"
            )

        # Append active WMC turns (chronological)
        context.extend(wmc_turns)

        self.logger.debug(
            f"MCC context built: "
            f"{len(wmc_turns)} WMC turns + "
            f"{len(relevant_episodes)} EMC episodes"
        )

        return context

    def assess_memory_schema(self) -> dict:
        """
        Assess the current stats of all memory cortex layers for logging and health checks.
    
        Returns:
            dict: Combined schema from WMC and EMC cortex layers
        """
        wmc_schema = self.wmc.assess_pmt_schema()            # Assess the PMT schema of WMC
        emc_schema = self.emc.get_stats()                    # Assess the engram complex of EMC
        return {                                             # return the current stats of all memory cortext layers
            "wmc": wmc_schema,
            "emc": emc_schema,
        }

    def report_memory_stats(self):
        """
        Report current memory cortex stats.
        Called by CNC after every turn for health monitoring.
    
        Todo:
            GUI — expose via ROS2 topic for real-time memory visualisation
        """
        stats = self.assess_memory_schema()                # Assess the current stats of all memory cortex layers
        wmc = stats["wmc"]                                 # Retrieve current stats of working memory
        emc = stats["emc"]                                 # Retrieve current stats of episodic memory
        self.logger.info(                                  # Log the current stats of all memory cortex layers 
            f"🧠 Memory stats:\n"
            f"   WMC: {wmc['pmt_count']} PMTs | "
            f"{wmc['sustained_chunks']}/{wmc['global_chunk_limit']} chunks ({wmc['chunk_occupancy']}%)\n"
            f"   EMC: {emc.get('episodes', 0)} episodes | "
            f"{emc.get('buffer_pending', 0)} pending embed | "
            f"{emc.get('db_size_mb', 0)} MB"
        )

    def forget_wm(self):
        """
        Forget working memory — called at conversation reset or session end.
        Does NOT clear EMC — episodic memory is permanent.
        """
        self.wmc.forget_pmt_schema()                                # Forget all sustaining PMT schema in working memory
        self.logger.info("🧹 Working memory forgotten")             # Log entry on successful working memory forgetting

    def close(self):
        """
        Gracefully close MCC and its memory cortex layers.
        WMC has no persistent resources to close,
        but EMC may have open file handles to the engram gateway that need to be released.
        """
        self.emc.close()                                            # Close the engram gateway
        self.logger.info("🗄️  MCC shutdown sequence complete")      # Log entry on successful MCC closure
