"""
MCC — Memory Coordination Core
================================
AuRoRA · Semantic Cognitive System (SCS)

Single memory interface for CNC — coordinates WMC and EMC.
CNC never touches WMC or EMC directly, only calls MCC.

Responsibilities:
    - Route new turns to WMC
    - Forward WMC overflow → EMC buffer (async, non-blocking)
    - Build context window for Cosmos (WMC turns + EMC search results)
    - Token budget management across the full context

Context window budget (Cosmos Reason2 2B, max_model_len=2048):
    System prompt + GRACE personality  ~300 tokens  (reserved)
    EMC injected context               ~300 tokens  (reserved)
    WMC active turns                   ~1400 tokens (WMC budget)
    ─────────────────────────────────────────────────────────
    Total                              ~2000 tokens (safe under 2048)

Future milestones:
    M2 — add SMC, 11pm reflection trigger
    M3 — add PMC, procedural skill retrieval
"""

import asyncio
from datetime import datetime
from pathlib import Path
from typing import Optional

from scs.wmc import WMC
from scs.emc import EMC


# ── Token budget for context assembly ────────────────────────────────────────
SYSTEM_PROMPT_RESERVE = 300   # tokens reserved for system prompt + personality
EMC_CONTEXT_RESERVE   = 300   # tokens reserved for injected EMC episodes
EMC_TOP_K             = 3     # max episodes injected per turn
EMC_MIN_SIMILARITY    = 0.25  # minimum cosine sim to include an episode
# ─────────────────────────────────────────────────────────────────────────────

# ── DB path ───────────────────────────────────────────────────────────────────
DEFAULT_DB_PATH = str(Path.home() / ".aurora" / "emc.db")
# ─────────────────────────────────────────────────────────────────────────────


class MCC:
    """
    Memory Coordination Core.

    Instantiated once by CNC at startup.
    All memory operations go through MCC — CNC calls:

        await mcc.add_turn(role, content)
        context = await mcc.build_context(user_input)
        mcc.get_stats()
    """

    def __init__(self, logger, db_path: str = DEFAULT_DB_PATH):
        self.logger = logger

        # Ensure DB directory exists
        Path(db_path).parent.mkdir(parents=True, exist_ok=True)

        # Initialise memory layers
        self.wmc = WMC(logger=logger)
        self.emc = EMC(db_path=db_path, logger=logger)

        self.logger.info(
            f"✅ MCC initialised\n"
            f"   WMC budget : {self.wmc.token_budget} tokens\n"
            f"   EMC db     : {db_path}\n"
            f"   EMC top-k  : {EMC_TOP_K} episodes per turn\n"
            f"   Context    : {SYSTEM_PROMPT_RESERVE + EMC_CONTEXT_RESERVE + self.wmc.token_budget} tokens total"
        )

    # ── Core API ──────────────────────────────────────────────────────────────

    async def add_turn(self, role: str, content: str):
        """
        Add a new conversation turn to memory.

        1. Push turn to WMC
        2. Forward any evicted turns to EMC buffer (non-blocking)

        Args:
            role:    "user" or "assistant"
            content: Message text
        """
        # Push to WMC — returns evicted turns synchronously (fast, in-memory)
        evicted = self.wmc.add_turn(role, content)

        # Forward evicted turns to EMC buffer
        # Run in executor so it never blocks the asyncio event loop
        if evicted:
            loop = asyncio.get_event_loop()
            await loop.run_in_executor(
                None, self._forward_to_emc_buffer, evicted
            )
            self.logger.debug(
                f"MCC forwarded {len(evicted)} evicted turn(s) → EMC buffer"
            )

    def _forward_to_emc_buffer(self, turns: list[dict]):
        """
        Write evicted WMC turns to EMC buffer.
        Runs in thread pool — never blocks asyncio loop.
        """
        for turn in turns:
            self.emc.buffer_append(
                role    = turn["role"],
                content = turn["content"],
            )

    async def build_context(self, user_input: str) -> list[dict]:
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

    # ── Utility ───────────────────────────────────────────────────────────────

    def get_stats(self) -> dict:
        """Return combined memory stats for logging and health checks."""
        wmc_stats = self.wmc.token_usage()
        emc_stats = self.emc.get_stats()
        return {
            "wmc": wmc_stats,
            "emc": emc_stats,
        }

    def log_stats(self):
        """Log current memory stats at INFO level."""
        stats = self.get_stats()
        w = stats["wmc"]
        e = stats["emc"]
        self.logger.info(
            f"🧠 Memory stats:\n"
            f"   WMC: {w['turns']} turns | "
            f"{w['used']}/{w['budget']} tokens ({w['percent']}%)\n"
            f"   EMC: {e.get('episodes', 0)} episodes | "
            f"{e.get('buffer_pending', 0)} pending embed | "
            f"{e.get('db_size_mb', 0)} MB"
        )

    def clear_wmc(self):
        """
        Clear working memory — called at conversation reset.
        Does NOT clear EMC — episodic memory is permanent.
        """
        self.wmc.clear()
        self.logger.info("🧹 WMC cleared via MCC")

    def close(self):
        """Shutdown all memory layers cleanly."""
        self.emc.close()
        self.logger.info("🗄️  MCC closed")
