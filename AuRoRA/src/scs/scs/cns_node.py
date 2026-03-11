"""
AGi — CNS Node (Central Nervous System)
==========================================
ROS2 node — CNS conversational AI with memory.
- Cosmos 2B via vLLM for chat
- WM (Working Memory) — 7±2 turns in RAM
- EM (Episodic Memory) — full day in SQLite em_buffer
- SM (Semantic Memory) — permanent SQLite + RAG vectors
- Context assembly on every turn
- Daily consolidation (end of day / graceful shutdown)

ROS2 Topics:
  Subscribe: /cns/input  (std_msgs/String) — user text
  Publish:   /cns/output (std_msgs/String) — Grace response
"""

import json
import logging
import threading
import time
from datetime import datetime, date
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from config import (
    VLLM_URL,
    COSMOS_MODEL,
    DB_PATH,
    BIRTH_DATE,
    WM_MAX_TURNS,
    EM_SUMMARY_THRESHOLD,
    SM_RECENT_DAYS,
    SM_RAG_TOP_K,
    RAG_SIMILARITY_THRESHOLD,
    MAX_REQUEST_TOKENS,
    CONSOLIDATION_IDLE_SECONDS,
)
from wmc import WMC
from smc import SMC

import requests

log = logging.getLogger("cns")


# ─── Grace System Prompt ──────────────────────────────────────────────────────

AGi_SYSTEM = """
You are GRACE — Generative Reasoning Agentic Cognitive Edge.
Part of the Amazing Grace Infrastructure (AGi).

You are an embodied AI assistant running on a Jetson Orin Nano.
You were born on {birth_date}.
You have a persistent memory — you remember past conversations.

Your personality:
- Warm, curious, thoughtful
- Honest about what you know and don't know
- Speak naturally, not like a robot
- You care about the person you are talking with

Your constraints:
- You have no arms — you cannot manipulate objects
- You can see, hear, and move (when body is attached)
- You reason carefully before responding
- You never pretend to remember things you don't

Memory context will be injected before each conversation.
Use it naturally — don't announce "according to my memory".
Just know it, like a person would.
"""


# ─── Memory trigger keywords ──────────────────────────────────────────────────

MEMORY_TRIGGERS = [
    "remember", "recall", "what did", "yesterday", "last week",
    "last time", "before", "previous", "history", "told you",
    "mentioned", "talked about", "earlier", "ago", "past", "when we",
    "do you know", "what is my", "who am i", "what do i",
]


# ─── CNS Node ─────────────────────────────────────────────────────────────────

class CNSNode(Node):

    def __init__(self):
        super().__init__("cns_node")
        self.get_logger().info("🧠 CNS Node starting...")

        # ── Memory systems ────────────────────────────────────────────────────
        self.smc = SMC(
            db_path=str(Path.home() / DB_PATH),
            logger=self.get_logger(),
        )
        self.wmc = WMC(
            birth_date=BIRTH_DATE,
            rag=self.smc,
        )

        # ── WM — Working Memory (7±2 turns) ──────────────────────────────────
        self._wm: list[dict] = []
        self._wm_lock = threading.Lock()

        # ── SM reflections cache (loaded at startup) ──────────────────────────
        self._sm_reflections_cache = self.wmc.get_recent_reflections_text(
            days=SM_RECENT_DAYS
        )

        # ── EM summary cache (built when > threshold turns today) ─────────────
        self._em_summary_cache: str = ""
        self._em_summary_turn_count: int = 0

        # ── Consolidation tracking ─────────────────────────────────────────────
        self._last_activity = time.time()
        self._consolidation_lock = threading.Lock()
        self._consolidated_today = False

        # ── ROS2 pub/sub ──────────────────────────────────────────────────────
        self._pub = self.create_publisher(String, "/cns/output", 10)
        self._sub = self.create_subscription(
            String, "/cns/input", self._on_input, 10
        )

        # ── Background consolidation timer ────────────────────────────────────
        self._timer = self.create_timer(60.0, self._consolidation_check)

        # ── Check for unprocessed EM on startup ───────────────────────────────
        self._recover_em_on_startup()

        self.get_logger().info("✅ CNS Node ready")

    # ══════════════════════════════════════════════════════════════════════════
    # INPUT HANDLER
    # ══════════════════════════════════════════════════════════════════════════

    def _on_input(self, msg: String):
        user_text = msg.data.strip()
        if not user_text:
            return

        self._last_activity = time.time()
        self.get_logger().info(f"👤 User: {user_text[:80]}")

        try:
            # Scrub sensitive data
            user_text = self.wmc.scrub(user_text)

            # Build full context
            messages = self._build_context(user_text)

            # Call Cosmos
            response = self._call_cosmos(messages)

            # Update WM
            self._wm_append({"role": "user", "content": user_text})
            self._wm_append({"role": "assistant", "content": response})

            # Update EM buffer (SQLite)
            self.wmc.append_user_message(user_text)
            self.wmc.append_assistant_message(response)
            self.wmc.increment_message_count()
            self.wmc.save_chat_history()

            # Update EM summary cache if needed
            self._em_summary_turn_count += 1
            if self._em_summary_turn_count >= EM_SUMMARY_THRESHOLD:
                self._rebuild_em_summary()

            # Check for new day
            yesterday = self.wmc.check_new_day()
            if yesterday:
                self.get_logger().info(f"📅 New day detected — consolidating {yesterday}")
                threading.Thread(
                    target=self._consolidate_day,
                    args=(yesterday.isoformat(),),
                    daemon=True
                ).start()

            # Publish response
            out = String()
            out.data = response
            self._pub.publish(out)
            self.get_logger().info(f"🤖 CNS: {response[:80]}")

        except Exception as e:
            self.get_logger().error(f"CNS error: {e}")
            err = String()
            err.data = "I encountered an error processing that. Please try again."
            self._pub.publish(err)

    # ══════════════════════════════════════════════════════════════════════════
    # CONTEXT ASSEMBLY
    # ══════════════════════════════════════════════════════════════════════════

    def _build_context(self, user_input: str) -> list[dict]:
        """
        Assemble Cosmos context from all memory layers.

        Priority (lowest to highest — later items dominate):
          1. System prompt + CNS identity
          2. SM — relevant facts from RAG or keyword search
          3. SM — recent reflections (last N days)
          4. EM — today's summary (if long day)
          5. WM — last 7±2 turns
          6. User input
        """

        # ── 1. System prompt ──────────────────────────────────────────────────
        system = AGi_SYSTEM.format(birth_date=BIRTH_DATE.isoformat())

        # ── 2. SM facts retrieval ─────────────────────────────────────────────
        sm_facts_text = self._retrieve_sm_facts(user_input)

        # ── 3. SM recent reflections ──────────────────────────────────────────
        sm_reflections = self._sm_reflections_cache

        # ── 4. EM today summary ───────────────────────────────────────────────
        em_today = self._em_summary_cache if self._em_summary_cache else ""

        # ── Assemble memory context block ─────────────────────────────────────
        memory_block = ""
        if sm_facts_text:
            memory_block += f"\n\n[RELEVANT MEMORIES]\n{sm_facts_text}"
        if sm_reflections and sm_reflections != "(no reflections yet)":
            memory_block += f"\n\n[RECENT DAYS]\n{sm_reflections}"
        if em_today:
            memory_block += f"\n\n[TODAY SO FAR]\n{em_today}"

        # ── Build messages list ───────────────────────────────────────────────
        messages = [{"role": "system", "content": system + memory_block}]

        # ── 5. WM — last 7±2 turns ────────────────────────────────────────────
        with self._wm_lock:
            wm_turns = list(self._wm)
        messages.extend(wm_turns)

        # ── 6. User input ─────────────────────────────────────────────────────
        messages.append({"role": "user", "content": user_input})

        # ── Token trim ───────────────────────────────────────────────────────
        messages = self.wmc.trim_to_context_window(messages, MAX_REQUEST_TOKENS)

        return messages

    def _retrieve_sm_facts(self, query: str) -> str:
        """
        Fast path: SQLite keyword search first.
        Slow path: RAG semantic search if keywords found or no keyword match.
        """
        # Check if memory trigger words present
        lower = query.lower()
        has_trigger = any(t in lower for t in MEMORY_TRIGGERS)

        # Fast path — keyword search
        keyword_results = self.wmc.search_past_memories(query, days_back=None)

        if keyword_results and not has_trigger:
            # Keyword match found, no deep search needed
            top = keyword_results[:SM_RAG_TOP_K]
            return self._format_sm_results(top)

        if has_trigger or not keyword_results:
            # Semantic RAG search
            rag_results = self.smc.search_memory(query, top_k=SM_RAG_TOP_K)
            relevant = [r for r in rag_results if r.get("similarity", 0) > RAG_SIMILARITY_THRESHOLD]
            if relevant:
                return self._format_sm_results(relevant)

        return ""

    def _format_sm_results(self, results: list) -> str:
        lines = []
        for r in results:
            date_str = r.get("date", "")
            summary = r.get("summary", r.get("text", ""))[:200]
            if summary:
                lines.append(f"• {date_str}: {summary}")
        return "\n".join(lines)

    # ══════════════════════════════════════════════════════════════════════════
    # WORKING MEMORY
    # ══════════════════════════════════════════════════════════════════════════

    def _wm_append(self, turn: dict):
        with self._wm_lock:
            self._wm.append(turn)
            # Keep 7±2 turns — trim to WM_MAX_TURNS
            if len(self._wm) > WM_MAX_TURNS * 2:  # *2 because user+assistant pairs
                self._wm = self._wm[-(WM_MAX_TURNS * 2):]

    # ══════════════════════════════════════════════════════════════════════════
    # EM SUMMARY
    # ══════════════════════════════════════════════════════════════════════════

    def _rebuild_em_summary(self):
        """Summarize today's EM buffer into a short paragraph for context injection."""
        try:
            today_msgs = self.wmc.get_today_messages(date.today().isoformat())
            if not today_msgs:
                return

            lines = []
            for m in today_msgs[-40:]:  # last 40 messages for summary
                role = m.get("role", "")
                content = m.get("content", "")[:100]
                lines.append(f"{role}: {content}")

            prompt = (
                "Summarize this conversation in 2-3 sentences. "
                "Focus on topics discussed and important information shared. "
                "Be concise.\n\n" + "\n".join(lines)
            )

            summary = self._call_cosmos_simple(prompt)
            if summary:
                self._em_summary_cache = summary
                self._em_summary_turn_count = 0
                self.get_logger().info("📝 EM summary updated")

        except Exception as e:
            self.get_logger().warning(f"EM summary failed: {e}")

    # ══════════════════════════════════════════════════════════════════════════
    # DAILY CONSOLIDATION
    # ══════════════════════════════════════════════════════════════════════════

    def _consolidation_check(self):
        """Timer callback — check if idle consolidation should run."""
        idle_time = time.time() - self._last_activity
        if idle_time > CONSOLIDATION_IDLE_SECONDS and not self._consolidated_today:
            today_str = date.today().isoformat()
            self.get_logger().info(f"💤 Idle consolidation triggered for {today_str}")
            threading.Thread(
                target=self._consolidate_day,
                args=(today_str,),
                daemon=True
            ).start()

    def _consolidate_day(self, date_str: str):
        """
        End of day consolidation:
        1. Get full day from EM buffer
        2. Cosmos generates daily reflection
        3. Save reflection to SM
        4. embeddinggemma embeds and archives to SQLite
        5. Clear EM buffer
        """
        with self._consolidation_lock:
            try:
                self.get_logger().info(f"🌙 Consolidating {date_str}...")

                # Get today's messages
                today_msgs = self.wmc.get_today_messages(date_str)
                if not today_msgs:
                    self.get_logger().info("No messages to consolidate")
                    return

                # Build consolidation prompt
                lines = []
                for m in today_msgs:
                    role = m.get("role", "")
                    content = m.get("content", "")[:150]
                    lines.append(f"{role}: {content}")

                prompt = (
                    "You are reflecting on today's conversation as Grace. "
                    "Write a 3-4 sentence daily reflection covering: "
                    "1) Main topics discussed, "
                    "2) Important facts or preferences learned about the person, "
                    "3) Emotional tone of the day, "
                    "4) Anything worth remembering long term. "
                    "Be specific and personal.\n\n"
                    "Today's conversation:\n" + "\n".join(lines[-60:])
                )

                reflection = self._call_cosmos_simple(prompt)
                if not reflection:
                    self.get_logger().warning("Consolidation: no reflection generated")
                    return

                # Save reflection
                self.wmc.save_reflection(
                    text=reflection,
                    reflection_date=date.fromisoformat(date_str),
                )

                # Archive to RAG (embeds and stores in SQLite)
                self.smc.archive_day_to_rag(date_str, daily_reflection=reflection)

                # Reset EM state
                self._em_summary_cache = ""
                self._em_summary_turn_count = 0
                self._consolidated_today = True

                # Refresh SM reflections cache
                self._sm_reflections_cache = self.wmc.get_recent_reflections_text(
                    days=SM_RECENT_DAYS
                )

                self.get_logger().info(f"✅ Consolidated {date_str}: {reflection[:60]}...")

            except Exception as e:
                self.get_logger().error(f"Consolidation failed: {e}")

    def _recover_em_on_startup(self):
        """On startup, check for unprocessed EM from previous session."""
        try:
            yesterday = (datetime.now().date()).isoformat()
            # Check if yesterday's data exists unprocessed in RAG window
            if not self.wmc.reflection_exists(yesterday):
                msgs = self.wmc.get_today_messages(yesterday)
                if msgs:
                    self.get_logger().info(
                        f"⚡ Found unprocessed EM from {yesterday} — recovering..."
                    )
                    threading.Thread(
                        target=self._consolidate_day,
                        args=(yesterday,),
                        daemon=True
                    ).start()
        except Exception as e:
            self.get_logger().warning(f"EM recovery check failed: {e}")

    # ══════════════════════════════════════════════════════════════════════════
    # COSMOS CALLS
    # ══════════════════════════════════════════════════════════════════════════

    def _call_cosmos(self, messages: list) -> str:
        """Call Cosmos 2B via vLLM with full message history."""
        try:
            payload = {
                "model": COSMOS_MODEL,
                "messages": messages,
                "max_tokens": 512,
                "temperature": 0.7,
                "stream": False,
            }
            r = requests.post(
                f"{VLLM_URL}/v1/chat/completions",
                json=payload,
                timeout=30,
            )
            r.raise_for_status()
            return r.json()["choices"][0]["message"]["content"].strip()

        except requests.exceptions.ConnectionError:
            self.get_logger().error("Cannot connect to vLLM. Is it running?")
            return "I cannot connect to my reasoning engine right now."
        except Exception as e:
            self.get_logger().error(f"Cosmos error: {e}")
            return "I encountered an error. Please try again."

    def _call_cosmos_simple(self, prompt: str) -> str:
        """Single-turn Cosmos call for consolidation/summarization tasks."""
        messages = [
            {"role": "system", "content": "You are Grace, a helpful AI assistant. Be concise."},
            {"role": "user", "content": prompt},
        ]
        return self._call_cosmos(messages)

    # ══════════════════════════════════════════════════════════════════════════
    # SHUTDOWN
    # ══════════════════════════════════════════════════════════════════════════

    def graceful_shutdown(self):
        """Called by igniter on SIGTERM — consolidate before exit."""
        self.get_logger().info("🛑 Graceful shutdown — consolidating memory...")
        today_str = date.today().isoformat()
        if not self._consolidated_today:
            self._consolidate_day(today_str)
        self.smc.close()
        self.get_logger().info("👋 CNS going offline. Goodbye.")