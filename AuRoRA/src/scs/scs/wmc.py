"""
AGi — WMC (Working Memory Cortex)
=================================
Manages active working memory for the CNS node.
- WM: Working memory (7±2 turns) via RAG window
- EM: Episodic buffer (today's full log) via SQLite em_buffer
- Daily reflections
- Token management / context trimming
- Sensitive data scrubbing
"""

import json
import logging
import re
import threading
from datetime import datetime, date
from pathlib import Path
from typing import Optional

from config import (
    CHAT_HISTORY_FILE,
    REFLECTIONS_FILE,
    COUNT_FILE,
    MAX_HISTORY_STORAGE,
    MAX_MESSAGES_PER_DAY,
    MAX_RECENT_MESSAGES,
    MAX_REFLECTIONS_LOAD,
    MAX_REQUEST_TOKENS,
)

log = logging.getLogger("cns.wmc")


class WMC:
    """
    Manages active working memory for the CNS node.
    Instantiate once and pass to CNSNode.
    """

    def __init__(self, birth_date: date, rag=None):
        self.birth_date = birth_date
        self.rag = rag  # Optional SQLiteVectorRAG instance

        # File paths
        self.chat_history_file = Path.home() / CHAT_HISTORY_FILE
        self.reflections_file  = Path.home() / REFLECTIONS_FILE

        # In-memory state
        self.chat_history   = self._load_chat_history()
        self.reflections    = self._load_reflections()

        # Today tracking
        self.today_start        = datetime.now().date()
        self.today_message_count = 0
        self._msg_count_lock    = threading.Lock()
        self._day_check_lock    = threading.Lock()

        # Restore today's count across restarts
        count_file = Path.home() / COUNT_FILE
        if count_file.exists():
            try:
                saved = json.loads(count_file.read_text())
                if saved.get("date") == date.today().isoformat():
                    self.today_message_count = saved.get("count", 0)
                    log.info(f"✅ Restored today's count: {self.today_message_count}")
            except Exception:
                pass

        log.info(f"Memory: {len(self.chat_history)} msgs, {len(self.reflections)} reflections")

    # ── Chat history ──────────────────────────────────────────────────────────

    def _load_chat_history(self) -> list:
        if self.chat_history_file.exists():
            try:
                data = json.loads(self.chat_history_file.read_text())
                return data.get("messages", [])
            except Exception as e:
                log.warning(f"Could not load chat history: {e}")
        return []

    def save_chat_history(self):
        """Save chat history; archive overflow to RAG."""
        if len(self.chat_history) > MAX_HISTORY_STORAGE:
            old = self.chat_history[:-MAX_HISTORY_STORAGE]
            if self.rag:
                self._archive_old_messages_to_rag(old)
            self.chat_history = self.chat_history[-MAX_HISTORY_STORAGE:]
        try:
            self.chat_history_file.write_text(
                json.dumps({"messages": self.chat_history}, indent=2)
            )
        except Exception as e:
            log.error(f"save_chat_history failed: {e}")

    def _archive_old_messages_to_rag(self, messages: list):
        if not self.rag:
            return
        try:
            date_str = date.today().isoformat()
            for msg in messages:
                self.rag.add_message_to_window(date_str, msg)
            log.info(f"Archived {len(messages)} messages to RAG")
        except Exception as e:
            log.warning(f"RAG archive failed: {e}")

    def append_user_message(self, content: str, has_image: bool = False):
        entry = {"role": "user", "content": content, "timestamp": datetime.now().isoformat()}
        if has_image:
            entry["has_image"] = True
        self.chat_history.append(entry)
        if self.rag:
            self.rag.add_message_to_window(date.today().isoformat(), entry)

    def append_assistant_message(self, content: str):
        entry = {"role": "assistant", "content": content, "timestamp": datetime.now().isoformat()}
        self.chat_history.append(entry)
        if self.rag:
            self.rag.add_message_to_window(date.today().isoformat(), entry)

    def get_recent_messages(self) -> list:
        """Return 7-day window via RAG, or fall back to tail of chat_history."""
        if self.rag:
            return self.rag.get_messages_in_window()
        return self.chat_history[-MAX_RECENT_MESSAGES:]

    # ── Token management ──────────────────────────────────────────────────────

    def estimate_tokens(self, messages: list) -> int:
        return sum(len(m.get("content", "")) // 4 for m in messages)

    def trim_to_context_window(self, messages: list, max_tokens: int = MAX_REQUEST_TOKENS) -> list:
        """Keep system messages; trim oldest conversation to fit token budget."""
        system_msgs = [m for m in messages if m.get("role") == "system"]
        convo       = [m for m in messages if m.get("role") != "system"]

        result = system_msgs.copy()
        used   = self.estimate_tokens(result)

        for msg in reversed(convo):
            cost = self.estimate_tokens([msg])
            if used + cost < max_tokens:
                result.append(msg)
                used += cost
            else:
                log.debug(f"Context trim: dropped old messages to fit {max_tokens} tokens")
                break

        # Restore chronological order
        tail = [m for m in result if m.get("role") != "system"]
        tail.reverse()
        return system_msgs + tail

    # ── Day / message count ───────────────────────────────────────────────────

    def increment_message_count(self):
        with self._msg_count_lock:
            self.today_message_count += 1
            count = self.today_message_count
        try:
            (Path.home() / COUNT_FILE).write_text(
                json.dumps({"date": date.today().isoformat(), "count": count})
            )
        except Exception:
            pass

    def check_new_day(self) -> Optional[date]:
        """
        Thread-safe new-day detection.
        Returns yesterday's date if a day boundary was crossed, else None.
        """
        with self._day_check_lock:
            today = datetime.now().date()
            if today != self.today_start:
                yesterday = self.today_start
                self.today_start = today
                with self._msg_count_lock:
                    self.today_message_count = 0
                return yesterday
        return None

    # ── Reflections ───────────────────────────────────────────────────────────

    def _load_reflections(self) -> list:
        if self.reflections_file.exists():
            try:
                data = json.loads(self.reflections_file.read_text())
                return data.get("reflections", [])
            except Exception as e:
                log.warning(f"Could not load reflections: {e}")
        return []

    def _save_reflections(self):
        if len(self.reflections) > 1000:
            self.reflections = self.reflections[-1000:]
            log.info("Trimmed old reflections (keeping last 1000)")
        data = {
            "birth_date":        self.birth_date.isoformat(),
            "total_reflections": len(self.reflections),
            "reflections":       self.reflections,
        }
        self.reflections_file.write_text(json.dumps(data, indent=2))

    def save_reflection(self, text: str, reflection_date: date = None, ascii_art: str = ""):
        reflection_date = reflection_date or date.today()
        age_days = (reflection_date - self.birth_date).days
        entry = {
            "day":           age_days,
            "date":          reflection_date.isoformat(),
            "reflection":    text,
            "ascii_art":     ascii_art,
            "message_count": self.today_message_count,
        }
        self.reflections.append(entry)
        self._save_reflections()
        log.info(f"💭 Reflection saved (day {age_days}): {text[:60]}...")

    def reflection_exists(self, date_str: str) -> bool:
        return any(r.get("date") == date_str for r in self.reflections)

    def get_recent_reflections_text(self, days: int) -> str:
        recent = self.reflections[-days:] if self.reflections else []
        if not recent:
            return "(no reflections yet)"
        return "\n\n".join(
            f"Day {r['day']} ({r['date']}) [{r.get('message_count', 0)} msgs]: {r['reflection']}"
            for r in recent
        )

    def should_load_reflections(self, message: str) -> bool:
        triggers = [
            "remember", "recall", "what did", "yesterday", "last week",
            "last time", "before", "previous", "history", "told you",
            "mentioned", "talked about", "earlier", "ago", "past", "when we",
        ]
        lower = message.lower()
        return any(t in lower for t in triggers)

    def build_reflection_summary(self, query: str = "") -> str:
        if not self.reflections:
            return ""
        recent = self.reflections[-MAX_REFLECTIONS_LOAD:]
        lines  = [f"Recent memory ({len(recent)} days):\n"]
        for r in recent:
            lines.append(f"D{r['day']}: {r['reflection'][:80]}")
        if self.rag and query:
            old = self.search_past_memories(query, days_back=90)
            if old:
                lines.append("\nRelevant older memories:")
                for m in old[:3]:
                    lines.append(f"• {m['date']}: {m['summary'][:60]}...")
        return "\n".join(lines)

    def search_past_memories(self, query: str, days_back: int = None) -> list:
        if not self.rag:
            return []
        return self.rag.search_memory(query, top_k=5, days_back=days_back)

    def get_today_messages(self, date_str: str) -> list:
        """Merge RAG + JSON history for a full day's messages."""
        rag_msgs = []
        if self.rag:
            rag_msgs = self.rag.get_full_day_messages(date_str) or []

        json_today = [
            m for m in self.chat_history
            if m.get("timestamp", "").startswith(date_str)
        ]
        if not json_today:
            json_today = self.chat_history[-MAX_MESSAGES_PER_DAY:]

        if rag_msgs:
            seen = {m.get("content", ""): True for m in rag_msgs}
            for m in json_today:
                if m.get("content", "") not in seen:
                    rag_msgs.append(m)
                    seen[m.get("content", "")] = True
            return rag_msgs
        return json_today if json_today else self.get_recent_messages()

    # ── Sensitive data scrubbing ──────────────────────────────────────────────

    _SCRUB_PATTERNS = [
        (r'(?i)(api[_\s-]?key|secret|token|bearer)[:\s=]+[A-Za-z0-9\-_\.]{16,}', '[API_KEY_REDACTED]'),
        (r'(?i)(password|passwd|pwd)[:\s=]+\S+',                                   '[PASSWORD_REDACTED]'),
        (r'\b(?:\d[ -]?){13,19}\b',                                                '[CARD_NUMBER_REDACTED]'),
        (r'(?i)(visa|passport|ssn|sin)[:\s#]+[A-Z0-9\-]{6,20}',                   '[ID_NUMBER_REDACTED]'),
        (r'\b\d{3}[-\s]?\d{2}[-\s]?\d{4}\b',                                      '[SSN_REDACTED]'),
        (r'(?i)(private[_\s]key|secret[_\s]key)[:\s=]+[A-Za-z0-9+/=]{20,}',       '[PRIVATE_KEY_REDACTED]'),
        (r'xox[bpaso]-[A-Za-z0-9\-]{10,}',                                         '[SLACK_TOKEN_REDACTED]'),
        (r'\b\d{8,12}:[A-Za-z0-9_\-]{30,}\b',                                      '[TELEGRAM_TOKEN_REDACTED]'),
        (r'gh[pousr]_[A-Za-z0-9]{36,}',                                            '[GITHUB_TOKEN_REDACTED]'),
        (r'(?i)(key|secret|token|auth)[=:]\s*["\']?[A-Za-z0-9+/\-_\.]{20,}["\']?','[SECRET_REDACTED]'),
    ]

    def scrub(self, text: str) -> str:
        if not text:
            return text
        count = 0
        for pattern, replacement in self._SCRUB_PATTERNS:
            text, n = re.subn(pattern, replacement, text)
            count += n
        if count:
            log.warning(f"🔒 Scrubbed {count} sensitive pattern(s)")
        return text

    def scrub_messages(self, messages: list) -> list:
        return [
            {**m, "content": self.scrub(m["content"])}
            if isinstance(m.get("content"), str) else m
            for m in messages
        ]

    # ── KPI helpers (used by reports.py) ─────────────────────────────────────

    def gather_kpis(self, days: int, rlhf=None) -> dict:
        recent       = self.reflections[-days:] if self.reflections else []
        total_msgs   = sum(r.get("message_count", 0) for r in recent)
        active_days  = sum(1 for r in recent if r.get("message_count", 0) > 0)
        offline_days = len(recent) - active_days
        avg          = round(total_msgs / max(active_days, 1), 1)

        rlhf_pos = rlhf_neg = rlhf_total = 0
        if rlhf:
            s          = rlhf.get_stats()
            rlhf_total = s.get("total_feedback", 0)
            rlhf_pos   = s.get("positive_count", 0)
            rlhf_neg   = s.get("negative_count", 0)
        pos_pct = round(rlhf_pos / max(rlhf_total, 1) * 100, 1)

        return {
            "period_days":               days,
            "total_conversations":       total_msgs,
            "active_days":               active_days,
            "offline_days":              offline_days,
            "avg_conversations_per_day": avg,
            "rlhf_total_feedback":       rlhf_total,
            "rlhf_positive_pct":         pos_pct,
            "rlhf_positive":             rlhf_pos,
            "rlhf_negative":             rlhf_neg,
        }