"""
AGi — SMC (Semantic Memory Cortex)
================================
Semantic Memory Cortex — permanent vector memory for the CNS.
- SQLite vector storage via embeddinggemma-300m on CPU
- 7-day rolling window (in-memory)
- Daily archival with semantic embeddings
- RAG semantic search (cosine similarity)
- Weekly journals and periodic reports
- em_buffer table for crash-safe episodic accumulation

Install:
    pip3 install sentence-transformers --break-system-packages
"""

import sqlite3
import json
import math
import time
from datetime import datetime, timedelta
from pathlib import Path


# ─── Embedding model config ───────────────────────────────────────────────────
EMBEDDING_MODEL = "google/embeddinggemma-300m-qat-q4_0-unquantized"
EMBEDDING_DIM   = 768          # all embeddinggemma-300m variants output 768 dims
# NOTE: activations do NOT support float16 — always runs in float32
# NOTE: QAT q4_0 version uses sub-200MB RAM — ideal for Jetson CPU
# ─────────────────────────────────────────────────────────────────────────────


class SentenceTransformerEmbedder:
    """
    Wrapper around sentence-transformers using google/embeddinggemma-300m.
    Runs on CPU — zero GPU memory impact, no Ollama dependency.
    """

    def __init__(self, logger):
        self.logger     = logger
        self._model     = None
        self._available = None
        self._cache: dict[str, list[float]] = {}

    # ------------------------------------------------------------------
    def _load_model(self):
        if self._model is not None:
            return True
        try:
            from sentence_transformers import SentenceTransformer
            self.logger.info(f"⏳ Loading {EMBEDDING_MODEL} (CPU)…")
            self._model = SentenceTransformer(EMBEDDING_MODEL)
            self.logger.info(f"✅ {EMBEDDING_MODEL} loaded on CPU")
            self._available = True
            return True
        except ImportError:
            self.logger.warn(
                "⚠️  sentence-transformers not installed.\n"
                "   Run: pip3 install sentence-transformers --break-system-packages"
            )
            self._available = False
            return False
        except Exception as exc:
            self.logger.warn(f"⚠️  Failed to load {EMBEDDING_MODEL}: {exc}")
            self._available = False
            return False

    # ------------------------------------------------------------------
    def is_available(self) -> bool:
        if self._available is not None:
            return self._available
        return self._load_model()

    # ------------------------------------------------------------------
    def encode(self, text: str, is_query: bool = False) -> list[float]:
        """
        Return embedding vector. Returns [] on failure so callers degrade
        falls back to keyword search on failure.

        Uses encode_query() for search queries and encode_document() for
        documents/memories — embeddinggemma is optimized for this distinction.
        """
        if not self._load_model():
            return []

        cache_key = f"{'q' if is_query else 'd'}:{text[:300]}"
        if cache_key in self._cache:
            return self._cache[cache_key]

        try:
            if is_query:
                vec = self._model.encode_query(text).tolist()
            else:
                vec = self._model.encode_document(text).tolist()
            self._cache[cache_key] = vec
            return vec
        except Exception as exc:
            self.logger.debug(f"Embed failed: {exc}")
            return []


# ─── Pure-Python cosine similarity ───────────────────────────────────────────

def _cosine(a: list[float], b: list[float]) -> float:
    if not a or not b or len(a) != len(b):
        return 0.0
    dot = sum(x * y for x, y in zip(a, b))
    na  = math.sqrt(sum(x * x for x in a))
    nb  = math.sqrt(sum(x * x for x in b))
    return dot / (na * nb) if na and nb else 0.0


# ─────────────────────────────────────────────────────────────────────────────

class SMC:
    """
    Semantic Memory Cortex — permanent vector memory.
    Powered by google/embeddinggemma-300m via sentence-transformers on CPU.
    """

    def __init__(self, db_path: str, logger):
        self.logger  = logger
        self.db_path = db_path

        # Embedder (CPU, no Ollama needed)
        self.embedder = SentenceTransformerEmbedder(logger)

        # SQLite
        self.conn = sqlite3.connect(db_path, check_same_thread=False)
        self.conn.row_factory = sqlite3.Row
        self._init_tables()

        # 7-day rolling window (in-memory)
        self.daily_messages:    dict[str, list] = {}
        self.weekly_journals:   list = []
        self.monthly_summaries: list = []

        self.logger.info("✅ SMC (embeddinggemma-300m / CPU) initialized")

    # ══════════════════════════════════════════════════════════════════════
    # SCHEMA
    # ══════════════════════════════════════════════════════════════════════

    def _init_tables(self):
        self.conn.executescript("""
            CREATE TABLE IF NOT EXISTS em_buffer (
                id        INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT    NOT NULL,
                role      TEXT    NOT NULL,
                content   TEXT    NOT NULL,
                date      TEXT    NOT NULL,
                processed BOOLEAN DEFAULT FALSE
            );

            CREATE INDEX IF NOT EXISTS idx_em_buffer_date
                ON em_buffer(date);

            CREATE TABLE IF NOT EXISTS daily_archives (
                id            INTEGER PRIMARY KEY AUTOINCREMENT,
                date          TEXT    NOT NULL,
                message_count INTEGER,
                full_text     TEXT,
                summary       TEXT,
                created_at    TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                UNIQUE(date)
            );

            CREATE TABLE IF NOT EXISTS memory_vectors (
                archive_id INTEGER PRIMARY KEY,
                embedding  TEXT,   -- JSON array of floats
                FOREIGN KEY (archive_id) REFERENCES daily_archives(id)
            );

            CREATE INDEX IF NOT EXISTS idx_daily_archives_date
                ON daily_archives(date);

            CREATE TABLE IF NOT EXISTS weekly_journals (
                id                INTEGER PRIMARY KEY AUTOINCREMENT,
                week_start        TEXT,
                week_end          TEXT,
                summary           TEXT,
                daily_reflections TEXT,
                created_at        TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            );

            CREATE TABLE IF NOT EXISTS periodic_summaries (
                id           INTEGER PRIMARY KEY AUTOINCREMENT,
                period_type  TEXT,
                period_start TEXT,
                period_end   TEXT,
                summary      TEXT,
                statistics   TEXT,
                created_at   TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            );
        """)
        self.conn.commit()

    # ══════════════════════════════════════════════════════════════════════
    # 7-DAY ROLLING WINDOW
    # ══════════════════════════════════════════════════════════════════════

    def add_message_to_window(self, date_str: str, message: dict):
        self.daily_messages.setdefault(date_str, []).append(message)

    def get_messages_in_window(self) -> list:
        dates = sorted(self.daily_messages)[-7:]
        msgs  = []
        for d in dates:
            msgs.extend(self.daily_messages[d])
        return msgs

    def get_full_day_messages(self, date_str: str) -> list:
        return self.daily_messages.get(date_str, [])

    def slide_window(self) -> str | None:
        dates = sorted(self.daily_messages)
        if len(dates) > 7:
            oldest = dates[0]
            self.logger.info(f"🗂️  Sliding window: archiving {oldest}")
            return oldest
        return None

    # ══════════════════════════════════════════════════════════════════════
    # ARCHIVE A DAY TO VECTOR DB
    # ══════════════════════════════════════════════════════════════════════

    def archive_day_to_rag(self, date_str: str, daily_reflection: str = ""):
        messages = self.daily_messages.get(date_str, [])
        if not messages:
            self.logger.info(f"No messages to archive for {date_str}")
            return

        try:
            lines = []
            for msg in messages:
                role    = msg.get("role", "unknown")
                content = msg.get("content", "")
                if msg.get("has_image"):
                    lines.append(f"{role}: [image message]")
                else:
                    lines.append(f"{role}: {content}")
            full_text = "\n".join(lines)

            self.logger.info(f"🔢 Embedding {date_str} ({len(messages)} msgs)…")
            vec = self.embedder.encode(full_text[:4000], is_query=False)
            embedding_json = json.dumps(vec) if vec else "[]"

            cur = self.conn.execute(
                """INSERT OR REPLACE INTO daily_archives
                       (date, message_count, full_text, summary)
                   VALUES (?, ?, ?, ?)""",
                [date_str, len(messages), full_text, daily_reflection],
            )
            archive_id = cur.lastrowid

            self.conn.execute(
                """INSERT OR REPLACE INTO memory_vectors (archive_id, embedding)
                   VALUES (?, ?)""",
                [archive_id, embedding_json],
            )
            self.conn.commit()

            self.logger.info(f"✅ Archived {date_str} → SQLite vector DB")

            # Remove from active window
            self.daily_messages.pop(date_str, None)

        except Exception as exc:
            self.logger.error(f"❌ Archive failed for {date_str}: {exc}")
            self.conn.rollback()

    # ══════════════════════════════════════════════════════════════════════
    # VECTOR SEARCH (pure-Python cosine)
    # ══════════════════════════════════════════════════════════════════════

    def search_memory(self, query: str, top_k: int = 5, days_back: int = None) -> list:
        """
        Semantic search over archived days.
        Returns list of dicts: {date, text, summary, message_count, similarity}
        """
        try:
            query_vec = self.embedder.encode(query, is_query=True)
            if not query_vec:
                return self._fallback_keyword_search(query, top_k, days_back)

            sql    = """SELECT da.date, da.full_text, da.summary,
                               da.message_count, mv.embedding
                        FROM memory_vectors mv
                        JOIN daily_archives da ON mv.archive_id = da.id"""
            params = []
            if days_back:
                sql += " WHERE da.date >= date('now', ?)"
                params.append(f"-{days_back} days")

            rows = self.conn.execute(sql, params).fetchall()
            if not rows:
                return []

            scored = []
            for row in rows:
                stored_vec = json.loads(row["embedding"] or "[]")
                sim = _cosine(query_vec, stored_vec)
                scored.append({
                    "date":          row["date"],
                    "text":          (row["full_text"] or "")[:500] + "…",
                    "summary":       row["summary"] or "",
                    "message_count": row["message_count"],
                    "similarity":    round(sim, 3),
                })

            scored.sort(key=lambda x: x["similarity"], reverse=True)
            top = scored[:top_k]
            self.logger.info(f"🔍 Memory search → {len(top)} results")
            return top

        except Exception as exc:
            self.logger.error(f"❌ Memory search failed: {exc}")
            return []

    def _fallback_keyword_search(self, query: str, top_k: int, days_back: int | None) -> list:
        """Simple LIKE-based fallback when embedder is unavailable."""
        words = [w for w in query.lower().split() if len(w) > 3]
        if not words:
            return []
        where_clause = " OR ".join(["LOWER(full_text) LIKE ?" for _ in words])
        params       = [f"%{w}%" for w in words]
        if days_back:
            where_clause += " AND date >= date('now', ?)"
            params.append(f"-{days_back} days")
        sql = f"""SELECT date, full_text, summary, message_count
                  FROM daily_archives WHERE {where_clause}
                  ORDER BY date DESC LIMIT {top_k}"""
        rows = self.conn.execute(sql, params).fetchall()
        return [
            {
                "date":          r["date"],
                "text":          (r["full_text"] or "")[:500] + "…",
                "summary":       r["summary"] or "",
                "message_count": r["message_count"],
                "similarity":    0.0,
            }
            for r in rows
        ]

    # ══════════════════════════════════════════════════════════════════════
    # WEEKLY JOURNALS
    # ══════════════════════════════════════════════════════════════════════

    def create_weekly_journal(self, start_date: str, end_date: str,
                              daily_reflections: list) -> str | None:
        if len(daily_reflections) < 7:
            return None
        lines = [f"Week of {start_date} to {end_date}\n"]
        for r in daily_reflections:
            lines.append(
                f"• Day {r.get('day')} ({r.get('date')}): "
                f"{r.get('message_count', 0)} msgs — {r.get('reflection', '')}"
            )
        summary = "\n".join(lines)
        self.conn.execute(
            """INSERT INTO weekly_journals
                   (week_start, week_end, summary, daily_reflections)
               VALUES (?, ?, ?, ?)""",
            [start_date, end_date, summary, json.dumps(daily_reflections)],
        )
        self.conn.commit()
        self.logger.info(f"📖 Weekly journal: {start_date} → {end_date}")
        return summary

    # ══════════════════════════════════════════════════════════════════════
    # PERIODIC REPORTS
    # ══════════════════════════════════════════════════════════════════════

    def generate_periodic_report(self, period_type: str, llm_generate_func) -> str | None:
        today = datetime.now().date()
        days_map = {"monthly": 30, "quarterly": 90, "yearly": 365}
        if period_type not in days_map:
            raise ValueError(f"Unknown period_type: {period_type}")

        start_date = (today - timedelta(days=days_map[period_type])).isoformat()
        end_date   = today.isoformat()

        archives = self.conn.execute(
            """SELECT date, summary, message_count
               FROM daily_archives
               WHERE date >= ? AND date <= ?
               ORDER BY date""",
            [start_date, end_date],
        ).fetchall()

        if not archives:
            self.logger.warn(f"No archives for {period_type} report")
            return None

        total_msgs = sum(r["message_count"] or 0 for r in archives)
        daily_sum  = [
            f"{r['date']}: {r['message_count']} msgs — {r['summary']}"
            for r in archives
        ]
        context = (
            f"{period_type.upper()} REPORT\n"
            f"Period: {start_date} to {end_date}\n"
            f"Total days: {len(archives)} | Total messages: {total_msgs}\n\n"
            f"Daily summaries:\n" + "\n".join(daily_sum) +
            f"\n\nGenerate a {period_type} reflection covering major themes, "
            f"emotional arc, growth, and outlook for the next {period_type}."
        )

        self.logger.info(f"📊 Generating {period_type} report…")
        summary = llm_generate_func(context)

        stats = json.dumps({
            "total_days":           len(archives),
            "total_messages":       total_msgs,
            "avg_messages_per_day": round(total_msgs / len(archives), 1),
        })
        self.conn.execute(
            """INSERT INTO periodic_summaries
                   (period_type, period_start, period_end, summary, statistics)
               VALUES (?, ?, ?, ?, ?)""",
            [period_type, start_date, end_date, summary, stats],
        )
        self.conn.commit()
        self.logger.info(f"✅ {period_type.capitalize()} report saved")
        return summary

    # ══════════════════════════════════════════════════════════════════════
    # STATS
    # ══════════════════════════════════════════════════════════════════════

    def get_statistics(self) -> dict:
        stats = {
            "active_days":     len(self.daily_messages),
            "active_messages": sum(len(m) for m in self.daily_messages.values()),
        }
        row = self.conn.execute(
            "SELECT COUNT(*) as d, SUM(message_count) as m FROM daily_archives"
        ).fetchone()
        stats["archived_days"]     = row["d"] or 0
        stats["archived_messages"] = row["m"] or 0
        stats["weekly_journals"]   = self.conn.execute(
            "SELECT COUNT(*) FROM weekly_journals"
        ).fetchone()[0]
        stats["periodic_reports"]  = self.conn.execute(
            "SELECT COUNT(*) FROM periodic_summaries"
        ).fetchone()[0]
        stats["embedding_model"]   = EMBEDDING_MODEL
        return stats

    # ══════════════════════════════════════════════════════════════════════
    # MIGRATION — re-embed existing DB to sentence-transformers dim 768
    # Run manually when ready: rag.rebuild_embeddings()
    # ══════════════════════════════════════════════════════════════════════

    def rebuild_embeddings(self) -> dict:
        """
        Re-embed all daily_archives rows already in THIS DB using
        sentence-transformers / embeddinggemma-300m (dim 768).

        Replaces old vectors (dim 2048 from Ollama embeddinggemma) with
        new vectors (dim 768) so semantic search works correctly.

        Safe to call multiple times — already-correct rows are re-embedded
        (idempotent, just overwrites with same result).

        Call this once after switching from Ollama embedder to sentence-transformers:
            from smc import SMC
            smc = SMC("/path/to/memory.db", logger)
            result = rag.rebuild_embeddings()

        Returns: {"rebuilt": N, "failed": N, "total": N}
        """
        self.logger.info("🔄 Rebuilding all embeddings → sentence-transformers dim 768…")

        rows = self.conn.execute(
            "SELECT da.id, da.date, da.full_text FROM daily_archives da ORDER BY da.date"
        ).fetchall()

        total   = len(rows)
        rebuilt = failed = 0

        self.logger.info(f"   Found {total} archived days to re-embed")

        for row in rows:
            archive_id = row["id"]
            date_str   = row["date"]
            full_text  = row["full_text"] or ""

            vec = self.embedder.encode(full_text[:4000], is_query=False)
            if not vec:
                self.logger.warn(f"   ⚠️  Could not embed {date_str} — skipping")
                failed += 1
                continue

            try:
                self.conn.execute(
                    """INSERT OR REPLACE INTO memory_vectors (archive_id, embedding)
                       VALUES (?, ?)""",
                    [archive_id, json.dumps(vec)],
                )
                self.conn.commit()
                rebuilt += 1
                self.logger.info(f"   ✅ Re-embedded {date_str}")
            except Exception as exc:
                self.conn.rollback()
                self.logger.error(f"   ❌ Failed {date_str}: {exc}")
                failed += 1

        result = {"rebuilt": rebuilt, "failed": failed, "total": total}
        self.logger.info(
            f"🏁 Rebuild done — rebuilt: {rebuilt}, failed: {failed}, total: {total}"
        )
        return result

    # ══════════════════════════════════════════════════════════════════════
    # EM BUFFER — crash-safe episodic accumulation
    # ══════════════════════════════════════════════════════════════════════

    def em_buffer_append(self, role: str, content: str, date_str: str):
        """Atomically write a turn to em_buffer. Survives crashes."""
        from datetime import datetime as _dt
        try:
            self.conn.execute(
                "INSERT INTO em_buffer (timestamp, role, content, date) VALUES (?, ?, ?, ?)",
                [_dt.now().isoformat(), role, content[:2000], date_str],
            )
            self.conn.commit()
        except Exception as e:
            self.logger.warning(f"em_buffer append failed: {e}")

    def em_buffer_get_day(self, date_str: str) -> list:
        """Get all unprocessed turns for a given date."""
        try:
            rows = self.conn.execute(
                "SELECT role, content, timestamp FROM em_buffer WHERE date = ? AND processed = FALSE ORDER BY id",
                [date_str],
            ).fetchall()
            return [{"role": r["role"], "content": r["content"], "timestamp": r["timestamp"]} for r in rows]
        except Exception as e:
            self.logger.warning(f"em_buffer get failed: {e}")
            return []

    def em_buffer_clear_day(self, date_str: str):
        """Clear processed em_buffer entries for a date."""
        try:
            self.conn.execute(
                "DELETE FROM em_buffer WHERE date = ?", [date_str]
            )
            self.conn.commit()
            self.logger.info(f"🧹 em_buffer cleared for {date_str}")
        except Exception as e:
            self.logger.warning(f"em_buffer clear failed: {e}")

    def em_buffer_has_unprocessed(self) -> list[str]:
        """Return list of dates with unprocessed em_buffer rows."""
        try:
            rows = self.conn.execute(
                "SELECT DISTINCT date FROM em_buffer WHERE processed = FALSE ORDER BY date"
            ).fetchall()
            return [r["date"] for r in rows]
        except Exception as e:
            self.logger.warning(f"em_buffer check failed: {e}")
            return []

    # ══════════════════════════════════════════════════════════════════════
    # CLEANUP
    # ══════════════════════════════════════════════════════════════════════

    def close(self):
        if self.conn:
            self.conn.close()
            self.logger.info("🗄️  SMC database closed")