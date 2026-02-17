"""
SimpleSQLiteRAG — Grace's RAG memory backend (EmbeddingGemma via Ollama)

Replaces the old sentence_transformers / numpy implementation with a pure
requests-based approach using EmbeddingGemma:300m-qat-q4_0 via Ollama.

Key changes vs old version:
  - No sentence_transformers, no numpy, no torch
  - Embeddings generated via Ollama HTTP API (embeddinggemma:300m-qat-q4_0)
  - Pure Python cosine similarity (~200MB model vs ~90MB MiniLM, much better quality)
  - Same public API — cnc.py needs zero changes beyond removing the old imports
  - Includes migrate_from_old_db() to preserve all old memories by re-embedding them

Memory architecture:
  ┌─────────────────────────────────────────────────┐
  │  7-day rolling window  (in-memory dict)         │
  │  → full message objects, instant retrieval      │
  ├─────────────────────────────────────────────────┤
  │  SQLite archive  (on-disk)                      │
  │  → daily_archives: full text + reflection       │
  │  → memory_vectors: JSON embedding arrays        │
  │  → weekly_journals, periodic_summaries          │
  └─────────────────────────────────────────────────┘

Usage:
    from scs.simple_sqlite_rag import SimpleSQLiteRAG

    rag = SimpleSQLiteRAG("/path/to/grace_memory.db", logger)

    # (Optional) one-time migration from old DB:
    rag.migrate_from_old_db("/path/to/old_grace_memory.db")
"""

import sqlite3
import json
import math
import time
import requests
from datetime import datetime, timedelta
from pathlib import Path


# ─── Embedding model config ───────────────────────────────────────────────────
EMBEDDING_MODEL   = "embeddinggemma:300m-qat-q4_0"
EMBEDDING_DIM     = 2048          # embeddinggemma output dimension
OLLAMA_EMBED_URL  = "http://localhost:11434/api/embed"
EMBED_TIMEOUT     = 15            # seconds per request
EMBED_RETRY       = 2             # retries on failure
# ─────────────────────────────────────────────────────────────────────────────


class OllamaEmbedder:
    """
    Thin wrapper around Ollama /api/embed.
    Handles retry logic and in-process caching.
    """

    def __init__(self, ollama_base_url: str, logger):
        self.url    = ollama_base_url.rstrip("/") + "/api/embed"
        self.logger = logger
        self._cache: dict[str, list[float]] = {}
        self._available: bool | None = None

    # ------------------------------------------------------------------
    def is_available(self) -> bool:
        if self._available is not None:
            return self._available
        try:
            r = requests.post(
                self.url,
                json={"model": EMBEDDING_MODEL, "input": "ping"},
                timeout=10,
            )
            self._available = (r.status_code == 200)
        except Exception:
            self._available = False

        if self._available:
            self.logger.info(f"✅ EmbeddingGemma ready ({EMBEDDING_MODEL})")
        else:
            self.logger.warn(
                f"⚠️  EmbeddingGemma not reachable at {self.url}. "
                f"Run: ollama pull {EMBEDDING_MODEL}"
            )
        return self._available

    # ------------------------------------------------------------------
    def encode(self, text: str) -> list[float]:
        """
        Return embedding vector. Returns [] on failure so callers degrade
        gracefully (falls back to keyword search).
        """
        if not self.is_available():
            return []

        cache_key = text[:300]
        if cache_key in self._cache:
            return self._cache[cache_key]

        for attempt in range(EMBED_RETRY + 1):
            try:
                r = requests.post(
                    self.url,
                    json={"model": EMBEDDING_MODEL, "input": text},
                    timeout=EMBED_TIMEOUT,
                )
                if r.status_code == 200:
                    vec = r.json().get("embeddings", [[]])[0]
                    if vec:
                        self._cache[cache_key] = vec
                        return vec
            except Exception as exc:
                if attempt < EMBED_RETRY:
                    time.sleep(1)
                else:
                    self.logger.debug(f"Embed failed after {EMBED_RETRY} retries: {exc}")
        return []


# ─── Pure-Python cosine similarity ───────────────────────────────────────────

def _cosine(a: list[float], b: list[float]) -> float:
    if not a or not b or len(a) != len(b):
        return 0.0
    dot  = sum(x * y for x, y in zip(a, b))
    na   = math.sqrt(sum(x * x for x in a))
    nb   = math.sqrt(sum(x * x for x in b))
    return dot / (na * nb) if na and nb else 0.0


# ─────────────────────────────────────────────────────────────────────────────

class SimpleSQLiteRAG:
    """
    Grace's vector memory — same public API as the old sentence_transformers
    version, but powered by EmbeddingGemma via Ollama.

    The SQLite schema is identical to the old version so existing .db files
    can be migrated (see migrate_from_old_db()).
    """

    def __init__(self, db_path: str, logger, ollama_base_url: str = "http://localhost:11434"):
        self.logger   = logger
        self.db_path  = db_path

        # Embedder
        self.embedder = OllamaEmbedder(ollama_base_url, logger)

        # SQLite
        self.conn = sqlite3.connect(db_path, check_same_thread=False)
        self.conn.row_factory = sqlite3.Row
        self._init_tables()

        # 7-day rolling window (in-memory)
        self.daily_messages:  dict[str, list] = {}
        self.weekly_journals:  list = []
        self.monthly_summaries: list = []

        self.logger.info("✅ SimpleSQLiteRAG (EmbeddingGemma) initialized")

    # ══════════════════════════════════════════════════════════════════════
    # SCHEMA
    # ══════════════════════════════════════════════════════════════════════

    def _init_tables(self):
        self.conn.executescript("""
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

            # Embed the day's text
            self.logger.info(f"🔢 Embedding {date_str} ({len(messages)} msgs)…")
            vec = self.embedder.encode(full_text[:4000])   # cap at 4K chars
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
            query_vec = self.embedder.encode(query)
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
    # MIGRATION  — one-time, run once, safe to call again (idempotent)
    # ══════════════════════════════════════════════════════════════════════

    def migrate_from_old_db(self, old_db_path: str) -> dict:
        """
        Re-embed all daily_archives rows from the old SQLite DB
        (which used all-MiniLM-L6-v2 / numpy) into THIS DB using EmbeddingGemma.

        Safe to call multiple times — rows already present in this DB are skipped.

        Returns: {"migrated": N, "skipped": N, "failed": N}
        """
        old_path = Path(old_db_path)
        if not old_path.exists():
            self.logger.warn(f"⚠️  Old DB not found: {old_db_path} — nothing to migrate")
            return {"migrated": 0, "skipped": 0, "failed": 0}

        self.logger.info(f"🔄 Migrating memory from {old_db_path} → EmbeddingGemma…")

        migrated = skipped = failed = 0

        try:
            old_conn = sqlite3.connect(old_db_path)
            old_conn.row_factory = sqlite3.Row

            rows = old_conn.execute(
                "SELECT date, message_count, full_text, summary FROM daily_archives ORDER BY date"
            ).fetchall()

            self.logger.info(f"   Found {len(rows)} archived days to migrate")

            for row in rows:
                date_str = row["date"]

                # Skip if already in new DB
                exists = self.conn.execute(
                    "SELECT 1 FROM daily_archives WHERE date = ?", [date_str]
                ).fetchone()
                if exists:
                    skipped += 1
                    continue

                full_text = row["full_text"] or ""
                summary   = row["summary"]   or ""

                # Re-embed with EmbeddingGemma
                vec = self.embedder.encode(full_text[:4000])
                if not vec:
                    self.logger.warn(f"   ⚠️  Could not embed {date_str} — embedder unavailable")
                    failed += 1
                    continue

                try:
                    cur = self.conn.execute(
                        """INSERT OR REPLACE INTO daily_archives
                               (date, message_count, full_text, summary)
                           VALUES (?, ?, ?, ?)""",
                        [date_str, row["message_count"], full_text, summary],
                    )
                    archive_id = cur.lastrowid

                    self.conn.execute(
                        """INSERT OR REPLACE INTO memory_vectors (archive_id, embedding)
                           VALUES (?, ?)""",
                        [archive_id, json.dumps(vec)],
                    )
                    self.conn.commit()
                    migrated += 1
                    self.logger.info(f"   ✅ Migrated {date_str}")

                except Exception as exc:
                    self.conn.rollback()
                    self.logger.error(f"   ❌ Failed to migrate {date_str}: {exc}")
                    failed += 1

            # Also migrate weekly_journals and periodic_summaries (no re-embedding needed)
            for table in ("weekly_journals", "periodic_summaries"):
                try:
                    old_rows = old_conn.execute(f"SELECT * FROM {table}").fetchall()
                    for r in old_rows:
                        cols = r.keys()
                        placeholders = ", ".join("?" for _ in cols)
                        vals = [r[c] for c in cols]
                        self.conn.execute(
                            f"INSERT OR IGNORE INTO {table} ({', '.join(cols)}) VALUES ({placeholders})",
                            vals,
                        )
                    self.conn.commit()
                    self.logger.info(f"   ✅ Copied {len(old_rows)} rows from {table}")
                except Exception as exc:
                    self.logger.warn(f"   ⚠️  Could not migrate {table}: {exc}")

            old_conn.close()

        except Exception as exc:
            self.logger.error(f"❌ Migration failed: {exc}")
            import traceback
            self.logger.error(traceback.format_exc())

        result = {"migrated": migrated, "skipped": skipped, "failed": failed}
        self.logger.info(
            f"🏁 Migration done — migrated: {migrated}, skipped: {skipped}, failed: {failed}"
        )
        return result

    # ══════════════════════════════════════════════════════════════════════
    # CLEANUP
    # ══════════════════════════════════════════════════════════════════════

    def close(self):
        if self.conn:
            self.conn.close()
            self.logger.info("🗄️  RAG database closed")
