"""
EMC — Episodic Memory Cortex
==============================
AuRoRA · Semantic Cognitive System (SCS)

Episodic memory for GRACE — "I remember that specific moment."
Stores individual conversation turns with semantic embeddings, forever.
No expiry — 1TB NVMe means GRACE remembers everything.

Architecture:
    WMC overflow → em_buffer (crash-safe raw intake)
                 → async embedding → episodes table (searchable)

Embedding:  google/embeddinggemma-300m-qat-q4_0 via sentence-transformers
            CPU-only, sub-200MB RAM, zero GPU impact
Similarity: Pure-Python cosine (no FAISS needed until millions of rows)
Storage:    SQLite — lightweight, Jetson-friendly, concurrent read/write

Retrieval:  Top-K semantic search over all episodes
            Used by MCC to inject relevant past context into Cosmos window

Buffer:     em_buffer table — crash-safe per-turn raw storage
            Async worker embeds buffer rows → episodes table
            Buffer rows deleted after successful embedding
Lifecycle:
    Binding → Encoding → Consolidation → Storing → Retrieval → Reinstatement
    
Daily flow:
    During day  → WMC evicts → em_buffer → async embed → episodes
    At 11pm     → MCC triggers reflection → SMC distils LTM (future M2)
"""

import asyncio
import json
import math
import sqlite3
import threading
from datetime import datetime
from pathlib import Path
from typing import Optional


# ── Embedding model ───────────────────────────────────────────────────────────
EMBEDDING_MODEL = "google/embeddinggemma-300m-qat-q4_0-unquantized"
EMBEDDING_DIM   = 768       # embeddinggemma-300m always outputs 768 dims
# NOTE: QAT q4_0 variant — sub-200MB RAM, CPU-only, float32 activations
# ─────────────────────────────────────────────────────────────────────────────

# ── Search defaults ───────────────────────────────────────────────────────────
DEFAULT_TOP_K         = 5    # episodes returned per search
DEFAULT_CONTENT_CHARS = 300  # truncate episode content in search results
# ─────────────────────────────────────────────────────────────────────────────


# ══════════════════════════════════════════════════════════════════════════════
# EMBEDDER
# ══════════════════════════════════════════════════════════════════════════════

class _Embedder:
    """
    CPU-only sentence-transformers wrapper for embeddinggemma-300m.
    Lazy-loads on first use. Caches recent embeddings to avoid re-encoding
    identical text (e.g. repeated short phrases).
    """

    def __init__(self, logger):
        self.logger     = logger
        self._model     = None
        self._available: Optional[bool] = None
        self._cache: dict[str, list[float]] = {}

    def _load(self) -> bool:
        if self._model is not None:
            return True
        try:
            from sentence_transformers import SentenceTransformer
            self.logger.info(f"⏳ Loading {EMBEDDING_MODEL} on CPU…")
            self._model     = SentenceTransformer(EMBEDDING_MODEL)
            self._available = True
            self.logger.info(f"✅ {EMBEDDING_MODEL} ready (CPU)")
            return True
        except ImportError:
            self.logger.warning(
                "⚠️  sentence-transformers not installed.\n"
                "   pip3 install sentence-transformers --break-system-packages"
            )
            self._available = False
            return False
        except Exception as exc:
            self.logger.warning(f"⚠️  Embedder load failed: {exc}")
            self._available = False
            return False

    def is_available(self) -> bool:
        if self._available is not None:
            return self._available
        return self._load()

    def encode(self, text: str, is_query: bool = False) -> list[float]:
        """
        Encode text to 768-dim vector.
        Uses encode_query() for search, encode_document() for storage.
        Returns [] on failure — callers fall back to keyword search.
        """
        if not self._load():
            return []

        key = f"{'q' if is_query else 'd'}:{text[:300]}"
        if key in self._cache:
            return self._cache[key]

        try:
            if is_query:
                vec = self._model.encode_query(text).tolist()
            else:
                vec = self._model.encode_document(text).tolist()
            # Keep cache small — evict oldest if over 512 entries
            if len(self._cache) >= 512:
                oldest = next(iter(self._cache))
                del self._cache[oldest]
            self._cache[key] = vec
            return vec
        except Exception as exc:
            self.logger.debug(f"Embed error: {exc}")
            return []


# ══════════════════════════════════════════════════════════════════════════════
# COSINE SIMILARITY
# ══════════════════════════════════════════════════════════════════════════════

def _cosine(a: list[float], b: list[float]) -> float:
    if not a or not b or len(a) != len(b):
        return 0.0
    dot = sum(x * y for x, y in zip(a, b))
    na  = math.sqrt(sum(x * x for x in a))
    nb  = math.sqrt(sum(x * x for x in b))
    return dot / (na * nb) if na and nb else 0.0


# ══════════════════════════════════════════════════════════════════════════════
# EMC
# ══════════════════════════════════════════════════════════════════════════════

class EMC:
    """
    Episodic Memory Cortex.

    Two-table SQLite design:
        em_buffer  — raw turns from WMC overflow (crash-safe intake)
        episodes   — embedded, searchable episodic memory (permanent)

    The async worker runs in a background thread, draining em_buffer →
    embedding → episodes continuously without blocking GRACE's responses.
    """

    def __init__(self, db_path: str, logger):
        self.logger   = logger
        self.db_path  = db_path

        # Embedder — CPU, lazy-loaded
        self._embedder = _Embedder(logger)

        # SQLite — WAL mode for concurrent reads during async writes
        self.conn = sqlite3.connect(db_path, check_same_thread=False)
        self.conn.row_factory = sqlite3.Row
        self.conn.execute("PRAGMA journal_mode=WAL;")
        self.conn.execute("PRAGMA synchronous=NORMAL;")
        self.conn.commit()
        self._init_tables()

        # Write lock — only async worker writes to episodes
        self._write_lock = threading.Lock()

        # Async worker
        self._worker_running = False
        self._worker_thread: Optional[threading.Thread] = None
        self._start_worker()

        self.logger.info(f"✅ EMC initialised → {db_path}")

    # ── Schema ────────────────────────────────────────────────────────────────

    def _init_tables(self):
        self.conn.executescript("""
            -- Raw turn intake from WMC overflow (crash-safe)
            CREATE TABLE IF NOT EXISTS em_buffer (
                id         INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp  TEXT    NOT NULL,
                role       TEXT    NOT NULL,
                content    TEXT    NOT NULL,
                date       TEXT    NOT NULL,
                processed  BOOLEAN DEFAULT FALSE
            );
            CREATE INDEX IF NOT EXISTS idx_em_buffer_processed
                ON em_buffer(processed);
            CREATE INDEX IF NOT EXISTS idx_em_buffer_date
                ON em_buffer(date);

            -- Embedded episodic memory (permanent, searchable)
            CREATE TABLE IF NOT EXISTS episodes (
                id         INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp  TEXT    NOT NULL,
                date       TEXT    NOT NULL,
                role       TEXT    NOT NULL,
                content    TEXT    NOT NULL,
                embedding  TEXT    NOT NULL,   -- JSON float array (768 dims)
                created_at TEXT    DEFAULT (datetime('now'))
            );
            CREATE INDEX IF NOT EXISTS idx_episodes_date
                ON episodes(date);
            CREATE INDEX IF NOT EXISTS idx_episodes_role
                ON episodes(role);
        """)
        self.conn.commit()

    # ── Buffer intake (called by MCC from asyncio loop) ───────────────────────

    def buffer_append(self, role: str, content: str) -> bool:
        """
        Atomically write an evicted WMC turn to em_buffer.
        Called by MCC — survives crashes, never blocks GRACE.

        Args:
            role:    "user" or "assistant"
            content: Turn text (truncated to 2000 chars for safety)

        Returns:
            True on success, False on failure
        """
        now      = datetime.now()
        date_str = now.date().isoformat()
        ts       = now.isoformat()

        try:
            self.conn.execute(
                "INSERT INTO em_buffer (timestamp, role, content, date) "
                "VALUES (?, ?, ?, ?)",
                [ts, role, content[:2000], date_str],
            )
            self.conn.commit()
            self.logger.debug(
                f"EMC buffer ← [{role}] {content[:40]}…"
            )
            return True
        except Exception as exc:
            self.logger.warning(f"EMC buffer_append failed: {exc}")
            return False

    # ── Async worker — buffer → embed → episodes ──────────────────────────────

    def _start_worker(self):
        """Start background embedding worker thread."""
        self._worker_running = True
        self._worker_thread  = threading.Thread(
            target=self._embed_worker,
            name="emc-embed-worker",
            daemon=True,
        )
        self._worker_thread.start()
        self.logger.info("🔄 EMC embedding worker started")

    def _embed_worker(self):
        """
        Background worker — drains em_buffer, embeds, stores in episodes.
        Runs forever, sleeps when buffer is empty.
        Processes one row at a time for stability under Jetson load.
        """
        # Worker-local SQLite connection (WAL allows concurrent access)
        worker_conn = sqlite3.connect(self.db_path, check_same_thread=False)
        worker_conn.row_factory = sqlite3.Row
        worker_conn.execute("PRAGMA journal_mode=WAL;")

        self.logger.info("⚙️  EMC worker running…")

        while self._worker_running:
            try:
                # Fetch one unprocessed buffer row
                row = worker_conn.execute(
                    "SELECT id, timestamp, role, content, date "
                    "FROM em_buffer WHERE processed = FALSE "
                    "ORDER BY id LIMIT 1"
                ).fetchone()

                if row is None:
                    # Buffer empty — sleep and check again
                    import time; time.sleep(2.0)
                    continue

                # Embed the turn content
                vec = self._embedder.encode(row["content"], is_query=False)

                if not vec:
                    # Embedder unavailable — skip for now, retry later
                    self.logger.warning(
                        f"EMC embed skipped (embedder unavailable): "
                        f"buffer id={row['id']}"
                    )
                    import time; time.sleep(5.0)
                    continue

                embedding_json = json.dumps(vec)

                with self._write_lock:
                    # Insert into episodes
                    worker_conn.execute(
                        "INSERT INTO episodes "
                        "(timestamp, date, role, content, embedding) "
                        "VALUES (?, ?, ?, ?, ?)",
                        [
                            row["timestamp"],
                            row["date"],
                            row["role"],
                            row["content"],
                            embedding_json,
                        ],
                    )
                    # Mark buffer row as processed
                    worker_conn.execute(
                        "UPDATE em_buffer SET processed = TRUE WHERE id = ?",
                        [row["id"]],
                    )
                    worker_conn.commit()

                self.logger.debug(
                    f"EMC embedded → episodes: [{row['role']}] "
                    f"{row['content'][:40]}… (date={row['date']})"
                )

            except Exception as exc:
                self.logger.error(f"EMC worker error: {exc}")
                import time; time.sleep(2.0)

        worker_conn.close()
        self.logger.info("EMC worker stopped")

    # ── Semantic search (called by MCC) ───────────────────────────────────────

    def search(
        self,
        query: str,
        top_k: int = DEFAULT_TOP_K,
        date_from: Optional[str] = None,
        date_to: Optional[str] = None,
    ) -> list[dict]:
        """
        Semantic search over all embedded episodes.
        Returns top-K most relevant episodes sorted by cosine similarity.

        Falls back to keyword search if embedder is unavailable.

        Args:
            query:     Natural language query
            top_k:     Number of results to return
            date_from: Optional ISO date string lower bound (inclusive)
            date_to:   Optional ISO date string upper bound (inclusive)

        Returns:
            List of dicts: {timestamp, date, role, content, similarity}
        """
        query_vec = self._embedder.encode(query, is_query=True)
        if not query_vec:
            return self._keyword_search(query, top_k, date_from, date_to)

        try:
            sql    = "SELECT timestamp, date, role, content, embedding FROM episodes"
            params = []
            where  = []

            if date_from:
                where.append("date >= ?")
                params.append(date_from)
            if date_to:
                where.append("date <= ?")
                params.append(date_to)
            if where:
                sql += " WHERE " + " AND ".join(where)

            rows = self.conn.execute(sql, params).fetchall()
            if not rows:
                return []

            scored = []
            for row in rows:
                stored_vec = json.loads(row["embedding"] or "[]")
                sim = _cosine(query_vec, stored_vec)
                scored.append({
                    "timestamp":  row["timestamp"],
                    "date":       row["date"],
                    "role":       row["role"],
                    "content":    row["content"][:DEFAULT_CONTENT_CHARS],
                    "similarity": round(sim, 4),
                })

            scored.sort(key=lambda x: x["similarity"], reverse=True)
            results = scored[:top_k]

            self.logger.debug(
                f"EMC search '{query[:30]}…' → "
                f"{len(results)} results "
                f"(top sim={results[0]['similarity'] if results else 0})"
            )
            return results

        except Exception as exc:
            self.logger.error(f"EMC search failed: {exc}")
            return []

    def _keyword_search(
        self,
        query: str,
        top_k: int,
        date_from: Optional[str],
        date_to: Optional[str],
    ) -> list[dict]:
        """Fallback keyword search when embedder is unavailable."""
        words = [w for w in query.lower().split() if len(w) > 3]
        if not words:
            return []

        where  = ["(" + " OR ".join(["LOWER(content) LIKE ?" for _ in words]) + ")"]
        params = [f"%{w}%" for w in words]

        if date_from:
            where.append("date >= ?")
            params.append(date_from)
        if date_to:
            where.append("date <= ?")
            params.append(date_to)

        sql = (
            f"SELECT timestamp, date, role, content FROM episodes "
            f"WHERE {' AND '.join(where)} "
            f"ORDER BY timestamp DESC LIMIT {top_k}"
        )
        try:
            rows = self.conn.execute(sql, params).fetchall()
            return [
                {
                    "timestamp":  r["timestamp"],
                    "date":       r["date"],
                    "role":       r["role"],
                    "content":    r["content"][:DEFAULT_CONTENT_CHARS],
                    "similarity": 0.0,
                }
                for r in rows
            ]
        except Exception as exc:
            self.logger.error(f"EMC keyword search failed: {exc}")
            return []

    # ── Episode retrieval for a specific date ─────────────────────────────────

    def get_episodes_for_date(self, date_str: str) -> list[dict]:
        """
        Return all episodes for a given date in chronological order.
        Used by MCC for 11pm reflection (future M2).
        """
        try:
            rows = self.conn.execute(
                "SELECT timestamp, role, content FROM episodes "
                "WHERE date = ? ORDER BY timestamp",
                [date_str],
            ).fetchall()
            return [
                {"timestamp": r["timestamp"], "role": r["role"], "content": r["content"]}
                for r in rows
            ]
        except Exception as exc:
            self.logger.error(f"EMC get_episodes_for_date failed: {exc}")
            return []

    # ── Buffer status ─────────────────────────────────────────────────────────

    def buffer_pending_count(self) -> int:
        """Return number of turns waiting to be embedded."""
        try:
            row = self.conn.execute(
                "SELECT COUNT(*) FROM em_buffer WHERE processed = FALSE"
            ).fetchone()
            return row[0] if row else 0
        except Exception:
            return 0

    # ── Stats ─────────────────────────────────────────────────────────────────

    def get_stats(self) -> dict:
        """Return EMC health and storage stats."""
        try:
            ep_row = self.conn.execute(
                "SELECT COUNT(*) as total, "
                "MIN(date) as oldest, MAX(date) as newest "
                "FROM episodes"
            ).fetchone()

            buf_row = self.conn.execute(
                "SELECT COUNT(*) as total, "
                "SUM(CASE WHEN processed=FALSE THEN 1 ELSE 0 END) as pending "
                "FROM em_buffer"
            ).fetchone()

            db_size_mb = round(Path(self.db_path).stat().st_size / 1_048_576, 2) \
                if Path(self.db_path).exists() else 0.0

            return {
                "episodes":         ep_row["total"]  if ep_row  else 0,
                "oldest_episode":   ep_row["oldest"] if ep_row  else None,
                "newest_episode":   ep_row["newest"] if ep_row  else None,
                "buffer_total":     buf_row["total"]   if buf_row else 0,
                "buffer_pending":   buf_row["pending"]  if buf_row else 0,
                "db_size_mb":       db_size_mb,
                "embedder_ready":   self._embedder.is_available(),
                "embedding_model":  EMBEDDING_MODEL,
            }
        except Exception as exc:
            self.logger.error(f"EMC get_stats failed: {exc}")
            return {}

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def cleanup_processed_buffer(self, keep_days: int = 1):
        """
        Remove processed em_buffer rows older than keep_days.
        Safe to call periodically — processed rows are already in episodes.
        """
        try:
            self.conn.execute(
                "DELETE FROM em_buffer "
                "WHERE processed = TRUE "
                "AND date < date('now', ?)",
                [f"-{keep_days} days"],
            )
            self.conn.commit()
            self.logger.debug("EMC buffer cleanup done")
        except Exception as exc:
            self.logger.warning(f"EMC buffer cleanup failed: {exc}")

    def close(self):
        """Stop worker and close DB connection."""
        self._worker_running = False
        if self._worker_thread:
            self._worker_thread.join(timeout=3.0)
        if self.conn:
            self.conn.close()
        self.logger.info("🗄️  EMC closed")
