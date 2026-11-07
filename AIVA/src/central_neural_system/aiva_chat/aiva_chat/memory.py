"""Small persistent turn history backed by SQLite."""
from __future__ import annotations

import sqlite3
from pathlib import Path


class ConversationMemory:
    """Store user and assistant turns and return a bounded recent context."""

    def __init__(self, path: Path, max_turns: int = 8) -> None:
        self.path = path
        self.max_turns = max(1, max_turns)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self._connect() as db:
            db.execute("PRAGMA journal_mode=WAL")
            db.execute(
                "CREATE TABLE IF NOT EXISTS turns ("
                "id INTEGER PRIMARY KEY AUTOINCREMENT, role TEXT NOT NULL, content TEXT NOT NULL)"
            )

    def _connect(self) -> sqlite3.Connection:
        return sqlite3.connect(self.path, timeout=5)

    def recent(self) -> list[dict[str, str]]:
        with self._connect() as db:
            rows = db.execute(
                "SELECT role, content FROM turns ORDER BY id DESC LIMIT ?",
                (self.max_turns * 2,),
            ).fetchall()
        return [{"role": role, "content": content} for role, content in reversed(rows)]

    def add_exchange(self, user_text: str, assistant_text: str) -> None:
        with self._connect() as db:
            db.executemany(
                "INSERT INTO turns(role, content) VALUES (?, ?)",
                [("user", user_text), ("assistant", assistant_text)],
            )
            db.execute(
                "DELETE FROM turns WHERE id NOT IN "
                "(SELECT id FROM turns ORDER BY id DESC LIMIT ?)",
                (self.max_turns * 2,),
            )
