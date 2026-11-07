"""PostgreSQL-backed conversation history and simple knowledge records."""
from __future__ import annotations

import psycopg


class GraceMemory:
    """Store recent turns and prepare document/chunk tables for ingestion."""

    def __init__(self, database_url: str, max_turns: int = 8) -> None:
        if not database_url:
            raise ValueError("Set AIVA_DATABASE_URL to Grace's PostgreSQL database")
        self.database_url = database_url
        self.max_turns = max(1, max_turns)
        self._initialize()

    def _connect(self) -> psycopg.Connection:
        return psycopg.connect(self.database_url)

    def _initialize(self) -> None:
        with self._connect() as db:
            db.execute("CREATE SCHEMA IF NOT EXISTS grace")
            db.execute(
                "CREATE TABLE IF NOT EXISTS grace.conversation_turns ("
                "id BIGINT GENERATED ALWAYS AS IDENTITY PRIMARY KEY, "
                "role TEXT NOT NULL CHECK (role IN ('user', 'assistant')), "
                "content TEXT NOT NULL, created_at TIMESTAMPTZ NOT NULL DEFAULT now())"
            )
            db.execute(
                "CREATE TABLE IF NOT EXISTS grace.knowledge_documents ("
                "id BIGINT GENERATED ALWAYS AS IDENTITY PRIMARY KEY, "
                "title TEXT NOT NULL, source TEXT, content_hash TEXT UNIQUE, "
                "created_at TIMESTAMPTZ NOT NULL DEFAULT now())"
            )
            db.execute(
                "CREATE TABLE IF NOT EXISTS grace.knowledge_chunks ("
                "id BIGINT GENERATED ALWAYS AS IDENTITY PRIMARY KEY, "
                "document_id BIGINT NOT NULL REFERENCES grace.knowledge_documents(id) ON DELETE CASCADE, "
                "chunk_index INTEGER NOT NULL, content TEXT NOT NULL, "
                "UNIQUE(document_id, chunk_index))"
            )

    def recent(self) -> list[dict[str, str]]:
        with self._connect() as db:
            rows = db.execute(
                "SELECT role, content FROM grace.conversation_turns "
                "ORDER BY id DESC LIMIT %s",
                (self.max_turns * 2,),
            ).fetchall()
        return [{"role": role, "content": content} for role, content in reversed(rows)]

    def add_exchange(self, user_text: str, assistant_text: str) -> None:
        with self._connect() as db:
            db.executemany(
                "INSERT INTO grace.conversation_turns(role, content) VALUES (%s, %s)",
                [("user", user_text), ("assistant", assistant_text)],
            )
            db.execute(
                "DELETE FROM grace.conversation_turns WHERE id NOT IN "
                "(SELECT id FROM grace.conversation_turns ORDER BY id DESC LIMIT %s)",
                (self.max_turns * 2,),
            )
