#!/usr/bin/env python3
"""
EMC Schema Migration — v1 → v2
================================
Adds M2a/M2b prep columns to emc_storage:

  Importance  : memory_strength, last_recalled_at, recall_count, novelty_score
  Versioning  : conflict, superseded_by, valid_from, valid_until

Safe to run multiple times — skips columns that already exist.
Updates schema_meta.schema_version from 1 to 2.

Usage:
    python3 migrate_emc_schema.py
    python3 migrate_emc_schema.py --db /custom/path/engram_complex.db
"""

import argparse
import sqlite3
import shutil
from datetime import datetime
from pathlib import Path


DB_PATH = Path.home() / ".agi" / "cns" / "engram_complex.db"
TARGET_VERSION = 2

NEW_COLUMNS = [
    # Importance columns (M2a)
    ("memory_strength",   "REAL"),
    ("last_recalled_at",  "TEXT"),
    ("recall_count",      "INTEGER DEFAULT 0"),
    ("novelty_score",     "REAL"),
    # Versioning columns (M2b)
    ("conflict",          "INTEGER DEFAULT 0"),
    ("superseded_by",     "INTEGER"),
    ("valid_from",        "TEXT"),
    ("valid_until",       "TEXT"),
]


def backup_db(db_path: Path) -> Path:
    """Create a timestamped backup before migration."""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_path = db_path.with_name(f"engram_complex.backup_{timestamp}.db")
    shutil.copy2(db_path, backup_path)
    print(f"  Backup created: {backup_path}")
    return backup_path


def get_schema_version(conn: sqlite3.Connection) -> int:
    """Read current schema version from schema_meta table."""
    try:
        row = conn.execute(
            "SELECT value FROM schema_meta WHERE key = 'schema_version'"
        ).fetchone()
        return int(row[0]) if row else 0
    except sqlite3.OperationalError:
        return 0


def get_existing_columns(conn: sqlite3.Connection, table: str) -> set[str]:
    """Return set of existing column names for a table."""
    rows = conn.execute(f"PRAGMA table_info({table})").fetchall()
    return {row[1] for row in rows}


def migrate(db_path: Path) -> None:
    print(f"\nEMC Schema Migration — v1 → v2")
    print(f"  Database : {db_path}")

    if not db_path.exists():
        print(f"  ERROR: Database not found at {db_path}")
        return

    # Backup first
    backup_db(db_path)

    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row

    try:
        current_version = get_schema_version(conn)
        print(f"  Current schema version: {current_version}")

        if current_version >= TARGET_VERSION:
            print(f"  Already at v{current_version} — nothing to do.")
            return

        existing_columns = get_existing_columns(conn, "emc_storage")
        added = []
        skipped = []

        for col_name, col_type in NEW_COLUMNS:
            if col_name in existing_columns:
                skipped.append(col_name)
                continue
            conn.execute(
                f"ALTER TABLE emc_storage ADD COLUMN {col_name} {col_type}"
            )
            added.append(col_name)
            print(f"  + Added column: {col_name} {col_type}")

        if skipped:
            print(f"  ~ Skipped (already exist): {', '.join(skipped)}")

        # Bump schema version
        conn.execute(
            "UPDATE schema_meta SET value = ? WHERE key = 'schema_version'",
            (str(TARGET_VERSION),)
        )

        conn.commit()
        print(f"\n  ✅ Migration complete — schema version: {TARGET_VERSION}")
        print(f"  Columns added : {len(added)}")
        print(f"  Rows affected : 0 (nullable columns, existing rows unchanged)")

    except Exception as e:
        conn.rollback()
        print(f"\n  ❌ Migration failed: {e}")
        print(f"  Database rolled back — restore from backup if needed.")
        raise

    finally:
        conn.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Migrate EMC schema v1 → v2")
    parser.add_argument(
        "--db",
        type=Path,
        default=DB_PATH,
        help=f"Path to engram_complex.db (default: {DB_PATH})"
    )
    args = parser.parse_args()
    migrate(args.db)