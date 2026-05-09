"""
Engram Complex Migration — v2 → v3
==================================
AuRoRA · Semantic Cognitive System (SCS)

Adds `user_id` column to `emc_storage` and `emc_staging` tables.
Required for per-user episodic memory filtering — engrams are now
scoped to a specific identity at the SQL layer rather than relying
on the PMT JSON content blob.

What this migration does:
    1. Validates the current schema version is v2 (refuses to run on anything else)
    2. Adds `user_id TEXT NOT NULL DEFAULT '<default_user>'` to emc_storage
    3. Adds `user_id TEXT NOT NULL DEFAULT '<default_user>'` to emc_staging
    4. Creates a B-tree index on emc_storage(user_id) for fast filtering
    5. Bumps schema_meta version from '2' to '3'

What this migration does NOT touch:
    PMT JSON content — the "user"/"assistant" role keys inside the content
    blob are GCE message schema role labels and must never change.
    user_id is a SQL-layer filter handle only.

    emc_vector (vec0 KNN index) — rowid-linked, no content columns to migrate.
    emc_lexical (FTS5 index)    — rowid-linked, no content columns to migrate.

Usage:
    Edit DEFAULT_USER below to match your active user identity,
    then run:

        python3 migrate_emc_v2_to_v3.py

    Run ONCE before restarting AuRoRA. The schema version gate in MSB
    will hard-stop the system if this migration has not been applied.

Safety:
    - Dry-run mode prints every SQL statement without executing (default ON).
    - Set DRY_RUN = False only when you are ready to commit.
    - A backup path is printed before any write — copy the DB file first.
"""

import sqlite3
from pathlib import Path

# ─── Configuration ────────────────────────────────────────────────────────────

# Absolute path to your engram complex SQLite file.
# Matches the path constructed in MCC.__init__() via AGi.ENTITY_GATEWAY.
# Example: /home/oppaai/.agi/scs/memory/engram.db
ENGRAM_GATEWAY: str = "/home/oppaai/.agi/scs/memory/engram.db"

# The user_id to backfill into all existing rows.
# All engrams written before this migration had no user_id — they belong
# to whoever was the sole user at that time.
# Change this to match your _active_user value in CNC (e.g. "oppaai").
DEFAULT_USER: str = "oppaai"

# Schema versions — migration refuses to run if DB is not on FROM_VERSION.
FROM_VERSION: int = 2
TO_VERSION:   int = 3

# Set to False when ready to commit changes to disk.
# While True, all SQL is printed but nothing is executed.
DRY_RUN: bool = True

# ─── Migration ────────────────────────────────────────────────────────────────

def connect(gateway: str) -> sqlite3.Connection:
    """
    Open a connection to the engram complex with WAL mode and row factory.
    Mirrors the connection strategy used by EngramComplex._connect_ecx().

    Args:
        gateway (str): Absolute path to the SQLite engram complex file.

    Returns:
        sqlite3.Connection: Configured connection ready for migration writes.

    Raises:
        FileNotFoundError : If the engram complex file does not exist at the given path.
        sqlite3.Error     : If the connection cannot be established.
    """
    if not Path(gateway).exists():
        raise FileNotFoundError(
            f"Engram complex not found at: {gateway}\n"
            f"Check ENGRAM_GATEWAY in this script matches your AGi entity gateway path."
        )
    conn = sqlite3.connect(gateway, check_same_thread=False)    # allow access from this script's thread
    conn.row_factory = sqlite3.Row                              # dict-like row access — column names usable as keys
    conn.execute("PRAGMA journal_mode=WAL;")                    # WAL mode — consistent with EngramComplex
    conn.execute("PRAGMA synchronous=NORMAL;")                  # matches EngramComplex — faster writes, safe enough
    conn.commit()
    return conn


def validate_version(conn: sqlite3.Connection) -> None:
    """
    Confirm the DB is on FROM_VERSION before applying any changes.
    Refuses to run on an unversioned DB, a wrong version, or an already-migrated DB.
    Hard-stops with a clear message — never silently proceeds on a bad state.

    Args:
        conn (sqlite3.Connection): Open connection to the engram complex.

    Raises:
        RuntimeError: If schema_meta is missing, version is wrong, or already migrated.
    """
    row = conn.execute(
        "SELECT value FROM schema_meta WHERE key = 'schema_version'"
    ).fetchone()

    if row is None:
        raise RuntimeError(
            "schema_meta missing or empty — this DB may be unversioned (pre-v2).\n"
            "Run the v1→v2 migration first, or inspect the DB manually."
        )

    current: int = int(row["value"])

    if current == TO_VERSION:
        raise RuntimeError(
            f"DB is already at schema v{TO_VERSION} — migration already applied. Nothing to do."
        )

    if current != FROM_VERSION:
        raise RuntimeError(
            f"Expected schema v{FROM_VERSION}, found v{current}.\n"
            f"This script only migrates v{FROM_VERSION} → v{TO_VERSION}.\n"
            f"Check whether a different migration script is needed."
        )

    print(f"✅ Schema version confirmed: v{current} — proceeding with migration.")


def column_exists(conn: sqlite3.Connection, table: str, column: str) -> bool:
    """
    Check whether a column already exists in a table.
    Used to make ALTER TABLE steps idempotent — safe to re-run if migration
    was interrupted after a partial write.

    Args:
        conn (sqlite3.Connection): Open connection to the engram complex.
        table (str)              : Table name to inspect.
        column (str)             : Column name to look for.

    Returns:
        bool: True if the column exists, False otherwise.
    """
    rows = conn.execute(f"PRAGMA table_info({table})").fetchall()   # SQLite PRAGMA returns one row per column
    return any(row["name"] == column for row in rows)               # match by column name field


def table_exists(conn: sqlite3.Connection, table: str) -> bool:
    """
    Check whether a table exists in the DB.
    Guards ALTER TABLE on emc_staging — staging is optional in the schema
    and may not exist if no PMTs have ever been evicted before a crash.

    Args:
        conn (sqlite3.Connection): Open connection to the engram complex.
        table (str)              : Table name to check.

    Returns:
        bool: True if the table exists, False otherwise.
    """
    row = conn.execute(
        "SELECT name FROM sqlite_master WHERE type='table' AND name=?",
        (table,)
    ).fetchone()
    return row is not None


def execute(conn: sqlite3.Connection, sql: str, params: list | None = None) -> None:
    """
    Execute a single SQL statement, or print it if DRY_RUN is active.
    All migration writes route through this function — makes dry-run
    behaviour consistent and easy to audit.

    Args:
        conn (sqlite3.Connection): Open connection to the engram complex.
        sql (str)                : SQL statement to execute.
        params (list | None)     : Optional bind parameters for the statement.
    """
    display = sql.strip()
    if params:
        display += f"  -- params: {params}"

    if DRY_RUN:
        print(f"  [DRY RUN] {display}")   # print but do not execute
    else:
        if params:
            conn.execute(sql, params)
        else:
            conn.execute(sql)
        print(f"  [EXECUTED] {display}")


def migrate(conn: sqlite3.Connection) -> None:
    """
    Apply all migration steps in order.
    Each step is idempotent — safe to re-run if the script was interrupted.

    Steps:
        1. Add user_id to emc_storage  (ALTER TABLE + backfill)
        2. Add user_id to emc_staging  (ALTER TABLE + backfill, if table exists)
        3. Create B-tree index on emc_storage(user_id)
        4. Bump schema_meta version to TO_VERSION

    Args:
        conn (sqlite3.Connection): Open connection to the engram complex.
    """

    # ── Step 1: emc_storage ───────────────────────────────────────────────────
    print("\n── Step 1: emc_storage ──────────────────────────────────────────")

    if not column_exists(conn, "emc_storage", "user_id"):
        # Add user_id column — NOT NULL with a default so existing rows are
        # immediately valid. SQLite requires DEFAULT when adding NOT NULL to
        # a populated table via ALTER TABLE.
        execute(conn,
            f"ALTER TABLE emc_storage ADD COLUMN user_id TEXT NOT NULL DEFAULT '{DEFAULT_USER}'"
        )
        # Explicit backfill — belt-and-suspenders. The DEFAULT above covers new
        # rows; this ensures existing rows carry the correct identity explicitly
        # rather than relying on the DEFAULT being preserved in future queries.
        execute(conn,
            "UPDATE emc_storage SET user_id = ? WHERE user_id IS NULL OR user_id = ''",
            [DEFAULT_USER]
        )
        print(f"  ✅ user_id column added to emc_storage (backfilled → '{DEFAULT_USER}')")
    else:
        print("  ⏭  user_id already exists in emc_storage — skipping.")

    # ── Step 2: emc_staging ───────────────────────────────────────────────────
    print("\n── Step 2: emc_staging ──────────────────────────────────────────")

    if not table_exists(conn, "emc_staging"):
        # Staging table is created lazily — may not exist if the system has
        # never crashed mid-encoding. Nothing to migrate; MSB will create it
        # with the correct schema (including user_id) on next boot.
        print("  ⏭  emc_staging does not exist — nothing to migrate (MSB will create it fresh).")
    elif not column_exists(conn, "emc_staging", "user_id"):
        execute(conn,
            f"ALTER TABLE emc_staging ADD COLUMN user_id TEXT NOT NULL DEFAULT '{DEFAULT_USER}'"
        )
        execute(conn,
            "UPDATE emc_staging SET user_id = ? WHERE user_id IS NULL OR user_id = ''",
            [DEFAULT_USER]
        )
        print(f"  ✅ user_id column added to emc_staging (backfilled → '{DEFAULT_USER}')")
    else:
        print("  ⏭  user_id already exists in emc_staging — skipping.")

    # ── Step 3: B-tree index on emc_storage(user_id) ─────────────────────────
    print("\n── Step 3: B-tree index ─────────────────────────────────────────")

    execute(conn,
        "CREATE INDEX IF NOT EXISTS idx_emc_storage_user_id ON emc_storage(user_id)"
    )
    print("  ✅ B-tree index on emc_storage(user_id) confirmed.")

    # ── Step 4: Bump schema version ───────────────────────────────────────────
    print("\n── Step 4: Schema version ───────────────────────────────────────")

    execute(conn,
        "UPDATE schema_meta SET value = ? WHERE key = 'schema_version'",
        [str(TO_VERSION)]
    )
    print(f"  ✅ schema_meta bumped: v{FROM_VERSION} → v{TO_VERSION}")

    if not DRY_RUN:
        conn.commit()                   # single commit — all steps land atomically
        print("\n✅ Migration committed.")
    else:
        print("\n⚠️  DRY RUN — no changes written. Set DRY_RUN = False to apply.")


# ─── Entry point ──────────────────────────────────────────────────────────────

def main() -> None:
    """
    Entry point — validates config, opens connection, runs migration.
    Prints a backup reminder before any write attempt.
    """
    print("=" * 60)
    print("MSB Schema Migration — v2 → v3")
    print("=" * 60)
    print(f"  Engram gateway : {ENGRAM_GATEWAY}")
    print(f"  Default user   : {DEFAULT_USER}")
    print(f"  Dry run        : {DRY_RUN}")
    print()

    if not DRY_RUN:
        print("⚠️  BACKUP REMINDER — copy your engram complex before proceeding:")
        print(f"    cp {ENGRAM_GATEWAY} {ENGRAM_GATEWAY}.v2.bak")
        print()
        confirm = input("Have you backed up the DB? Type 'yes' to continue: ").strip().lower()
        if confirm != "yes":
            print("Aborted — no changes made.")
            return

    conn = connect(ENGRAM_GATEWAY)

    try:
        validate_version(conn)  # hard-stop if DB is not on FROM_VERSION
        migrate(conn)           # apply all steps
    except (RuntimeError, FileNotFoundError) as e:
        print(f"\n❌ Migration aborted: {e}")
    finally:
        conn.close()            # always release connection — even on failure
        print("\n🔒 Connection closed.")


if __name__ == "__main__":
    main()
