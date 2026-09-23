"""
migrate_engram_complex.py
=========================
AuRoRA · EMC schema migration — old → new

Changes applied to engram_complex.db
─────────────────────────────────────
  schema_meta : schema_version 3 → 4

  emc_storage : content → trace        (rename via table rebuild)
                reformat trace data   (JSON {user,assistant} → plain text)
                DROP memory_strength   (removed in new schema)
                DROP novelty_score     (removed in new schema)

  emc_staging : content → trace        (rename via table rebuild)

  emc_lexical : DROP + rebuild FTS5 to index `trace` instead of `content`
                (FTS5 virtual tables cannot be ALTER'd — rebuild required)

  emc_vector  : no change — rowid linkage is unaffected

Note: SQLite has no DROP COLUMN before 3.35 and no RENAME COLUMN before 3.25.
      This script uses table rebuild for both — safe on all SQLite versions
      shipped with Jetson JetPack.

Usage
─────
  python migrate_engram_complex.py [--db PATH] [--dry-run] [--no-backup]

  --db PATH      Path to engram_complex.db  (default: ./engram_complex.db)
  --dry-run      Print SQL plan only — no writes
  --no-backup    Skip backup (not recommended)

Safety
──────
  - Timestamped .bak created before any mutation
  - All DDL runs in a single transaction — full rollback on any error
  - Row counts validated before and after
  - Idempotent — already-applied steps are detected and skipped
"""

import argparse
import json
import shutil
import sqlite3
import sys
from datetime import datetime
from pathlib import Path


# ── CLI ────────────────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Migrate engram_complex.db to new EMC schema")
    p.add_argument("--db",        default="engram_complex.db",
                   help="Path to engram_complex.db (default: ./engram_complex.db)")
    p.add_argument("--dry-run",   action="store_true",
                   help="Print SQL plan only — no writes")
    p.add_argument("--no-backup", action="store_true",
                   help="Skip backup (not recommended)")
    return p.parse_args()


# ── Helpers ────────────────────────────────────────────────────────────────────

def log(msg: str) -> None:
    print(f"  {msg}")

def section(title: str) -> None:
    print(f"\n{'─' * 60}")
    print(f"  {title}")
    print(f"{'─' * 60}")

def col_names(conn: sqlite3.Connection, table: str) -> set[str]:
    try:
        return {r[1] for r in conn.execute(f"PRAGMA table_info({table})")}
    except sqlite3.OperationalError:
        return set()

def row_count(conn: sqlite3.Connection, table: str) -> int:
    try:
        return conn.execute(f"SELECT COUNT(*) FROM {table}").fetchone()[0]
    except sqlite3.OperationalError:
        return -1

def fts_sql(conn: sqlite3.Connection, table: str) -> str:
    row = conn.execute(
        "SELECT sql FROM sqlite_master WHERE type='table' AND name=?", (table,)
    ).fetchone()
    return row[0] if row else ""

def _safe_default(dflt: str, typ: str) -> str:
    """
    Normalize a DEFAULT value from PRAGMA table_info for use in CREATE TABLE.

    PRAGMA table_info returns the stored default without the DEFAULT keyword:
        (datetime('now'))  — expression with parens — pass through as-is
        datetime('now')    — expression without parens — wrap in parens
        '0'                — quoted numeric for INTEGER/REAL — strip quotes
        'active'           — quoted TEXT string — pass through as-is
    """
    # Quoted numeric for INTEGER/REAL — strip spurious quotes e.g. '0' → 0
    if typ.upper() in ("INTEGER", "REAL") and dflt.startswith("'") and dflt.endswith("'"):
        inner = dflt[1:-1]
        try:
            float(inner)    # confirm actually numeric before stripping
            return inner
        except ValueError:
            return dflt     # not numeric — leave quoted

    # Bare SQL expression (contains a function call but no wrapping parens)
    # e.g. datetime('now') → (datetime('now'))
    if "(" in dflt and not dflt.startswith("(") and not dflt.startswith("'"):
        return f"({dflt})"

    return dflt             # already correct — (datetime('now')), '0', 'text' etc.


# ── Backup ─────────────────────────────────────────────────────────────────────

def backup(db_path: Path) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    bak   = db_path.with_suffix(f".{stamp}.bak")
    shutil.copy2(db_path, bak)
    log(f"Backup → {bak}")
    return bak


# ── Schema version ────────────────────────────────────────────────────────────

SCHEMA_VERSION_OLD = 3
SCHEMA_VERSION_NEW = 4


# ── Preflight ──────────────────────────────────────────────────────────────────

def preflight(conn: sqlite3.Connection) -> dict:
    storage_cols = col_names(conn, "emc_storage")
    staging_cols = col_names(conn, "emc_staging")
    fts          = fts_sql(conn, "emc_lexical")

    # Read current schema_version from schema_meta
    row = conn.execute(
        "SELECT value FROM schema_meta WHERE key = 'schema_version'"
    ).fetchone()
    current_version = int(row[0]) if row else None

    return {
        "storage_cols"        : storage_cols,
        "staging_cols"        : staging_cols,
        "fts"                 : fts,
        "current_version"     : current_version,
        "storage_done"        : "trace" in storage_cols and "content" not in storage_cols,
        "staging_done"        : "trace" in staging_cols and "content" not in staging_cols,
        "fts_done"            : "trace" in fts and "content" not in fts,
        "storage_count_before": row_count(conn, "emc_storage"),
        "staging_count_before": row_count(conn, "emc_staging"),
    }


def check_schema_version(state: dict) -> None:
    """Abort if the DB is not at the expected source version.
    Also accepts v4 already stamped (from a previously failed run) — will reset to v3 then re-migrate.
    """
    v = state["current_version"]
    if v is None:
        print(f"\n  ERROR: schema_meta table missing or has no schema_version.")
        print(f"         Cannot migrate safely — inspect DB manually.")
        sys.exit(1)
    if v == SCHEMA_VERSION_NEW:
        print(f"\n  NOTE: schema_version already {SCHEMA_VERSION_NEW} — likely from a previously failed run.")
        print(f"        Will reset to {SCHEMA_VERSION_OLD} and re-migrate cleanly.")
        return   # allowed — reset happens inside transaction
    if v != SCHEMA_VERSION_OLD:
        print(f"\n  ERROR: Expected schema_version {SCHEMA_VERSION_OLD}, found {v}.")
        print(f"         This script migrates {SCHEMA_VERSION_OLD} → {SCHEMA_VERSION_NEW} only.")
        sys.exit(1)


def reset_schema_version_if_needed(conn: sqlite3.Connection, state: dict, dry_run: bool) -> None:
    """If version was pre-stamped to v4 by a failed prior run, reset it to v3 before migrating."""
    if state["current_version"] == SCHEMA_VERSION_NEW:
        sql = "UPDATE schema_meta SET value = ? WHERE key = 'schema_version'"
        log(f"{'[DRY RUN] ' if dry_run else ''}Resetting schema_version {SCHEMA_VERSION_NEW} → {SCHEMA_VERSION_OLD} (prior failed run)")
        if not dry_run:
            conn.execute(sql, (str(SCHEMA_VERSION_OLD),))


def migrate_schema_version(conn: sqlite3.Connection, dry_run: bool) -> None:
    section(f"schema_meta  (schema_version {SCHEMA_VERSION_OLD} → {SCHEMA_VERSION_NEW})")
    sql = "UPDATE schema_meta SET value = ? WHERE key = 'schema_version'"
    log(f"{'[DRY RUN] ' if dry_run else ''}UPDATE schema_meta SET value = '{SCHEMA_VERSION_NEW}' WHERE key = 'schema_version'")
    if not dry_run:
        conn.execute(sql, (str(SCHEMA_VERSION_NEW),))
        log(f"schema_version updated to {SCHEMA_VERSION_NEW} ✓")


# ── emc_storage ────────────────────────────────────────────────────────────────

def migrate_storage(conn: sqlite3.Connection, state: dict, dry_run: bool) -> None:
    section("emc_storage  (content→trace, drop memory_strength, drop novelty_score)")

    if state["storage_done"]:
        log("Already migrated — skipping.")
        return

    if "content" not in state["storage_cols"]:
        log("WARNING: `content` not found in emc_storage — inspect manually.")
        return

    DROP_COLS   = {"memory_strength", "novelty_score"}
    RENAME_COLS = {"content": "trace"}

    pragma       = conn.execute("PRAGMA table_info(emc_storage)").fetchall()
    new_col_defs = []
    select_exprs = []

    for _, name, typ, notnull, dflt, pk in pragma:
        if name in DROP_COLS:
            continue
        new_name = RENAME_COLS.get(name, name)
        if pk:
            new_col_defs.append(f'"{new_name}" INTEGER PRIMARY KEY AUTOINCREMENT')
        else:
            parts = [f'"{new_name}" {typ}']
            if notnull:
                parts.append("NOT NULL")
            if dflt is not None:
                parts.append(f"DEFAULT {_safe_default(dflt, typ)}")
            new_col_defs.append(" ".join(parts))
        select_exprs.append(f'"{name}"')

    stmts = [
        ("CREATE TABLE emc_storage_new",
         "CREATE TABLE emc_storage_new (\n    " + ",\n    ".join(new_col_defs) + "\n)"),
        ("INSERT INTO emc_storage_new … SELECT FROM emc_storage",
         f"INSERT INTO emc_storage_new SELECT {', '.join(select_exprs)} FROM emc_storage"),
        ("DROP TABLE emc_storage",
         "DROP TABLE emc_storage"),
        ("ALTER TABLE emc_storage_new RENAME TO emc_storage",
         "ALTER TABLE emc_storage_new RENAME TO emc_storage"),
    ]

    # Print full CREATE TABLE DDL so syntax errors are visible
    log(f"Full DDL:\n{stmts[0][1]}")

    for label, sql in stmts:
        log(f"{'[DRY RUN] ' if dry_run else ''}{label}")
        if not dry_run:
            conn.execute(sql)

    if not dry_run:
        log("emc_storage migrated ✓")


# ── reformat trace data ────────────────────────────────────────────────────────

def reformat_trace_data(conn: sqlite3.Connection, dry_run: bool) -> None:
    """
    Reformat emc_storage.trace from old JSON content to new plain-text format.

    Old format (content column, stored as JSON):
        {"user": "...", "assistant": "..."}

    New format (trace column, plain text):
        {user_id} said: "{user_prompt}"
        You replied: "{ai_response}"

    Rows that are already plain text (not valid JSON) are left unchanged —
    idempotent: safe to re-run if migration was interrupted mid-batch.
    Must run AFTER migrate_storage and BEFORE rebuild_fts — FTS indexes
    the reformatted trace, not the raw JSON.
    """
    section("emc_storage  (reformat trace: JSON → plain text)")

    if dry_run:
        log("[DRY RUN] SELECT id, user_id, trace FROM emc_storage → reformat each row")
        return

    rows = conn.execute("SELECT id, user_id, trace FROM emc_storage").fetchall()
    reformatted = 0
    skipped     = 0

    for row in rows:
        raw = row["trace"]
        try:
            parsed = json.loads(raw)                                        # attempt JSON parse — old format
            user_turn      = parsed.get("user", "")
            assistant_turn = parsed.get("assistant", "")
            new_trace = (
                f'{row["user_id"]} said: "{user_turn}"\n'
                f'You replied: "{assistant_turn}"'
            )
            conn.execute(
                "UPDATE emc_storage SET trace = ? WHERE id = ?",
                (new_trace, row["id"]),
            )
            reformatted += 1
        except (json.JSONDecodeError, KeyError):
            skipped += 1                                                    # already plain text or malformed — leave as-is

    log(f"Reformatted {reformatted} row(s), skipped {skipped} (already plain text) ✓")


# ── emc_staging ────────────────────────────────────────────────────────────────

def migrate_staging(conn: sqlite3.Connection, state: dict, dry_run: bool) -> None:
    section("emc_staging  (content→trace)")

    if state["staging_done"]:
        log("Already migrated — skipping.")
        return

    if "content" not in state["staging_cols"]:
        log("WARNING: `content` not found in emc_staging — inspect manually.")
        return

    RENAME_COLS  = {"content": "trace"}
    pragma       = conn.execute("PRAGMA table_info(emc_staging)").fetchall()
    new_col_defs = []
    select_exprs = []

    for _, name, typ, notnull, dflt, pk in pragma:
        new_name = RENAME_COLS.get(name, name)
        if pk:
            new_col_defs.append(f'"{new_name}" INTEGER PRIMARY KEY AUTOINCREMENT')
        else:
            parts = [f'"{new_name}" {typ}']
            if notnull:
                parts.append("NOT NULL")
            if dflt is not None:
                parts.append(f"DEFAULT {_safe_default(dflt, typ)}")
            new_col_defs.append(" ".join(parts))
        select_exprs.append(f'"{name}"')

    stmts = [
        ("CREATE TABLE emc_staging_new",
         "CREATE TABLE emc_staging_new (\n    " + ",\n    ".join(new_col_defs) + "\n)"),
        ("INSERT INTO emc_staging_new … SELECT FROM emc_staging",
         f"INSERT INTO emc_staging_new SELECT {', '.join(select_exprs)} FROM emc_staging"),
        ("DROP TABLE emc_staging",
         "DROP TABLE emc_staging"),
        ("ALTER TABLE emc_staging_new RENAME TO emc_staging",
         "ALTER TABLE emc_staging_new RENAME TO emc_staging"),
    ]

    for label, sql in stmts:
        log(f"{'[DRY RUN] ' if dry_run else ''}{label}")
        if not dry_run:
            conn.execute(sql)

    if not dry_run:
        log("emc_staging migrated ✓")


# ── emc_lexical FTS5 ───────────────────────────────────────────────────────────

def rebuild_fts(conn: sqlite3.Connection, state: dict, dry_run: bool) -> None:
    section("emc_lexical  (FTS5 rebuild — index `trace` instead of `content`)")

    if state["fts_done"]:
        log("Already indexes `trace` — skipping.")
        return

    if not state["fts"]:
        log("emc_lexical not found — skipping.")
        return

    stmts = [
        ("DROP TABLE IF EXISTS emc_lexical",
         "DROP TABLE IF EXISTS emc_lexical"),
        ("CREATE VIRTUAL TABLE emc_lexical USING fts5(trace, …)",
         ("CREATE VIRTUAL TABLE emc_lexical USING fts5(\n"
          "    trace,\n"
          "    content='emc_storage',\n"
          "    content_rowid='id',\n"
          "    tokenize='porter ascii'\n"
          ")")),
        ("INSERT INTO emc_lexical(rowid, trace) SELECT id, trace FROM emc_storage",
         "INSERT INTO emc_lexical(rowid, trace) SELECT id, trace FROM emc_storage"),
        ("INSERT INTO emc_lexical(emc_lexical) VALUES('optimize')",
         "INSERT INTO emc_lexical(emc_lexical) VALUES('optimize')"),
    ]

    for label, sql in stmts:
        log(f"{'[DRY RUN] ' if dry_run else ''}{label}")
        if not dry_run:
            conn.execute(sql)

    if not dry_run:
        log("emc_lexical rebuilt ✓")


# ── B-tree index guard ─────────────────────────────────────────────────────────

def ensure_btree_index(conn: sqlite3.Connection, dry_run: bool) -> None:
    section("B-tree index  (user_id, timestamp)")
    sql = "CREATE INDEX IF NOT EXISTS emc_storage_user_timestamp ON emc_storage (user_id, timestamp)"
    log(f"{'[DRY RUN] ' if dry_run else ''}CREATE INDEX IF NOT EXISTS emc_storage_user_timestamp …")
    if not dry_run:
        conn.execute(sql)
        log("B-tree index ensured ✓")


# ── Validation ─────────────────────────────────────────────────────────────────

def validate(conn: sqlite3.Connection, state: dict) -> bool:
    section("Validation")
    ok = True

    # Schema version
    row = conn.execute("SELECT value FROM schema_meta WHERE key = 'schema_version'").fetchone()
    actual_version = int(row[0]) if row else None
    if actual_version == SCHEMA_VERSION_NEW:
        log(f"schema_version = {actual_version} ✓")
    else:
        log(f"ERROR: schema_version = {actual_version}, expected {SCHEMA_VERSION_NEW}!")
        ok = False

    def check_count(label, before):
        nonlocal ok
        after = row_count(conn, label)
        if before == after:
            log(f"{label}: {after} rows ✓")
        else:
            log(f"ERROR {label}: {before} rows before → {after} after — mismatch!")
            ok = False

    check_count("emc_storage", state["storage_count_before"])
    check_count("emc_staging",  state["staging_count_before"])

    new_storage = col_names(conn, "emc_storage")
    new_staging = col_names(conn, "emc_staging")

    for expected, present, table in (
        ("trace",   True,  "emc_storage", ),
        ("trace",   True,  "emc_staging"),
        ("content", False, "emc_storage"),
        ("content", False, "emc_staging"),
        ("memory_strength", False, "emc_storage"),
        ("novelty_score",   False, "emc_storage"),
    ):
        cols = new_storage if table == "emc_storage" else new_staging
        found = expected in cols
        if found == present:
            state_str = "present" if present else "absent"
            log(f"`{expected}` correctly {state_str} in {table} ✓")
        else:
            state_str = "missing" if present else "still present"
            log(f"ERROR: `{expected}` {state_str} in {table}!")
            ok = False

    return ok


# ── Main ───────────────────────────────────────────────────────────────────────

def main() -> None:
    args    = parse_args()
    db_path = Path(args.db)

    print(f"\n{'═' * 60}")
    print(f"  AuRoRA · EMC schema migration")
    print(f"  DB   : {db_path.resolve()}")
    print(f"  Mode : {'DRY RUN — no writes' if args.dry_run else 'LIVE'}")
    print(f"{'═' * 60}")

    if not db_path.exists():
        print(f"\nERROR: {db_path} not found.")
        sys.exit(1)

    if not args.no_backup and not args.dry_run:
        backup(db_path)

    conn = sqlite3.connect(str(db_path))
    conn.row_factory = sqlite3.Row

    try:
        state = preflight(conn)

        section("Pre-flight checks")
        log(f"schema_version : {state['current_version']}")
        log(f"emc_storage    : {state['storage_count_before']} rows")
        log(f"emc_staging    : {state['staging_count_before']} rows")

        check_schema_version(state)  # abort if not v3

        if not args.dry_run:
            conn.execute("PRAGMA journal_mode=WAL")
            conn.execute("BEGIN")

        try:
            reset_schema_version_if_needed(conn, state, args.dry_run)  # fix pre-stamped v4 from failed run
            migrate_schema_version(conn, args.dry_run)
            migrate_storage(conn, state, args.dry_run)
            reformat_trace_data(conn, args.dry_run)     # must run after storage rebuild, before FTS
            migrate_staging(conn, state, args.dry_run)
            rebuild_fts(conn, state, args.dry_run)
            ensure_btree_index(conn, args.dry_run)

            if not args.dry_run:
                conn.execute("COMMIT")
                log("\nTransaction committed ✓")

        except Exception as e:
            if not args.dry_run:
                conn.execute("ROLLBACK")
            print(f"\n  ERROR — rolled back: {e}")
            raise

        if not args.dry_run:
            ok = validate(conn, state)
            result = conn.execute("PRAGMA integrity_check").fetchone()[0]
            log(f"integrity_check: {result} ✓" if result == "ok" else f"WARNING: {result}")
            if not ok:
                print("\n  WARNING: validation issues detected — review above.")
                sys.exit(2)

        print(f"\n{'═' * 60}")
        print(f"  {'DRY RUN complete — no changes made.' if args.dry_run else 'Migration complete ✓'}")
        print(f"{'═' * 60}\n")

    finally:
        conn.close()


if __name__ == "__main__":
    main()