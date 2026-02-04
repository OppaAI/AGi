import sqlite3
import json
from pathlib import Path
from datetime import datetime

DB_PATH = "./logs/ledger.db"

def query_anomalies(limit=20, level_filter=None, system_filter=None):
    """
    Query the ledger database with optional filters.
    
    Args:
        limit: Number of records to return
        level_filter: Filter by level (e.g., "ERROR", "CRITICAL")
        system_filter: Filter by system (e.g., "VCS", "SCS")
    """
    if not Path(DB_PATH).exists():
        print("No ledger found. The robot hasn't done anything yet!")
        return
    
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    
    # Build query with optional filters
    query = "SELECT correlation_id, timestamp, level, proc_id, system, channels, message_preview FROM events"
    conditions = []
    params = []
    
    if level_filter:
        conditions.append("level = ?")
        params.append(level_filter)
    
    if system_filter:
        conditions.append("system = ?")
        params.append(system_filter)
    
    if conditions:
        query += " WHERE " + " AND ".join(conditions)
    
    query += " ORDER BY timestamp DESC LIMIT ?"
    params.append(limit)
    
    try:
        cursor.execute(query, params)
        rows = cursor.fetchall()
        
        if not rows:
            print(f"No events found matching the criteria.")
            return
        
        # Header
        print(f"{'CORR_ID':<12} | {'TIMESTAMP':<20} | {'LEVEL':<10} | {'PROC_ID':<20} | {'CHANNELS':<35} | {'MESSAGE'}")
        print("-" * 150)
        
        for row in rows:
            corr_id, ts, lvl, proc_id, system, channels_json, msg = row
            
            # Truncate correlation_id for display
            corr_short = corr_id[:8] if corr_id else "N/A"
            
            # Format timestamp (truncate for readability)
            ts_short = ts[:19] if ts and len(ts) > 19 else ts or "N/A"
            
            # Parse channels JSON
            try:
                channels_list = json.loads(channels_json) if channels_json else []
                channels_str = ",".join(channels_list)[:33]  # Truncate if too long
            except (json.JSONDecodeError, TypeError):
                channels_str = "N/A"
            
            # Truncate message if too long
            msg_display = (msg[:60] + "...") if msg and len(msg) > 60 else (msg or "N/A")
            
            # Color coding based on level
            level_colored = colorize_level(lvl)
            
            print(f"{corr_short:<12} | {ts_short:<20} | {level_colored:<16} | {proc_id:<20} | {channels_str:<35} | {msg_display}")
        
        # Summary stats
        print("\n" + "=" * 150)
        print_summary(cursor, level_filter, system_filter)
        
    except sqlite3.OperationalError as e:
        print(f"Ledger table doesn't exist yet or schema mismatch: {e}")
        print("Did the EEE system initialize the ledger?")
    finally:
        conn.close()


def print_summary(cursor, level_filter=None, system_filter=None):
    """Print summary statistics from the ledger."""
    
    # Count by level
    query = "SELECT level, COUNT(*) FROM events"
    conditions = []
    params = []
    
    if level_filter:
        conditions.append("level = ?")
        params.append(level_filter)
    
    if system_filter:
        conditions.append("system = ?")
        params.append(system_filter)
    
    if conditions:
        query += " WHERE " + " AND ".join(conditions)
    
    query += " GROUP BY level ORDER BY COUNT(*) DESC"
    
    cursor.execute(query, params)
    level_counts = cursor.fetchall()
    
    print("📊 SUMMARY:")
    for level, count in level_counts:
        level_colored = colorize_level(level)
        print(f"  {level_colored:<16}: {count}")
    
    # Total events
    total_query = "SELECT COUNT(*) FROM events"
    if conditions:
        total_query += " WHERE " + " AND ".join(conditions)
    cursor.execute(total_query, params)
    total = cursor.fetchone()[0]
    print(f"  {'TOTAL':<10}: {total}")


def colorize_level(level):
    """Add ANSI color codes to level for terminal output."""
    colors = {
        "CRITICAL": "\033[1;31m",  # Bold Red
        "ERROR": "\033[31m",       # Red
        "WARNING": "\033[33m",     # Yellow
        "INFO": "\033[32m",        # Green
        "DEBUG": "\033[90m",       # Gray
    }
    reset = "\033[0m"
    
    color = colors.get(level.upper(), "")
    return f"{color}{level:<8}{reset}" if color else f"{level:<8}"


def query_by_correlation(correlation_id):
    """Find all instances of an event across channels by correlation ID."""
    if not Path(DB_PATH).exists():
        print("No ledger found.")
        return
    
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    
    query = """
        SELECT timestamp, level, proc_id, system, subsystem, module, 
               channels, message_preview, content_hash
        FROM events 
        WHERE correlation_id LIKE ?
    """
    
    # Allow partial correlation ID (just first 8 chars)
    search_pattern = f"{correlation_id}%"
    cursor.execute(query, (search_pattern,))
    row = cursor.fetchone()
    
    if row:
        ts, lvl, proc_id, system, subsystem, module, channels_json, msg, content_hash = row
        channels = json.loads(channels_json) if channels_json else []
        
        print(f"\n🔍 Event Details (Correlation ID: {correlation_id})")
        print("=" * 80)
        print(f"Timestamp:   {ts}")
        print(f"Level:       {colorize_level(lvl)}")
        print(f"Proc ID:     {proc_id}")
        print(f"  System:    {system}")
        print(f"  Subsystem: {subsystem or 'N/A'}")
        print(f"  Module:    {module or 'N/A'}")
        print(f"Message:     {msg}")
        print(f"Content Hash: {content_hash[:16]}...")
        print(f"Routed to:   {', '.join(channels)}")
        print("\n💡 To view full details in other channels:")
        print(f"  activity.log: grep '{correlation_id[:8]}' logs/activity.log")
        print(f"  anomaly.jsonl: grep '{correlation_id}' logs/anomaly.jsonl | jq")
    else:
        print(f"No event found with correlation ID starting with: {correlation_id}")
    
    conn.close()


def check_duplicates(time_window=5):
    """Find duplicate events within a time window (for debugging deduplication)."""
    if not Path(DB_PATH).exists():
        print("No ledger found.")
        return
    
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    
    query = """
        SELECT content_hash, COUNT(*) as count, 
               GROUP_CONCAT(correlation_id) as corr_ids,
               GROUP_CONCAT(level) as levels,
               MIN(timestamp) as first_seen,
               MAX(timestamp) as last_seen,
               message_preview
        FROM events
        GROUP BY content_hash
        HAVING count > 1
        ORDER BY count DESC
    """
    
    cursor.execute(query)
    duplicates = cursor.fetchall()
    
    if duplicates:
        print(f"\n⚠️  Found {len(duplicates)} duplicate event patterns:")
        print("-" * 80)
        for hash_val, count, corr_ids, levels, first, last, msg in duplicates:
            print(f"Hash: {hash_val[:16]}... (appeared {count} times)")
            print(f"  Message: {msg}")
            print(f"  First: {first}")
            print(f"  Last:  {last}")
            print(f"  Levels: {levels}")
            print(f"  IDs: {corr_ids}")
            print()
    else:
        print("✅ No duplicates found - deduplication is working!")
    
    conn.close()


def show_recent_errors(minutes=60):
    """Show errors from the last N minutes."""
    if not Path(DB_PATH).exists():
        print("No ledger found.")
        return
    
    from datetime import datetime, timedelta
    
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    
    # Calculate cutoff time
    cutoff = (datetime.now() - timedelta(minutes=minutes)).isoformat()
    
    query = """
        SELECT correlation_id, timestamp, level, proc_id, message_preview
        FROM events
        WHERE level IN ('ERROR', 'CRITICAL') AND timestamp > ?
        ORDER BY timestamp DESC
    """
    
    cursor.execute(query, (cutoff,))
    rows = cursor.fetchall()
    
    if rows:
        print(f"\n🚨 Errors in the last {minutes} minutes ({len(rows)} found):")
        print("-" * 100)
        for corr_id, ts, lvl, proc_id, msg in rows:
            level_colored = colorize_level(lvl)
            print(f"{ts[:19]} | {level_colored:<16} | {proc_id:<20} | {msg}")
    else:
        print(f"✅ No errors in the last {minutes} minutes")
    
    conn.close()


if __name__ == "__main__":
    import sys
    
    # Example usage with command-line arguments
    if len(sys.argv) > 1:
        cmd = sys.argv[1].lower()
        
        if cmd == "errors":
            print("🔴 Showing only ERROR and CRITICAL events:\n")
            query_anomalies(limit=50, level_filter="ERROR")
            print("\n")
            query_anomalies(limit=50, level_filter="CRITICAL")
        
        elif cmd == "vcs":
            print("🤖 Showing only VCS system events:\n")
            query_anomalies(limit=50, system_filter="VCS")
        
        elif cmd == "scs":
            print("🛠️  Showing only SCS system events:\n")
            query_anomalies(limit=50, system_filter="SCS")
        
        elif cmd == "dupes":
            check_duplicates()
        
        elif cmd == "recent":
            minutes = int(sys.argv[2]) if len(sys.argv) > 2 else 60
            show_recent_errors(minutes)
        
        elif cmd.startswith("corr:"):
            corr_id = cmd.split(":", 1)[1]
            query_by_correlation(corr_id)
        
        else:
            print("📖 Usage:")
            print("  python read_ledger_db.py              # Show last 20 events")
            print("  python read_ledger_db.py errors       # Show errors/criticals only")
            print("  python read_ledger_db.py vcs          # Show VCS system events")
            print("  python read_ledger_db.py scs          # Show SCS system events")
            print("  python read_ledger_db.py dupes        # Check for duplicates")
            print("  python read_ledger_db.py recent [60]  # Errors in last N minutes")
            print("  python read_ledger_db.py corr:UUID    # Find by correlation ID")
    else:
        # Default: show last 20 events
        query_anomalies()