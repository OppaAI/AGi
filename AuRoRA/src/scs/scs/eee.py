"""
EEE (Emergency and Exception Event) Module

Welcome to my Emergency and Exception Event (EEE) Module!
This module is designed to handle, broadcast, and record emergency and exception events across the system,
providing a unified and centralized interface for alerting my other systems of any anomalies, and ensuring
that they are addressed promptly and effectively, thus maintaining system stability.
"""

# System modules
import atexit
from collections import deque
from datetime import datetime
import gzip
import hashlib
import json
import logging
from pathlib import Path
import queue
import shutil
import signal
import sqlite3
import threading
import time
import traceback
import uuid

# AGi modules
from scs.sdv import GzipRotatingFileHandler

# (TODO) Later make this configurable via config file or environment variable (e.g. /var/log/agisys)
LOG_PATH = "./logs"  # Location of log files
LEDGER_DB_FILE = "ledger.db"  # Ledger database file name
MASTER_LOG_FILE = "activity.log"  # Master log file name (40% of the logs)
ERROR_LOG_FILE = "anomaly.jsonl"   # Error log file name (10% of the logs)

class RelayNodeLimiter:
    """
    Token bucket algorithm to limit the message flow rate relaying through the node.
    Prevents message floods while allowing bursts of critical events.
    
    Example:
        throttle = RelayNodeLimiter(rate=1.0, burst=3)
        if throttle.allow():
            # Process event
        else:
            # Drop event (throttled)
    """
    
    def __init__(self, rate: float = 1.0, burst: int = 3) -> None:
        """
        Initialize token bucket.
        
        Args:
            rate: Tokens refilled per second (Hz)
            burst: Maximum token capacity (burst size)
        """
        self.rate: float = rate                         # The rate it allows the message to flow
        self.burst: int = burst                         # The capacity it allows the message to burst out
        self.tokens: float = burst                      # Start with full bucket
        self.last_update: float = time.monotonic()
        self._lock: threading.Lock = threading.Lock()
    
    def allow(self) -> bool:
        """
        Check if event is allowed (has tokens available).
        
        Returns:
            True if event should be processed, False if throttled
        """
        with self._lock:
            now: float = time.monotonic()
            elapsed: float = now - self.last_update
            
            # Refill tokens based on elapsed time
            self.tokens: float = min(self.burst, self.tokens + elapsed * self.rate)
            self.last_update: float = now
            
            # Check if we have at least 1 token
            if self.tokens >= 1.0:
                self.tokens: float = self.tokens - 1.0
                return True
            
            return False
    
    def get_status(self) -> dict:
        """
        Get current throttle status (for debugging).
        """
        with self._lock:
            return {
                "tokens": round(self.tokens, 2),
                "rate": self.rate,
                "burst": self.burst
            }

class EEEAggregator:
    """
    This is my centralized Emergency and Exception Event (EEE) Aggregator module, which collects and manages all emergency
    and exception events across the system. It provides a unified interface for logging, alerting, and handling critical
    events, ensuring that they are addressed promptly and effectively.

    Each event entry goes through the following process to ensure standardized logging:
    1. Ingestion: Event is ingested from various subsystems/modules by dispatching loggers to the appropriate logger.
    2. Filtration: Event is then filtered to restructure into a standardized format for uniformity.
    3. Propagation: Event is propagated to multiple loggers/handlers based on severity level (by TALLE).
        
    Usage:
        eee = EEEAggregator(caller_id="VCS.VTC")
        eee.info("Hello, World!")
        eee.critical("RED ALERT! My core is overheating!")
        
    That's all you need to do to log messages from anywhere in the system.
    It automatically sets up logging on first use, and creates log files as needed.
    """
    
    # Class-level flags to ensure setup happens only once
    _initialized: bool = False
    _log_queue: queue.Queue = queue.Queue(maxsize=10000)  # Bounded queue
    _dropped_events: int = 0  # Counter for dropped events
    _worker_started: bool = False  # Worker thread started flag
    _thread_lock: threading.Lock = threading.Lock()  # Thread lock
    _shutdown_registered: bool = False  # Shutdown registered flag
    _plugins: dict = {}  # Plugin dictionary
    _ledger_queue: deque = deque()  # Ledger queue
    _ledger_flush_timer: threading.Timer | None = None
    _ledger_lock = threading.Lock()
    _shutting_down: bool = False
    _shutdown_complete: bool = False  # ← add with other class variables
    _db_conn: sqlite3.Connection | None = None  # Single shared DB connection
    BATCH_SIZE = 50
    FLUSH_INTERVAL = 0.5  # seconds

    _throttles: dict = {}  # Per-proc_id throttles
    _throttle_config = {
        "enabled": True,
        "default_rate": 1.0,   # 1 event per second
        "burst_size": 3,       # Allow 3-event bursts
        "critical_bypass": True  # CRITICAL level bypasses throttling
    }

    def __init__(self, caller_id: str):
        """
        This is where the logger is created and initialized.
        It auto-initializes the logging system on first use, including root and subsystem loggers, and
        dynamically registers the subsystem logger if not already done.
        
        Args:
            caller_id: The system/node/module which is calling this logger (e.g. "VCS.VTC.Pump")
                    The system name is derived from the caller_id by taking the first element of the split.
                    eg. "VCS.VTC.Pump" -> "VCS"
                    If no caller_id is provided, caller_id defaults to "SYSTEM.UNKNOWN".
        """
        
        # Ensure caller_id is valid
        if not caller_id or not isinstance(caller_id, str):
            caller_id = "SYSTEM.UNKNOWN"
        
        # Create the logging system if not already initialized
        if not EEEAggregator._initialized:
            with EEEAggregator._thread_lock:
                if not EEEAggregator._initialized:  # Double-check
                    EEEAggregator.Propagator.engage_TALLE_to_propagate()
                    EEEAggregator._register_shutdown_handlers()
                    EEEAggregator._initialized = True

        # Establish the specific logger for the caller
        # This logger will have NO handlers of its own, and relies 100% on propagation
        self.logger = logging.getLogger(caller_id)

        # Start the "Postman" thread if it's not running
        if not EEEAggregator._worker_started:
            with EEEAggregator._thread_lock:
                if not EEEAggregator._worker_started:
                    threading.Thread(target=self._async_worker, daemon=True).start()
                    EEEAggregator._worker_started = True

    def _async_worker(self):
        """
        This is the ONLY thread that touches the disk.
        """
        log_dir = Path(LOG_PATH)
        log_dir.mkdir(parents=True, exist_ok=True)

        # ✅ Create handlers ONCE before the loop
        master_handler = GzipRotatingFileHandler(
            log_dir / MASTER_LOG_FILE, maxBytes=5*1024*1024, backupCount=5, encoding='utf-8'
        )
        master_handler.setFormatter(EEEAggregator.Filtrator.pass_thru_EEE_filters())
        
        anomaly_handler = GzipRotatingFileHandler(
            log_dir / ERROR_LOG_FILE, maxBytes=2*1024*1024, backupCount=5, encoding='utf-8'
        )
        anomaly_handler.setFormatter(logging.Formatter('%(message)s'))

        # ✅ THE LEDGER: Initialize SQLite ONCE - single persistent connection
        EEEAggregator._db_conn = sqlite3.connect(log_dir / LEDGER_DB_FILE, check_same_thread=False)
        EEEAggregator._db_conn.execute("PRAGMA journal_mode=WAL;")  # Crash-safe, concurrent
        EEEAggregator._db_conn.execute("PRAGMA synchronous=NORMAL;")  # Balanced safety/speed (still durable)
        EEEAggregator._db_conn.execute("PRAGMA wal_autocheckpoint=100;")  # Auto-checkpoint every 100 pages
        EEEAggregator._db_conn.execute("""
            CREATE TABLE IF NOT EXISTS events (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                correlation_id TEXT NOT NULL UNIQUE,
                timestamp TEXT NOT NULL,
                level TEXT NOT NULL,
                proc_id TEXT NOT NULL,
                system TEXT,
                node TEXT,
                module TEXT,
                message_preview,
                channels TEXT,
                content_hash TEXT,
                created_at REAL DEFAULT (julianday('now'))
            )
        """)
        EEEAggregator._db_conn.execute("CREATE INDEX IF NOT EXISTS idx_correlation_id ON events(correlation_id)")
        EEEAggregator._db_conn.execute("CREATE INDEX IF NOT EXISTS idx_timestamp ON events(timestamp)")
        EEEAggregator._db_conn.execute("CREATE INDEX IF NOT EXISTS idx_level ON events(level)")
        EEEAggregator._db_conn.execute("CREATE INDEX IF NOT EXISTS idx_system ON events(system)")
        EEEAggregator._db_conn.execute("CREATE INDEX IF NOT EXISTS idx_content_hash ON events(content_hash)")
        EEEAggregator._db_conn.commit()

        # Start periodic flush timer
        def _start_ledger_flush():
            with EEEAggregator._ledger_lock:
                if EEEAggregator._ledger_flush_timer is None:
                    def flush_loop():
                        if EEEAggregator._shutting_down:
                            return
                        EEEAggregator._flush_ledger_batch()
                        EEEAggregator._ledger_flush_timer = threading.Timer(
                            EEEAggregator.FLUSH_INTERVAL, flush_loop
                        )
                        EEEAggregator._ledger_flush_timer.daemon = True
                        EEEAggregator._ledger_flush_timer.start()

        _start_ledger_flush()

        def _start_ledger_backup():
            def backup_loop():
                log_dir = Path(LOG_PATH)
                db_path = log_dir / LEDGER_DB_FILE
                if db_path.exists() and db_path.stat().st_size > 50 * 1024 * 1024:  # 50MB threshold
                    backup_path = db_path.with_suffix(f'.backup.{int(time.time())}.gz')
                    try:
                        with open(db_path, 'rb') as f_in:
                            with gzip.open(backup_path, 'wb') as f_out:
                                shutil.copyfileobj(f_in, f_out)
                        print(f"[EEE] Ledger backed up to {backup_path.name}")
                        # Optional: VACUUM to compact (safe) - use separate connection
                        temp_conn = sqlite3.connect(db_path)
                        temp_conn.execute("VACUUM;")
                        temp_conn.close()
                    except Exception as e:
                        print(f"[EEE] Ledger backup failed: {e}")
                # Check hourly
                threading.Timer(3600, backup_loop).start()
            backup_loop()

        _start_ledger_backup()

        text_formatter = EEEAggregator.Filtrator.pass_thru_EEE_filters()

        # Deduplication cache: content_hash -> (timestamp, correlation_id)
        recent_hashes = {}
        DEDUP_WINDOW = 5.0  # 5 seconds
        
        # Throttling stats (for monitoring)
        throttled_count = 0

        system_handlers = {}  # proc_id -> handler (create on first use)

        while True:
            # Wait for a log entry
            timestamp, level, caller, msg, evidence = EEEAggregator._log_queue.get()

            # Generate a unique correlation ID for this event
            correlation_id = str(uuid.uuid4())
            
            # Parse the caller ID into its components
            parts = caller.split('.')
            system = parts[0].upper() if len(parts) > 0 else "SYSTEM"
            node = parts[1].upper() if len(parts) > 1 else "UNKNOWN"
            module = parts[2].upper() if len(parts) > 2 else "UNKNOWN"
            proc_id = "::".join([p.upper() for p in parts])
            
            # Get the level name
            # Starting Python 3.11, level_map = logging.getLevelNamesMapping().get(level, f"LVL-{level}")
            lvl_name = logging.getLevelName(level)

            # Create content hash for deduplication
            content_hash = hashlib.sha256(f"{caller}:{msg}".encode()).hexdigest()
            
            # Deduplication check
            now = time.time()
            if content_hash in recent_hashes:
                last_seen, last_corr_id = recent_hashes[content_hash]
                if now - last_seen < DEDUP_WINDOW:
                    # Skip this event
                    EEEAggregator._log_queue.task_done()
                    continue

            # Update deduplication cache
            recent_hashes[content_hash] = (now, correlation_id)

            # Clean old entries from cache, keep recent 5 seconds (dedup window)
            recent_hashes = {h: (t, c) for h, (t, c) in recent_hashes.items() if now - t <= DEDUP_WINDOW}
            
            # Throttle check
            should_throttle = False
            
            if EEEAggregator._throttle_config["enabled"]:
                # CRITICAL level bypasses throttling
                if level >= logging.CRITICAL and EEEAggregator._throttle_config["critical_bypass"]:
                    should_throttle = False
                else:
                    # Get or create throttle for this proc_id
                    if proc_id not in EEEAggregator._throttles:
                        EEEAggregator._throttles[proc_id] = RelayNodeLimiter(
                            rate=EEEAggregator._throttle_config["default_rate"],
                            burst=EEEAggregator._throttle_config["burst_size"]
                        )
                    
                    throttle = EEEAggregator._throttles[proc_id]
                    
                    # Check if allowed
                    if not throttle.allow():
                        should_throttle = True
                        throttled_count += 1
                        
                        # Every 100 throttled events, log a warning
                        if throttled_count % 100 == 0:
                            print(f"[EEE] Throttled {throttled_count} events so far. Proc: {proc_id}")
            
            # Skip event if throttled
            if should_throttle:
                EEEAggregator._log_queue.task_done()
                continue
            
            # Truncate message for ledger preview
            message_preview = msg[:80] + "..." if len(msg) > 80 else msg

            # Determine which channels this event should propagate to
            channels = ["awareness", "ledger", "master_log", "system_log"]
            if level >= logging.WARNING:
                channels.append("error_log")
            
            # Create LogRecord to reuse the existing ISO 8601 formatter
            record = logging.LogRecord(caller, level, "", 0, msg, None, None)
            record.created = timestamp
            record.msecs = (timestamp - int(timestamp)) * 1000
            record.name = proc_id  # Already formatted as SYSTEM::NODE::MODULE

            # Generate ISO 8601 timestamp using the custom formatter
            iso_timestamp = text_formatter.formatTime(record)

            # Generate plain text line using the custom formatter
            plain_text_line = text_formatter.format(record)

            # Add correlation ID to the plain text line
            parts_split = plain_text_line.split('|', 1)
            plain_text_line = f"{parts_split[0]} | CID:{correlation_id[:8]} | {parts_split[1]}"
            if evidence:
                plain_text_line += f" | DATA: {json.dumps(evidence)}"

            # Call plugins BEFORE disk writes
            skip_disk = False
            for plugin_name, callback in list(EEEAggregator._plugins.items()):  # Use list() to avoid dict modification during iteration
                try:
                    if callback(level, msg, evidence or {}, {
                        "correlation_id": correlation_id,
                        "proc_id": proc_id,
                        "system": system,
                        "node": node,
                        "module": module
                    }):
                        # Plugin handled it - skip disk for non-critical
                        if level < logging.CRITICAL:
                            skip_disk = True
                            break  # Exit plugin loop early
                except Exception as e:
                    # Use print instead of self.logger to avoid recursion
                    print(f"[EEE] Plugin {plugin_name} crashed: {e}")
                    traceback.print_exc()
                    # Auto-disable broken plugin:
                    EEEAggregator._plugins.pop(plugin_name, None)
                    print(f"[EEE] Plugin {plugin_name} DISABLED")

            # Skip disk writes if plugin handled it
            if skip_disk:
                EEEAggregator._log_queue.task_done()
                continue  # ✅ Now this continues the while True loop

            try:
                # 📦 ARCHIVE A: MASTER LOG (Post-mortem)
                master_handler.emit(record)

                # 🔧 ARCHIVE B: SYSTEM LOG (Subsystem isolation)
                system_prefix = caller.split('.')[0].lower()
                if system_prefix not in system_handlers:
                    system_handlers[system_prefix] = GzipRotatingFileHandler(
                        log_dir / f"{system_prefix}.log", maxBytes=3*1024*1024, backupCount=3, encoding='utf-8'
                    )
                    system_handlers[system_prefix].setFormatter(text_formatter)
                system_handlers[system_prefix].emit(record)  # Use same formatted record

                # 🔍 ARCHIVE C: ANOMALY VAULT (JSONL for Cloud/ML)
                # Severe warnings, Errors, and Fatals
                if level >= logging.WARNING:
                    # We use a clean payload for JSONL, distinct from the text line
                    payload = {
                        "correlation_id": correlation_id,
                        "timestamp": iso_timestamp,
                        "level": lvl_name,
                        "proc_id": proc_id,
                        "system": system,
                        "node": node,
                        "module": module,
                        "message": msg,
                        "traceback": evidence.get("traceback") if evidence else None,
                        "context": {k: v for k, v in evidence.items() if k != "traceback"} if evidence else None,
                        "routed_to": channels
                    }

                    # JSON string as message
                    payload_str = json.dumps(payload)

                    # Dummy LogRecord for JSONL as message
                    json_record = logging.LogRecord(
                        name=proc_id,           # For consistency
                        level=level,
                        pathname="",            # Unused
                        lineno=0,               # Unused
                        msg=payload_str,        # This becomes the line
                        args=None,
                        exc_info=None
                    )
                    json_record.created = timestamp # For potential time use

                    # Write JSON line via anomaly_handler
                    anomaly_handler.emit(json_record)
    
                # 📊 THE LEDGER (SQLite Metadata) - Batched insert
                ledger_args = (
                    correlation_id,
                    iso_timestamp,
                    lvl_name,
                    proc_id,
                    system,
                    node,
                    module,
                    message_preview,
                    json.dumps(channels),
                    content_hash
                )
                with EEEAggregator._ledger_lock:
                    EEEAggregator._ledger_queue.append(ledger_args)
                    # Immediate flush if batch full (for low-volume bursts)
                    if len(EEEAggregator._ledger_queue) >= EEEAggregator.BATCH_SIZE:
                        EEEAggregator._flush_ledger_batch()

            except Exception as e:
                # If the disk fails, print to console for now
                print(f"[EEE] Failed to write to archive: {e}")
                traceback.print_exc()

            EEEAggregator._log_queue.task_done()

    @classmethod
    def _flush_ledger_batch(cls) -> None:
        """
        Flush batched ledger entries to SQLite.
        Uses the single persistent connection from _db_conn.
        """
        with cls._ledger_lock:
            if not cls._ledger_queue:
                return
            
            batch = list(cls._ledger_queue)
            cls._ledger_queue.clear()

            try:
                # ✅ Use the single persistent connection
                if cls._db_conn is None:
                    print("[EEE] Database connection not initialized!")
                    return
                
                def _do_write():
                    cls._db_conn.executemany("""
                        INSERT INTO events
                        (correlation_id, timestamp, level, proc_id, system, node, module, message_preview, channels, content_hash)
                        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                    """, batch)
                    cls._db_conn.commit()

                t = threading.Thread(target=_do_write, daemon=True)
                t.start()
                t.join(timeout=1.0)
                if t.is_alive():
                    print("[EEE] Ledger flush timed out - skipping remaining writes")
            except Exception as e:
                print(f"[EEE] Ledger batch flush failed: {e}")
                traceback.print_exc()
                # Re-queue on failure (optional)
                # cls._ledger_queue.extendleft(reversed(batch))
                
    @classmethod
    def get_throttle_status(cls) -> dict:
        """
        Get current throttling status for all proc_ids.
        Useful for debugging and monitoring
        
        Returns:
            Dictionary of proc_id -> throttle status
        """
        status = {
            "enabled": cls._throttle_config["enabled"],
            "config": cls._throttle_config,
            "active_throttles": {}
        }
        
        for proc_id, throttle in cls._throttles.items():
            status["active_throttles"][proc_id] = throttle.get_status()
    
        return status

    @classmethod
    def shutdown(cls):
        if not cls._worker_started or cls._shutdown_complete:
            return
        cls._shutdown_complete = True  # ← prevent double call
        print("[EEE] t=0 shutdown start")

        print("[EEE] t=A draining queue...")
        deadline = time.monotonic() + 1.0
        while cls._log_queue.qsize() > 0 and time.monotonic() < deadline:
            time.sleep(0.05)
        print(f"[EEE] t=B queue drained, dropped={cls._log_queue.qsize()}")

        cls._shutting_down = True          # ← stop flush_loop from running
        if cls._ledger_flush_timer:
            cls._ledger_flush_timer.cancel()
            cls._ledger_flush_timer = None
        print("[EEE] t=C timer cancelled")
        # skip _flush_ledger_batch() - SQLite WAL handles incomplete transactions safely
        print("[EEE] t=D skipping ledger flush")

        if cls._db_conn:
            try:
                cls._db_conn.close()
                cls._db_conn = None
                print("[EEE] t=E db closed")
            except Exception as e:
                print(f"[EEE] t=E db close error: {e}")

        print("[EEE] t=F calling GzipRotatingFileHandler.shutdown()...")
        GzipRotatingFileHandler.shutdown()
        print("[EEE] t=G done")
    
    @classmethod
    def _register_shutdown_handlers(cls):
        """Register shutdown handlers for clean exit."""
        if cls._shutdown_registered:
            return
        
        cls._shutdown_registered = True
        print("[EEE] Shutdown handlers registered.")
    
    @classmethod
    def register_plugin(cls, name: str, callback):
        """Register a plugin to receive events before disk write."""
        cls._plugins[name] = callback
        print(f"[EEE] Plugin registered: {name}")
    
    @classmethod
    def unregister_plugin(cls, name: str):
        """Unregister a plugin."""
        if name in cls._plugins:
            del cls._plugins[name]
            print(f"[EEE] Plugin unregistered: {name}")

    class Filtrator:
        """
        Filtrator:
        A submodule in my Emergency and Exception Event (EEE) module that filters of the entries of emergency and exception
        events, which converts raw log entries into a standardized format for uniformity across the system.

        It includes two main components:
        1. pass_thru_EEE_time_filter: Custom formatter to format time in ISO 8601 format with milliseconds and timezone.
        2. pass_thru_EEE_header_filter: Custom filter to standardize log headers by replacing dots with colons and uppercasing text.
        """
        class pass_thru_EEE_time_formatter(logging.Formatter):
            """
            This is a custom formatter to format time in ISO 8601 format with milliseconds and timezone.
            It overrides the default time formatting to ensure compliance with ISO 8601 which is the industry Gold Standard for timestamps.
            e.g. 2025-01-15T12:34:56.789-08:00
            """
            def formatTime(self, record, datefmt=None):
                # Convert record creation time to a local datetime object
                local_datetime = datetime.fromtimestamp(record.created).astimezone()
                # %Y-%m-%dT%H:%M:%S -> Date and Time
                # .%03d -> 3-digit milliseconds
                # %z -> Timezone offset (+HHMM)
                # Manually insert the colon in the offset to be strictly ISO compliant

                timezone_offset = local_datetime.strftime('%z')
                timezone_iso = f"{timezone_offset[:3]}:{timezone_offset[3:]}"
                
                return local_datetime.strftime('%Y-%m-%dT%H:%M:%S') + f".{int(record.msecs):03d}{timezone_iso}"
            
        class pass_thru_EEE_header_filter(logging.Filter):
            """
            This is a custom filter to standardize the log format.
            It replaces dots with colons and uppers the text to standardize subsystem/module names.
            eg. "vcs.vtc.pump" becomes "VCS::VTC::PUMP"
            """
            def filter(self, record: logging.LogRecord) -> bool:
                # Standardize the log header format by replacing dots with colons and uppercasing text.
                record.name = record.name.replace('.', '::').upper()
                return True
        
        @classmethod
        def pass_thru_EEE_filters(cls) -> logging.Formatter:
            """
            This is the method to pass the log entry through the time formatter.
            """
            # Standardized log format for all log entries
            # (e.g. 2025-01-15T12:34:56.789-08:00 | INFO | [SYSTEM::ROOT] | This is a log message)
            EEE_FORMAT: str = "%(asctime)s | %(levelname)s | [%(name)s] | %(message)s"

            return cls.pass_thru_EEE_time_formatter(EEE_FORMAT)

        @classmethod
        def pass_filtrate_to_handler(cls, EEE_handler: logging.Handler):
            """
            This is the method to pass the filtered entry format to the given handler.
            It contains the standardized time formatter and header filter and applies them to the handler.
            
            Args:
                handler: The logging handler to which the filter and formatter will be applied.
            """
            # Pass the handler through the time filter
            EEE_handler.setFormatter(cls.pass_thru_EEE_filters())
            
            # Pass the handler through the header filter
            EEE_handler.addFilter(cls.pass_thru_EEE_header_filter())
            
            return EEE_handler
            
    class Propagator:
        """
        Propagator:
        A submodule in my Emergency and Exception Event (EEE) module that propagates the log entries of emergency and
        exception events to the appropriate loggers/handlers based on severity level.

        It implements the Triple-Aggregation Logger & Ledger Entry (TALLE) which is composed of:
        - Master log file: All logs (DEBUG and above)
        - Error log file: Errors only (ERROR and above)
        - System log files: Separate log files for each system (DEBUG and above)
        - plus Console output: Info and above (INFO and above)
        """
        
        _log_path = LOG_PATH

        @classmethod
        def engage_TALLE_to_propagate(cls) -> None:
            """
            Sets up the Root Logger, The Master Logger, The Error Logger, and the Console. This is the orchestrator that
            propagates the event entries to the appropriate loggers/handlers.
            """
            # Set up root logger for the entire system
            root = logging.getLogger()
            root.setLevel(logging.DEBUG)
            
            # Clear existing handlers to avoid duplication
            root.handlers.clear()

            # Console output for info and above
            console = logging.StreamHandler()
            console.setLevel(logging.INFO)

            # Apply industry-standard formatting to the console handler
            root.addHandler(EEEAggregator.Filtrator.pass_filtrate_to_handler(console))

    """
    These are the public API for the other modules to call logging methods.
    They are simply pass-through methods to the underlying logger,
    which is already set up with the appropriate handlers and formatters.
    They are setup to log at different levels:
        - info: messages that convey general information
        - warning: warning messages that indicate potential issues
        - error: error messages that indicate failures but not causing system shutdown
        - critical: critical messages that indicate severe failures causing system shutdown
        - debug: debug messages for development and troubleshooting
        - exception: exception messages with traceback
    
    Usage:
        eee = EEEAggregator(caller_id="VCS.VTC")
        eee.info("Hello, World!")
        eee.critical("RED ALERT! My core is overheating!")
    """
    def info(self, msg: str, evidence: dict = None):
        """ Pushes to queue and returns immediately. No waiting for disk I/O. """
        self._dispatch(logging.INFO, msg, evidence)
    
    def warning(self, msg: str, evidence: dict = None):
        """ Pushes to queue and returns immediately. No waiting for disk I/O. """
        self._dispatch(logging.WARNING, msg, evidence)
    
    def error(self, msg: str, evidence: dict = None):
        """ Pushes to queue and returns immediately. No waiting for disk I/O. """
        self._dispatch(logging.ERROR, msg, evidence)
    
    def critical(self, msg: str, evidence: dict = None):
        """ Pushes to queue and returns immediately. No waiting for disk I/O. """
        self._dispatch(logging.CRITICAL, msg, evidence)
    
    def debug(self, msg: str, evidence: dict = None):
        """ Pushes to queue and returns immediately. No waiting for disk I/O. """
        self._dispatch(logging.DEBUG, msg, evidence)

    def exception(self, msg: str, evidence: dict = None):
        """
        Captures the full stack trace AND any physical evidence 
        provided by the sensors at the time of the crash.
        """
        if evidence is None:
            evidence = {}
            
        # Capture the traceback as a string so it can be stored in the Ledger
        evidence["traceback"] = traceback.format_exc()
        
        # Dispatch to the background worker (Async)
        self._dispatch(logging.ERROR, msg, evidence)

    def _dispatch(self, level: int, msg: str, evidence: dict):
        """Package the event and drop it in the mail"""
        try:
            self._log_queue.put((time.time(), level, self.logger.name, msg, evidence), timeout=0.1)  # Block 100ms max
        except queue.Full:
            EEEAggregator._dropped_events += 1
            if EEEAggregator._dropped_events % 100 == 0:
                print(f"[EEE] WARNING: Dropped {EEEAggregator._dropped_events} events (queue full!)")
                
        # Trigger the console output immediately
        if evidence:
            self.logger.log(level, f"{msg} | Data: {json.dumps(evidence)}")
        else:
            self.logger.log(level, msg)


def get_logger(caller_id: str = "SCS.EEE") -> EEEAggregator:
    """
    This is the public API for the other modules to call logging methods.
    It simply returns an instance of the EEEAggregator class.
    """
    return EEEAggregator(caller_id)


if __name__ == "__main__":

    # Simple test of the TALLE logger
    # Inject the TALLE logger into the global namespace
    logger = get_logger("SCS.EEE.TEST")

    logger.info("This is a test message.")

    # Test the system logger
    # Test INFO logging entries
    print("Sending 10 test logs...")
    for i in range(10):
        logger.info(f"Test message {i+1}")
        time.sleep(0.1)
        
    # Test critical and exception tracing
    try:
        x = 1 / 0
    except ZeroDivisionError:
        logger.critical("Robot is UNDER ATTACK!")
        logger.exception("This is an exception message.")
    
    print("\nTesting throttling with burst...")
    print("Config:", EEEAggregator._throttle_config)
    print()
    
    # Send 10 rapid INFO messages (should be throttled after burst of 3)
    print("Sending 10 rapid INFO messages (rate=1Hz, burst=3):")
    for i in range(10):
        logger.info(f"Rapid message {i+1}")
        time.sleep(0.05)  # 50ms between messages (20 msgs/sec - way over limit!)
    
    print("\nWaiting 2 seconds for bucket to refill...")
    time.sleep(2)
    
    # Send 3 more (should go through after refill)
    print("Sending 3 more messages (bucket refilled):")
    for i in range(3):
        logger.info(f"After refill message {i+1}")
        time.sleep(0.05)
    
    # Test CRITICAL bypass
    print("\nTesting CRITICAL bypass (should always go through):")
    for i in range(5):
        logger.critical(f"CRITICAL message {i+1} - bypasses throttle")
        time.sleep(0.05)
    
    print("\n--- Throttle Status ---")
    print(EEEAggregator.get_throttle_status())
    
    # Flush
    EEEAggregator.shutdown()
    
    print("\nCheck logs - you should see:")
    print("  - First 3 INFO messages (burst)")
    print("  - GAPS in sequence (throttled messages)")
    print("  - 3 messages after refill")
    print("  - ALL 5 CRITICAL messages (bypass)")

    print("\nExiting (shutdown should flush queue automatically)...")
