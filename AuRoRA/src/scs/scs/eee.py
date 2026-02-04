"""
EEE (Emergency and Exception Event) Module

Welcome to my Emergency and Exception Event (EEE) Module!
This module is designed to handle, broadcast, and record emergency and exception events across the system,
providing a unified and centralized interface for alerting my other systems of any anomalies, and ensuring
that they are addressed promptly and effectively, thus maintaining system stability.
"""

# System modules
import atexit
from datetime import datetime
import hashlib
import json
import logging
from pathlib import Path
import queue
import signal
import sqlite3
import threading
import time
import traceback
import uuid

#(TODO) Later make this configurable via config file or environment variable (e.g. /var/log/agisys)
LOG_PATH = "./logs"  # Location of log files
LEDGER_DB_FILE = "ledger.db"  # Ledger database file name
MASTER_LOG_FILE = "activity.log"  # Master log file name (40% of the logs)
ERROR_LOG_FILE = "anomaly.jsonl"   # Error log file name (10% of the logs)

class TokenBucketThrottle:
    """
    Token bucket algorithm for rate limiting per conduit.
    Prevents log floods while allowing bursts of critical events.
    
    Example:
        throttle = TokenBucketThrottle(rate=1.0, burst=3)
        if throttle.allow():
            # Process event
        else:
            # Drop event (throttled)
    """
    
    def __init__(self, rate: float = 1.0, burst: int = 3):
        """
        Initialize token bucket.
        
        Args:
            rate: Tokens refilled per second (Hz)
            burst: Maximum token capacity (burst size)
        """
        self.rate = rate          # Events per second
        self.burst = burst        # Burst capacity
        self.tokens = burst       # Start with full bucket
        self.last_update = time.monotonic()
        self._lock = threading.Lock()
    
    def allow(self) -> bool:
        """
        Determine whether an event may proceed by refilling the token bucket and consuming one token if available.
        
        This method is thread-safe; it refills tokens based on elapsed time up to the configured burst capacity and, if at least one token is available, consumes one.
        
        Returns:
            True if a token was consumed and the event is allowed, False otherwise.
        """
        with self._lock:
            now = time.monotonic()
            elapsed = now - self.last_update
            
            # Refill tokens based on elapsed time
            self.tokens = min(self.burst, self.tokens + elapsed * self.rate)
            self.last_update = now
            
            # Check if we have at least 1 token
            if self.tokens >= 1.0:
                self.tokens -= 1.0
                return True
            
            return False
    
    def get_status(self) -> dict:
        """
        Report the token bucket's current state for debugging.
        
        Returns:
            status (dict): Mapping with keys:
                - "tokens": Current token count rounded to two decimal places.
                - "rate": Refill rate in tokens per second.
                - "burst": Maximum token capacity (burst size).
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
    _initialized = False
    _log_queue: queue.Queue = queue.Queue()
    _worker_started: bool = False
    _thread_lock = threading.Lock()
    _shutdown_registered = False
    _plugins: dict = {}  # plugin_name -> callback

    _throttles: dict = {}  # Per-proc_id throttles
    _throttle_config = {
        "enabled": True,
        "default_rate": 1.0,   # 1 event per second
        "burst_size": 3,       # Allow 3-event bursts
        "critical_bypass": True  # CRITICAL level bypasses throttling
    }

    def __init__(self, caller_id: str):
        """
        Initialize an EEEAggregator for the given caller and ensure the global logging subsystem and background worker are started.
        
        Parameters:
            caller_id (str): Dot-separated identifier for the caller (e.g., "VCS.VTC.Pump"). The leading element is treated as the system name.
                If `caller_id` is missing or not a string, it defaults to "SYSTEM.UNKNOWN".
        
        Description:
            Creates a logger instance named by `caller_id` (no local handlers; relies on propagation), performs one-time global initialization
            of the logging/propagation infrastructure if needed, registers shutdown handlers, and ensures the asynchronous worker thread is running.
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
        Background worker that consumes aggregated log events and persists them to durable stores.
        
        Runs in a dedicated thread and processes entries from the internal queue until shutdown:
        - Applies deduplication (short time window) and per-proc_id rate throttling.
        - Invokes registered plugins before any disk writes; plugins may prevent persistence for non-critical events.
        - Persists events to the master plaintext log, a per-system plaintext log, a JSONL anomaly log for warnings/errors,
          and an SQLite ledger table with structured metadata and a content hash.
        - Produces an ISO 8601 timestamp and a short message preview for ledger entries.
        - Handles and reports disk or plugin errors to the console but continues processing subsequent events.
        
        This is the only thread that performs disk I/O for the aggregator.
        """
        log_dir = Path(LOG_PATH)
        log_dir.mkdir(parents=True, exist_ok=True)

        # THE LEDGER: Initialize SQLite here
        db_conn = sqlite3.connect(log_dir / LEDGER_DB_FILE)
        db_conn.execute("""
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
        db_conn.execute("CREATE INDEX IF NOT EXISTS idx_correlation_id ON events(correlation_id)")
        db_conn.execute("CREATE INDEX IF NOT EXISTS idx_timestamp ON events(timestamp)")
        db_conn.execute("CREATE INDEX IF NOT EXISTS idx_level ON events(level)")
        db_conn.execute("CREATE INDEX IF NOT EXISTS idx_system ON events(system)")
        db_conn.execute("CREATE INDEX IF NOT EXISTS idx_content_hash ON events(content_hash)")
        db_conn.commit()

        text_formatter = EEEAggregator.Filtrator.pass_thru_EEE_filters()

        # Deduplication cache: content_hash -> (timestamp, correlation_id)
        recent_hashes = {}
        DEDUP_WINDOW = 5.0  # 5 seconds
        
         # Throttling stats (for monitoring)
        throttled_count = 0

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
                        EEEAggregator._throttles[proc_id] = TokenBucketThrottle(
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
            parts = plain_text_line.split('|', 1)
            plain_text_line = f"{parts[0]} | CID:{correlation_id[:8]} | {parts[1]}"
            if evidence:
                plain_text_line += f" | DATA: {json.dumps(evidence)}"

            # Call plugins BEFORE disk writes
            skip_disk = False
            for plugin_name, callback in EEEAggregator._plugins.items():
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
                    print(f"[EEE] Plugin {plugin_name} error: {e}")

            # Skip disk writes if plugin handled it
            if skip_disk:
                EEEAggregator._log_queue.task_done()
                continue  # ✅ Now this continues the while True loop

            try:
                # 📦 ARCHIVE A: MASTER LOG (Post-mortem)
                with open(log_dir / MASTER_LOG_FILE, "a", encoding="utf-8") as f:
                    f.write(plain_text_line + "\n")

                # 🔧 ARCHIVE B: SYSTEM LOG (Subsystem isolation)
                system_prefix = caller.split('.')[0].lower()
                with open(log_dir / f"{system_prefix}.log", "a", encoding="utf-8") as f:
                    f.write(plain_text_line + "\n")

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
                    with open(log_dir / ERROR_LOG_FILE, "a", encoding="utf-8") as f:
                        f.write(json.dumps(payload) + "\n")

                # 📊 THE LEDGER (SQLite Metadata)
                db_conn.execute("""
                    INSERT INTO events
                    (correlation_id, timestamp, level, proc_id, system, node, module, message_preview, channels, content_hash)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """, (
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
                ))
                db_conn.commit()

            except Exception as e:
                # (TODO) If the disk fails, print to console for now
                print(f"Failed to write to archive: {e}")

            EEEAggregator._log_queue.task_done()
            
    @classmethod
    def get_throttle_status(cls) -> dict:
        """
        Get current throttling status for all proc_ids.
        Useful for debugging and monitoring.
        
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
        """
        Shuts down the background worker by waiting for any queued events to be written to disk.
        
        If the worker was started, blocks until the internal log queue is empty and prints brief status messages to stdout; no value is returned.
        """
        if cls._worker_started:
            print("[EEE] Flushing pending log entries...")
            cls._log_queue.join()  # Wait for queue to empty
            print(f"[EEE] All events written. Shutdown complete.")
    
    @classmethod
    def _register_shutdown_handlers(cls):
        """
        Register process exit handlers once so the aggregator can shut down cleanly.
        
        This idempotent method attaches an atexit handler and installs SIGINT/SIGTERM handlers to invoke the class shutdown routine during normal exit or on interrupt/terminate signals; subsequent calls have no effect.
        """
        if cls._shutdown_registered:
            return
        
        # Register with atexit (normal program exit)
        atexit.register(cls.shutdown)
        
        # Register signal handlers (Ctrl+C, kill)
        def signal_handler(signum, frame):
            """
            Handle incoming POSIX signals by initiating a graceful shutdown of the EEE aggregator and then delegating to normal process termination.
            
            Parameters:
                signum (int): Numeric signal identifier received (e.g., SIGINT, SIGTERM).
                frame (types.FrameType | None): Current stack frame at the time of signal delivery.
            
            Behavior:
                Prints a shutdown notice and invokes the aggregator's shutdown routine. After shutdown:
                - If the signal is SIGINT, raises KeyboardInterrupt to propagate an interrupt.
                - If the signal is SIGTERM, calls sys.exit(0) to exit cleanly.
                - For other signals, returns after performing shutdown.
            
            Raises:
                KeyboardInterrupt: When `signum` is `signal.SIGINT`.
                SystemExit: When `signum` is `signal.SIGTERM` (via `sys.exit(0)`).
            """
            print(f"\n[EEE] Signal {signum} received. Initiating graceful shutdown...")
            cls.shutdown()
            # Re-raise to allow normal termination
            if signum == signal.SIGINT:
                raise KeyboardInterrupt
            elif signum == signal.SIGTERM:
                import sys
                sys.exit(0)
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        cls._shutdown_registered = True
        print("[EEE] Shutdown handlers registered.")
    
    @classmethod
    def register_plugin(cls, name: str, callback):
        """Register a plugin to receive events before disk write."""
        cls._plugins[name] = callback
        print(f"[EEE] Plugin registered: {name}")
    
    @classmethod
    def unregister_plugin(cls, name: str):
        """
        Unregisters a previously registered plugin by name.
        
        Removes the plugin entry from the class plugin registry if present; if no plugin with the given name exists the method is a no-op. Prints a confirmation message to stdout when a plugin is removed.
        
        Parameters:
            name (str): Name of the plugin to remove.
        """
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
                """
                Format a LogRecord timestamp as an ISO 8601 local datetime with millisecond precision and a colon-separated timezone offset.
                
                Parameters:
                    record (logging.LogRecord): Log record whose `created` and `msecs` fields are used to build the timestamp.
                    datefmt (str | None): Ignored by this formatter.
                
                Returns:
                    str: Timestamp in the form `YYYY-MM-DDTHH:MM:SS.sss±HH:MM` using the system local timezone.
                """
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
                """
                Normalize the log record's name by replacing dots with double colons and uppercasing it.
                
                This mutates `record.name` in-place so downstream handlers see the standardized header format.
                
                Parameters:
                    record (logging.LogRecord): The log record to modify.
                
                Returns:
                    bool: `True` to allow the record to be processed.
                """
                record.name = record.name.replace('.', '::').upper()
                return True
        
        @classmethod
        def pass_thru_EEE_filters(cls) -> logging.Formatter:
            """
            Return a logging.Formatter configured for EEE messages with standardized timestamp and header formatting.
            
            The returned formatter uses an ISO 8601 timestamp with milliseconds and timezone offset and the format:
            "%(asctime)s | %(levelname)s | [%(name)s] | %(message)s".
            
            Returns:
                logging.Formatter: Formatter instance configured with the EEE time formatter and standard layout.
            """
            # Standardized log format for all log entries
            # (e.g. 2025-01-15T12:34:56.789-08:00 | INFO | [SYSTEM::ROOT] | This is a log message)
            EEE_FORMAT: str = "%(asctime)s | %(levelname)s | [%(name)s] | %(message)s"

            return cls.pass_thru_EEE_time_formatter(EEE_FORMAT)

        @classmethod
        def pass_filtrate_to_handler(cls, EEE_handler: logging.Handler):
            """
            Apply the module's standard formatter and header filter to a logging.Handler.
            
            Parameters:
                EEE_handler (logging.Handler): Handler to configure with the standard EEE formatter and header filter.
            
            Returns:
                logging.Handler: The same handler instance after applying the formatter and filter.
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
            Configure system-wide logging propagation by initializing the root logger and a console handler.
            
            Sets the root logger level to DEBUG, clears any existing handlers to avoid duplication, and attaches a console StreamHandler set to INFO that is configured with the module's standard formatter and header filter for consistent timestamps and headers.
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
        """
        Enqueues an INFO-level event for processing by the aggregator.
        
        Parameters:
        	msg (str): Human-readable message describing the event.
        	evidence (dict, optional): Additional structured context or diagnostic data to attach to the event.
        """
        self._dispatch(logging.INFO, msg, evidence)
    
    def warning(self, msg: str, evidence: dict = None):
        """
        Enqueue a WARNING-level event for asynchronous processing; returns immediately.
        
        Parameters:
        	msg (str): Human-readable event message.
        	evidence (dict, optional): Additional context to attach to the event (e.g., traceback, diagnostics). Omit or pass None when there is no extra context.
        """
        self._dispatch(logging.WARNING, msg, evidence)
    
    def error(self, msg: str, evidence: dict = None):
        """
        Enqueues an ERROR-level event for handling, propagation, and durable recording.
        
        Parameters:
        	msg (str): Human-readable event message.
        	evidence (dict, optional): Additional structured context to attach to the event (for example traceback, context fields, or other metadata).
        """
        self._dispatch(logging.ERROR, msg, evidence)
    
    def critical(self, msg: str, evidence: dict = None):
        """
        Enqueue a CRITICAL-level event for asynchronous handling.
        
        Parameters:
        	msg (str): Human-readable message describing the event.
        	evidence (dict, optional): Additional context or payload to include with the event (for example traceback or diagnostic metadata).
        """
        self._dispatch(logging.CRITICAL, msg, evidence)
    
    def debug(self, msg: str, evidence: dict = None):
        """
        Log a debug-level event for this logger and enqueue it for processing.
        
        Parameters:
            msg (str): Human-readable message describing the event.
            evidence (dict, optional): Additional contextual data to include with the event.
        """
        self._dispatch(logging.DEBUG, msg, evidence)

    def exception(self, msg: str, evidence: dict = None):
        """
        Record the current exception traceback and dispatch an ERROR-level event with optional sensor evidence.
        
        If `evidence` is omitted, creates an empty dict, injects the current exception traceback into `evidence["traceback"]`, and enqueues the message and evidence for processing by the aggregator's background worker.
        
        Parameters:
            msg (str): Human-readable message describing the context of the exception.
            evidence (dict, optional): Additional context or sensor data to include with the event; will be mutated to include the traceback.
        """
        if evidence is None:
            evidence = {}
            
        # Capture the traceback as a string so it can be stored in the Ledger
        evidence["traceback"] = traceback.format_exc()
        
        # Dispatch to the background worker (Async)
        self._dispatch(logging.ERROR, msg, evidence)
    def _dispatch(self, level: int, msg: str, evidence: dict):
        """
        Enqueue an event for asynchronous processing and emit an immediate console log record.
        
        Parameters:
        	level (int): Logging level (e.g., logging.INFO, logging.WARNING).
        	msg (str): Human-readable message describing the event.
        	evidence (dict): Optional structured context or payload to accompany the event.
        """
        self._log_queue.put((time.time(), level, self.logger.name, msg, evidence))

        # Trigger the console output immediately
        self.logger.log(level, f"{msg} | Data: {json.dumps(evidence)}")

def get_logger(caller_id: str = "SCS.EEE") -> EEEAggregator:
    """
    Create or retrieve an EEEAggregator for the given caller identifier.
    
    Parameters:
        caller_id (str): Dot-delimited identifier used as the logger's name (e.g., "SYSTEM.NODE.MODULE"). Defaults to "SCS.EEE".
    
    Returns:
        EEEAggregator: An EEEAggregator instance configured for the provided caller_id.
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
    
    print("Testing throttling with burst...")
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