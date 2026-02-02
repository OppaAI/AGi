"""
EEE (Emergency and Exception Event) Module

Welcome to my Emergency and Exception Event (EEE) Module!
This module is designed to handle, broadcast, and record emergency and exception events across the system,
providing a unified and centralized interface for alerting my other systems of any anomalies, and ensuring
that they are addressed promptly and effectively, thus maintaining system stability.
"""

# System modules
import builtins
from datetime import datetime
import logging
import logging.handlers
from pathlib import Path
import threading
from typing import Any, Set, TYPE_CHECKING

# Callable type hints for global injection
if TYPE_CHECKING:
    global_logger: Any
    system_logger: Any

#(TODO) Later make this configurable via config file or environment variable (e.g. /var/log/agisys)
LOG_PATH = "./logs"  # Location of log files
MASTER_LOG_FILE = "activity.log"  # Master log file name (40% of the logs)
ERROR_LOG_FILE = "anomaly.log"   # Error log file name (10% of the logs)

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
        eee.fatal("RED ALERT! My core is overheating!")
        
    That's all you need to do to log messages from anywhere in the system.
    It automatically sets up logging on first use, and creates log files as needed.
    """
    
    # Class-level flags to ensure setup happens only once
    _initialized = False
    _system_registry: Set[str] = set()
    _thread_lock = threading.Lock()
    
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
                    EEEAggregator._initialized = True
        
        # Derive the system name from the caller ID
        parts = [p for p in caller_id.split('.') if p.strip()]
        system = parts[0].upper() if parts else "SYSTEM"

        # Register the system of the caller if not already registered
        if system not in EEEAggregator._system_registry:
            with EEEAggregator._thread_lock:
                if system not in EEEAggregator._system_registry:
                    EEEAggregator.Propagator.build_system_logger_to_propagate(system)
                    EEEAggregator._system_registry.add(system)

        # Establish the specific logger for the caller
        # This logger will have NO handlers of its own, and relies 100% on propagation
        self.logger = logging.getLogger(caller_id)

    class Injector:
        """
        Injector:
        A submodule in my Emergency and Exception Event (EEE) module that injects the TALLE listener automatically
        into the global namespace, or manually into my other systems, to ensure that all emergency and exception events
        that may arise in my systems are logged and handled properly for prompt attention.
        """

        # Class-level flag to ensure injection happens only once
        _already_injected = False

        @classmethod
        def inject_logger_to_builtins(cls)-> None:
            """
            Injects the TALLE listener automatically into the global namespace, or manually into my other systems,
            to ensure that all emergency and exception events that may arise in my systems are logged and handled properly
            for prompt attention.
            """
            # Prevent double injection, once is enough
            if cls._already_injected:
                return
        
            # Inject the TALLE instance for generic/system-wide logger
            setattr(builtins, "global_logger", EEEAggregator("SCS.EEE"))

            # Inject the TALLE factory for system/module-specific loggers
            setattr(builtins, "system_logger", lambda caller_id: EEEAggregator(caller_id))

            # Mark the system as injected
            cls._already_injected = True

            # Log the successful injection
            global_logger.info("Builtins hijacked successfully. I'm 40% global variables!")

        @classmethod
        def is_ready(cls)-> bool:
            """
            This flag is used to ensure that the logger is only injected once, and to prevent double injection.
            
            Returns:
                True if the injector has already injected the logger into the global namespace, False otherwise.
            """
            return cls._already_injected

    class Filtrator:
        """
        Filtrator:
        A submodule in my Emergency and Exception Event (EEE) module that filters of the entries of emergency and exception
        events, which converts raw log entries into a standardized format for uniformity across the system.

        It includes two main components:
        1. pass_thru_EEE_time_filter: Custom formatter to format time in ISO 8601 format with milliseconds and timezone.
        2. pass_thru_EEE_header_filter: Custom filter to standardize log headers by replacing dots with colons and uppercasing text.
        """
        class pass_thru_EEE_time_filter(logging.Formatter):
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
        def pass_filtrate_to_handler(cls, EEE_handler: logging.Handler):
            """
            This is the method to pass the filtered entry format to the given handler.
            It contains the standardized time formatter and header filter and applies them to the handler.
            
            Args:
                handler: The logging handler to which the filter and formatter will be applied.
            """
            # Standardized log format for all log entries
            # (e.g. 2025-01-15T12:34:56.789-08:00 | INFO | [SYSTEM::ROOT] | This is a log message)
            EEE_FORMAT: str = "%(asctime)s | %(levelname)s | [%(name)s] | %(message)s"

            # Pass the handler through the time filter
            EEE_handler.setFormatter(cls.pass_thru_EEE_time_filter(EEE_FORMAT))
            
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
        def engage_TALLE_to_propagate(cls)-> None:
            """
            Sets up the Root Logger, The Master Logger, The Error Logger, and the Console. This is the orchestrator that
            propagates the event entries to the appropriate loggers/handlers.
            """
            # Create log directory if it doesn't exist
            log_path = Path(cls._log_path).resolve()
            log_path.mkdir(parents=True, exist_ok=True)
            
            # Set up root logger for the entire system
            root = logging.getLogger()
            root.setLevel(logging.DEBUG)
            
            # Clear existing handlers to avoid duplication
            root.handlers.clear()

            # Master log file that logs all activity events
            cls._add_rotating_handler(
                logger=root,
                path=log_path / MASTER_LOG_FILE, 
                level=logging.DEBUG, 
                size=50*1024*1024, 
                backups=5
            )
            
            # Error log file that logs all errors and above
            cls._add_rotating_handler(
                logger=root,
                path=log_path / ERROR_LOG_FILE, 
                level=logging.ERROR, 
                size=10*1024*1024, 
                backups=3
            )

            # Console output for info and above
            console = logging.StreamHandler()
            console.setLevel(logging.INFO)

            # Apply industry-standard formatting to the console handler
            root.addHandler(EEEAggregator.Filtrator.pass_filtrate_to_handler(console))

        @classmethod
        def build_system_logger_to_propagate(cls, system: str)-> None:
            """
            Sets up a specific system (e.g., 'VCS') to have its own log file
            while ensuring it propagates its data up to all of the TALLE loggers.

            Args:
                system: The system of the caller which is calling this logger (e.g. "VCS")
            
            Returns:
                None
            """
            # Create the logger for the specified system
            system_logger = logging.getLogger(system)
            system_logger.setLevel(logging.DEBUG)
            
            # Propagate the log entries to the Master/Error/Console. This is where the magic happens
            system_logger.propagate = True
            
            log_path = Path(cls._log_path).resolve()
            filename = log_path / f"{system.lower()}.log"

            # Strict check for existing handlers to prevent "double-logging"
            if not any(isinstance(h, logging.handlers.RotatingFileHandler) 
                    and h.baseFilename == str(filename.absolute())
                    for h in system_logger.handlers):
                
                cls._add_rotating_handler(
                    system_logger, filename, 
                    logging.DEBUG, 20*1024*1024, 3
                )

        @staticmethod
        def _add_rotating_handler(logger: logging.Logger, path: Path, level: int, size: int, backups: int)-> None:
            """
            Adds a rotating file handler to the given logger.

            Args:
                logger: The logger to which the handler will be added.
                path: The path to the log file.
                level: The logging level for the handler.
                size: The maximum size of the log file in bytes.
                backups: The number of backup log files to keep.
            
            Returns:
                None
            """
            handler = logging.handlers.RotatingFileHandler(
                path, maxBytes=size, backupCount=backups
            )
            handler.setLevel(level)
            # Assuming Filtrator is your class that attaches the Formatter/Filter
            handler = EEEAggregator.Filtrator.pass_filtrate_to_handler(handler)
            logger.addHandler(handler)
        
    """
    These are the public API for the other modules to call logging methods.
    They are simply pass-through methods to the underlying logger,
    which is already set up with the appropriate handlers and formatters.
    They are setup to log at different levels:
        - info: messages that convey general information
        - warn: warning messages that indicate potential issues
        - error: error messages that indicate failures but not causing system shutdown
        - fatal: fatal/critical messages that indicate severe failures causing system shutdown
        - debug: debug messages for development and troubleshooting
        - exception: exception messages with traceback
    
    Usage:
        eee = EEEAggregator(caller_id="VCS.VTC")
        eee.info("Hello, World!")
        eee.fatal("RED ALERT! My core is overheating!")
    """
    def info(self, msg: str):
        """Log info message"""
        self.logger.info(msg)
    
    def warn(self, msg: str):
        """Log warning message"""
        self.logger.warning(msg)
    
    def error(self, msg: str):
        """Log error message"""
        self.logger.error(msg)
    
    def fatal(self, msg: str):
        """Log fatal/critical message"""
        self.logger.critical(msg)
    
    def debug(self, msg: str):
        """Log debug message"""
        self.logger.debug(msg)

    def exception(self, msg: str):
        """Log exception message with traceback"""
        self.logger.exception(msg)

if __name__ == "__main__":

    # Simple test of the TALLE logger
    # Inject the TALLE logger into the global namespace
    EEEAggregator.Injector.inject_logger_to_builtins()

    # Create a system logger
    system_logger = getattr(builtins, "system_logger")
    test_logger = system_logger("TEST.MODULE")

    # Test the system logger
    try:
        x = 1 / 0
    except ZeroDivisionError:
        test_logger.exception("This is an exception message.")