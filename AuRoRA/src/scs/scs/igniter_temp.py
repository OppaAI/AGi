"""
Igniter (Temporary) Module

Welcome to my Igniter Module! 
Currently, this temporary module is designed to simulate the ignition sequence (bootstrap) of the robot's system.
It serves as a placeholder until the full Igniter Module is developed.

This module includes the following features that load at startup:
    - EEEAggregator: Centralized Emergency and Exception Event Aggregator for logging and handling critical events.
"""

# System modules
import logging
import logging.handlers
from pathlib import Path
import threading
from typing import Set

# Log entry format constants
# Standardized log format for all log entries (e.g. 2025-01-15 12:34:56.789 | INFO | [SYSTEM::ROOT] | This is a log message)
LOG_ENTRY_FORMAT = "%(asctime)s.%(msecs)03d | %(levelname)s | [%(name)s] | %(message)s"
# Standardized time format for all log entries (e.g. 2025-01-15 12:34:56)
ISO_TIME_FORMAT = "%Y-%m-%d %H:%M:%S"
LOG_FORMATTER = logging.Formatter(LOG_ENTRY_FORMAT, ISO_TIME_FORMAT)

LOG_PATH = "./logs"  # Location of log files (TODO) Later make this configurable via config file or environment variable (e.g. /var/log/agisys)

class EEEHeaderFilter(logging.Filter):
    """
    This is a custom filter to standardize the log format.
    It replaces dots with colons and uppers the text to standardize subsystem/module names.
    eg. "vcs.vtc.pump" becomes "VCS::VTC::PUMP"
    """
    def filter(self, record):
        record.name = record.name.replace('.', '::').upper()
        return True
    
class EEEAggregator():
    """
    This is my centralized Emergency and Exception Event (EEE) Aggregator, which
    collects and manages all emergency and exception events across the system.
    It provides a unified interface for logging, alerting, and handling critical
    events, ensuring that they are addressed promptly and effectively.
        
    Usage:
        eee = EEEAggregator(caller="VCS.VTC")
        eee.info("Hello, World!")
        eee.fatal("RED ALERT! My core is overheating!")
        
    That's all you need to do to log messages from anywhere in the system.
    It automatically sets up logging on first use, and creates log files as needed.
    """
    
    # Class-level flag to ensure setup happens only once
    _initialized = False
    _systems: Set[str] = set()
    _thread_lock = threading.Lock()
    
    def __init__(self, caller: str):
        """
        This is where the logger is created.
        It auto-initializes the logging system on first use, including root and subsystem loggers,
        while also dynamically registers the subsystem logger if not already done.        
        
        Args:
            caller: The system/node/module which is calling this logger (e.g. "VCS.VTC.Pump")
        """
        # Auto-initialize on first use
        if not EEEAggregator._initialized:
            with EEEAggregator._thread_lock:
                if not EEEAggregator._initialized:  # Double-check
                    EEEAggregator._setup_logging_system(caller)
                    EEEAggregator._initialized = True
        
        # Set up the logger for this instance
        self.logger = logging.getLogger(caller)

        # Dynamically register system file if new
        parts = [p for p in caller.split('.') if p.strip()]
        if parts:
            system = parts[0].upper()
            if system not in EEEAggregator._systems:
                with EEEAggregator._thread_lock:
                    if system not in EEEAggregator._systems:
                        EEEAggregator._setup_system_logger(system)
                        EEEAggregator._systems.add(system)
    
    @staticmethod
    def _setup_logging_system(caller: str):
        """
        This is the setup for the entire logging system.
        It creates the log directory, root logger, and robot system logger,
        and configures log handlers for file and console output at different levels.

        This is a Triple-Action Logging & Ledgering Implementation (TALLI):
        - Master log file: All logs (DEBUG and above)
        - Error log file: Errors only (ERROR and above)
        - System log files: Separate log files for each system (DEBUG and above)
        - plus Console output: Info and above (INFO and above)

        Args:
            caller: The system/node/module which is calling this logger (e.g. "VCS.VTC.Pump")
        """
        
        # Create log directory if it doesn't exist
        log_path = Path(LOG_PATH).resolve()
        log_path.mkdir(parents=True, exist_ok=True)
        
        # Set up root logger for the entire system
        root_logger = logging.getLogger()
        root_logger.setLevel(logging.DEBUG)
        root_logger.addFilter(EEEHeaderFilter())
        
        # Clear existing handlers to avoid duplication
        root_logger.handlers.clear()
                
        # Master log file that logs all messages
        master_logger = logging.handlers.RotatingFileHandler(
            log_path / "robot.log",
            maxBytes=50*1024*1024,  # 50MB
            backupCount=5
        )
        master_logger.setLevel(logging.DEBUG)
        master_logger.setFormatter(LOG_FORMATTER)
        root_logger.addHandler(master_logger)
        
        # Error log file that logs errors only
        error_logger = logging.handlers.RotatingFileHandler(
            log_path / "errors.log",
            maxBytes=10*1024*1024,  # 10MB
            backupCount=3
        )
        error_logger.setLevel(logging.ERROR)
        error_logger.setFormatter(LOG_FORMATTER)
        root_logger.addHandler(error_logger)

        # Console output for info and above
        console = logging.StreamHandler()
        console.setLevel(logging.INFO)
        console.setFormatter(LOG_FORMATTER)
        root_logger.addHandler(console)

        # Set up system logger for the caller's system
        system = caller.split('.')[0] if caller else "SYSTEM"
        EEEAggregator._setup_system_logger(system)
    
    @staticmethod
    def _setup_system_logger(system: str):
        """
        This is the setup for a specific system's logger.
        It creates a log file for the specified system.
        
        Args:
            system: The system/node/module which is calling this logger (e.g. "VCS")
        """
        
        # Create log directory if it doesn't exist
        log_path = Path(LOG_PATH).resolve()
        log_path.mkdir(parents=True, exist_ok=True)

        system_logger = logging.getLogger(system)
        system_logger.setLevel(logging.DEBUG)
        system_logger.propagate = True
        
        logger_file = str(log_path / f"{system.lower()}.log")

        # Check if handler for this file already exists
        if not any(isinstance(h, logging.handlers.RotatingFileHandler) and h.baseFilename == logger_file for h in system_logger.handlers):

            # Create a new handler if it doesn't exist
            system_handler = logging.handlers.RotatingFileHandler(
                logger_file,
                maxBytes=20*1024*1024,  # 20MB
                backupCount=3
            )
            system_handler.setLevel(logging.DEBUG)
            system_handler.setFormatter(LOG_FORMATTER)
            system_logger.addHandler(system_handler)
        
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
        eee = EEEAggregator(caller="VCS.VTC")
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