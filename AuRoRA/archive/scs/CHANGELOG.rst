^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package Semantic Cognitive System
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**System Name:** Semantic Cognitive System (SCS)

**Description:** This system is the thinking/reasoning/cognitive/guardrail, basically the command centre, of AGi robots, that receive inputs from users, environments and sensors, and provides feedbacks to different
systems in the robots and to users.

Future Implementation Roadmap
-----------------------------
* Add the following modules to Igniter:
* - Bio-Logic Clock: Use a centralized ROS timer for all modules and nodes in all systems in the whole robot
* - Memory: Add robot specs and user/admin configurations to YAML files and load during startup

Forthcoming
-----------
* Initial Implementation
* Added Igniter (Temporary) as a placeholder for the future robot ignition sequence (bootstrap)
* Added EEEAggregator TALLE error logger to centralize logging of emergency and exception events:
* - Injector: Collects log entries from various modules and ingests them into the logger
* - Filtrator: Standardizes log entry format
* - Propagator: Propagates log entries to multiple loggers/handlers based on severity level
*     - TALLE uses 3 channels + 1 console output (for debugging) for logging and ledgering of emergency and exception events:
*         - Master log: All logs (DEBUG and above)
*         - Error log: Errors only (ERROR and above)
*         - System log files: Separate log files for each system (DEBUG and above)
*         - Console output: Info and above (INFO and above)
* Added Deduplication mechanism to prevent log floods
* Added Token Bucket Throttle to rate limit logging
* Added plugin system to EEEAggregator to allow for easy extension
* Added EOS ROS Bridge plugin to bridge EEE to ROS 2 diagnostics and rosout
* Added Reflex Plugin to bridge EEE to ROS 2 diagnostics
* Added Awarenes sPlugin to bridge EEE to ROS 2 rosout
* Added Health query service to check the health of the robot
* Add File rotation + gzip,  Batch Ledger inserts, Richer health response, Per-proc throttling in Reflex, and STALE support
* Updated to allow more robust CID in Awareness (/rosout) and added metrics topic (/eee/metrics)

0.1.3 (2026-01-26)
------------------
* N/A

0.1.2 (2026-01-01)
------------------
* N/A

0.1.1 (2025-12-26)
------------------
* N/A

0.1.0 (2025-12-19)
------------------
* N/A

===============================
SCS Future Implementation Roadmap
===============================

This section documents planned and discussed features for the
Semantic Cognitive System (SCS). These items reflect architectural
decisions, future extensions, and long-term evolution goals.

The roadmap is organized by subsystem and implementation phase.

--------------------------------
1. Core VCS Architecture
--------------------------------

- TBD
