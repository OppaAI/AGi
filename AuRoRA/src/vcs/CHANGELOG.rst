^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package Vital Circulatory System
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**System Name:** Vital Circulatory System (VCS)

**Description:** This system monitors and manages the vital signs of AGi robots, including their operational status, user interactions, and network connectivity. It provides real-time feedback and alerts for critical conditions, ensuring the smooth operation of the AGi ecosystem.

Future Implementation Roadmap
-----------------------------
* Add the following modules to VTC:
* - Regulator: Normalizes raw sensor data into proper format and regulates the oscillation rhythm according to RTT
* - Oscillator: Packs and encodes sensor data into vital pulse signal and publishes at oscillation rhythm
* - Orchestrator: Monitors vital pulse signal and response, detects disconnections, and triggers safety interlocks


Forthcoming
-----------
* Added launch file to launch vtc and vcc nodes based on role parameter
* Changed the threaded terminal print to use rclpy logger
* Added QoS Profile to vital pulse signal and response
* Established the framework to implement PRO2 architectural for VTC:
* - Pump: Collects glob from different lifestream flow channels
* Decoupled the Pump Module to its own module to have better modularity
* Optimized lifestream collection using Triple Action Pump (TAP) technique to prioritize crucial glob while conserving memory footprints
* Implemented impurity detection and removal to ensure clean glob is harvested
* Transitioned the naming schema to a "Biological Hardware" schema with more evocative and memorable terminologies
* Added a standalone validation block in the pump moudle for real-time verification of its integrity


0.1.2 (2026-01-01)
------------------
* Fixed JSON Fragility issue by changing to ROS msg (#5)
* Fixed Logic Conflict of vital_pulse and vital_feedback by utilizing centralized output (#6)
* Optimized the codes in the nodes and packages for better performance and readability
* Combined back the vital core (vcc and vtc) functions into its own package, to be prepared for modularization

0.1.1 (2025-12-26)
------------------
* Fixed Time Logic issue by using ROS time instead of system clock (#3)
* Fixed print Bottleneck issue by using separate thread for terminal display (#2)
* Separated vital core (vcc and vtc) functions into its own package:
  - vcc - Vital Central Core (AIVA/VCS)
  - vtc - Vital Terminal Core (AuRoRA/VCS)

0.1.0 (2025-12-19)
------------------
* Initial release
* Implemented 2 nodes: vital_pulse_analyzer and vital_pulse_collector
* vital_pulse_collector (AuRoRA/VCS): Collects vital pulses from robot and sends to server
* vital_pulse_analyzer (AIVA/VCS): Receives vital pulses from robots, tracks connected robots/users, and detects timeouts/disconnections

===============================
VCS Future Implementation Roadmap
===============================

This section documents planned and discussed features for the
Vital Control System (VCS). These items reflect architectural
decisions, future extensions, and long-term evolution goals.

The roadmap is organized by subsystem and implementation phase.

--------------------------------
1. Core VCS Architecture
--------------------------------

- Formalize VCS as a distributed heartbeat and network-awareness system
- Maintain a biologically-inspired design:
  - Pump
  - Oscillator
  - Regulator
  - Pacemaker
- Ensure deterministic fast-path for network liveness and feedback
- Separate fast diagnostics from deep historical analysis

--------------------------------
2. Robot-side VCS (VTC Node)
--------------------------------

Planned features for the Vital Terminal Core (robot side):

- Unified VTC node containing:
  - Pump (raw metric gathering)
  - Oscillator (tag generation, protobuf wrapping)
  - Regulator (ROS publish / subscribe)
  - Pacemaker (OPM calculation and network liveness)
- Baseline OPM (60) with adaptive scaling based on RTT
- Automatic fallback to baseline OPM when:
  - Network timeout occurs
  - RTT exceeds threshold
  - Negative OPM persists for defined grace cycles
- Local lightweight model activation when server is offline
- Maximum OPM cap to avoid resource saturation
- Separate server_linked / network_state flag in feedback message

--------------------------------
3. Server-side VCS (VCC Nodes)
--------------------------------

Planned server architecture:

- Fast-path Analyzer Node:
  - Decrypt incoming Vital Pulse
  - Compute RTT
  - Determine OPM and color state
  - Send immediate Vital Feedback
- Slow Diagnostics Node (future):
  - Historical analysis
  - Statistical aggregation
  - Network quality profiling
  - Long-term health reporting

--------------------------------
4. Messaging and Data Model
--------------------------------

- ROS messages used only as transport envelopes
- Vital data payload encoded as Protobuf
- Encrypted payload carried inside ROS msg
- Stable ROS message fields:
  - tag
  - timestamp(s)
  - seq
  - encrypted protobuf payload
  - OPM
  - server_linked
- Tag generation based on:
  - ROS time
  - Sequence number
- Tag usable for:
  - Request/response matching
  - Encryption key derivation
  - Logging index
  - Visualization and replay

--------------------------------
5. Metrics Collection (Pump)
--------------------------------

Incremental metric rollout strategy:

- Initial metrics:
  - CPU temperature
  - GPU temperature
- Future metrics:
  - Memory usage
  - Battery level
  - Voltage
  - Sensor health flags
- Per-metric adjustable sampling frequency
- Pump acts as a data wrapper only (no decision logic)

--------------------------------
6. Configuration and Parameters
--------------------------------

- All constants externalized into YAML
- Separation of configuration domains:

  Admin (Critical, reboot required):
    - baseline_opm
    - max_opm
    - timeout thresholds
    - jitter tolerance
    - network classification logic

  User (Non-critical, runtime adjustable):
    - display colors
    - refresh rates
    - UI toggles

- ROS parameter enforcement:
  - Critical parameters reject runtime modification
  - UI parameters support live update
- Future WebUI support:
  - User mode for UI tuning
  - Admin mode for core tuning with reboot

--------------------------------
7. Logging and Storage Strategy
--------------------------------

Robot-side logging:

- Short-term memory (STM):
  - ROS 2 bag with MCAP + Protobuf
  - High-speed read for visualization and replay
- Logging includes:
  - tag
  - timestamps
  - encrypted protobuf payload

Server-side logging:

- Short-term memory:
  - Optional MCAP recording
- Long-term memory (LTM):
  - PostgreSQL for aggregated and administrative data
- NAS usage:
  - Backup and archival only
  - Not used for live database storage

--------------------------------
8. Visualization and Replay
--------------------------------

Planned visualization features:

- Vital Display Console (VCS Heart View)
- Real-time OPM and network state visualization
- Historical replay:
  - Animate Vital Pulse travel path
  - Visualize RTT and pulse speed
- Doctor-style inspection mode:
  - Live status
  - Past time windows (hourly, daily, weekly)

--------------------------------
9. Security and Encryption
--------------------------------

- Dedicated CryptoCore node (future)
- Encryption applied after protobuf serialization
- Tag used as part of encryption context
- Encrypted data logged without decryption
- Decryption only performed by authorized analysis components

--------------------------------
10. Adaptive Intelligence (Future)
--------------------------------

- Adaptive Small Language Model (SLM) integration
- Learning acceptable RTT / jitter patterns
- Dynamic tuning of:
  - OPM scaling
  - Timeout thresholds
- Gradual migration from static rules to learned behavior
- Integration after LLM and core VCS stabilization

--------------------------------
11. System Evolution Principles
--------------------------------

- Architecture-first, implementation-second
- Fast-path determinism over feature density
- Clear separation of:
  - Control vs visualization
  - Runtime vs persistent configuration
  - Short-term vs long-term memory
- Backward compatibility of recorded data
- Incremental, non-breaking extensions