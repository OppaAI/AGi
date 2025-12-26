# Changelog

This repository contains multiple ROS packages. For granular technical details, see the package-specific logs:
- [VCS Changelog (Server)](./AIVA/src/vcs/CHANGELOG.rst)
- [VCS Changelog (Robot)](./AuRoRA/src/vcs/CHANGELOG.rst)

## 0.1.1 - 2025-12-26
### Changed
- [VCS] Separated vital core (vcc and vtc) functions into its own package:
  - vcc - Vital Central Core (AIVA/VCS)
  - vtc - Vital Terminal Core (AuRoRA/VCS)

### Fixed
- [VCS] Fixed Time Logic issue by using ROS time instead of system clock ([#3](https://github.com/OppaAI/AGi-ROS/pull/3)) (@OppaAI)
- [VCS] Fixed print Bottleneck issue by using separate thread for terminal display ([#2](https://github.com/OppaAI/AGi-ROS/pull/2)) (@OppaAI)

## 0.1.0 - 2025-12-19
[VCS] _Initial non-production release for testing._

### Added
- [VCS] 2 nodes: vital_pulse_analyzer and vital_pulse_collector
- [VCS] vital_pulse_collector (AuRoRA/VCS): Collects vital pulses from robot and sends to server
- [VCS] vital_pulse_analyzer (AIVA/VCS): Receives vital pulses from robots, tracks connected robots/users, and detects timeouts/disconnections

