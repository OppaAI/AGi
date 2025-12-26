^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vcs
^^^^^^^^^^^^^^^^^^^^^^^^^

System Name: Vital Circulatory System (VCS)
Description: This system monitors and manages the vital signs of AGi robots, including their operational status, user interactions, and network connectivity. It provides real-time feedback and alerts for critical conditions, ensuring the smooth operation of the AGi ecosystem.

0.1.0 (2025-12-19)
------------------
* Initial release
* Implemented 2 nodes: vital_pulse_analyzer and vital_pulse_collector
* vital_pulse_collector (AuRoRA/VCS): Collects vital pulses from robot and sends to server
* vital_pulse_analyzer (AIVA/VCS): Receives vital pulses from robots, tracks connected robots/users, and detects timeouts/disconnections

0.1.1 (2025-12-26)
------------------
* Fixed Time Logic issue by using ROS time instead of system clock (#3)
* Fixed print Bottleneck issue by using separate thread for terminal display (#2)
* Separated vital core (vcc and vtc) functions into its own package:
  - vcc - Vital Central Core (AIVA/VCS)
  - vtc - Vital Terminal Core (AuRoRA/VCS)
