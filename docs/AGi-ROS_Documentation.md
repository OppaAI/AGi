# AGi-ROS Comprehensive Documentation

## 1. Overview
AGi-ROS is a ROS 2 (Humble) project that explores a modular, bio-inspired approach to building intelligent robotics systems. The repository is split into two main programs:

- **AIVA**: the server-side "AI Virtual Assistant" responsible for coordination and analysis services.
- **AuRoRA**: the robot-side "Autonomous Rover Robot Assistant" responsible for sensing, local autonomy, and ROS 2 node execution.

A foundational subsystem in the current codebase is the **Vital Circulatory System (VCS)**, which provides a heartbeat mechanism for monitoring robot/server connectivity and vital telemetry. The VCS is built around a **Vital Terminal Core (VTC)** on the robot and a **Vital Central Core (VCC)** on the server, communicating over a custom `VitalPulse` ROS message.

## 2. Repository Layout
```
AGi-ROS/
├── AIVA/                    # Server-side ROS 2 packages and utilities
│   ├── src/
│   │   └── vcs/              # VCS server implementation (VCC)
│   └── vp/                   # VitalPulse message definition
├── AuRoRA/                   # Robot-side ROS 2 packages and utilities
│   ├── src/
│   │   ├── vcs/              # VCS robot implementation (VTC)
│   │   ├── scs/              # Supporting system components
│   │   └── vp/               # VitalPulse message definition (build support)
├── docs/                     # Documentation and design notes
└── README.md                 # High-level project overview
```

## 3. Core Concepts
### 3.1 AIVA (Server) and AuRoRA (Robot)
AIVA and AuRoRA represent the two ends of the system:

- **AuRoRA** hosts robot-local sensing, health telemetry, and low-latency control loops.
- **AIVA** aggregates signals from one or more robots, provides monitoring, and is designed to host higher-level reasoning services.

### 3.2 Vital Circulatory System (VCS)
The VCS is a communication layer for robot health and liveness. It exposes a heartbeat-like flow of telemetry and uses periodic feedback to determine connection health.

Key elements:
- **VTC (Vital Terminal Core)**: Robot-side node that gathers system telemetry, packages it into `VitalPulse` messages, and publishes them on a fixed cadence. Implemented in `vcs/vtc.py` and supported by the `Pump` module in `vcs/pump.py` for data collection and aggregation.
- **VCC (Vital Central Core)**: Server-side node that receives pulses, computes RTT and connection status, and publishes feedback to the robot. Implemented in `vcs/vcc.py`.
- **VitalPulse message**: Custom ROS message that carries robot identity, timestamps, and CPU/GPU temperature payloads.

## 4. VCS Architecture
### 4.1 Message Definition
The VCS uses the `VitalPulse` message type:
```
string robot_id
string user_id
builtin_interfaces/Time timestamp
float64 vital_pulse_opm
float64 cpu_temp
float64 gpu_temp
```

### 4.2 Robot-Side VTC (Vital Terminal Core)
The VTC node is responsible for:
- Collecting health data using the **Pump** module (e.g., CPU/GPU temps and system status).
- Publishing heartbeat pulses at a computed oscillation rate.
- Receiving feedback to detect RTT and connection liveness.

The current VTC implementation in `vcs/vtc.py` includes placeholders for a full "pump → regulator → oscillator → orchestrator" pipeline and already handles:
- Pump timing and data snapshotting.
- Publishing `VitalPulse` messages with CPU/GPU temperatures.
- Tracking feedback and RTT.

### 4.3 Server-Side VCC (Vital Central Core)
The VCC node is responsible for:
- Subscribing to the VTC heartbeat topic.
- Tracking last pulse time, RTT, and robot identity.
- Publishing feedback pulses back to the robot.
- Displaying status in the terminal for quick monitoring.

The current VCC implementation in `vcs/vcc.py` shows the server-side analyzer logic, including timeout handling and status output.

### 4.4 Topics and Namespaces
Namespaces are constructed as:
```
/<robot_id>/<system_name>/<node_name>
```
For the VCS nodes in the current implementation:
- Namespace base: `/<robot_id>/VCS`
- Topics:
  - `vital_pulse_signal` (robot → server)
  - `vital_pulse_response` (server → robot)

## 5. Implementation Details
### 5.1 Pump Module (Robot-Side)
The `Pump` class in `vcs/pump.py` reads system vitals and maintains a "stream of consciousness" buffer. It uses a multi-rate sampling strategy (HI/MID/LO channels) and stores time-series data in fixed-size deques. This design supports both high-frequency and low-frequency telemetry while keeping memory bounded.

### 5.2 QoS Profiles
Both VTC and VCC use a `BEST_EFFORT` QoS profile with short lifetimes, matching the nature of heartbeat signals. Liveliness and deadline policies are configured to reflect expected heartbeat intervals and to detect connection loss quickly.

## 6. Build & Run (VCS Demo)
### 6.1 Build the ROS 2 Workspace
From the repository root:
```bash
colcon build --symlink-install
source install/setup.bash
```

### 6.2 Launch the VCS Nodes
The `vcs.launch.py` file can run either side based on the `role` argument:

Robot-side (VTC):
```bash
ros2 launch vcs vcs.launch.py role:=robot
```

Server-side (VCC):
```bash
ros2 launch vcs vcs.launch.py role:=server
```

> Note: The AuRoRA launch file includes the `scs` igniter node in addition to the VTC/VCC nodes.

## 7. Roadmap (Suggested Doc Expansion)
This documentation can be expanded over time to include:
- Full agent loop design (sense → reason → plan → act).
- LLM adapter architecture and prompt pipelines.
- Mapping between ROS 2 actions/services and high-level skills.
- Simulation workflows (Gazebo/Ignition).
- Safety constraints and guardrails for physical robot deployment.

## 8. How to Cite / Acknowledge
If you plan to reference AGi-ROS in a publication:
- Mention the VCS subsystem as the current implemented backbone.
- Cite the repository and link to the `docs/AGi-ROS_Paper.tex` preprint template for the formal write-up.

---

## Appendix A: Suggested Documentation TODOs
- Add a diagram of the full agent stack.
- Provide per-package API references and ROS interface docs.
- Add unit/integration test workflows.
- Provide sample configs for robot and server deployments.
