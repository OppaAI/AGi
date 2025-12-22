# Vital Circulatory System (VCS) for Robotics

## Overview

The **Vital Circulatory System (VCS)** is a conceptual framework for managing the "heartbeat" and vital signals of robots (VTC) and servers (VCC) in a distributed ROS2 architecture. It mimics human physiology at a simplified level to organize systems, modules, and nodes for robotics operations.  

The Vital Circulatory System (VCS) is designed to:  
- Continuously monitor the health of robots through **heartbeat signals**.  
- Enable the server to process and respond to robot vital data in real-time.  
- Verify and maintain a reliable connection between robots and the server.

---

## System Architecture

The system is divided into:

| Type    | Name             | Module | Node / Executable        |
|--------|-----------------|--------|-------------------------|
| Robot  | AuRoRA              | VTC    | vp_generator            |
| Server | AIVA       | VCC    | vp_analyzer             |

- **VTC (Vital Terminal Core)** – Robot module for sending vital data.  
- **VCC (Vital Central Core)** – Server module for receiving and analyzing robot vital data.  

---

### Namespace / Topic Naming Convention

We follow this hierarchical structure in ROS2:

```

/<Robot_or_Server_Name>/<System_Name>/<Module_Name>/<Node_Name>

````

- **Robot/Server Name**: Identifies the individual machine (`AuRoRA`, `AIVA`)  
- **System Name**: Logical grouping (`vcs`)  
- **Module Name**: Functional module (`vtc` for robot, `vcc` for server)  
- **Node Name**: Individual executable node (`vp_generator`, `vp_analyzer`)  

**Topics** are within the namespace:

- `/robot_AuRoRA/vcs/vtc/vp_generator/vital_pulse`  
- `/robot_AuRoRA/vcs/vtc/vp_generator/vital_feedback`  
- `/AIVA/vcs/vcc/vp_analyzer/vital_pulse` (subscription)

---

## System Diagram

```mermaid
flowchart TD
    Robot["AuRoRA (Robot VCS)"]
    Server["AIVA (Server VCS)"]

    subgraph RobotModule ["Robot VTC Module"]
        vp_generator["vp_generator (Node/Executable)"]
    end

    subgraph ServerModule ["Server VCC Module"]
        vp_analyzer["vp_analyzer (Node/Executable)"]
    end

    Robot --> vp_generator
    Server --> vp_analyzer

    vp_generator -->|vital_pulse| vp_analyzer
    vp_analyzer -->|vital_feedback| vp_generator

````

---

## Naming Conventions

| Concept           | ROS2 Equivalent         | Example                               |
| ----------------- | ----------------------- | ------------------------------------- |
| System            | Namespace (optional)    | `vcs`                                 |
| Module            | Sub-namespace / Package | `vtc` (robot), `vcc` (server)         |
| Node              | Node Name / Executable  | `vp_generator`, `vp_analyzer`         |
| Function / Action | Function called in node | `generate_pulse()`, `analyze_pulse()` |

* **Node names** are nouns (objects).
* **Function names / executables** are verbs (actions).

---

## Build ROS2 Worksapce
Go to the directory of the repo (eg. ~/AGi_ROS)

```bash
# Build ROS2 workspace
colcon build
source install/setup.bash
```

---

## Usage Examples

### Robot (VTC)

```bash
# Run vital pulse generator
ros2 run vtc vp_generator
```

### Server (VCC)

```bash
# Run vital pulse analyzer
ros2 run vcc vp_analyzer
```

### Topics Overview

```bash
ros2 topic list
# /<robotID>/vcs/vtc/vp_generator/vital_pulse
# /<robotID>/vcs/vtc/vp_generator/vital_feedback
```

---

## Code 

### vp_generator.py (Robot VTC)
Periodically publishes lightweight pulses containing telemetry data and timestamp:

→ [`AuRoRA/src/vcs/vcs/vp_generator.py`](https://github.com/OppaAI/AGi-ROS/blob/main/AuRoRA/src/vcs/vcs/vp_generator.py)


### vp_analyzer.py (Server VCC)
Monitors incoming pulses, tracks connected robots/users, and detects timeouts/disconnections in real time.  
→ [`AIVA/src/vcs/vcs/vp_analyzer.py`](https://github.com/OppaAI/AGi-ROS/blob/main/AIVA/src/vcs/vcs/vp_analyzer.py)


---

## Contributing

* Use consistent naming conventions: noun for nodes, verb for functions/executables.
* Keep modules organized within the system namespace (`vcs`).
* Ensure topic names are descriptive of the action or data transmitted.

---

## License

GPL-3.0-only

---

## Author

OppaAI

```

