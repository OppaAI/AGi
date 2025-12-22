# Vital Circulatory System (VCS) for Robotics

## Overview

The **Vital Circulatory System (VCS)** is a conceptual framework for managing the "heartbeat" and vital signals of robots (VTC) and servers (VCC) in a distributed ROS2 architecture. It mimics human physiology at a simplified level to organize systems, modules, and nodes for robotics operations.  

VCS is designed to:  
- Ensure continuous monitoring of robot health via **heartbeat signals**.  
- Allow servers to process and respond to robot vital data.  
- Provide a clear naming convention for nodes, packages, and topics to map the system intuitively.

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
    Robot[AuRoRA (Robot VTC)]
    Server[AIVA (Server VCC)]

    Robot -->|vital_pulse| Server
    Server -->|vital_feedback| Robot

    subgraph Robot Module
        vp_generator["vp_generator (Node/Executable)"]
    end
    Robot --> vp_generator

    subgraph Server Module
        vp_analyzer["vp_analyzer (Node/Executable)"]
    end
    Server --> vp_analyzer
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

## Installation

```bash
# Clone repository
git clone https://github.com/<username>/vcs.git
cd vcs

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
# /robot_AuRoRA/vcs/vtc/vp_generator/vital_pulse
# /robot_AuRoRA/vcs/vtc/vp_generator/vital_feedback
```

---

## Code Examples

### vp_generator.py (Robot VTC)

```python
#!/usr/bin/env python3
# [Robot node code snippet here]
```

### vp_analyzer.py (Server VCC)

```python
#!/usr/bin/env python3
# [Server node code snippet here]
```

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

---

😊：This is a clean, ready-to-push **full README** for your VCS system.  

If you like, I can **also add a second Mermaid diagram showing multiple robots and multiple modules communicating with the server**, to make it visually obvious how VCS scales. Do you want me to do that?
```
