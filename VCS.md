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

### Namespace / Topic Naming Convention

We follow this hierarchical structure in ROS2:
/<Robot_or_Server_Name>/<System_Name>/<Module_Name>/<Node_Name>

- **Robot/Server Name**: Identifies the individual machine (`Jon`, `Grace_Home`)  
- **System Name**: Logical grouping (`vcs`)  
- **Module Name**: Functional module (`vtc` for robot, `vcc` for server)  
- **Node Name**: Individual executable node (`vp_generator`, `vp_analyzer`)  

**Topics** are within the namespace:

- `/robot_jon/vcs/vtc/vp_generator/vital_pulse`  
- `/robot_jon/vcs/vtc/vp_generator/vital_feedback`  
- `/Grace_Home/vcs/vcc/vp_analyzer/vital_pulse` (subscription)

---

## System Diagram

```mermaid
flowchart TD
    Robot[Jon (Robot VTC)]
    Server[Grace_Home (Server VCC)]

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

###Naming Conventions
Concept	ROS2 Equivalent	Example
System	Namespace (optional)	vcs
Module	Sub-namespace / Package	vtc (robot), vcc (server)
Node	Node Name / Executable	vp_generator, vp_analyzer
Function / Action	Function called in node	generate_pulse(), analyze_pulse()

Node names are nouns (objects).

Function names / executables are verbs (actions).

###Usage Examples
Robot (VTC)
# Run vital pulse generator
ros2 run vtc vp_generator


Contributing

Use consistent naming conventions: noun for nodes, verb for functions/executables.

Keep modules organized within the system namespace (vcs).

Ensure topic names are descriptive of the action or data transmitted.

License

GPL-3.0-only

---

😊：This structure explains your **VCS**, shows **namespace/module/node hierarchy**, includes **Mermaid diagram**, installation, usage, and coding guidance.  

If you want, I can also **expand the diagram** to show **multiple robot nodes, multiple modules, and message flows**, making it more visual for GitHub. Do you want me to do that next?


Author

OppaAI
