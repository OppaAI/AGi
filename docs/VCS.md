# Vital Circulatory System (VCS) for Robotics

## Overview

The **Vital Circulatory System (VCS)** is a conceptual framework for managing the "heartbeat" and vital signals of robots (VTC) and servers (VCC) in a distributed ROS2 architecture. It mimics human physiology at a simplified level to organize systems, modules, and nodes for robotics operations, and provides robust, mutual awareness between robot and server, ensuring reliable operation in distributed AGI robot setups.
Inspired by biological circulatory systems, VCS acts as the "blood flow" that keeps the entire system alive and self-aware.

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

```mermaid
sequenceDiagram
    autonumber
    participant VTC as VTC (VitalTerminalCore)
    participant Pump as Pump (module)
    participant Reg as Regulator (module)
    participant Osc as Oscillator (module)
    participant Orc as Orchestrator (module)
    participant VCC as VCC (VitalPulseAnalyzer)
    participant Monitor as Operator / Dashboard

    Note over VTC,VCC: namespaces & nodes (from code)
    Note over VTC: node="vital_terminal_core", namespace="ROBOT_ID/VCS"
    Note over VCC: node="vital_pulse_analyzer", namespace="ROBOT_ID/VCS" or "AIVA/VCS"

    loop periodic heartbeat (oscillation_timer)
        VTC->>Pump: run_pump_cycle()  %% collect telemetry (body_temp etc.)
        Pump-->>VTC: vital_dump["pump"].glob (temperature, telemetry, etc.)
        VTC->>Reg: commit_vital_dump("pump", vital_dump)
        Reg-->>VTC: normalized payload (filtered telemetry)
        VTC->>Osc: oscillate_vital_pulse()  %% compute current_opm, timestamp
        Osc-->>VTC: VitalPulse msg (VitalPulse)
        Note right of VTC: VitalPulse fields: robot_id, user_id, timestamp (builtin_interfaces/Time), cpu_temp, gpu_temp, vital_pulse_opm
        VTC->>VCC: publish VitalPulse on topic "vital_pulse_signal" (QoS: BEST_EFFORT, Liveliness/Deadline/Lifespan configured)
        activate VCC
        VCC->>VCC: pulse_callback(msg)
        Note right of VCC: actions: record_identity(robot_id,user_id), record_remote_pulse(opm,timestamp)
        VCC->>VCC: update VitalCentralCore state (connected robots, last pulse time)
        VCC->>VCC: prepare feedback VitalPulse (robot_id=SERVER_ID, user_id=remote_robot_id, timestamp=now, vital_pulse_opm=pulse)
        VCC-->>VTC: publish feedback on "vital_pulse_response" (same QoS)
        deactivate VCC
        VTC->>VTC: feedback_callback(msg)  %% compute RTT, update last_feedback_time, vc_linked True
        alt RTT acceptable / vc_linked
            VTC->>Orc: notify linked status / clear timeout counters
        else timeout or missing feedback
            VTC->>Orc: trigger safety_interlock() / vc_linked False
            Orc-->>Monitor: alert / log (display update)
        end
        VTC->>Monitor: display_tick() updates terminal snapshot (heartbeat, opm, RTT)
    end

    Note over VTC,VCC: Topic examples (from docs)
    Note over VTC: "/<robotID>/vcs/vtc/vp_generator/vital_pulse" and "/<robotID>/vcs/vtc/vp_generator/vital_feedback"
    Note over VCC: "/AIVA/vcs/vcc/vp_analyzer/vital_pulse" (subscription) / publishes "vital_pulse_response"

    rect rgba(200,255,200,0.15)
    Note over VTC,VCC: QoS details (from code)
    Note over VTC,VCC: QoSProfile: depth=1, reliability=BEST_EFFORT, history=KEEP_LAST, durability=VOLATILE,
    Note over VTC,VCC: liveliness=AUTOMATIC, liveliness_lease_duration=VITAL_PULSE_TIMEOUT*1.5, lifespan=VITAL_PULSE_TIMEOUT, deadline~VITAL_PULSE_TIMEOUT*1.1
    end
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



```mermaid

graph TB
    subgraph Robot
        R1[VTC - Vital Pulse Generator 0.1.0]
        R2[Robot ID : SAID TBD]
        R3[Ner-ID]
        R4[Health Data]
        R5[Open GPU]
        R6[Vital Pulse Threat]
        R7[Vital Pulse Poke]
        R8[Vital Feedback]
    end

    subgraph Servers
        S1[VCC - Vital Pulse Receptor 0.1.0]
        S2[Robot ID : SAID TBD]
        S3[Ner-ID]
        S4[Health Data]
        S5[Open GPU]
        S6[Vital Pulse Threat]
        S7[Vital Pulse Poke]
        S8[Vital Feedback]
    end

    R1 -->|Vital Pulse Threat| S1
    R2 -->|Vital Pulse Threat| S1
    R6 -->|Vital Pulse Poke| S7
    R7 -->|Vital Feedback| R8
    R8 -->|Vital Feedback| S8
    S1 -->|Vital Feedback| R8
    S6 -->|Vital Feedback| R7
    S7 -->|Vital Feedback| R6
    R4 -->|Health Data| S4
    S5 -->|Health Data| R5
    R3 -->|Robot ID| S2
    S3 -->|Robot ID| R2
    R5 -->|Open GPU| S5
    S5 -->|Open GPU| R5

````
