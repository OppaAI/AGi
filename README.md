# AGi — Autonomous General Intelligence

**AuRoRA** · Autonomous Robot with Reasoning Architecture  
**Author:** [OppaAI](https://github.com/OppaAI) · Beautiful British Columbia, Canada  
**License:** GPL-3.0-only

---

A clean-slate rebuild of my autonomous robot project, starting from first principles.  
After building [ERIC](https://github.com/OppaAI/eric) for the NVIDIA Cosmos Cookoff 2026, I learned what I would do differently — proper ROS2 architecture from day one, a biologically-inspired memory system, and a foundation that can grow into full autonomy.

The goal: build an autonomous ground robot that can explore nature with me, powered by on-device AI with no cloud dependency.

---

## Hardware

| Component | Model |
|---|---|
| SBC | Jetson Orin Nano Super 8GB |
| Robot | Waveshare UGV Beast (tracked) |
| LiDAR | YDLIDAR D500 360° |
| Depth Camera | OAK-D Lite (stereo + YOLO) |
| Pan-tilt + Webcam | USB |
| Storage | 1TB NVMe |

---

## Stack

- **Cosmos Reason2 2B** via vLLM — vision + reasoning brain
- **ROS2 Humble** — full native architecture from day one
- **embeddinggemma-300m** — CPU-only semantic embeddings
- **SQLite** — lightweight on-device memory storage
- **rosbridge** — WebSocket bridge to web GUI

---

## Repository Structure

```
AGi/
├── AuRoRA/          # Robot workspace (Jetson Orin Nano)
│   └── src/
│       └── scs/     # Semantic Cognitive System
│           └── scs/
│               ├── cnc.py   # Central Neural Core (ROS2 node)
│               ├── mcc.py   # Memory Coordination Core
│               ├── wmc.py   # Working Memory Cortex
│               └── emc.py   # Episodic Memory Cortex
│
└── AIVA/            # Server workspace (PC) — future
    └── src/
```

---

## Roadmap

### Phase 1 — Chatbot with Memory
| Milestone | Description | Status |
|---|---|---|
| M1 | Chatbot + Working Memory (WMC) + Episodic Memory (EMC) | 🟢 In Progress |
| M2 | + Semantic Memory (SMC) + 11pm daily reflection | ⬜ Planned |
| M3 | + Procedural Memory (PMC) | ⬜ Planned |

### Phase 2 — Hardware / Autonomy
| Milestone | Description | Status |
|---|---|---|
| M4 | Motors + LiDAR + OAK-D integration | ⬜ Planned |
| M5 | Navigation + SLAM | ⬜ Planned |
| M6 | Agentic mission execution | ⬜ Planned |

---

## Memory Architecture

GRACE's memory is modelled on the human brain — four cortices, each serving a distinct role, coordinated by the MCC.

```mermaid
flowchart TD
    GUI["GUI — AGi.html\nrosbridge ws://jetson:9090"]
    ROS_IN["/cns/neural_input"]
    ROS_OUT["/gce/response"]

    subgraph SCS["SCS — Semantic Cognitive System"]
        CNC["CNC\nCentral Neural Core\nROS2 node · asyncio"]
        MCC["MCC\nMemory Coordination Core"]
        WMC["WMC\nWorking Memory Cortex\ndeque · 1400 tokens"]
        EMC["EMC\nEpisodic Memory Cortex\nforever · SQLite"]
        BUF["em_buffer\ncrash-safe intake"]
    end

    COSMOS["Cosmos Reason2 2B\nvLLM · localhost:8000"]
    DB[("emc.db\nSQLite WAL")]
    EMBED["embeddinggemma-300m\nCPU · 768-dim"]

    GUI -->|user message| ROS_IN
    ROS_IN --> CNC
    CNC --> MCC
    MCC --> WMC
    MCC --> EMC
    WMC -->|overflow| BUF
    BUF -->|async embed| EMBED
    EMBED --> EMC
    EMC --- DB
    MCC -->|context window| CNC
    CNC -->|POST stream| COSMOS
    COSMOS -->|SSE tokens| CNC
    CNC -->|publish chunks| ROS_OUT
    ROS_OUT --> GUI
```

---

## Conversation Sequence

A single conversation turn — from user input to GRACE's response.

```mermaid
sequenceDiagram
    participant GUI as GUI
    participant CNC as CNC
    participant MCC as MCC
    participant WMC as WMC
    participant EMC as EMC
    participant COSMOS as Cosmos vLLM

    GUI->>CNC: /cns/neural_input
    activate CNC

    CNC->>MCC: add_turn("user", msg)
    MCC->>WMC: add_turn()
    WMC-->>MCC: evicted turns (if full)
    MCC-->>EMC: buffer_append(evicted) [async]

    CNC->>MCC: build_context(user_input)
    par concurrent
        MCC->>WMC: get_turns()
        WMC-->>MCC: active turns
    and
        MCC->>EMC: search(query, top_k=3)
        EMC-->>MCC: relevant past episodes
    end
    MCC-->>CNC: messages[] with memory context

    CNC->>COSMOS: POST /v1/chat/completions stream=true
    loop streaming tokens
        COSMOS-->>CNC: token
        CNC-->>GUI: /gce/response {type: start/delta}
    end
    COSMOS-->>CNC: DONE
    CNC-->>GUI: /gce/response {type: done}

    CNC->>MCC: add_turn("assistant", response)
    deactivate CNC
```

---

## Quick Start

```bash
# 1. Clone
git clone https://github.com/OppaAI/AGi ~/AGi
cd ~/AGi/AuRoRA

# 2. Install deps
rosdep install --from-paths src --ignore-src -r -y
pip3 install httpx sentence-transformers Pillow "numpy<2" --break-system-packages

# 3. Build
colcon build --packages-select scs
source install/setup.bash

# 4. Start Cosmos vLLM
bash launch/cosmos.sh
# Wait ~3 minutes for: "Application startup complete"

# 5. Start GRACE
ros2 run scs cnc

# 6. Start rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# 7. Open GUI
# Serve AGi.html from Jetson:
python3 -m http.server 9413 --directory src/scs/scs
# Open in browser: http://<jetson-ip>:9413/AGi.html
```

---

## Built by

Solo developer — Beautiful British Columbia, Canada. No CS/ML degree.  
Just curiosity, a tracked robot, and NVIDIA Cosmos Reason 2 on a Jetson.

Previous project: [ERIC — Edge Robotics Innovation by Cosmos](https://github.com/OppaAI/eric)  
Built for the NVIDIA Cosmos Cookoff 2026.
