# AGi — Autonomous General Intelligence

**AuRoRA** · Autonomous Robot with Reasoning Architecture  
**Author:** [OppaAI](https://github.com/OppaAI) · Beautiful British Columbia, Canada

[![Repo](https://img.shields.io/badge/Repo-OppaAI%2FAGi-76B900)](https://github.com/OppaAI/AGi)
![Status](https://img.shields.io/badge/Status-experimental-orange.svg)
[![License](https://img.shields.io/badge/License-GPL--3.0-blue.svg)](https://opensource.org/licenses/GPL-3.0)

![ARM](https://img.shields.io/badge/ARM64-aarch64-0091BD?logo=arm)
![LLM](https://img.shields.io/badge/Model-Cosmos%20Reason2%202B-76B900?logo=nvidia)
![JetPack](https://img.shields.io/badge/JetPack-6.2.2-76B900?logo=nvidia)
![CUDA](https://img.shields.io/badge/CUDA-12.6-76B900?logo=nvidia)

For more comprehensive doucmentation: [![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/OppaAI/AGi)

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
| LiDAR | YDLIDAR D500 360 |
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

### Phase 2 — Voice Output

| Milestone | Description | Status |
|---|---|---|
| M4 | TTS on robot — Piper CPU streaming | ⬜ Planned |
| M5 | TTS in web GUI — browser audio playback | ⬜ Planned |

### Phase 3 — Multimodal Input

| Milestone | Description | Status |
|---|---|---|
| M6 | Image input — camera + vision | ⬜ Planned |
| M7 | ASR — on-device speech to text | ⬜ Planned |
| M8 | Messaging — Gmail, Slack, Telegram | ⬜ Planned |

### Phase 4 — Hardware / Autonomy

| Milestone | Description | Status |
|---|---|---|
| M9 | Motors + LiDAR + OAK-D integration | ⬜ Planned |
| M10 | Navigation + SLAM | ⬜ Planned |
| M11 | Agentic mission execution | ⬜ Planned |

---

## Architecture

```mermaid
flowchart TD
    GUI[GUI - AGi.html]
    IN[cns/neural_input]
    OUT[gce/response]

    subgraph SCS[Semantic Cognitive System]
        CNC[CNC - Central Neural Core]
        MCC[MCC - Memory Coordination Core]
        WMC[WMC - Working Memory Cortex]
        EMC[EMC - Episodic Memory Cortex]
        BUF[em_buffer - crash-safe intake]
    end

    COSMOS[Cosmos Reason2 2B - vLLM]
    DB[(emc.db - SQLite)]
    EMBED[embeddinggemma-300m - CPU]

    GUI -->|user message| IN
    IN --> CNC
    CNC --> MCC
    MCC --> WMC
    MCC --> EMC
    WMC -->|overflow| BUF
    BUF -->|async embed| EMBED
    EMBED --> EMC
    EMC --- DB
    MCC -->|context window| CNC
    CNC -->|stream request| COSMOS
    COSMOS -->|SSE tokens| CNC
    CNC -->|chunks| OUT
    OUT --> GUI
```

---

## Conversation Sequence

```mermaid
sequenceDiagram
    participant GUI
    participant CNC
    participant MCC
    participant WMC
    participant EMC
    participant COSMOS as Cosmos vLLM

    GUI->>CNC: /cns/neural_input
    activate CNC

    CNC->>MCC: add_turn user
    MCC->>WMC: add_turn
    WMC-->>MCC: evicted turns
    MCC-->>EMC: buffer_append async

    CNC->>MCC: build_context
    par concurrent
        MCC->>WMC: get_turns
        WMC-->>MCC: active turns
    and
        MCC->>EMC: search top-k
        EMC-->>MCC: past episodes
    end
    MCC-->>CNC: messages with context

    CNC->>COSMOS: POST stream
    loop tokens
        COSMOS-->>CNC: token
        CNC-->>GUI: start / delta
    end
    CNC-->>GUI: done

    CNC->>MCC: add_turn assistant
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
pip3 install -r requirements.txt --break-system-packages

# 3. Build
colcon build --packages-select scs
source install/setup.bash

# 4. Start Cosmos vLLM
bash launch/cosmos.sh
# Wait ~3 min for: Application startup complete

# 5. Start GRACE
ros2 run scs cnc

# 6. Start rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# 7. Open GUI
python3 -m http.server 9413 --directory src/scs/scs
# Open: http://<jetson-ip>:9413/AGi.html
```

---

## Built by

Solo developer — Beautiful British Columbia, Canada. No CS/ML degree.  
Just curiosity, a tracked robot, and NVIDIA Cosmos Reason 2 on a Jetson.

Previous project: [ERIC — Edge Robotics Innovation by Cosmos](https://github.com/OppaAI/eric)  
Built for the NVIDIA Cosmos Cookoff 2026.
