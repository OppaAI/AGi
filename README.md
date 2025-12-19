# AGi-ROS (ROS2 Humble Project with LLM to achieve AGi)

[![repo](https://img.shields.io/badge/repo-OppaAI%2FAGi--ROS-darkcyan)](https://github.com/OppaAI/AGi-ROS)
![status](https://img.shields.io/badge/build-pending-lightgrey)
[![license: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0.en.html)

![Python 3.10](https://img.shields.io/badge/python-3.10-blue.svg)
![ROS2 Humble](https://img.shields.io/badge/ROS-2%20Humble-blue.svg)


AGi-ROS (AGi for ROS) is a modular framework to build and run intelligent agents that interact with robots and simulators through ROS. AGi-ROS combines perception adapters, planning modules, LLM-based reasoning, and execution components to enable high-level tasking, autonomous decision-making, and integration with standard robotics stacks.
I am only a one man team with limited programming capability since the era of QBASIC, and superficial knowledge of ML/DL/Agentic/Robotics.
This project may last for decades, but I just want to see how far I could go.

> WARNING: This README is only a raw blanket file with starter configurations, preliminary outlines, examples and roadmaps. May contain inaccurate and misleading information. Will update shortly to reflect my current progress... Throughout the time I will go through my notes to write a comphrensive guidance document...

This repo consists of 2 separate directories:
- AIVA - AI Virtual Assistant (Server for containing LLM and various models)
- AuRoRA - Autonomous Rover Robot Assistant (Robot unit with autonomous navigation and thinking/reasoning and simplified Agentic framework)

Table of Contents
- [Why AGi-ROS?](#why-agi-ros)
- [Features](#features)
- [Supported Platforms](#supported-platforms)
- [Quickstart](#quickstart)
  - [Prerequisites](#prerequisites)
  - [Clone & Build (ROS 2)](#clone--build-ros-2)
  - [Run a Demo](#run-a-demo)
  - [Docker (optional)](#docker-optional)
- [Configuration](#configuration)
  - [LLM & API keys](#llm--api-keys)
  - [Example agent config (YAML)](#example-agent-config-yaml)
- [Architecture](#architecture)
- [Usage Examples](#usage-examples)
  - [Python API example](#python-api-example)
  - [ROS 2 launch example](#ros-2-launch-example)
- [Testing & CI](#testing--ci)
- [Development](#development)
- [Contributing](#contributing)
- [Roadmap](#roadmap)
- [License](#license)
- [Contact](#contact)
- [Acknowledgements](#acknowledgements)

Why AGi-ROS?
------------
Robotics systems need robust reasoning, planning, and natural-language driven tasking. AGi-ROS provides:
- A modular agent loop (sense → reason → plan → act).
- Pluggable LLM connectors and prompt templates for task decomposition.
- Standard ROS node interfaces for perception, localization, planning, and control.
- Simulation integration for Gazebo / Ignition / other simulators.
- Safety and constraint layers for executing high-level commands in real-world hardware.

Features
--------
- Agent core with memory, context, and action planners.
- LLM connectors (OpenAI, local LLMs) — adapter pattern to add more providers.
- Perception adapters: camera, LiDAR, object detector (placeholder nodes).
- Planner adapters: symbolic task planner, motion planner bridge.
- Execution adapters: ROS action clients, service wrappers, hardware controllers.
- Sim integration with example launch files for quickly testing in simulation.
- Config-driven behavior (YAML/JSON) for reproducible experiments.

Supported Platforms
-------------------
- Primary target: ROS 2 (Humble — check branch-specific docs)
- Ubuntu 22.04 (match ROS distro)
- Python 3.10

Quickstart
----------

Prerequisites
- Git
- A supported ROS installation (ROS 2 recommended)
- Python 3.10 and pip
- colcon (for ROS 2 builds)
- LLM API credentials (if using cloud LLMs)

Clone & Build (ROS 2)
1. Clone the repo:
   ```
   git clone https://github.com/OppaAI/AGi-ROS.git
   cd AGi-ROS
   ```
2. Install any system dependencies (example, adapt to your distro):
   ```
   # Ubuntu example for ROS 2 dependencies (adjust based on distro)
   sudo apt update
   sudo apt install -y python3-pip python3-colcon-common-extensions
   ```
3. Build (ROS 2):
   ```
   colcon build --symlink-install
   source install/setup.bash
   ```
4. Install Python dependencies (if package is separate):
   ```
   pip3 install -r requirements.txt
   ```

Run a Demo
----------
- Example (replace `demo.launch.py` with actual launch file in this repo):
  ```
  source install/setup.bash
  ros2 launch agi_ros demo.launch.py
  ```
- Control via ROS topics/services or a provided CLI example.

Docker (optional)
-----------------
A Docker image can provide a reproducible environment. Example (replace Dockerfile path as needed):
```
docker build -t oppaai/agi-ros:latest .
docker run --rm -it --network host --privileged oppaai/agi-ros:latest /bin/bash
```
Note: Use `--network host` to make ROS networking and LLM API access easier; adapt security settings to your environment.

Configuration
-------------
AGi-ROS is configuration-driven. Typical location: `config/agent.yaml` (create or edit for your deployment).

LLM & API keys
- Store API keys in environment variables or in a secrets file excluded from version control.
- Example env:
  ```
  export OPENAI_API_KEY="sk-..."
  export AGI_ROS_CONFIG=./config/agent.yaml
  ```

Example agent config (YAML)
```
agent:
  name: "agi_agent"
  llm_provider: "openai"    # openai, local, llama, etc.
  llm_model: "gpt-4o-mini"
  max_tokens: 1024
  temperature: 0.2

planner:
  type: "symbolic"
  planner_service: "/planner/plan"

perception:
  camera_topic: "/camera/image_raw"
  detector: "yolov8"

execution:
  navigation_action: "/navigate_to"
  safety_layer: true
```

Architecture
------------
AGi-ROS is organized as modular ROS packages/nodes:

- agent_core (node)
  - Maintains memory, context, and orchestrates the loop.
  - Communicates with LLM adapters for reasoning and decomposition.

- llm_adapter (node/library)
  - Abstracts LLM provider APIs and handles prompts, caching, and rate limits.

- perception_node(s)
  - Ingests sensor data and publishes high-level observations.

- planner_node
  - Converts goals into subgoals and generates plans/actions.

- execution_node
  - Executes concrete robot actions (calls actions/services), monitors status.

- simulator_bridge
  - Integrates Gazebo/Ignition for safe testing.

Communication diagram (textual)
agent_core -> llm_adapter (reasoning)
agent_core -> planner_node (task decomposition)
planner_node -> execution_node (action commands)
perception_node -> agent_core (observations)
execution_node -> robot/hardware (action execution)
All components communicate using standard ROS topics, services, and actions.

Usage Examples
--------------

Python API example
```
from agi_ros.agent import Agent

agent = Agent(config_path="config/agent.yaml")
result = agent.run(goal="Deliver the package to waypoint B")
print("Result:", result)
```

ROS 2 launch example
```
ros2 launch agi_ros agent_launch.py config:=config/agent.yaml
# Or with override:
ros2 launch agi_ros agent_launch.py agent_name:=test_agent
```

Testing & CI
------------
- Unit tests should be under `tests/` and runnable with pytest.
- Integration tests can spin up a simulator and run scenarios.
- CI pipeline: run linting (flake8/ruff), unit tests, build packages (colcon).
- Add GitHub Actions workflows for automatic test runs (example workflow file: `.github/workflows/ci.yml`).

Development
-----------
- Use a Python virtualenv or depend on ROS workspace isolation.
- Run linters and formatters:
  ```
  ruff check .
  ruff format .
  black .
  ```
- Run tests:
  ```
  pytest tests/
  ```

Contributing
------------
Thanks for your interest! Please follow these steps:
1. Fork the repo and create a feature branch: `git checkout -b feat/my-feature`
2. Write tests for new behavior.
3. Run linters and tests locally.
4. Open a PR with a clear description and link to any relevant issues.

Please read and follow the repository's [CONTRIBUTING.md] if present. If not present, adhere to these rules:
- Commit messages: "type(scope): short summary" (e.g., "feat(agent): add retry on LLM timeout")
- One logical change per PR
- Keep PRs small and focused

Roadmap
-------
Planned items:
- First stable release with ROS 2 Humble support
- Additional LLM adapters for on-prem LLMs (Llama, Mistral)
- Prebuilt Docker images and CI artifacts
  
License
-------
This repository does not include a LICENSE file by default in this README. If you'd like to use a permissive license, consider MIT:

```
MIT License
Copyright (c) YEAR OppaAI
```

Replace YEAR and verify legal requirements. If a different license is desired, add the appropriate LICENSE file.

Contact
-------
Maintainers: OppaAI
Repository: [OppaAI/AGi-ROS](https://github.com/OppaAI/AGi-ROS)
For questions or support, open an issue or reach out via the repo discussion.

Acknowledgements
----------------
- ROS community and simulation tools (Gazebo, Ignition)
- Open-source LLM connector projects and robotics research
- Contributors and collaborators

Notes & Next Steps
------------------
- This README contains example commands and placeholders for launch filenames, config locations, and license choice. Replace placeholders with actual file names and configuration from your repository.
- If you want, I can:
  - Commit this README.md to the repository (create a branch and open a PR).
  - Generate matching example `config/agent.yaml`, example launch file(s), and CI workflow.
  - Tailor the README to a specific ROS distro or add exact commands for your repo layout.

Tell me which next step you want and any specifics (ROS distro, license choice, whether to create files/PRs), and I'll proceed.
