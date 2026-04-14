# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

AuRoRA is a cognitive architecture system for AGi (Artificial General Intelligence) consisting of two main ROS2 packages:

1. **SCS (Semantic Cognitive System)** - Provides a CLI interface for chatting with GRACE (the AI core)
2. **HRS (Homeostatic Regulation System)** - Manages cognitive architecture parameters and homeostatic regulation

The system is designed to run on NVIDIA Jetson hardware with Docker containerized LLM inference (Cosmos-Reason2-2B).

## Key Components

### SCS Package (`src/scs/`)
- **CLI Interface** (`src/scs/scs/cli.py`): Terminal interface for interacting with GRACE
  - Publishes user input to `/cns/neural_input` topic
  - Subscribes to `/gce/response` for streaming responses
  - Supports web search via `/web <message>` prefix
  - Features colored output and streaming response display

### HRS Package (`src/hrs/`)
- **Parameter Management** (`src/hrs/hrs/hrp.py`): Homeostatic Regulation Parameters
  - Three-tier constant hierarchy:
    - [STATIC]: Frozen hardware/architecture limits (admin-only)
    - [INTRINSIC]: GRACE's self-tuning cognitive parameters
    - [EXTRINSIC]: Per-user preferences
  - Manages cognitive resources like cortical capacity, memory systems, etc.

## Development Commands

### Building the Workspace
```bash
# Build all packages
colcon build

# Build specific packages
colcon build --packages-select scs hrs

# Rebuild with symlink installation (for active development)
colcon build --packages-select scs hrs --symlink-install
```

### Running the System
```bash
# Start the Cosmos LLM server (required first)
./launch.sh
# or
./cosmos.sh

# Source the ROS2 workspace
source install/setup.bash

# Run the SCS CLI interface
ros2 run scs cli
# or directly
ros2 run scs cli.py
```

### Testing
```bash
# Run all tests
colcon test

# Test specific packages
colcon test --packages-select scs
colcon test --packages-select hrs

# View test results
colcon test-result --verbose
```

### Individual Test Scripts
```bash
# Test SQLite vector operations (EMC validation)
python3 tests/test_sqlite_vec.py

# Test GRACE UI interface
python3 tests/grace_ui.py
```

### Linting and Code Quality
```bash
# Check Python formatting
flake8 src/ scs/ hrs/

# Check docstring conventions
pydocstyle src/ scs/ hrs/
```

## Development Workflow

1. **Start Dependencies**: Launch the Cosmos LLM server first (`./launch.sh`)
2. **Build Workspace**: `colcon build --symlink-install` for active development
3. **Source Environment**: `source install/setup.bash`
4. **Make Changes**: Edit source files in `src/`
5. **Rebuild**: `colcon build --packages-select <package> --symlink-install`
6. **Test**: Run relevant test scripts or `colcon test`
7. **Run**: `ros2 run scs cli` to test CLI interface

## Important Topics

The SCS CLI communicates via these ROS2 topics:
- **Input**: `/cns/neural_input` (published by CLI, consumed by GRACE)
- **Response**: `/gce/response` (published by GRACE, consumed by CLI)

Response messages are JSON with these types:
- `start`: Beginning of response stream
- `delta`: Incremental response tokens
- `done`: Response completion
- `error`: Error occurred

## Configuration

### HRS Parameters
Located in `src/hrs/hrs/hrp.py`, organized by:
- **STATIC**: Hardware limits (never change at runtime)
- **INTRINSIC**: Self-tuning parameters (modified by GRACE)
- **EXTRINSIC**: User preferences (loaded per session)

Key cognitive parameters include:
- Cortical capacity and reserves
- Memory encoding and recall limits
- Working memory slot limits (based on Miller's Law)
- Relevance thresholds and timeouts

## Troubleshooting

### Common Issues
1. **LLM Server Not Ready**: Wait for "Application startup complete" in docker logs
2. **ROS2 Not Sourced**: Run `source install/setup.bash` after building
3. **Package Not Found**: Ensure workspace is built with `colcon build`
4. **Port Conflicts**: Check for existing Docker containers using `docker ps`

### Logs
- Build logs: `log/build_*/`
- Docker logs: `docker logs -f cosmos-reason2-2b`
- ROS2 logs: Standard ROS2 logging mechanisms

## Architecture Notes

This system follows a cognitive architecture inspired by:
- **Central Nervous System (CNS)**: Main processing unit
- **Episodic Memory Cortex (EMC)**: Long-term memory storage
- **Working Memory Cortex (WMC)**: Active reasoning workspace
- **Homeostatic Regulation System (HRS)**: Parameter management

Data flows:
1. User types in SCS CLI → Published to `/cns/neural_input`
2. GRACE processes input → Publishes response to `/gce/response`
3. SCS CLI receives response → Displays to user with formatting