# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

AuRoRA is a cognitive architecture system for AGi (Artificial General Intelligence) consisting of two main ROS2 packages:

1. **SCS (Semantic Cognitive System)** - Provides a CLI interface for chatting with GRACE (the AI core)
2. **HRS (Homeostatic Regulation System)** - Manages cognitive architecture parameters and homeostatic regulation

The system is designed to run on NVIDIA Jetson hardware with Docker containerized LLM inference (Cosmos-Reason2-2B).

## Key Components

### SCS Package (`src/scs/`)
- **Central Neural Core** (`cnc.py`): ROS2 node that interfaces with the Cosmos LLM via HTTP
- **Memory Coordination Core** (`mcc.py`): Coordinates between working memory, episodic memory, and contextual processing
- **Working Memory Cortex** (`wmc.py`): Implements Miller's Law 7±2 slot-based working memory
- **Episodic Memory Cortex** (`emc.py`): Handles long-term memory storage using SQLite with WAL mode
- **Memory Storage Bank** (`msb.py`: Shared infrastructure layer providing encoding, vector math, storage utilities, and convergence fusion
- **CLI Interface**: Originally in `src/scs/scs/cli.py` but enhanced version moved to `tests/grace_ui.py`

### HRS Package (`src/hrs/`)
- **Homeostatic Regulation Parameters** (`hrp.py`): Three-tier constant hierarchy:
  - `[STATIC]`: Frozen hardware/architecture limits (admin-only)
  - `[INTRINSIC]`: GRACE's self-tuning cognitive parameters  
  - `[EXTRINSIC]`: Per-user preferences

## Development Commands

### Building the Workspace
```bash
# Build all packages
colcon build

# Build specific packages
colcon build --packages-select scs hrs

# Rebuild with symlink installation (for active development)
colcon build --packages-select scs hrs --symlink-install

# Clean rebuild
colcon build --packages-select scs hrs --symlink-install --cmake-force-configure
```

### Running the System
```bash
# Start the Cosmos LLM server (required first)
./launch.sh
# or
./cosmos.sh

# Source the ROS2 workspace
source install/setup.bash

# Run the SCS CLI interface (original)
ros2 run scs cli

# Run the enhanced CLI directly (recommended for development)
python3 tests/grace_ui.py

# Alternative: Run CLI with specific ROS2 arguments
ros2 run scs cli --ros-args --log-level info
```

### Testing
```bash
# Run all tests
colcon test

# Test specific packages
colcon test --packages-select scs
colcon test --packages-select hrs

# View detailed test results
colcon test-result --verbose

# Individual test scripts
python3 tests/test_sqlite_vec.py          # Validate EMC SQLite vector operations
python3 tests/grace_ui.py                 # Run enhanced CLI (interactive)
```

### Linting and Code Quality
```bash
# Check Python formatting (from repo root)
flake8 AuRoRA/src/

# Check docstring conventions  
pydocstyle AuRoRA/src/

# Run both checks
flake8 AuRoRA/src/ && pydocstyle AuRoRA/src/
```

### Individual Component Testing
```bash
# Test MSB (Memory Storage Bank) directly
cd AuRoRA/src
python3 -c "
from scs.scs.msb import EncodingEngine, unit_normalize, semantic_match
import logging
logging.basicConfig(level=logging.INFO)
eng = EncodingEngine(logging.getLogger(), 'all-MiniLM-L6-v2', cache_limit=10)
print('Encoding engine available:', eng.is_available)
"
```

## Development Workflow

1. **Start Dependencies**: Launch the Cosmos LLM server first (`./launch.sh` or `./cosmos.sh`)
2. **Build Workspace**: `colcon build --symlink-install` for active development
3. **Source Environment**: `source install/setup.bash`
4. **Make Changes**: Edit source files in `AuRoRA/src/`
5. **Rebuild**: `colcon build --packages-select <package> --symlink-install`
6. **Test**: Run relevant test scripts or `colcon test --packages-select <package>`
7. **Run**: `python3 tests/grace_ui.py` to test CLI interface

## Important Topics

### SCS CLI Communication
The SCS CLI communicates via these ROS2 topics:
- **Input**: `/aurora/grace/input` (published by CLI, consumed by CNC)
- **Response**: `/aurora/grace/response` (published by CNC, consumed by CLI)

Response messages are JSON with these types:
- `start`: Beginning of response stream (`{"type": "start", "content": "<first chunk>"}`)
- `chunk`: Incremental response tokens (`{"type": "chunk", "content": "<delta>"}`)
- `end`: Response completion (`{"type": "end", "content": "<full response>"}`)
- `error`: Error occurred (`{"type": "error", "content": "<error message>"}`)

### MSB (Memory Storage Bank) Key Features
The MSB module provides shared infrastructure for all memory cortices:

**Encoding Engine**:
- Sentence-transformers model wrapper with LRU caching
- Automatic fallback to lexical search when encoding engine unavailable
- Cue vs episode encoding separation for different caching strategies

**Vector Operations**:
- Unit normalization for cosine-equivalent L2 search
- Semantic matching via cosine similarity (fallback when sqlite-vec unavailable)
- Vector packing/unpacking for SQLite storage

**Storage Utilities**:
- SQLite WAL mode connection factory for concurrent access
- Dynamic schema generation from EngramSchema definitions
- Automatic index creation (sqlite-vec vector index + FTS5 lexical index)

**Search & Retrieval**:
- KNN semantic search via sqlite-vec ANN
- Cosine similarity fallback pool (stratified recent/oldest sampling)
- FTS5 lexical search with OR-based keyword matching
- Reciprocal Rank Fusion (RRF) for combining semantic + lexical results

**Episodic Buffer**:
- Crash-safe staging area for unencoded episodes
- Batch processing for encoding and consolidation
- Automatic cleanup after successful storage

### HRS Parameters
Located in `AuRoRA/src/hrs/hrs/hrp.py`, organized by:
- **STATIC**: Hardware limits (never change at runtime)
- **INTRINSIC**: Self-tuning parameters (modified by GRACE)
- **EXTRINSIC**: User preferences (loaded per session)

Key cognitive parameters include:
- Cortical capacity and reserves (WMC slots based on Miller's Law)
- Memory encoding and recall limits
- Working memory timeouts and decay rates
- Relevance thresholds for memory retrieval
- Homeostatic regulation intervals

### Architecture Notes

Data flows in the cognitive architecture:
1. User types in SCS CLI → Published to `/aurora/grace/input`
2. CNC processes input → Sends to MCC for memory coordination
3. MCC builds context from WMC + EMC → Returns to CNC
4. CNC streams request to Cosmos LLM → Receives SSE tokens
5. CNC sends chunks to GUI via `/aurora/grace/response`
6. MCC stores assistant turn in WMC → Triggers EMC consolidation when needed

Memory cortex interaction pattern:
- Encoding: Raw text → EncodingEngine → Normalized vector → Storage
- Recall: Cue text → EncodingEngine → Vector search → RRF fusion → Results

## Troubleshooting

### Common Issues
1. **LLM Server Not Ready**: Wait for "Application startup complete" in docker logs
   - Check status: `docker logs -f cosmos-reason2-2b`
   - Alternative server: `./cosmos.sh` (uses different GPU memory settings)

2. **ROS2 Not Sourced**: Run `source install/setup.bash` after building
   - Verify sourcing: `echo $ROS_PACKAGE_PATH` should contain install paths

3. **Package Not Found**: Ensure workspace is built with `colcon build`
   - Rebuild specific package: `colcon build --packages-select scs --symlink-install`

4. **Port Conflicts**: Check for existing Docker containers using `docker ps`
   - Stop conflicting service: `docker stop <container_name>`

5. **Encoding Engine Issues**: 
   - Missing sentence-transformers: `pip3 install sentence-transformers --break-system-packages`
   - Missing sqlite-vec: `pip3 install sqlite-vec --break-system-packages`

### Logs
- Build logs: `AuRoRA/log/build_*/`
- Docker logs: `docker logs -f cosmos-reason2-2b`
- ROS2 logs: Standard ROS2 logging mechanisms (configured via launch files)
- Console output: Watch terminal where you launched components

## Memory System Specifics

### EMC (Episodic Memory Cortex) Storage Schema
- Primary table: `emc_storage` (episodes with temporal metadata)
- Vector table: `emc_vector` (sqlite-vec embeddings for semantic search)
- Lexical table: `emc_lexical` (FTS5 for keyword search)
- Staging buffer: `emc_staging` (crash-safe intake for encoding worker)

### Key Algorithms
- **Forgetting Curve**: Ebbinghaus R = e^(−t/S) where S is importance score
- **Memory Convergence**: RRF = 1/(k + rank_semantic) + 1/(k + rank_lexical) with k=60
- **Capacity Limits**: WMC = 7±2 slots (Miller's Law), Visuospatial sketchpad = 4±1 (Cowan)
- **Importance Scoring**: 3-dimension (SMC similarity, novelty via embeddinggemma, content signals)

## Performance Considerations

### GPU Memory Management
- Cosmos server uses `--gpu-memory-utilization 0.45` (launch.sh) or `0.50` (cosmos.sh)
- Adjust based on model size and concurrent requests
- Monitor with: `nvidia-smi` (when Docker has GPU access)

### Caching Strategies
- EncodingEngine LRU cache (default 256 entries) prevents re-encoding identical texts
- Separate caching for cues vs episodes to optimize recall patterns
- Cache imprint based on first 300 characters to balance hit rate vs false positives

### Database Optimization
- SQLite WAL mode enables concurrent reads during async writes
- NORMAL synchronous mode balances safety vs performance
- Vector index (sqlite-vec) provides ANN search when available
- FTS5 with porter+unicode61 tokenization for robust lexical search