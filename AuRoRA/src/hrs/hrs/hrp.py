"""
HRP — Homeostatic Regulation Parameters
=======================================
AuRoRA · Homeostatic Regulation System (HRS)

Shared constants for the robot's cognitive architecture —
single source of truth for all parameters across the robot's systems.
"""

# System libraries
from pathlib import Path    # For handling DB file paths

# Memory system parameters
# Memory Coordination Core parameters
SYSTEM_PROMPT_RESERVE = 300   # tokens reserved for system prompt + personality
EMC_CONTEXT_RESERVE   = 300   # tokens reserved for injected EMC episodes
EMC_TOP_K             = 3     # max episodes injected per turn
EMC_MIN_SIMILARITY    = 0.25  # minimum cosine sim to include an episode
EMC_DB_PATH           = str(Path.home() / ".aurora" / "emc.db")

# Working Memory Cortex parameters
UNITS_PER_CHUNK                = 4     # Rough English approximation (1 chunk ≈ 4 character units)
PMT_OVERHEAD                   = 4     # Overhead chunks per PMT (role label + chat template formatting)
DEFAULT_WMC_GLOBAL_CHUNK_LIMIT = 1440  # Global chunk limit for WMC (tunable based on LLM context window)
DEFAULT_WMC_PMT_SLOT_LIMIT     = 7     # PMT slot limit for WMC (Miller's Law centre point 7±2)
DEFAULT_WMC_PMT_SLOT_BUFFER    = 2     # PMT slot buffer for WMC (Miller's Law ±2 flexibility)
