"""
HRP — Homeostatic Regulation Parameters
=======================================
AuRoRA · Homeostatic Regulation System (HRS)

Shared constants for the robot's cognitive architecture —
single source of truth for all parameters across the robot's systems.
"""

# Memory system parameters
UNITS_PER_CHUNK                = 4     # Rough English approximation (1 chunk ≈ 4 character units)
PMT_OVERHEAD                   = 4     # Overhead chunks per PMT (role label + chat template formatting)
DEFAULT_WMC_GLOBAL_CHUNK_LIMIT = 1440  # Global chunk limit for WMC (tunable based on LLM context window)
DEFAULT_WMC_PMT_SLOT_LIMIT     = 7     # PMT slot limit for WMC (Miller's Law centre point 7±2)
DEFAULT_WMC_PMT_SLOT_BUFFER    = 2     # PMT slot buffer for WMC (Miller's Law ±2 flexibility)
