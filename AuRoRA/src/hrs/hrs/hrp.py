"""
HRP — Homeostatic Regulation Parameters
=======================================
AuRoRA · Homeostatic Regulation System (HRS)

Shared constants for GRACE's cognitive architecture —
single source of truth for all parameters across GRACE's systems.
"""

# ── Memory system parameters ───────────────────────────────────────────────────
# These parameters govern the memory system, which manages the active conversation context for GRACE.
UNITS_PER_CHUNK = 4  # rough English approximation (1 chunk ≈ 4 units)
SEGMENT_OVERHEAD = 4  # overhead per event segment (role label + formatting)

DEFAULT_CHUNK_CAPACITY = 1440  # default chunk capacity for WMC, tunable based on LLM context window and hardware constraints
DEFAULT_EVENT_SEGMENT_CAPACITY = 7  # default event segment capacity for WMC, based on Miller's Law 7±2
DEFAULT_EVENT_SEGMENT_BUFFER = 2  # default event segment buffer for WMC, based on Miller's Law 7±2, to allow some flexibility if chunks are small

# ─────────────────────────────────────────────────────────────────────────────