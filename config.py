"""
GRACE — Configuration
======================
Central config for all Grace systems.
Edit this file to tune behaviour.
"""

from datetime import date

# ─── vLLM / Cosmos ───────────────────────────────────────────────────────────
VLLM_URL             = "http://localhost:8000"
COSMOS_MODEL         = "nvidia/Cosmos-Reason1-2B"   # adjust to your vLLM model name
VLLM_WARMUP_RETRIES  = 20
VLLM_WARMUP_INTERVAL = 5   # seconds between retries

# ─── Grace identity ───────────────────────────────────────────────────────────
GRACE_BIRTH_DATE = date(2026, 1, 1)   # adjust to actual birth date

# ─── Memory ──────────────────────────────────────────────────────────────────
GRACE_DB_PATH      = ".grace/grace_memory.db"   # relative to home dir
CHAT_HISTORY_FILE  = ".grace/chat_history.json"
REFLECTIONS_FILE   = ".grace/reflections.json"

# WM — Working Memory
WM_MAX_TURNS = 9   # 7±2 — max turns kept in RAM (counts pairs, so 9 pairs = 18 messages)

# EM — Episodic Memory
EM_SUMMARY_THRESHOLD = 20   # rebuild EM summary every N new turns

# SM — Semantic Memory
SM_RECENT_DAYS       = 3    # days of reflections always loaded into context
SM_RAG_TOP_K         = 5    # max SM results from RAG search
RAG_SIMILARITY_THRESHOLD = 0.45   # min cosine similarity to include result

# Token management
MAX_REQUEST_TOKENS   = 3072   # max tokens sent to Cosmos per request
MAX_HISTORY_STORAGE  = 500    # max messages kept in chat_history.json
MAX_MESSAGES_PER_DAY = 200    # max messages loaded for one day
MAX_RECENT_MESSAGES  = 20     # fallback if RAG unavailable
MAX_REFLECTIONS_LOAD = 7      # max reflection days loaded into context

# ─── Consolidation ────────────────────────────────────────────────────────────
CONSOLIDATION_IDLE_SECONDS = 1800   # 30 min idle triggers consolidation
