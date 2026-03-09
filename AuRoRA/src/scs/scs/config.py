"""
AuRoRA — Configuration
All constants and environment variables in one place.
LLM backend: Cosmos/vLLM (OpenAI-compatible API)
"""

import os
from pathlib import Path

try:
    from dotenv import load_dotenv
    load_dotenv(dotenv_path=Path.home() / "AGi/.env")
    DOTENV_AVAILABLE = True
except ImportError:
    DOTENV_AVAILABLE = False
    print("⚠️  python-dotenv not installed — using system env only")

# ── Identity ──────────────────────────────────────────────────────────────────
ROBOT_NAME      = os.getenv("ROBOT_NAME", "Grace")
ROBOT_BIRTH_FILE = Path.home() / ".grace_birth_date"  # YYYY-MM-DD text file

# ── LLM — Cosmos via vLLM (OpenAI-compatible) ────────────────────────────────
# vLLM serves Cosmos on localhost with the OpenAI /v1/chat/completions endpoint.
# Start with:  vllm serve nvidia/Cosmos-Reason1-7B --port 8000
VLLM_BASE_URL   = os.getenv("VLLM_BASE_URL", "http://localhost:8000/v1")
VLLM_MODEL      = os.getenv("VLLM_MODEL",    "nvidia/Cosmos-Reason1-7B")
VLLM_API_KEY    = os.getenv("VLLM_API_KEY",  "not-needed")  # vLLM ignores this
VLLM_KEEP_ALIVE = -1   # not applicable to vLLM (model stays loaded)

# Small fast model for ASCII art / tag generation (still via vLLM)
# Swap to a smaller Cosmos variant or keep the same model.
VLLM_FAST_MODEL = os.getenv("VLLM_FAST_MODEL", VLLM_MODEL)

# ── Memory files ──────────────────────────────────────────────────────────────
CHAT_HISTORY_FILE  = ".chat_history.json"
REFLECTIONS_FILE   = ".daily_reflections.json"

# ── Web search — SearXNG ──────────────────────────────────────────────────────
SEARXNG_URL        = os.getenv("SEARXNG_URL", "http://127.0.0.1:8080")
ENABLE_AUTO_SEARCH = os.getenv("ENABLE_AUTO_SEARCH", "true").lower() == "true"

# ── Slack ─────────────────────────────────────────────────────────────────────
SLACK_BOT_TOKEN      = os.getenv("SLACK_BOT_TOKEN", "")
SLACK_APP_TOKEN      = os.getenv("SLACK_APP_TOKEN", "")        # Socket Mode
SLACK_CHAT_CHANNEL   = os.getenv("SLACK_CHAT_CHANNEL",   "#chat-with-robot")
SLACK_DAILY_CHANNEL  = os.getenv("SLACK_DAILY_CHANNEL",  "#daily-reflection")
SLACK_PROGRESS_CHANNEL = os.getenv("SLACK_PROGRESS_CHANNEL", "#project-progress")
SLACK_ALERTS_CHANNEL = os.getenv("SLACK_ALERTS_CHANNEL", "#chat-with-robot")
SLACK_CHANNEL        = SLACK_CHAT_CHANNEL   # legacy alias

# ── Telegram ──────────────────────────────────────────────────────────────────
TELEGRAM_BOT_TOKEN = os.getenv("TELEGRAM_BOT_TOKEN", "")
TELEGRAM_CHAT_ID   = os.getenv("TELEGRAM_CHAT_ID",   "")

# ── Hugo blog ─────────────────────────────────────────────────────────────────
HUGO_BLOG_DIR    = os.getenv("HUGO_BLOG_DIR",    str(Path.home() / "grace-blog"))
HUGO_AUTO_DEPLOY = os.getenv("HUGO_AUTO_DEPLOY", "true").lower() == "true"
HUGO_GIT_REMOTE  = os.getenv("HUGO_GIT_REMOTE",  "origin")
HUGO_GIT_BRANCH  = os.getenv("HUGO_GIT_BRANCH",  "main")

# ── Gmail ─────────────────────────────────────────────────────────────────────
GMAIL_ADDRESS   = os.getenv("GRACE_GMAIL_ADDRESS",      "")
GMAIL_APP_PASS  = os.getenv("GRACE_GMAIL_APP_PASSWORD", "")
GMAIL_POLL_SECS = int(os.getenv("GRACE_GMAIL_POLL_SECS", "300"))

# ── Context / memory limits ───────────────────────────────────────────────────
# Cosmos-7B fits comfortably at 8192; bump for larger variants.
SAFE_CONTEXT_SIZES = {
    "3b-7b":  8192,
    "13b+":  16384,
}
MAX_MESSAGES_BY_SIZE = {
    "3b-7b": 20,
    "13b+":  40,
}

MEMORY_WINDOW_DAYS   = 7
MAX_MESSAGES_PER_DAY = 200
MAX_REFLECTIONS_LOAD = 20
MAX_HISTORY_STORAGE  = 10000
SUMMARIZE_THRESHOLD  = 100
MAX_RECENT_MESSAGES  = 30
MAX_REQUEST_TOKENS   = 12000

# ── Search limits ─────────────────────────────────────────────────────────────
MAX_SEARCH_RESULTS   = 5
SEARCH_CONTENT_CHARS = 500

# ── .env template (print on first run if missing keys) ───────────────────────
ENV_TEMPLATE = """
# AuRoRA / Grace .env — copy to ~/AGi/.env and fill in values

ROBOT_NAME=Grace

# LLM (vLLM serving Cosmos)
VLLM_BASE_URL=http://localhost:8000/v1
VLLM_MODEL=nvidia/Cosmos-Reason1-7B

# Slack
SLACK_BOT_TOKEN=
SLACK_APP_TOKEN=
SLACK_CHAT_CHANNEL=#chat-with-robot
SLACK_DAILY_CHANNEL=#daily-reflection
SLACK_PROGRESS_CHANNEL=#project-progress
SLACK_ALERTS_CHANNEL=#chat-with-robot

# Telegram
TELEGRAM_BOT_TOKEN=
TELEGRAM_CHAT_ID=

# Gmail
GRACE_GMAIL_ADDRESS=
GRACE_GMAIL_APP_PASSWORD=
GRACE_GMAIL_POLL_SECS=300

# Blog
HUGO_BLOG_DIR=~/grace-blog
HUGO_AUTO_DEPLOY=true
HUGO_GIT_REMOTE=origin
HUGO_GIT_BRANCH=main

# Search
SEARXNG_URL=http://127.0.0.1:8080
ENABLE_AUTO_SEARCH=true
"""
