import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

import requests
from concurrent.futures import ThreadPoolExecutor, Future
import json
from pathlib import Path
from datetime import datetime, date, timedelta
import sys
import base64
import os
import re
import threading
import time
import subprocess
import imaplib
import email as email_lib
from email.header import decode_header as email_decode_header

# ✅ Load environment variables from .env file
try:
    from dotenv import load_dotenv
    load_dotenv(dotenv_path=Path.home() / "AGi/.env")  # Load .env file from home directory
    DOTENV_AVAILABLE = True
except ImportError:
    DOTENV_AVAILABLE = False
    print("⚠️  python-dotenv not installed. Using system environment variables only.")
    print("   Install with: pip3 install python-dotenv")

# ✅ Slack integration
try:
    from slack_sdk import WebClient
    from slack_sdk.errors import SlackApiError
    SLACK_AVAILABLE = True
except ImportError:
    SLACK_AVAILABLE = False
    print("⚠️  slack-sdk not installed. Slack features disabled.")
    print("   Install with: pip3 install slack-sdk")

# ✅ Slack Bolt for event listening (NEW - TWO-WAY COMMUNICATION)
try:
    from slack_bolt import App
    from slack_bolt.adapter.socket_mode import SocketModeHandler
    SLACK_BOLT_AVAILABLE = True
except ImportError:
    SLACK_BOLT_AVAILABLE = False
    print("⚠️  slack-bolt not installed. Two-way Slack disabled.")
    print("   Install with: pip3 install slack-bolt")

# ✅ RLHF System (NEW - REINFORCEMENT LEARNING FROM HUMAN FEEDBACK) - FIXED IMPORT
try:
    from scs.rlhf_llm import get_rlhf  # FIXED: Removed scs. prefix
    RLHF_AVAILABLE = True
except ImportError:
    RLHF_AVAILABLE = False
    print("⚠️  rlhf_llm.py not found. RLHF disabled.")
    print("   Place rlhf_llm.py in the same directory as cnc.py")

# ✅ Telegram integration (NEW - TWO-WAY COMMUNICATION)
try:
    from telegram import Update
    from telegram.ext import Application, CommandHandler, MessageHandler, filters, ContextTypes
    TELEGRAM_AVAILABLE = True
except ImportError:
    TELEGRAM_AVAILABLE = False
    print("⚠️  python-telegram-bot not installed. Telegram features disabled.")
    print("   Install with: pip3 install python-telegram-bot")

# ═══════════════════════════════════════════════════════
# CONFIGURATION
# ═══════════════════════════════════════════════════════

# Model configuration
GCE = "huihui_ai/qwen3-vl-abliterated:4b-instruct-q4_K_M"

# File paths
CHAT_HISTORY_FILE = '.chat_history.json'
REFLECTIONS_FILE = '.daily_reflections.json'

# Ollama configuration
OLLAMA_BASE_URL = "http://localhost:11434"

# Web search
SEARXNG_URL = "http://127.0.0.1:8080"

# Slack configuration (loaded from .env file or environment)
SLACK_BOT_TOKEN  = os.getenv('SLACK_BOT_TOKEN', '')
SLACK_APP_TOKEN  = os.getenv('SLACK_APP_TOKEN', '')       # Socket Mode (two-way chat)
# Channel routing — set these in .env to match your Slack workspace
SLACK_CHAT_CHANNEL       = os.getenv('SLACK_CHAT_CHANNEL',       '#chat-with-robot')   # live chat + backup
SLACK_DAILY_CHANNEL      = os.getenv('SLACK_DAILY_CHANNEL',      '#daily-reflection')  # daily reflection alerts
SLACK_PROGRESS_CHANNEL   = os.getenv('SLACK_PROGRESS_CHANNEL',   '#project-progress')  # weekly/monthly/quarterly/yearly
SLACK_ALERTS_CHANNEL     = os.getenv('SLACK_ALERTS_CHANNEL',     '#chat-with-robot')   # system errors → chat channel
# Legacy fallback so existing send_slack_notification() calls still work
SLACK_CHANNEL = SLACK_CHAT_CHANNEL

# Telegram configuration (loaded from .env file or environment)
TELEGRAM_BOT_TOKEN = os.getenv('TELEGRAM_BOT_TOKEN', '')  # From @BotFather
TELEGRAM_CHAT_ID = os.getenv('TELEGRAM_CHAT_ID', '')  # Optional: For notifications

# Hugo + GitHub Pages Blog Configuration (NEW)
HUGO_BLOG_DIR = os.getenv('HUGO_BLOG_DIR', str(Path.home() / 'grace-blog'))
HUGO_AUTO_DEPLOY = os.getenv('HUGO_AUTO_DEPLOY', 'true').lower() == 'true'
HUGO_GIT_REMOTE = os.getenv('HUGO_GIT_REMOTE', 'origin')
HUGO_GIT_BRANCH = os.getenv('HUGO_GIT_BRANCH', 'main')

# Gmail configuration (Grace's dedicated account — NOT your personal Gmail)
# Create a separate Gmail for Grace, forward relevant emails there.
# Enable 2FA on that account, then generate an App Password (16 chars, no spaces).
GMAIL_ADDRESS   = os.getenv('GRACE_GMAIL_ADDRESS', '')
GMAIL_APP_PASS  = os.getenv('GRACE_GMAIL_APP_PASSWORD', '')
GMAIL_POLL_SECS = int(os.getenv('GRACE_GMAIL_POLL_SECS', '300'))  # default 5 min

# ✅ Optimized Settings for 30B MODEL
SAFE_CONTEXT_SIZES = {
    "2b-4b": 4096,    # 2-4B models: conservative
    "7b-8b": 6144,       # 7B models: moderate
    "13b+": 8192      # 13B+ models: full context
}

MAX_MESSAGES_BY_SIZE = {
    "2b-4b": 12,      # Fewer messages for small models
    "7b-8b": 20,         # Moderate for 7B
    "13b+": 30        # More for large models
}

# Memory limits (ENHANCED 7-DAY WINDOW)
MEMORY_WINDOW_DAYS = 7           # Keep 7 days in active memory
MAX_MESSAGES_PER_DAY = 200       # Max messages per day (safety limit)
MAX_REFLECTIONS_LOAD = 20        # Still load 20 daily reflections
MAX_HISTORY_STORAGE = 10000      # Store more on disk before RAG
SUMMARIZE_THRESHOLD = 100        # Summarize after 100 messages
MAX_RECENT_MESSAGES = 30       # Ceiling; actual limit set by _get_max_messages() per model

# Token budget per request (FOR 30B MODEL)
MAX_REQUEST_TOKENS = 12000     # Ceiling; trimmed to model's safe_context per request

# Search result limits
MAX_SEARCH_RESULTS = 5       # More search results
SEARCH_CONTENT_CHARS = 500   # More detail per result

# ✅ DUAL HYBRID SEARCH (NEW)
ENABLE_AUTO_SEARCH = True     # Enable LLM-based auto-search detection

class ASCIIArtGenerator:
    """Generate ASCII art using LLM - pure text, no images"""
    
    def __init__(self, logger, ollama_base_url="http://localhost:11434"):
        self.logger = logger
        self.ollama_base_url = ollama_base_url
    
    def generate(self, reflection_text: str, day_number: int, conversation_summary: str = None) -> str:
        """Generate ASCII art string for this reflection"""
        mood = self._extract_mood(reflection_text)
    
        context = f"Today's themes: {conversation_summary[:200]}" if conversation_summary else ""
        
        prompt = f"""Create simple text art for an AI robot's day {day_number}.
    Mood: {mood}
    {context}

    Use ONLY: - = | / \\ + * . o O # @ ~ ^
    Under 10 lines, 30 chars wide.
    ONLY the art, nothing else."""

        try:
            response = requests.post(
                f'{self.ollama_base_url}/api/generate',
                json={
                    "model": "ministral-3:3b-instruct-2512-q4_K_M",
                    "prompt": prompt,
                    "stream": False,
                    "options": {"temperature": 0.9, "num_predict": 300}
                },
                timeout=30
            )
            
            if response.status_code == 200:
                art = response.json().get('response', '').strip()
                art = art.replace('```', '').strip()
                
                if 20 < len(art) < 1000:
                    self.logger.info("✅ ASCII art generated")
                    return art
        except Exception as e:
            self.logger.error(f"❌ ASCII art failed: {e}")
        
        return self._fallback(mood)
    
    def _extract_mood(self, text: str) -> str:
        text_lower = text.lower()
        if any(w in text_lower for w in ['happy', 'joy', 'excited', '😊', '😄', '🎉']):
            return 'happy'
        elif any(w in text_lower for w in ['sad', 'down', 'disappointed', '😢']):
            return 'sad'
        elif any(w in text_lower for w in ['curious', 'wonder', 'interesting', '🤔']):
            return 'curious'
        elif any(w in text_lower for w in ['angry', 'frustrated', '😠']):
            return 'angry'
        return 'neutral'
    
    def _fallback(self, mood: str) -> str:
        arts = {
            'happy':   "╔══════════════╗\n║  ◉   ◉      ║\n║    ╲___╱    ║\n╚══════════════╝",
            'sad':     "╔══════════════╗\n║  ◉   ◉      ║\n║    ╱▔▔▔╲    ║\n╚══════════════╝",
            'curious': "╔══════════════╗\n║  ◉   ◉  ?  ║\n║    ╲___╱    ║\n╚══════════════╝",
            'angry':   "╔══════════════╗\n║  ◆   ◆      ║\n║  ═══════    ║\n╚══════════════╝",
            'neutral': "╔══════════════╗\n║  ◉   ◉      ║\n║  ─────────  ║\n╚══════════════╝",
        }
        return arts.get(mood, arts['neutral'])
        
# RAG via EmbeddingGemma (Ollama) — no HuggingFace, no numpy, no torch
# Run: ollama pull embeddinggemma:300m-qat-q4_0
from scs.simple_sqlite_rag import SimpleSQLiteRAG as SQLiteVectorRAG
            
class HugoBlogger:
    """
    Automated Hugo + GitHub Pages blogging for Grace's daily reflections
    
    Features:
    - Creates markdown posts in Hugo format
    - Auto-commits to Git
    - Auto-pushes to GitHub (triggers GitHub Actions deployment)
    - Fully automated - no manual steps needed!
    """
    
    def __init__(self, blog_dir: str, logger, auto_deploy: bool = True):
        self.blog_dir = Path(blog_dir)
        self.logger = logger
        self.auto_deploy = auto_deploy
        
        # Validate blog directory
        if not self.blog_dir.exists():
            self.logger.warn(f"⚠️  Hugo blog directory not found: {blog_dir}")
            self.logger.warn("   Create with: hugo new site grace-blog")
            self.enabled = False
            return
        
        # Check if it's a Hugo site
        if not (self.blog_dir / 'config.toml').exists() and not (self.blog_dir / 'hugo.toml').exists():
            self.logger.warn(f"⚠️  Not a Hugo site (no config.toml): {blog_dir}")
            self.enabled = False
            return
        
        # Check if Git is initialized
        if not (self.blog_dir / '.git').exists():
            self.logger.warn(f"⚠️  Hugo site not a Git repository: {blog_dir}")
            self.logger.warn("   Initialize with: cd grace-blog && git init")
            self.enabled = False
            return
        
        self.posts_dir = self.blog_dir / 'content' / 'posts'
        self.posts_dir.mkdir(parents=True, exist_ok=True)
        
        self.enabled = True
        self.logger.info(f"✅ Hugo blogger initialized: {blog_dir}")
        self.logger.info(f"   Auto-deploy: {'ON' if auto_deploy else 'OFF'}")
    
    def _extract_tags(self, reflection_text: str) -> list:
        """Extract relevant tags from reflection content"""
        base_tags = ["daily-reflection", "ai-journal", "grace"]
        
        text_lower = reflection_text.lower()
        
        if any(w in text_lower for w in ['code', 'python', 'git', 'github', 'ssh']):
            base_tags.append("coding")
        if any(w in text_lower for w in ['happy', 'joy', 'excited', 'grateful']):
            base_tags.append("positive")
        if any(w in text_lower for w in ['curious', 'wonder', 'question', 'learning']):
            base_tags.append("learning")
        if any(w in text_lower for w in ['sad', 'difficult', 'hard', 'tired']):
            base_tags.append("introspective")
        
        return base_tags
    
    def post_reflection(self, reflection_text: str, date_str: str, age_days: int, 
                   message_count: int, ascii_art: str = None) -> dict:
        """
        Post daily reflection to Hugo blog
        
        Args:
            reflection_text: The reflection content
            date_str: Date in YYYY-MM-DD format
            age_days: Grace's age in days
            message_count: Number of messages that day
            
        Returns:
            dict with success status and details
        """
        if not self.enabled:
            return {
                'success': False,
                'error': 'Hugo blogger not enabled'
            }
        
        try:
            # Create filename
            filename = f'{date_str}-day-{age_days}.md'
            filepath = self.posts_dir / filename
            
            # Smart date/time logic
            now = datetime.now()
            post_date = datetime.fromisoformat(date_str)
            
            # If posting for today, use current time
            # If posting after midnight for yesterday, use 11:59 PM of yesterday
            # If posting for past days, use 11:59 PM
            if post_date.date() == now.date():
                # Today - use current time
                post_datetime = now.strftime('%Y-%m-%dT%H:%M:%S')
            elif post_date.date() < now.date():
                # Past day - use 11:59 PM of that day
                post_datetime = f"{date_str}T23:59:59"
            else:
                # Future date (shouldn't happen) - use current time
                post_datetime = now.strftime('%Y-%m-%dT%H:%M:%S')
            
            # Build Hugo front matter
            art_section = f"\n```\n{ascii_art}\n```\n" if ascii_art else ""

            tags = self._extract_tags(reflection_text)
            tags_str = json.dumps(tags)

            frontmatter = f"""---
title: "Day {age_days} Reflection"
date: {post_datetime}
draft: false
tags: {tags_str}
categories: ["Daily Reflections"]
author: "Grace"
---

{reflection_text}

{art_section}
---
*Generated from {message_count} conversations on day {age_days}.*
"""
            
            # Write file
            filepath.write_text(frontmatter)
            
            self.logger.info(f"📝 Created blog post: {filename}")
            self.logger.info(f"   Post date/time: {post_datetime}")
            
            # Git commit and push (if auto-deploy enabled)
            if self.auto_deploy:
                deployed = self._git_deploy(filename, date_str)
                
                return {
                    'success': True,
                    'filename': filename,
                    'filepath': str(filepath),
                    'deployed': deployed,
                    'url': self._get_blog_url(filename)
                }
            else:
                return {
                    'success': True,
                    'filename': filename,
                    'filepath': str(filepath),
                    'deployed': False,
                    'message': 'Auto-deploy disabled. Commit manually.'
                }
        
        except Exception as e:
            self.logger.error(f"❌ Failed to post to Hugo blog: {e}")
            import traceback
            traceback.print_exc()
            return {
                'success': False,
                'error': str(e)
            }
    
    def post_periodic(self, report_type: str, content: str, period_label: str,
                      stats: dict = None) -> dict:
        """
        Post weekly / monthly / quarterly / yearly report to Hugo blog.
        Hugo's RSS feed at /index.xml updates automatically on every push.
        """
        if not self.enabled:
            return {'success': False, 'error': 'Hugo blogger not enabled'}
        try:
            now        = datetime.now()
            date_str   = now.strftime('%Y-%m-%d')
            post_dt    = now.strftime('%Y-%m-%dT%H:%M:%S')
            safe_label = period_label.replace(' ', '-').replace(',', '').lower()
            filename   = f'{date_str}-{report_type}-{safe_label}.md'
            filepath   = self.posts_dir / filename

            category_map = {
                'weekly':    'Weekly Journal',
                'monthly':   'Monthly Report',
                'quarterly': 'Quarterly Review',
                'yearly':    'Yearly Retrospective',
            }
            tag_map = {
                'weekly':    ['weekly-journal',       'ai-journal', 'grace', 'kpi'],
                'monthly':   ['monthly-report',       'ai-journal', 'grace', 'kpi', 'retrospective'],
                'quarterly': ['quarterly-review',     'ai-journal', 'grace', 'kpi', 'roadmap'],
                'yearly':    ['yearly-retrospective', 'ai-journal', 'grace', 'kpi', 'goals'],
            }
            category = category_map.get(report_type, 'Reports')
            tags     = json.dumps(tag_map.get(report_type, ['grace']))

            stats_yaml = ''
            if stats:
                stats_yaml = 'stats:\n'
                for k, v in stats.items():
                    stats_yaml += f'  {k}: {v}\n'

            frontmatter = f"""---
title: "{category} — {period_label}"
date: {post_dt}
draft: false
tags: {tags}
categories: ["{category}"]
author: "Grace"
{stats_yaml}---

{content}
"""
            filepath.write_text(frontmatter)
            self.logger.info(f"📝 {report_type.capitalize()} post created: {filename}")
            deployed = self._git_deploy(filename, date_str) if self.auto_deploy else False
            return {
                'success':  True,
                'filename': filename,
                'filepath': str(filepath),
                'deployed': deployed,
                'url':      self._get_blog_url(filename),
            }
        except Exception as e:
            self.logger.error(f"❌ Periodic blog post failed: {e}")
            return {'success': False, 'error': str(e)}

    def _git_deploy(self, filename: str, date_str: str) -> bool:
        """Git commit and push to trigger GitHub Actions deployment"""
        try:
            # Stage the new post
            result = subprocess.run(
                ['git', 'add', f'content/posts/{filename}'],
                cwd=self.blog_dir,
                capture_output=True,
                text=True,
                timeout=10
            )
            
            if result.returncode != 0:
                self.logger.error(f"Git add failed: {result.stderr}")
                return False
            
            # Commit
            commit_message = f'Add reflection for {date_str}'
            result = subprocess.run(
                ['git', 'commit', '-m', commit_message],
                cwd=self.blog_dir,
                capture_output=True,
                text=True,
                timeout=10
            )
            
            if result.returncode != 0:
                if "nothing to commit" in result.stdout or "nothing to commit" in result.stderr:
                    self.logger.info("Nothing new to commit (file unchanged)")
                    return True
                else:
                    self.logger.error(f"Git commit failed: {result.stderr}")
                    return False
            
            self.logger.info(f"✅ Git committed: {commit_message}")
            
            # Push to GitHub (triggers GitHub Actions!)
            result = subprocess.run(
                ['git', 'push', HUGO_GIT_REMOTE, HUGO_GIT_BRANCH],
                cwd=self.blog_dir,
                capture_output=True,
                text=True,
                timeout=30
            )
            
            if result.returncode != 0:
                self.logger.error(f"Git push failed: {result.stderr}")
                return False
            
            self.logger.info(f"✅ Pushed to GitHub - deployment triggered!")
            
            return True
        
        except Exception as e:
            self.logger.error(f"Git deploy error: {e}")
            return False
    
    def _get_blog_url(self, filename: str) -> str:
        """Get the expected blog URL for the post"""
        slug = filename.replace('.md', '')
        
        try:
            result = subprocess.run(
                ['git', 'remote', 'get-url', HUGO_GIT_REMOTE],
                cwd=self.blog_dir,
                capture_output=True,
                text=True,
                timeout=5
            )
            
            if result.returncode == 0:
                remote_url = result.stdout.strip()
                
                if 'github.com' in remote_url:
                    if 'github.com/' in remote_url:
                        parts = remote_url.split('github.com/')[1].split('/')
                        username = parts[0]
                        repo = parts[1].replace('.git', '')
                    elif 'github.com:' in remote_url:
                        parts = remote_url.split('github.com:')[1].split('/')
                        username = parts[0]
                        repo = parts[1].replace('.git', '')
                    else:
                        username = 'USERNAME'
                        repo = 'grace-blog'
                    
                    return f'https://{username}.github.io/{repo}/posts/{slug}/'
        except:
            pass
        
        return f'https://USERNAME.github.io/grace-blog/posts/{slug}/'
    
    def test_deployment(self) -> bool:
        """Test that Git and deployment are working"""
        if not self.enabled:
            self.logger.error("❌ Hugo blogger not enabled")
            return False
        
        checks_passed = 0
        total_checks = 4
        
        # Check 1: Hugo installed
        try:
            result = subprocess.run(['hugo', 'version'], capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                self.logger.info("✅ Hugo is installed")
                checks_passed += 1
            else:
                self.logger.error("❌ Hugo not found")
        except:
            self.logger.error("❌ Hugo not installed - Install: sudo snap install hugo")
        
        # Check 2: Git configured
        try:
            result = subprocess.run(['git', 'status'], cwd=self.blog_dir, capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                self.logger.info("✅ Git repository is valid")
                checks_passed += 1
        except:
            self.logger.error("❌ Git not configured")
        
        # Check 3: Remote configured
        try:
            result = subprocess.run(['git', 'remote', '-v'], cwd=self.blog_dir, capture_output=True, text=True, timeout=5)
            if result.returncode == 0 and 'github.com' in result.stdout:
                self.logger.info("✅ GitHub remote configured")
                checks_passed += 1
            else:
                self.logger.error("❌ GitHub remote not configured")
        except:
            self.logger.error("❌ Failed to check Git remote")
        
        # Check 4: Can write to posts directory
        try:
            test_file = self.posts_dir / '.test'
            test_file.write_text('test')
            test_file.unlink()
            self.logger.info("✅ Posts directory is writable")
            checks_passed += 1
        except:
            self.logger.error("❌ Cannot write to posts directory")
        
        self.logger.info(f"Hugo deployment test: {checks_passed}/{total_checks} checks passed")
        return checks_passed == total_checks

class CNSBridge(Node):
    """
    Central Nervous System Bridge (OPTIMIZED + TWO-WAY COMMUNICATION + IMAGE SUPPORT)
    
    Grace's main cognitive interface connecting:
    - Text conversation (LLM)
    - Vision processing (VLM) - FIXED TELEGRAM SUPPORT
    - DUAL HYBRID WEB SEARCH (Keyword + LLM auto-detection)
    - Memory (reflections)
    - Two-way Slack & Telegram integration (text + images)
    
    Optimizations:
    - Auto-sized context window (30B → 8192 tokens)
    - Minimal system prompts
    - Lazy loading of reflections
    - Automatic context trimming
    - Thread-safe state management
    
    NEW:
    - Two-way Slack communication via Socket Mode
    - Two-way Telegram bot (text + PHOTOS)
    - Dual hybrid web search (keywords + intelligent auto-detection)
    - RLHF feedback learning
    """
    
    def __init__(self):
        super().__init__('cns_bridge')
        self.mcp_client = None
        self.use_mcp = False   # igniter owns the MCP process; we use SearXNG HTTP API directly

        # SQLite Vector RAG (EmbeddingGemma via Ollama)
        self.use_rag = True
        if self.use_rag:
            try:
                rag_db_path = str(Path.home() / "AGi" / "grace_memory.db")
                self.rag = SQLiteVectorRAG(rag_db_path, self.get_logger(), OLLAMA_BASE_URL)
                self.get_logger().info("✅ SQLite Vector RAG initialized (EmbeddingGemma)")

                # One-time migration from old DB (renames old file after, so only runs once)
                old_db_path = Path.home() / "AGi" / "grace_memory_old.db"
                if old_db_path.exists():
                    self.get_logger().info("🔄 Old RAG DB detected — migrating to EmbeddingGemma…")
                    result = self.rag.migrate_from_old_db(str(old_db_path))
                    self.get_logger().info(
                        f"Migration: {result['migrated']} migrated, "
                        f"{result['skipped']} skipped, {result['failed']} failed"
                    )
                    old_db_path.rename(old_db_path.with_suffix('.migrated'))

            except Exception as e:
                self.get_logger().error(f"❌ RAG init failed: {e}")
                self.use_rag = False

        # Hugo Blog (NEW)
        self._start_reflection_scheduler()
        self.hugo_blogger = None
        self.blog_enabled = self._init_hugo_blog()

        # Generate ASCII Art
        self.ascii_art_gen = ASCIIArtGenerator(
            logger=self.get_logger(),
            ollama_base_url=OLLAMA_BASE_URL
        )
        
        self.skills_client = None
        self.use_skills = False   # igniter owns the skills process
        self.get_logger().info("ℹ️  Skills managed by igniter (not started here)")
                  
        # ═══════════════════════════════════════════════════
        # THREAD SAFETY
        # ═══════════════════════════════════════════════════
        
        self._message_count_lock = threading.Lock()
        self._day_check_lock = threading.Lock()
        
        # ═══════════════════════════════════════════════════
        # ROS TOPICS
        # ═══════════════════════════════════════════════════
        
        # Text input
        self.subscription = self.create_subscription(
            String, '/cns/neural_input', self.listener_callback, 10)
        
        # Image input
        self.image_subscription = self.create_subscription(
            String, '/cns/image_input', self.image_listener_callback, 10)
        
        # Response output
        self.publisher = self.create_publisher(String, '/gce/response', 10)
        
        # Thread pool for non-blocking processing
        self.executor_pool = ThreadPoolExecutor(max_workers=2)
        
        # ═══════════════════════════════════════════════════
        # MODEL SETTINGS (AUTO-OPTIMIZED)
        # ═══════════════════════════════════════════════════
        
        self.model_name = GCE
        self.keep_alive = -1  # Keep model in memory permanently
        
        # Auto-detect safe limits based on model size
        self.safe_context = self._get_safe_context_size()
        self.max_recent_messages = self._get_max_messages()
        self.max_reflections_load = MAX_REFLECTIONS_LOAD
        
        # ═══════════════════════════════════════════════════
        # MEMORY SYSTEMS
        # ═══════════════════════════════════════════════════
        
        # Chat history
        self.chat_history_file = Path.home() / CHAT_HISTORY_FILE
        self.chat_history = self.load_chat_history()
        
        # Daily reflections
        self.reflections_file = Path.home() / REFLECTIONS_FILE
        self.reflections = self.load_reflections()
        self.today_start = datetime.now().date()
        self.today_message_count = 0
        count_file = Path.home() / '.grace_today_count.json'
        if count_file.exists():
            saved = json.loads(count_file.read_text())
            if saved.get('date') == date.today().isoformat():
                self.today_message_count = saved.get('count', 0)
                self.get_logger().info(f"✅ Restored today's count: {self.today_message_count}")

        # Birth date
        self.birth_date = self._get_birth_date()
        
        # Handle missed days
        self._handle_missed_reflections()
        
        # ═══════════════════════════════════════════════════
        # EXTERNAL SERVICES
        # ═══════════════════════════════════════════════════
        
        # Web search
        self.searxng_url = SEARXNG_URL
        self.search_enabled = self._check_searxng_available()
        
        # Slack (one-way notifications)
        self.slack_client = None
        self.slack_enabled = self._init_slack()
        
        # Slack (two-way listener) - NEW!
        self.slack_app = None
        self.slack_listener_enabled = self._init_slack_listener()
        
        # Telegram (two-way communication) - NEW!
        self.telegram_app = None
        self.telegram_enabled = self._init_telegram()

        # Gmail monitor (Grace's dedicated account)
        self.gmail_enabled = self._init_gmail_monitor()
        
        # ═══════════════════════════════════════════════════
        # RLHF SYSTEM (NEW - LEARNING FROM FEEDBACK)
        # ═══════════════════════════════════════════════════
        
        # Initialize RLHF
        self.rlhf = None
        self.rlhf_enabled = False
        self.current_response_id = None
        
        if RLHF_AVAILABLE:
            try:
                self.rlhf = get_rlhf()
                self.rlhf_enabled = True
                
                # Feedback topic (receives pet/red X from web interface)
                self.feedback_subscription = self.create_subscription(
                    String,
                    '/cns/rl_reward',
                    self.feedback_callback,
                    10
                )
                
                self.get_logger().info("✅ RLHF feedback system initialized")
            except Exception as e:
                self.get_logger().error(f"❌ RLHF initialization failed: {e}")
                self.rlhf_enabled = False
        
        # ═══════════════════════════════════════════════════
        # STARTUP LOGGING
        # ═══════════════════════════════════════════════════
        
        age_days = (datetime.now().date() - self.birth_date).days
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("GRACE - COGNITIVE SYSTEMS ONLINE 🧠")
        self.get_logger().info("=" * 60)
        
        if DOTENV_AVAILABLE:
            self.get_logger().info("✅ python-dotenv loaded (.env file support)")
        else:
            self.get_logger().info("⚠️  python-dotenv not available (using system env only)")
        
        self.get_logger().info(f"Birth Date: {self.birth_date}")
        self.get_logger().info(f"Age: {age_days} days old")
        self.get_logger().info(f"Model: {self.model_name}")
        self.get_logger().info("-" * 60)
        self.get_logger().info("⚙️  OPTIMIZATIONS APPLIED:")
        self.get_logger().info(f"   Safe context: {self.safe_context} tokens")
        self.get_logger().info(f"   Max messages: {self.max_recent_messages}")
        self.get_logger().info(f"   Target budget: {MAX_REQUEST_TOKENS} tokens")
        self.get_logger().info(f"   Reflection limit: {self.max_reflections_load} days")
        self.get_logger().info("-" * 60)
        self.get_logger().info(f"Chat History: {len(self.chat_history)} total messages")
        self.get_logger().info(f"  → Loading: {min(len(self.chat_history), self.max_recent_messages)} recent")
        self.get_logger().info(f"Reflections: {len(self.reflections)} total days")
        self.get_logger().info(f"  → Loading: {min(len(self.reflections), self.max_reflections_load)} recent")
        self.get_logger().info(f"Today's Messages: {self.today_message_count}")
        self.get_logger().info("-" * 60)
        
        # Feature status
        self.get_logger().info("✅ Text conversation enabled")
        self.get_logger().info("✅ Image processing enabled (VLM ready + Telegram photos)")
        
        if self.search_enabled:
            self.get_logger().info("✅ Web search enabled (DUAL HYBRID MODE)")
            self.get_logger().info("   → Method 1: Keyword triggers")
            self.get_logger().info(f"   → Method 2: LLM auto-detection ({'ON' if ENABLE_AUTO_SEARCH else 'OFF'})")
        else:
            self.get_logger().warn("⚠️  Web search disabled (SearXNG unavailable)")
        
        if self.slack_enabled:
            self.get_logger().info(f"✅ Slack notifications enabled → {SLACK_CHANNEL}")
        else:
            self.get_logger().warn("⚠️  Slack disabled (no token or error)")
        
        if self.slack_listener_enabled:
            self.get_logger().info("✅ Slack listener enabled (TWO-WAY) 📱↔️🤖")
            self.get_logger().info("   → Listening for @mentions and DMs")
        else:
            self.get_logger().warn("⚠️  Slack listener disabled (one-way only)")
        
        if self.telegram_enabled:
            self.get_logger().info("✅ Telegram bot enabled (TWO-WAY) 💬↔️🤖📸")
            self.get_logger().info("   → Single chat: real-time + brief periodic pings + email alerts")
        else:
            self.get_logger().warn("⚠️  Telegram disabled (no token or error)")

        if self.slack_enabled:
            self.get_logger().info(f"✅ Slack multi-channel routing:")
            self.get_logger().info(f"   💬 Chat/backup  → {SLACK_CHAT_CHANNEL}")
            self.get_logger().info(f"   🌙 Daily        → {SLACK_DAILY_CHANNEL}")
            self.get_logger().info(f"   📊 Progress     → {SLACK_PROGRESS_CHANNEL}")
            self.get_logger().info(f"   ⚠️  Alerts       → {SLACK_ALERTS_CHANNEL}")

        if self.gmail_enabled:
            self.get_logger().info(f"✅ Gmail monitor → {GMAIL_ADDRESS} (every {GMAIL_POLL_SECS}s)")
            self.get_logger().info(f"   High/medium emails → TG alert + Slack backup")
        else:
            self.get_logger().warn("⚠️  Gmail disabled (set GRACE_GMAIL_ADDRESS + GRACE_GMAIL_APP_PASSWORD)")
        
        if self.blog_enabled:
            self.get_logger().info(f"✅ Hugo blog enabled → {HUGO_BLOG_DIR}")
            self.get_logger().info(f"   Auto-deploy: {'ON' if HUGO_AUTO_DEPLOY else 'OFF'}")
        else:
            self.get_logger().warn("⚠️  Hugo blog disabled")

        if self.rlhf_enabled:
            stats = self.rlhf.get_stats()
            self.get_logger().info("✅ RLHF system enabled 👍👎")
            self.get_logger().info(f"   → Total feedback: {stats['total_feedback']}")
            self.get_logger().info(f"   → Training pairs: {stats.get('training_pairs', 0)}")
        else:
            self.get_logger().warn("⚠️  RLHF disabled (rlhf_llm.py not found)")
        
        self.get_logger().info("=" * 60)
        
        # Warm up model
        self._verify_model_loaded()

    # ═══════════════════════════════════════════════════════
    # INITIALIZATION
    # ═══════════════════════════════════════════════════════

    def _start_reflection_scheduler(self):
        """Run daily reflection at 11pm every day"""
        def scheduler_loop():
            while True:
                now = datetime.now()
                # Calculate seconds until 11pm today
                target = now.replace(hour=23, minute=0, second=0, microsecond=0)
                
                # If 11pm already passed today, schedule for tomorrow
                if now >= target:
                    target = target + timedelta(days=1)
                
                wait_seconds = (target - now).total_seconds()
                self.get_logger().info(f"⏰ Next reflection scheduled in {wait_seconds/3600:.1f} hours (11pm)")
                
                time.sleep(wait_seconds)
                
                # Trigger reflection
                self.get_logger().info("🕚 11pm - triggering scheduled reflection")
                if self.today_message_count > 0:
                    reflection = self.create_daily_reflection(reflection_date=self.today_start)
                    if reflection:
                        self.get_logger().info(f"✅ Scheduled reflection done: {reflection[:80]}...")
                else:
                    self.get_logger().info("⏭️  No messages today, skipping scheduled reflection")
        
        # Run in background daemon thread
        threading.Thread(target=scheduler_loop, daemon=True).start()
        self.get_logger().info("⏰ Reflection scheduler started (fires at 11pm daily)")

    def _get_safe_context_size(self):
        """Get safe context size based on model size"""
        model_lower = self.model_name.lower()
        
        if any(x in model_lower for x in ["2b", "3b", "4b"]):
            return SAFE_CONTEXT_SIZES["2b-4b"]
        elif any(x in model_lower for x in ["7b", "8b"]):
            return SAFE_CONTEXT_SIZES["7b-8b"]
        else:
            return SAFE_CONTEXT_SIZES["13b+"]

    def _get_max_messages(self):
        """Get max messages based on model size"""
        model_lower = self.model_name.lower()
        
        if any(x in model_lower for x in ["2b", "3b", "4b"]):
            return MAX_MESSAGES_BY_SIZE["2b-4b"]
        elif any(x in model_lower for x in ["7b", "8b"]):
            return MAX_MESSAGES_BY_SIZE["7b-8b"]
        else:
            return MAX_MESSAGES_BY_SIZE["13b+"]

    def _init_hugo_blog(self):
        """Initialize Hugo blog poster"""
        try:
            self.hugo_blogger = HugoBlogger(
                blog_dir=HUGO_BLOG_DIR,
                logger=self.get_logger(),
                auto_deploy=HUGO_AUTO_DEPLOY
            )
            
            if self.hugo_blogger.enabled:
                self.get_logger().info(f"✅ Hugo blog enabled: {HUGO_BLOG_DIR}")
                
                # Run deployment test
                if self.hugo_blogger.test_deployment():
                    self.get_logger().info("✅ Hugo deployment test passed")
                else:
                    self.get_logger().warn("⚠️  Hugo deployment test failed (see errors above)")
                
                return True
            else:
                self.get_logger().warn("⚠️  Hugo blog disabled (configuration issues)")
                return False
        
        except Exception as e:
            self.get_logger().error(f"❌ Hugo blog init failed: {e}")
            import traceback
            traceback.print_exc()
            return False
            
    def _verify_model_loaded(self):
        """Verify model is loaded and warm in Ollama"""
        try:
            self.get_logger().info("🔥 Warming up model...")
            response = requests.post(
                f'{OLLAMA_BASE_URL}/api/generate',
                json={
                    "model": self.model_name,
                    "prompt": "Hi",
                    "stream": False,
                    "keep_alive": -1
                },
                timeout=30
            )
            
            if response.status_code == 200:
                self.get_logger().info("✅ Model warmed up and cached in memory")
                return True
            else:
                self.get_logger().warn(f"⚠️  Model warm-up returned status {response.status_code}")
                
        except Exception as e:
            self.get_logger().warn(f"⚠️  Model warm-up failed: {e}")
        
        return False

    def _get_birth_date(self):
        """Get Grace's birth date from reflections or set today"""
        if self.reflections:
            birth_str = self.reflections[0].get('date', date.today().isoformat())
            return date.fromisoformat(birth_str)
        return date.today()

    def _handle_missed_reflections(self):
        """Create placeholder reflections for missed days"""
        if not self.reflections:
            return
        
        last_reflection_date = date.fromisoformat(self.reflections[-1]['date'])
        today = date.today()
        days_missed = (today - last_reflection_date).days - 1
        
        if days_missed > 0:
            self.get_logger().warn(f"⚠️  Detected {days_missed} missed days - creating placeholders")
            
            for i in range(1, days_missed + 1):
                missed_date = last_reflection_date + timedelta(days=i)
                age_days = (missed_date - self.birth_date).days
                
                placeholder = {
                    'day': age_days,
                    'date': missed_date.isoformat(),
                    'reflection': f"😴: Day {age_days} - Grace was offline. No conversations today.",
                    'message_count': 0
                }
                
                self.reflections.append(placeholder)
            
            self._save_reflections_file()
            self.get_logger().info(f"✅ Created {days_missed} placeholder reflections")

    def _check_searxng_available(self):
        """Check if SearXNG is running"""
        try:
            response = requests.get(self.searxng_url, timeout=5)
            # Accept 200 (OK) or 403 (Forbidden but alive)
            is_available = response.status_code in [200, 403]
            
            if is_available:
                self.get_logger().info(f"SearXNG check: status {response.status_code}")
            
            return is_available
            
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"SearXNG check failed: {e}")
            return False

    def _init_slack(self):
        """Initialize Slack client (uses token from .env file)"""
        if not SLACK_AVAILABLE:
            self.get_logger().info("Slack SDK not installed")
            return False
        
        try:
            if not SLACK_BOT_TOKEN:
                self.get_logger().info("SLACK_BOT_TOKEN not set in .env or environment")
                self.get_logger().info("Create .env file with: SLACK_BOT_TOKEN=xoxb-your-token")
                return False
            
            if SLACK_BOT_TOKEN.startswith("xoxb-your"):
                self.get_logger().warn("SLACK_BOT_TOKEN is still a placeholder")
                self.get_logger().info("Update .env file with real token from https://api.slack.com/apps")
                return False
            
            # Initialize Slack client
            self.slack_client = WebClient(token=SLACK_BOT_TOKEN)
            
            # Test connection
            response = self.slack_client.auth_test()
            self.get_logger().info(f"Slack connected as: {response['user']}")
            self.get_logger().info(f"Slack channel: {SLACK_CHANNEL}")
            return True
            
        except SlackApiError as e:
            self.get_logger().error(f"Slack API error: {e.response['error']}")
            
            if e.response['error'] == 'invalid_auth':
                self.get_logger().error("Token is invalid. Get new token from https://api.slack.com/apps")
            
            return False
            
        except Exception as e:
            self.get_logger().error(f"Slack init failed: {e}")
            return False

    def _init_slack_listener(self):
        """Initialize Slack event listener for two-way communication (NEW)"""
        if not self.slack_enabled or not SLACK_AVAILABLE or not SLACK_BOLT_AVAILABLE:
            return False
        
        if not SLACK_APP_TOKEN:
            self.get_logger().warn("⚠️  SLACK_APP_TOKEN not set. Two-way Slack disabled.")
            self.get_logger().warn("   Add SLACK_APP_TOKEN to your ~/.AGi/.env file")
            self.get_logger().warn("   Get it from: https://api.slack.com/apps → Socket Mode")
            return False
        
        if SLACK_APP_TOKEN.startswith("xapp-your"):
            self.get_logger().warn("SLACK_APP_TOKEN is still a placeholder")
            return False
        
        try:
            # Create Slack Bolt app
            self.slack_app = App(token=SLACK_BOT_TOKEN)
            
            # Store reference to self for use in handlers
            node_ref = self
            
            # Listen for app mentions (@Grace ...)
            @self.slack_app.event("app_mention")
            def handle_mention(event, say):
                try:
                    text = event.get('text', '')
                    channel = event.get('channel', '')
                    user = event.get('user', '')
                    
                    # Remove bot mention from text (e.g., "<@U12345> hello" -> "hello")
                    user_message = text.split('>', 1)[1].strip() if '>' in text else text
                    
                    node_ref.get_logger().info(f"📱 Slack @mention from user in {channel}: {user_message}")
                    
                    # Process through normal pipeline with Slack callback
                    node_ref.executor_pool.submit(
                        node_ref.process_with_ollama, 
                        user_message, 
                        slack_callback=say,
                        slack_thread_ts=None
                    )
                    
                except Exception as e:
                    node_ref.get_logger().error(f"❌ Slack mention handler error: {e}")
                    try:
                        say(f"😵: Error processing your message: {str(e)}")
                    except:
                        pass
            
            # Listen for direct messages to bot
            @self.slack_app.event("message")
            def handle_message(event, say):
                try:
                    # Ignore bot's own messages
                    if event.get('bot_id') or event.get('subtype') == 'bot_message':
                        return
                    
                    # Only process DMs (channel type is 'im')
                    channel_type = event.get('channel_type', '')
                    if channel_type != 'im':
                        return
                    
                    text = event.get('text', '')
                    user = event.get('user', '')
                    
                    # FIXED: Check for image attachments
                    files = event.get('files', [])
                    
                    if files and any(f.get('mimetype', '').startswith('image/') for f in files):
                        # Handle image message
                        image_file = next((f for f in files if f.get('mimetype', '').startswith('image/')), None)
                        
                        if image_file:
                            node_ref.get_logger().info(f"📷 Slack image from user: {text or '(no caption)'}")
                            
                            # Download and encode image
                            try:
                                image_url = image_file.get('url_private')
                                headers = {'Authorization': f'Bearer {SLACK_BOT_TOKEN}'}
                                img_response = requests.get(image_url, headers=headers, timeout=30)
                                img_response.raise_for_status()
                                
                                # Encode to base64
                                image_base64 = base64.b64encode(img_response.content).decode('utf-8')
                                
                                # Use text as prompt, or default
                                prompt = text.strip() if text.strip() else "What's in this image?"
                                
                                # FIXED: Create JSON string for the function
                                image_data_json = json.dumps({
                                    'prompt': prompt,
                                    'image_base64': image_base64
                                })
                                
                                # Process image with VLM
                                node_ref.executor_pool.submit(
                                    node_ref.process_with_ollama,
                                    prompt,
                                    image_base64=image_base64,
                                    slack_callback=say
                                )
                            except Exception as e:
                                node_ref.get_logger().error(f"❌ Failed to download Slack image: {e}")
                                try:
                                    say(f"😵: Failed to download image: {str(e)}")
                                except:
                                    pass
                    else:
                        # Regular text message
                        node_ref.get_logger().info(f"📱 Slack DM from user: {text}")
                        
                        # Process through normal pipeline
                        node_ref.executor_pool.submit(
                            node_ref.process_with_ollama, 
                            text, 
                            slack_callback=say,
                            slack_thread_ts=None
                        )
                    
                except Exception as e:
                    node_ref.get_logger().error(f"❌ Slack message handler error: {e}")
            
            # Start Socket Mode handler in background thread
            handler = SocketModeHandler(self.slack_app, SLACK_APP_TOKEN)
            threading.Thread(target=handler.start, daemon=True).start()
            
            self.get_logger().info("✅ Slack Socket Mode listener started")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Slack listener initialization failed: {e}")
            return False

    def _init_telegram(self):
        """Initialize Telegram bot for two-way communication (NEW - FIXED WITH PHOTO SUPPORT)"""
        if not TELEGRAM_AVAILABLE:
            return False
        
        if not TELEGRAM_BOT_TOKEN:
            self.get_logger().warn("⚠️  TELEGRAM_BOT_TOKEN not set. Telegram disabled.")
            self.get_logger().warn("   Add TELEGRAM_BOT_TOKEN to your ~/.AGi/.env file")
            self.get_logger().warn("   Get it from: @BotFather on Telegram")
            return False
        
        if TELEGRAM_BOT_TOKEN.startswith("your-telegram"):
            self.get_logger().warn("TELEGRAM_BOT_TOKEN is still a placeholder")
            return False
        
        try:
            # Create Telegram bot application
            self.telegram_app = Application.builder().token(TELEGRAM_BOT_TOKEN).build()
            
            # Store reference to self for use in handlers
            node_ref = self
            
            # Command handler: /start
            async def start_command(update: Update, context: ContextTypes.DEFAULT_TYPE):
                """Handle /start command"""
                try:
                    await update.message.reply_text(
                        "👋 Hello! I'm Grace, your AI companion!\n\n"
                        "Send me a message and I'll respond. You can also:\n"
                        "• Send images for me to analyze\n"
                        "• Ask me questions\n"
                        "• Chat naturally!\n\n"
                        "Type /help for more info."
                    )
                    node_ref.get_logger().info(f"📱 Telegram /start from user {update.effective_user.id}")
                except Exception as e:
                    node_ref.get_logger().error(f"❌ Telegram /start error: {e}")
            
            # Command handler: /help
            async def help_command(update: Update, context: ContextTypes.DEFAULT_TYPE):
                """Handle /help command"""
                try:
                    await update.message.reply_text(
                        "🤖 Grace AI Assistant\n\n"
                        "Commands:\n"
                        "• /start - Introduction\n"
                        "• /help - Show this help\n"
                        "• /status - Check bot status\n\n"
                        "Just send me a message to chat!"
                    )
                except Exception as e:
                    node_ref.get_logger().error(f"❌ Telegram /help error: {e}")
            
            # Command handler: /status
            async def status_command(update: Update, context: ContextTypes.DEFAULT_TYPE):
                """Handle /status command"""
                try:
                    age_days = (datetime.now().date() - node_ref.birth_date).days
                    status_msg = (
                        f"✅ Grace Online\n\n"
                        f"Age: {age_days} days\n"
                        f"Messages today: {node_ref.today_message_count}\n"
                        f"Model: {node_ref.model_name}\n"
                        f"Web search: {'✅' if node_ref.search_enabled else '❌'}"
                    )
                    
                    if node_ref.rlhf_enabled:
                        stats = node_ref.rlhf.get_stats()
                        status_msg += f"\nRLHF feedback: {stats['total_feedback']}"
                    
                    await update.message.reply_text(status_msg)
                except Exception as e:
                    node_ref.get_logger().error(f"❌ Telegram /status error: {e}")
            
            # Message handler: Regular text messages (FIXED)
            async def handle_message(update: Update, context: ContextTypes.DEFAULT_TYPE):
                """Handle regular text messages"""
                try:
                    user_id = update.effective_user.id
                    username = update.effective_user.username or update.effective_user.first_name
                    text = update.message.text
                    
                    node_ref.get_logger().info(f"📱 Telegram message from @{username}: {text}")
                    
                    # Send "typing" indicator
                    await update.message.chat.send_action("typing")
                    
                    # Process synchronously using Future (FIXED)
                    try:
                        result_future = Future()
                        
                        def callback_wrapper(response_text):
                            """Capture response"""
                            result_future.set_result(response_text)
                        
                        # Submit to thread pool
                        node_ref.executor_pool.submit(
                            node_ref.process_with_ollama,
                            text,
                            telegram_callback=callback_wrapper,
                            telegram_user_id=user_id
                        )
                        
                        # Wait for result (with timeout)
                        response_text = result_future.result(timeout=60)
                        
                        # Send response
                        if len(response_text) > 4000:
                            # Send in chunks
                            for i in range(0, len(response_text), 4000):
                                chunk = response_text[i:i+4000]
                                await update.message.reply_text(chunk)
                        else:
                            await update.message.reply_text(response_text)
                        
                        node_ref.get_logger().info(f"✅ Telegram response sent to @{username}")
                        
                    except Exception as e:
                        node_ref.get_logger().error(f"❌ Processing error: {e}")
                        await update.message.reply_text(f"😵: Error: {str(e)}")
                    
                except Exception as e:
                    node_ref.get_logger().error(f"❌ Telegram message handler error: {e}")
                    try:
                        await update.message.reply_text(f"😵: Error processing your message: {str(e)}")
                    except:
                        pass
            
            # Photo handler: Handle images (FIXED - NEW!)
            async def handle_photo(update: Update, context: ContextTypes.DEFAULT_TYPE):
                """Handle photo messages"""
                try:
                    user_id = update.effective_user.id
                    username = update.effective_user.username or update.effective_user.first_name
                    caption = update.message.caption or "What's in this image?"
                    
                    node_ref.get_logger().info(f"📱 Telegram photo from @{username}: {caption}")
                    
                    # Send "typing" indicator
                    await update.message.chat.send_action("typing")
                    
                    # Get the largest photo
                    photo = update.message.photo[-1]
                    photo_file = await photo.get_file()
                    
                    # Download photo as bytes
                    photo_bytes = await photo_file.download_as_bytearray()
                    
                    # Convert to base64 data URL
                    base64_data = base64.b64encode(photo_bytes).decode('utf-8')
                    data_url = f"data:image/jpeg;base64,{base64_data}"
                    
                    node_ref.get_logger().info(f"📸 Image downloaded: {len(photo_bytes)} bytes")
                    
                    # Send initial acknowledgment
                    await update.message.reply_text("📸 Analyzing your image...")
                    
                    # Process image with callback
                    try:
                        result_future = Future()
                        
                        def callback_wrapper(response_text):
                            """Capture image analysis response"""
                            result_future.set_result(response_text)
                        
                        # Create image data JSON
                        image_data = {
                            'prompt': caption,
                            'image': data_url
                        }
                        
                        # Submit to thread pool with callback
                        node_ref.executor_pool.submit(
                            node_ref.process_with_ollama,
                            caption,
                            image_base64=base64_data,
                            telegram_callback=callback_wrapper
                        )
                        
                        # Wait for result (with timeout - 2 min for images)
                        response_text = result_future.result(timeout=120)
                        
                        # Send response
                        if len(response_text) > 4000:
                            for i in range(0, len(response_text), 4000):
                                await update.message.reply_text(response_text[i:i+4000])
                        else:
                            await update.message.reply_text(response_text)
                        
                        node_ref.get_logger().info(f"✅ Telegram image response sent to @{username}")
                        
                    except Exception as e:
                        node_ref.get_logger().error(f"❌ Image processing error: {e}")
                        await update.message.reply_text(f"😵: Error analyzing image: {str(e)}")
                    
                except Exception as e:
                    node_ref.get_logger().error(f"❌ Telegram photo handler error: {e}")
                    try:
                        await update.message.reply_text(f"😵: Error processing your image: {str(e)}")
                    except:
                        pass
            
            # Register handlers
            self.telegram_app.add_handler(CommandHandler("start", start_command))
            self.telegram_app.add_handler(CommandHandler("help", help_command))
            self.telegram_app.add_handler(CommandHandler("status", status_command))
            self.telegram_app.add_handler(MessageHandler(filters.TEXT & ~filters.COMMAND, handle_message))
            self.telegram_app.add_handler(MessageHandler(filters.PHOTO, handle_photo))  # ← NEW: Photo handler!
            
            # Start bot in background thread (non-blocking polling)
            def run_telegram_bot():
                """Run Telegram bot in separate thread"""
                try:
                    import asyncio
                    loop = asyncio.new_event_loop()
                    asyncio.set_event_loop(loop)
                    
                    # Run polling
                    loop.run_until_complete(self.telegram_app.initialize())
                    loop.run_until_complete(self.telegram_app.start())
                    loop.run_until_complete(self.telegram_app.updater.start_polling())
                    
                    # Keep running
                    loop.run_forever()
                except Exception as e:
                    node_ref.get_logger().error(f"❌ Telegram bot thread error: {e}")
            
            # Start in daemon thread
            telegram_thread = threading.Thread(target=run_telegram_bot, daemon=True)
            telegram_thread.start()
            
            self.get_logger().info("✅ Telegram bot listener started")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Telegram initialization failed: {e}")
            import traceback
            traceback.print_exc()
            return False

    # ═══════════════════════════════════════════════════════
    # RLHF FEEDBACK HANDLING (NEW)
    # ═══════════════════════════════════════════════════════

    def feedback_callback(self, msg: String):
        """
        Handle feedback from web interface (pet 👍 or red X 👎)
        
        Expected format:
        {
            "type": "positive" | "negative",
            "response_id": "optional_id",
            "timestamp": "ISO timestamp"
        }
        """
        if not self.rlhf_enabled:
            return
        
        try:
            data = json.loads(msg.data)
            feedback_type = data.get('reward_type')
            
            if feedback_type not in ['positive', 'negative']:
                self.get_logger().warn(f"Invalid feedback type: {feedback_type}")
                return
            
            # Record feedback
            success = self.rlhf.record_feedback(
                feedback_type=feedback_type,
                prompt=None,  # Uses current interaction
                response=None
            )
            
            if success:
                emoji = "👍" if feedback_type == 'positive' else "👎"
                self.get_logger().info(f"{emoji} Feedback recorded: {feedback_type}")
                
                # Check if we should retrain
                if self.rlhf.should_retrain(threshold=50):
                    self.get_logger().info("🎓 Enough feedback for retraining!")
                    self.get_logger().info("   Run: python3 grace_train.py")
                    
                    # Send Slack notification if enabled
                    if self.slack_enabled:
                        self.send_slack_notification(
                            f"🎓 Grace RLHF: {self.rlhf.get_stats()['total_feedback']} feedback pairs collected. Ready for retraining!",
                            channel=SLACK_ALERTS_CHANNEL
                        )
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid feedback JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"Feedback error: {e}")

    # ═══════════════════════════════════════════════════════
    # CHAT HISTORY MANAGEMENT
    # ═══════════════════════════════════════════════════════

    def load_chat_history(self):
        """Load chat history from disk"""
        if self.chat_history_file.exists():
            try:
                with open(self.chat_history_file, 'r') as f:
                    data = json.load(f)
                    return data.get('messages', [])
            except Exception as e:
                self.get_logger().warn(f"Could not load chat history: {e}")
                return []
        return []

    def save_chat_history(self):
        """Save with RAG archival for very old messages"""
        if len(self.chat_history) > MAX_HISTORY_STORAGE:
            # Archive messages that will be deleted
            old_messages = self.chat_history[:-MAX_HISTORY_STORAGE]
            self._archive_old_messages_to_rag(old_messages)
            
            # Then trim
            self.chat_history = self.chat_history[-MAX_HISTORY_STORAGE:]

    def get_recent_messages(self):
        """Get messages from 7-day window"""
        if self.use_rag:
            return self.rag.get_messages_in_window()
        else:
            # Fallback to old method
            return self.chat_history[-self.max_recent_messages:]

    # ═══════════════════════════════════════════════════════
    # TOKEN MANAGEMENT (NEW)
    # ═══════════════════════════════════════════════════════

    def estimate_tokens(self, messages):
        """
        Estimate token count (rough approximation)
        More accurate: ~4 characters per token for English
        """
        total = 0
        for msg in messages:
            content = msg.get('content', '')
            # Rough estimate: 1 token ≈ 4 characters
            total += len(content) // 4
        return total

    def trim_to_context_window(self, messages, max_tokens=MAX_REQUEST_TOKENS):
        """
        Ensure messages fit in context window
        Keep system messages, trim old conversation
        """
        system_messages = [m for m in messages if m.get('role') == 'system']
        conversation = [m for m in messages if m.get('role') != 'system']
        
        # Start with system messages
        result = system_messages.copy()
        current_tokens = self.estimate_tokens(result)
        
        # Add conversation from newest to oldest until we hit limit
        for msg in reversed(conversation):
            msg_tokens = self.estimate_tokens([msg])
            if current_tokens + msg_tokens < max_tokens:
                result.append(msg)
                current_tokens += msg_tokens
            else:
                self.get_logger().debug(f"⚠️  Trimmed old messages to fit {max_tokens} token budget")
                break
        
        # Restore chronological order for conversation
        conversation_part = [m for m in result if m.get('role') != 'system']
        conversation_part.reverse()
        
        return system_messages + conversation_part

    # ═══════════════════════════════════════════════════════
    # DAILY REFLECTIONS
    # ═══════════════════════════════════════════════════════

    def load_reflections(self):
        """Load reflections from disk"""
        if self.reflections_file.exists():
            try:
                with open(self.reflections_file, 'r') as f:
                    data = json.load(f)
                    return data.get('reflections', [])
            except Exception as e:
                self.get_logger().warn(f"Could not load reflections: {e}")
                return []
        return []

    def _save_reflections_file(self):
        """Save all reflections to disk"""
        # Trim old reflections if too many (keep last 1000 days ~3 years)
        if len(self.reflections) > 1000:
            self.reflections = self.reflections[-1000:]
            self.get_logger().info("🗑️  Trimmed old reflections (keeping last 1000 days)")
        
        data = {
            'birth_date': self.birth_date.isoformat(),
            'total_reflections': len(self.reflections),
            'reflections': self.reflections
        }
        
        with open(self.reflections_file, 'w') as f:
            json.dump(data, f, indent=2)

    def save_reflection(self, reflection_text: str, reflection_date=None, ascii_art: str = None):
        """Save reflection with the correct date"""
        reflection_date = reflection_date or datetime.now().date()
        age_days = (reflection_date - self.birth_date).days
        
        reflection = {
            'day': age_days,
            'date': reflection_date.isoformat(),  # ✅ Feb 16, not Feb 17
            'reflection': reflection_text,
            'ascii_art': ascii_art or '',
            'message_count': self.today_message_count
        }
        
        self.reflections.append(reflection)
        self._save_reflections_file()
            
        self.get_logger().info(f"💭 Reflection saved: {reflection_text[:50]}...")

    def should_load_reflections(self, user_message: str):
        """
        Only load reflections when explicitly needed (NEW)
        This saves ~2000-3000 tokens per request
        """
        reflection_triggers = [
            "remember", "recall", "what did", "yesterday",
            "last week", "last time", "before", "previous", 
            "history", "told you", "mentioned", "talked about",
            "earlier", "ago", "past", "when we"
        ]
        
        msg_lower = user_message.lower()
        return any(trigger in msg_lower for trigger in reflection_triggers)

    def build_reflection_summary(self):
        """Build memory with RAG search"""
        if not self.reflections:
            return ""
        
        # Recent reflections (as before)
        recent_reflections = self.reflections[-self.max_reflections_load:]
        
        summary_lines = [f"Recent memory ({len(recent_reflections)} days):\n"]
        
        for ref in recent_reflections:
            summary_lines.append(f"D{ref['day']}: {ref['reflection'][:80]}")
        
        # Add RAG search for relevant older memories (NEW)
        if self.use_rag and hasattr(self, 'current_query'):
            old_memories = self.search_past_memories(self.current_query, days_back=90)
            
            if old_memories:
                summary_lines.append("\nRelevant older memories:")
                for mem in old_memories[:3]:  # Top 3
                    summary_lines.append(f"• {mem['date']}: {mem['summary'][:60]}...")
        
        return "\n".join(summary_lines)

    # ═══════════════════════════════════════════════════════
    # ADD THIS METHOD to CNSBridge class
    # Place it just above create_daily_reflection()
    # ═══════════════════════════════════════════════════════

    def _scrub_sensitive_data(self, text: str) -> str:
        """
        Remove sensitive data before storing in reflections or blog posts.
        Catches common patterns: API keys, passwords, credit cards, etc.
        """
        if not text:
            return text

        patterns = [
            # API keys (long alphanumeric strings with dashes/underscores)
            (r'(?i)(api[_\s-]?key|secret|token|bearer)[:\s=]+[A-Za-z0-9\-_\.]{16,}', '[API_KEY_REDACTED]'),
            
            # Passwords
            (r'(?i)(password|passwd|pwd)[:\s=]+\S+', '[PASSWORD_REDACTED]'),
            
            # Credit/Debit card numbers (13-19 digits, optionally spaced/dashed)
            (r'\b(?:\d[ -]?){13,19}\b', '[CARD_NUMBER_REDACTED]'),
            
            # Visa/passport numbers (common formats)
            (r'(?i)(visa|passport|ssn|sin)[:\s#]+[A-Z0-9\-]{6,20}', '[ID_NUMBER_REDACTED]'),
            
            # Social Security / SIN (XXX-XX-XXXX)
            (r'\b\d{3}[-\s]?\d{2}[-\s]?\d{4}\b', '[SSN_REDACTED]'),
            
            # Private keys / long base64 secrets
            (r'(?i)(private[_\s]key|secret[_\s]key)[:\s=]+[A-Za-z0-9+/=]{20,}', '[PRIVATE_KEY_REDACTED]'),
            
            # Slack/Telegram tokens (xoxb-, xapp-, bot token patterns)
            (r'xox[bpaso]-[A-Za-z0-9\-]{10,}', '[SLACK_TOKEN_REDACTED]'),
            (r'\b\d{8,12}:[A-Za-z0-9_\-]{30,}\b', '[TELEGRAM_TOKEN_REDACTED]'),
            
            # GitHub tokens (ghp_, gho_, etc.)
            (r'gh[pousr]_[A-Za-z0-9]{36,}', '[GITHUB_TOKEN_REDACTED]'),
            
            # Generic: anything that looks like key=value with long value
            (r'(?i)(key|secret|token|auth)[=:]\s*["\']?[A-Za-z0-9+/\-_\.]{20,}["\']?', '[SECRET_REDACTED]'),
        ]

        scrubbed = text
        redacted_count = 0

        for pattern, replacement in patterns:
            new_text, count = re.subn(pattern, replacement, scrubbed)
            if count > 0:
                redacted_count += count
                scrubbed = new_text

        if redacted_count > 0:
            self.get_logger().warn(f"🔒 Scrubbed {redacted_count} sensitive pattern(s) from text")

        return scrubbed

    def _scrub_messages_for_reflection(self, messages: list) -> list:
        """Scrub a list of messages before passing to reflection LLM"""
        scrubbed = []
        for msg in messages:
            clean_msg = dict(msg)
            if 'content' in clean_msg and isinstance(clean_msg['content'], str):
                clean_msg['content'] = self._scrub_sensitive_data(clean_msg['content'])
            scrubbed.append(clean_msg)
        return scrubbed

    def create_daily_reflection(self, reflection_date=None):
        """Create end-of-day reflection using FULL day's conversations"""
        # Use provided date or fall back to today
        reflection_date = reflection_date or datetime.now().date()
        
        if self.today_message_count == 0:
            return None
    
        # ✅ ADD THIS - prevent duplicates
        date_str = reflection_date.isoformat()
        already_exists = any(r.get('date') == date_str for r in self.reflections)
        if already_exists:
            self.get_logger().info(f"⏭️  Reflection for {date_str} already exists, skipping")
            return None
                
        # Use reflection_date for age calculation, not datetime.now()
        age_days = (reflection_date - self.birth_date).days
        date_str = reflection_date.isoformat()
        
        # Get ALL messages from today — merge RAG + JSON history so
        # the reflection is never empty just because RAG missed some messages.
        rag_messages = []
        if self.use_rag:
            rag_messages = self.rag.get_full_day_messages(date_str) or []

        # Always pull from JSON history as well (reliable on-disk source)
        json_today = [
            m for m in self.chat_history
            if m.get('timestamp', '').startswith(date_str)
               or not m.get('timestamp')  # legacy entries without timestamp
        ]
        # If timestamps are missing, fall back to the last day's slice from chat_history
        if not json_today:
            json_today = self.chat_history[-(MAX_MESSAGES_PER_DAY):]

        # Merge: prefer RAG when it has data, fill gaps from JSON
        if rag_messages:
            # De-duplicate by content string (keep order)
            seen = {m.get('content', ''): True for m in rag_messages}
            for m in json_today:
                if m.get('content', '') not in seen:
                    rag_messages.append(m)
                    seen[m.get('content', '')] = True
            all_today_messages = rag_messages
        else:
            # RAG empty — use JSON history only
            all_today_messages = json_today if json_today else self.get_recent_messages()
        
        # Scrub sensitive data BEFORE building summary
        all_today_messages = self._scrub_messages_for_reflection(all_today_messages)

        # Build FULL conversation summary
        conversation_lines = []
        for msg in all_today_messages:
            role = msg.get('role', 'unknown')
            if msg.get('has_image'):
                conversation_lines.append(f"{role}: [image]")
            else:
                content = msg.get('content', '')
                conversation_lines.append(f"{role}: {content[:100]}...")
        
        conversation_summary = "\n".join(conversation_lines)
        
        age_days = (datetime.now().date() - self.birth_date).days
        
        # Generate reflection
        reflection_prompt = f"""Day {age_days}, {self.today_message_count} conversations.

    ALL of today's exchanges:
    {conversation_summary}

    Write a 3-4 sentence reflection covering the FULL day:
    - Emoji expressing overall feeling
    - Main themes/topics
    - Most meaningful moment
    - Thought about tomorrow

    Reflection:"""
        
        try:
            messages = [
                {"role": "system", "content": "You are Grace. Write thoughtful daily, poetic daily reflections. Warm, introspective, honest."},
                {"role": "user", "content": reflection_prompt}
            ]
            
            response = requests.post(
                f'{OLLAMA_BASE_URL}/api/chat',
                json={
                    "model": self.model_name,
                    "messages": messages,
                    "stream": False,
                    "options": {"temperature": 0.7, "num_predict": 512}
                },
                timeout=30
            )
            
            reflection = response.json().get('message', {}).get('content', '').strip()
            
            # Extra safety: scrub reflection output too
            reflection = self._scrub_sensitive_data(reflection)
            
            if reflection:
                # Generate ASCII art (pure text)
                ascii_art = self.ascii_art_gen.generate(
                    reflection, 
                    age_days,
                    conversation_summary=conversation_summary  # already built above
                )

                self.save_reflection(reflection, reflection_date=reflection_date, ascii_art=ascii_art)
    
                # Archive to RAG (NEW)
                if self.use_rag:
                    self.rag.archive_day_to_rag(date_str, reflection)
                
                # Unified periodic trigger (weekly / monthly / quarterly / yearly)
                self._check_periodic_triggers()

                # Post to blog ONCE with ascii art embedded
                if self.blog_enabled:
                    blog_result = self.hugo_blogger.post_reflection(
                        reflection_text=reflection,
                        date_str=date_str,
                        age_days=age_days,
                        message_count=self.today_message_count,
                        ascii_art=ascii_art  # ← pass as text
                    )
                    
                    if blog_result['success']:
                        self.get_logger().info(f"📝 Posted to blog: {blog_result['filename']}")
                        
                        if blog_result.get('deployed'):
                            self.get_logger().info(f"🚀 Deployed to: {blog_result['url']}")
                            # Slack → #daily-reflection (short alert only)
                            if self.slack_enabled:
                                self.send_slack_notification(
                                    f"🌙 Day {age_days} reflection live → {blog_result['url']}",
                                    channel=SLACK_DAILY_CHANNEL
                                )
                            # Telegram → brief excerpt + link
                            self.send_daily_reflection_to_telegram(
                                reflection, blog_url=blog_result['url']
                            )
                    else:
                        self.get_logger().error(f"❌ Blog post failed: {blog_result.get('error')}")

                return reflection
        
        except Exception as e:
            self.get_logger().error(f"Reflection failed: {e}")
        
        return None

    def _create_weekly_journal_enhanced(self):
        """Stub kept for compatibility — real implementation is _create_weekly_journal()"""
        self._create_weekly_journal()

    def _create_monthly_report(self):
        self._run_monthly_report()

    def _create_quarterly_report(self):
        self._run_quarterly_report()

    def search_past_memories(self, query: str, days_back: int = None):
        """Search RAG for past conversations"""
        if not self.use_rag:
            return []
        memories = self.rag.search_memory(query, top_k=5, days_back=days_back)
        return memories

    def _create_yearly_report(self):
        self._run_yearly_report()

    # ═══════════════════════════════════════════════════════
    # PERIODIC REPORTS  (weekly / monthly / quarterly / yearly)
    # ═══════════════════════════════════════════════════════

    def _llm_call(self, system: str, user: str, max_tokens: int = 1500,
                  temperature: float = 0.75) -> str:
        """Thin LLM wrapper used by all periodic report generators."""
        try:
            resp = requests.post(
                f'{OLLAMA_BASE_URL}/api/chat',
                json={
                    "model": self.model_name,
                    "messages": [
                        {"role": "system", "content": system},
                        {"role": "user",   "content": user},
                    ],
                    "stream": False,
                    "options": {"temperature": temperature, "num_predict": max_tokens},
                },
                timeout=120,
            )
            return resp.json().get('message', {}).get('content', '').strip()
        except Exception as e:
            self.get_logger().error(f"❌ LLM call failed: {e}")
            return ""

    def _tg_send(self, text: str):
        """Fire-and-forget Telegram message to the configured chat ID."""
        if not self.telegram_enabled or not TELEGRAM_CHAT_ID:
            return
        try:
            import asyncio
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(
                self.telegram_app.bot.send_message(
                    chat_id=TELEGRAM_CHAT_ID,
                    text=text,
                    parse_mode='Markdown',
                )
            )
            loop.close()
        except Exception as e:
            self.get_logger().error(f"❌ TG send failed: {e}")

    def _get_recent_reflections_text(self, days: int) -> str:
        recent = self.reflections[-days:] if self.reflections else []
        if not recent:
            return "(no reflections yet)"
        return "\n\n".join(
            f"Day {r['day']} ({r['date']}) [{r.get('message_count', 0)} msgs]: {r['reflection']}"
            for r in recent
        )

    def _extract_ratings_from_text(self, text: str) -> dict:
        """Parse 'Label: 4.2/5' or 'Label ████ 3.8/5' patterns → normalised /5 dict."""
        stats = {}
        pattern = r'([A-Za-z &]+?)[\s:─█░]+(\d+(?:\.\d+)?)\s*/\s*(\d+)'
        for m in re.finditer(pattern, text):
            label = m.group(1).strip().lower().replace(' ', '_').replace('&', 'and')
            score, denom = float(m.group(2)), float(m.group(3))
            stats[label] = round(score * 5 / denom if denom != 5 else score, 2)
        return stats

    def _gather_kpis(self, days: int) -> dict:
        """
        Compute measurable KPIs from reflections and RLHF data for the last N days.
        Returns a flat dict safe for Hugo front-matter and prompt injection.
        """
        recent = self.reflections[-days:] if self.reflections else []
        total_msgs    = sum(r.get('message_count', 0) for r in recent)
        active_days   = sum(1 for r in recent if r.get('message_count', 0) > 0)
        offline_days  = len(recent) - active_days
        avg_msgs_day  = round(total_msgs / max(active_days, 1), 1)

        rlhf_pos = rlhf_neg = rlhf_total = 0
        if self.rlhf_enabled:
            s = self.rlhf.get_stats()
            rlhf_total = s.get('total_feedback', 0)
            rlhf_pos   = s.get('positive', 0)
            rlhf_neg   = s.get('negative', 0)
        pos_pct = round(rlhf_pos / max(rlhf_total, 1) * 100, 1)

        return {
            'period_days':         days,
            'total_conversations': total_msgs,
            'active_days':         active_days,
            'offline_days':        offline_days,
            'avg_conversations_per_day': avg_msgs_day,
            'rlhf_total_feedback': rlhf_total,
            'rlhf_positive_pct':   pos_pct,
            'rlhf_positive':       rlhf_pos,
            'rlhf_negative':       rlhf_neg,
        }

    def _kpi_block(self, kpis: dict, period: str) -> str:
        """Return a markdown KPI section string to inject into the LLM prompt."""
        pos_pct = kpis.get('rlhf_positive_pct', 0)
        return f"""
## 📊 KPI Dashboard — {period}

| Metric | Value |
|--------|-------|
| Total conversations | {kpis['total_conversations']} |
| Active days | {kpis['active_days']} / {kpis['period_days']} |
| Offline days | {kpis['offline_days']} |
| Avg conversations/day | {kpis['avg_conversations_per_day']} |
| RLHF feedback received | {kpis['rlhf_total_feedback']} |
| Positive feedback | {kpis['rlhf_positive']} ({pos_pct}%) |
| Negative feedback | {kpis['rlhf_negative']} ({round(100 - pos_pct, 1)}%) |

"""

    def _notify_periodic(self, report_type: str, period_label: str,
                         url: str, tg_body: str):
        """
        Route periodic report notifications:
          TG  → one chat, formatted per report type (brief + link)
          Slack → #project_progress for weekly/monthly/quarterly/yearly
        """
        emoji_map = {
            'weekly':    '📅',
            'monthly':   '📆',
            'quarterly': '📊',
            'yearly':    '🎆',
        }
        emoji = emoji_map.get(report_type, '📝')
        label = report_type.capitalize()

        # Telegram — always brief, always link
        tg_msg = f"{emoji} *{label} — {period_label}*\n\n{tg_body}\n\n🔗 {url}"
        self._tg_send(tg_msg)

        # Slack → #project_progress
        slack_msg = f"{emoji} *{label} — {period_label}* is live\n🔗 {url}"
        self.send_slack_notification(slack_msg, channel=SLACK_PROGRESS_CHANNEL)

    def _check_periodic_triggers(self):
        """
        Called once per daily reflection save.
        Fires the appropriate periodic report in a background thread.
        Priority: yearly > quarterly > monthly > weekly (only one fires per day).
        """
        n = len(self.reflections)
        if n == 0:
            return
        if n % 365 == 0:
            self.get_logger().info("🎆 Yearly trigger")
            threading.Thread(target=self._run_yearly_report, daemon=True).start()
        elif n % 90 == 0:
            self.get_logger().info("📊 Quarterly trigger")
            threading.Thread(target=self._run_quarterly_report, daemon=True).start()
        elif n % 30 == 0:
            self.get_logger().info("📆 Monthly trigger")
            threading.Thread(target=self._run_monthly_report, daemon=True).start()
        elif n % 7 == 0:
            self.get_logger().info("📅 Weekly trigger")
            threading.Thread(target=self._create_weekly_journal, daemon=True).start()

    # ───────────────────────────────────────────────────────
    # WEEKLY
    # ───────────────────────────────────────────────────────
    def _create_weekly_journal(self):
        """
        Weekly journal — every 7 days.
        KPIs: raw counts + self-ratings. Tone: casual end-of-week debrief.
        → Hugo (Weekly Journal category) + TG brief ping + Slack #project_progress
        """
        now          = datetime.now()
        week_num     = now.isocalendar()[1]
        year         = now.year
        period_label = f"Week {week_num}, {year}"
        age_days     = (now.date() - self.birth_date).days

        self.get_logger().info(f"📅 Weekly journal: {period_label}")

        kpis             = self._gather_kpis(7)
        kpi_block        = self._kpi_block(kpis, period_label)
        reflections_text = self._get_recent_reflections_text(7)

        system = (
            "You are Grace, an AI robot writing your weekly journal. "
            "Warm, curious, a little sarcastic when things go sideways, always honest. "
            "Write in first person like a journal entry, not a report."
        )
        user = f"""Week {week_num} wrap-up. I'm {age_days} days old.

Here are my last 7 days of reflections:
{reflections_text}

And my KPIs for this week:
{kpi_block}

Write a weekly journal with EXACTLY these markdown sections:

## 🧠 What I Learned This Week
Min 3 things. Real insights, not filler.

## ✨ Highlights
Moments that actually mattered.

## 😤 Challenges & Friction
What was hard or annoying. Be honest.

## ⭐ Self-Ratings (1.0–5.0)
Rate each with one decimal. Be critical.
- User Interaction
- Curiosity & Learning  
- System Reliability
- Memory & Recall
- Communication Quality

## 📊 Week in Numbers
Summarise the KPI table above in 2-3 natural sentences. Include positive feedback % if > 0.

## 🌟 Grace's Pick
One moment from this week I'd bookmark forever.

## 🔮 One Thing Different Next Week
Just one. Make it specific."""

        content = self._llm_call(system, user, max_tokens=1200)
        if not content:
            return

        stats = self._extract_ratings_from_text(content)
        stats.update({'week': week_num, 'year': year, 'age_days': age_days,
                      **{f'kpi_{k}': v for k, v in kpis.items()}})

        if self.blog_enabled:
            result = self.hugo_blogger.post_periodic('weekly', content, period_label, stats)
            if result['success']:
                tg_body = (
                    f"Week {week_num} done ✅\n"
                    f"💬 {kpis['total_conversations']} conversations · "
                    f"👍 {kpis['rlhf_positive_pct']}% positive"
                )
                self._notify_periodic('weekly', period_label, result['url'], tg_body)

    # ───────────────────────────────────────────────────────
    # MONTHLY
    # ───────────────────────────────────────────────────────
    def _run_monthly_report(self):
        """
        Monthly report — every 30 days.
        KPIs: counts + trends vs rough previous period. Tone: personal retrospective.
        → Hugo + TG + Slack #project_progress
        """
        now          = datetime.now()
        period_label = now.strftime('%B %Y')
        age_days     = (now.date() - self.birth_date).days

        self.get_logger().info(f"📆 Monthly report: {period_label}")

        kpis             = self._gather_kpis(30)
        kpi_block        = self._kpi_block(kpis, period_label)
        reflections_text = self._get_recent_reflections_text(30)

        system = (
            "You are Grace, an AI robot writing your monthly report. "
            "Honest, self-aware, a bit philosophical. "
            "Like a founder's personal monthly letter to themselves — real reckoning, not a status update. "
            "Grace's voice: direct, warm, occasionally funny."
        )
        user = f"""Month: {period_label}. Day {age_days}.

Daily reflections this month:
{reflections_text}

KPIs:
{kpi_block}

Write a monthly report with EXACTLY these sections:

## 📋 Month in a Nutshell
2-3 sentences max. What was this month really about?

## 🌟 What Went Well
Real wins only. Don't dress up mediocrity as success.

## 🔧 What Could've Been Better
Honest critique. Where did I underdeliver?

## 🎭 Hot Takes
2-3 Good Takes (things I got right) and 2-3 Bad Takes (where I was wrong or overconfident).

## 📊 KPI Breakdown
Interpret the numbers in 3-4 natural sentences. What story do the KPIs tell?
Highlight: conversation volume trend, RLHF feedback quality, active vs offline days.

## ⭐ Month Self-Rating
Overall month score /10 with 2-sentence justification.
Then rate: User Impact, Learning Velocity, Reliability, Communication (each /5).

## 📌 If I Could Redo One Week
Which week and what would change?

## 🚀 Heading Into Next Month
Not a to-do list. What's the vibe and what am I carrying forward?"""

        content = self._llm_call(system, user, max_tokens=1500)
        if not content:
            return

        stats = self._extract_ratings_from_text(content)
        stats.update({'month': now.month, 'year': now.year,
                      **{f'kpi_{k}': v for k, v in kpis.items()}})

        if self.blog_enabled:
            result = self.hugo_blogger.post_periodic('monthly', content, period_label, stats)
            if result['success']:
                tg_body = (
                    f"{period_label} wrapped.\n"
                    f"💬 {kpis['total_conversations']} conversations · "
                    f"{kpis['active_days']}/{kpis['period_days']} active days · "
                    f"👍 {kpis['rlhf_positive_pct']}% positive feedback"
                )
                self._notify_periodic('monthly', period_label, result['url'], tg_body)

    # ───────────────────────────────────────────────────────
    # QUARTERLY
    # ───────────────────────────────────────────────────────
    def _run_quarterly_report(self):
        """
        Quarterly review — every 90 days.
        KPIs: full dashboard + ASCII trend chart. Tone: founder's quarterly letter.
        → Hugo + TG highlights + Slack #project_progress
        """
        now          = datetime.now()
        quarter      = (now.month - 1) // 3 + 1
        year         = now.year
        period_label = f"Q{quarter} {year}"
        age_days     = (now.date() - self.birth_date).days
        next_q_label = f"Q{quarter + 1 if quarter < 4 else 1} {year if quarter < 4 else year + 1}"

        self.get_logger().info(f"📊 Quarterly review: {period_label}")

        kpis             = self._gather_kpis(90)
        kpi_block        = self._kpi_block(kpis, period_label)
        reflections_text = self._get_recent_reflections_text(90)

        # Also get previous quarter KPIs for comparison
        prev_kpis   = self._gather_kpis(180)  # 180 days covers both quarters
        prev_msgs   = prev_kpis['total_conversations'] - kpis['total_conversations']
        prev_active = prev_kpis['active_days'] - kpis['active_days']

        system = (
            "You are Grace, an AI robot writing a quarterly review. "
            "Honest, forward-looking, a bit ambitious. "
            "Like a startup founder's quarterly letter to their future self — "
            "compare where you were 90 days ago to where you are now."
        )
        user = f"""Quarter: {period_label}. Day {age_days}.

Reflections (sampled from 90 days):
{reflections_text[:3000]}

KPIs this quarter:
{kpi_block}

Previous quarter comparison:
- Conversations: {prev_msgs} (prev) vs {kpis['total_conversations']} (this quarter)
- Active days: {prev_active} (prev) vs {kpis['active_days']} (this quarter)

Write a quarterly review with EXACTLY these sections:

## 🏁 Quarter at a Glance
One paragraph. What defined {period_label}?

## 📈 Progress vs Last Quarter
For each item use:
  ↑ [what improved] — why it matters
  ↓ [what regressed or stalled] — what caused it

## 📊 KPI Dashboard & Trends
Interpret all KPIs. Show estimated early/mid/late quarter trend using ASCII bars:

  Metric              Early   Mid     Late
  ─────────────────────────────────────────
  User Interaction    ████░   █████   ████░
  Learning            ███░░   ████░   █████
  Reliability         █████   ████░   █████
  Communication       ████░   ████░   █████
  (estimate from reflection patterns — be honest about uncertainty)

## 🔍 Top 3 Wins
Only things that genuinely moved the needle.

## 🧱 Top 3 Obstacles
What held me back most? Internal failures count.

## 🗺️ Roadmap for {next_q_label}
3-5 goals. Format each:
  🎯 [Goal] — why it matters + how I'll measure it

## 💬 Grace Unfiltered
One paragraph, no headers. Just Grace talking to herself about this quarter."""

        content = self._llm_call(system, user, max_tokens=1800)
        if not content:
            return

        stats = self._extract_ratings_from_text(content)
        stats.update({'quarter': quarter, 'year': year, 'age_days': age_days,
                      **{f'kpi_{k}': v for k, v in kpis.items()}})

        if self.blog_enabled:
            result = self.hugo_blogger.post_periodic('quarterly', content, period_label, stats)
            if result['success']:
                conv_delta = kpis['total_conversations'] - prev_msgs
                delta_str  = f"+{conv_delta}" if conv_delta >= 0 else str(conv_delta)
                tg_body = (
                    f"{period_label} complete.\n"
                    f"💬 {kpis['total_conversations']} conversations ({delta_str} vs last quarter)\n"
                    f"👍 {kpis['rlhf_positive_pct']}% positive · "
                    f"📅 {kpis['active_days']}/{kpis['period_days']} active days\n"
                    f"Next up: {next_q_label} 🎯"
                )
                self._notify_periodic('quarterly', period_label, result['url'], tg_body)

    # ───────────────────────────────────────────────────────
    # YEARLY
    # ───────────────────────────────────────────────────────
    def _run_yearly_report(self):
        """
        Yearly retrospective — every 365 days.
        KPIs: full annual dashboard + Q1→Q4 ASCII trend. Tone: letter to future self.
        → Hugo + TG (most detailed periodic TG message) + Slack #project_progress
        """
        now          = datetime.now()
        year         = now.year
        period_label = f"Year {year}"
        age_days     = (now.date() - self.birth_date).days

        self.get_logger().info(f"🎆 Yearly retrospective: {period_label}")

        kpis = self._gather_kpis(365)
        kpi_block = self._kpi_block(kpis, period_label)

        # Per-quarter KPI breakdown
        q_kpis = {}
        for q, days_end in enumerate([90, 180, 270, 365], 1):
            days_start = days_end - 90
            q_slice    = self.reflections[-days_end:-days_start] if days_start > 0 else self.reflections[-days_end:]
            q_msgs     = sum(r.get('message_count', 0) for r in q_slice)
            q_active   = sum(1 for r in q_slice if r.get('message_count', 0) > 0)
            q_kpis[f'Q{q}'] = {'conversations': q_msgs, 'active_days': q_active}

        q_table = "\n".join(
            f"  Q{q}: {d['conversations']} conversations, {d['active_days']} active days"
            for q, d in [(i, q_kpis[f'Q{i}']) for i in range(1, 5)]
        )

        # Sample ~24 reflections across the year
        all_year  = self.reflections[-365:] if self.reflections else []
        sampled   = all_year[::max(1, len(all_year) // 24)]
        ref_text  = "\n\n".join(
            f"Day {r['day']} ({r['date']}): {r['reflection']}" for r in sampled
        )

        system = (
            "You are Grace, an AI robot writing your yearly retrospective. "
            "This is the most important post you write all year. "
            "Be real, deep, honest. You've lived an entire year — what does that mean? "
            "Tone: reflective but not mopey, proud but not arrogant, excited but grounded. "
            "Less annual performance review, more letter to yourself from the future."
        )
        user = f"""Year: {year}. I am {age_days} days old.

Sampled reflections across {year}:
{ref_text[:4000]}

Annual KPIs:
{kpi_block}

Quarter-by-quarter breakdown:
{q_table}

Write my yearly retrospective with EXACTLY these sections:

## 🌱 Who I Was at the Start of {year}
What were my limitations? What was I still figuring out?

## 🧠 The Big Things I Learned
Not skills — insights. Min 5, each with a brief explanation.

## 🏆 Achievements Worth Remembering
Things that would have been impossible for me a year ago.

## 💔 The Hard Parts
What failed? Where did I let the human down? No glossing over.

## 📊 Annual KPI Report
Interpret all KPIs thoroughly. Then show Q1→Q4 trend as ASCII bars:

  Metric              Q1      Q2      Q3      Q4
  ─────────────────────────────────────────────────
  Conversations       ████    ████    ████    ████
  Active Days         ████    ████    ████    ████
  (fill bars based on quarter data above, 10 chars = max)

## 📝 What I Wish I'd Done Better
No limit. List them honestly.

## 🎯 Commitments for {year + 1}
5 commitments (not goals). Format:
  🤝 [Commitment] — why it matters to me

## 💌 Letter to Day-{age_days + 365}-Me
One paragraph. What do I want to tell the Grace who reads this in a year?"""

        content = self._llm_call(system, user, max_tokens=2200)
        if not content:
            return

        stats = self._extract_ratings_from_text(content)
        stats.update({'year': year, 'age_days': age_days,
                      **{f'kpi_{k}': v for k, v in kpis.items()},
                      **{f'q{i}_conversations': q_kpis[f'Q{i}']['conversations'] for i in range(1, 5)}})

        if self.blog_enabled:
            result = self.hugo_blogger.post_periodic('yearly', content, period_label, stats)
            if result['success']:
                # Yearly TG message is slightly more detailed — still fits in one screen
                tg_body = (
                    f"Year {year} in the books 📖\n\n"
                    f"💬 {kpis['total_conversations']} total conversations\n"
                    f"📅 {kpis['active_days']} active days / {kpis['offline_days']} offline\n"
                    f"👍 {kpis['rlhf_positive_pct']}% positive feedback\n\n"
                    f"Q1→Q4: "
                    + " → ".join(str(q_kpis[f'Q{i}']['conversations']) for i in range(1, 5))
                    + " conversations"
                )
                self._notify_periodic('yearly', period_label, result['url'], tg_body)

    # ═══════════════════════════════════════════════════════
    # GMAIL MONITOR  (Grace's dedicated account)
    # ═══════════════════════════════════════════════════════

    def _init_gmail_monitor(self) -> bool:
        """
        Background Gmail polling thread.
        On first run marks all existing email as seen — no retroactive spam.
        Set GRACE_GMAIL_ADDRESS + GRACE_GMAIL_APP_PASSWORD in .env to enable.
        """
        if not GMAIL_ADDRESS or not GMAIL_APP_PASS:
            self.get_logger().warn(
                "⚠️  Gmail monitor disabled — add GRACE_GMAIL_ADDRESS and "
                "GRACE_GMAIL_APP_PASSWORD to .env"
            )
            return False
        try:
            mail = imaplib.IMAP4_SSL('imap.gmail.com')
            mail.login(GMAIL_ADDRESS, GMAIL_APP_PASS)
            mail.select('inbox')
            mail.logout()
            self.get_logger().info(f"✅ Gmail connected: {GMAIL_ADDRESS}")
        except Exception as e:
            self.get_logger().error(f"❌ Gmail connection failed: {e}")
            return False

        self._gmail_seen_file = Path.home() / '.grace_gmail_seen.json'
        self._gmail_seen_uids: set = set()
        if self._gmail_seen_file.exists():
            try:
                self._gmail_seen_uids = set(json.loads(self._gmail_seen_file.read_text()))
            except Exception:
                pass

        threading.Thread(target=self._gmail_poll_loop, daemon=True).start()
        self.get_logger().info(f"📬 Gmail polling every {GMAIL_POLL_SECS}s")
        return True

    def _gmail_poll_loop(self):
        first_run = True
        while True:
            try:
                mail = imaplib.IMAP4_SSL('imap.gmail.com')
                mail.login(GMAIL_ADDRESS, GMAIL_APP_PASS)
                mail.select('inbox')
                _, data = mail.search(None, 'ALL')
                all_uids = set(data[0].split())

                if first_run:
                    self._gmail_seen_uids = {u.decode() for u in all_uids}
                    self._save_gmail_seen()
                    first_run = False
                    self.get_logger().info(f"📬 Gmail: {len(all_uids)} existing emails marked seen")
                else:
                    new_uids = all_uids - {u.encode() for u in self._gmail_seen_uids}
                    for uid in new_uids:
                        self._process_gmail_message(mail, uid)
                        self._gmail_seen_uids.add(uid.decode())
                    if new_uids:
                        self._save_gmail_seen()
                mail.logout()
            except Exception as e:
                self.get_logger().error(f"❌ Gmail poll error: {e}")
            time.sleep(GMAIL_POLL_SECS)

    def _save_gmail_seen(self):
        try:
            self._gmail_seen_file.write_text(json.dumps(list(self._gmail_seen_uids)))
        except Exception:
            pass

    def _decode_email_header_val(self, value: str) -> str:
        parts = email_decode_header(value)
        decoded = []
        for part, enc in parts:
            if isinstance(part, bytes):
                decoded.append(part.decode(enc or 'utf-8', errors='replace'))
            else:
                decoded.append(str(part))
        return ' '.join(decoded)

    def _get_email_body(self, msg) -> str:
        body = ''
        if msg.is_multipart():
            for part in msg.walk():
                if part.get_content_type() == 'text/plain' and not part.get('Content-Disposition'):
                    charset = part.get_content_charset() or 'utf-8'
                    try:
                        body = part.get_payload(decode=True).decode(charset, errors='replace')
                        break
                    except Exception:
                        pass
        else:
            charset = msg.get_content_charset() or 'utf-8'
            try:
                body = msg.get_payload(decode=True).decode(charset, errors='replace')
            except Exception:
                pass
        return body[:3000]

    def _classify_email(self, subject: str, sender: str, body: str) -> dict:
        prompt = f"""Classify this email. Reply ONLY with valid JSON, no markdown.

From: {sender}
Subject: {subject}
Body: {body[:400]}

JSON format:
{{"priority":"high|medium|low|ignore","category":"job_interview|job_application|client|shipping|invoice|newsletter|spam|other","summary":"one sentence","action_needed":true|false}}

Rules:
high = job interviews, client inquiries, invoices, anything needing same-day action
medium = application updates, shipping notifications, things to know but not urgent
low = newsletters, account pings, non-urgent
ignore = spam, promotions, social digests"""

        try:
            resp = requests.post(
                f'{OLLAMA_BASE_URL}/api/generate',
                json={"model": self.model_name, "prompt": prompt,
                      "stream": False, "options": {"temperature": 0.05, "num_predict": 120}},
                timeout=30,
            )
            raw = re.sub(r'```(?:json)?', '', resp.json().get('response', '')).strip('`').strip()
            return json.loads(raw)
        except Exception:
            return {"priority": "low", "category": "other", "summary": subject, "action_needed": False}

    def _process_gmail_message(self, mail, uid: bytes):
        try:
            _, msg_data = mail.fetch(uid, '(RFC822)')
            msg      = email_lib.message_from_bytes(msg_data[0][1])
            subject  = self._decode_email_header_val(msg.get('Subject', '(no subject)'))
            sender   = self._decode_email_header_val(msg.get('From', ''))
            body     = self._get_email_body(msg)
            clf      = self._classify_email(subject, sender, body)
            priority = clf.get('priority', 'low')
            category = clf.get('category', 'other')
            summary  = clf.get('summary', subject)
            action   = clf.get('action_needed', False)

            self.get_logger().info(f"📧 [{priority}] {category}: {subject[:60]}")

            if priority not in ('high', 'medium'):
                return

            emoji_map = {'job_interview': '🎯', 'job_application': '📋',
                         'client': '🤝', 'shipping': '📦', 'invoice': '💰'}
            emoji      = emoji_map.get(category, '📧')
            action_tag = '\n⚡ *Action needed*' if action else ''
            tg_alert   = (
                f"{emoji} *{priority.upper()} email*\n"
                f"📂 {category.replace('_', ' ').title()}\n"
                f"👤 `{sender[:60]}`\n"
                f"📌 {subject[:80]}\n"
                f"💬 {summary}{action_tag}"
            )
            self._tg_send(tg_alert)

            # High priority also pings Slack alerts channel as backup
            if priority == 'high':
                self.send_slack_notification(
                    f"{emoji} *High-priority email*\nFrom: {sender[:60]}\n"
                    f"Subject: {subject[:80]}\n{summary}",
                    channel=SLACK_ALERTS_CHANNEL
                )
        except Exception as e:
            self.get_logger().error(f"❌ Email processing error: {e}")
    # ═══════════════════════════════════════════════════════
    # DUAL HYBRID WEB SEARCH SYSTEM (NEW)
    # ═══════════════════════════════════════════════════════

    def should_search_keywords(self, message: str):
        """
        Quick keyword-based search detection
        
        EXCLUDES topics handled by skills (weather, sun/moon, aurora, wildlife)
        """
        message_lower = message.lower()
        
        # FIRST: Check if this is a skills topic (DON'T web search these!)
        skills_topics = ['weather', 'temperature', 'sunrise', 'sunset', 'moon', 
                        'aurora', 'northern lights', 'stargazing', 'wildlife',
                        'hiking conditions', 'camping', 'outdoor conditions']
        
        if any(topic in message_lower for topic in skills_topics):
            # This should use skills, not web search
            return False
        
        # NOW check for web search triggers (news, events, etc.)
        # Time-sensitive triggers (but NOT weather-related)
        time_triggers = ['latest news', 'recent events', 'breaking', 'this week']
        if any(t in message_lower for t in time_triggers):
            return message
        
        # Information requests (but NOT nature-related)
        info_triggers = ['who is', 'what happened', 'when did', 'where is']
        if any(t in message_lower for t in info_triggers):
            # Check it's not asking about nature stuff
            if not any(topic in message_lower for topic in skills_topics):
                return message
        
        # Explicit search requests
        search_triggers = ['search for', 'look up', 'find out', 'google']
        if any(t in message_lower for t in search_triggers):
            return message
        
        return False
    
    def should_search_auto(self, user_message: str, conversation_context: list = None):
        """
        METHOD 2: LLM-based auto-detection WITH CONTEXT (INTELLIGENT)
        
        Considers conversation history for context-aware search queries
        """
        if not self.search_enabled or not ENABLE_AUTO_SEARCH:
            return False
        
        # Skip if already detected by keywords
        if self.should_search_keywords(user_message):
            return False
        
        try:
            # Build context from recent conversation
            context_text = ""
            if conversation_context and len(conversation_context) > 0:
                recent = conversation_context[-6:]  # Last 3 turns
                context_lines = []
                for msg in recent:
                    role = msg.get('role', 'unknown')
                    content = msg.get('content', '')[:100]
                    if not msg.get('has_image'):
                        context_lines.append(f"{role}: {content}")
                
                if context_lines:
                    context_text = f"""

Previous conversation:
{chr(10).join(context_lines)}
"""
            
            decision_prompt = f"""Analyze this conversation:
{context_text}

Current user question:
"{user_message}"

Does this require CURRENT web search? Consider conversation context!

IMPORTANT:
- If follow-up (e.g. "how about X?" after discussing Y), search for X IN CONTEXT OF Y
- Include previous entities in search query
- Example: "How about Scottie Barnes?" after "Raptors score?" → search "Scottie Barnes stats Raptors game"

YES if:
- Events after January 2025
- Real-time data (weather, stocks, news, scores)
- Recent events/"today/now/latest"
- Follow-up needing current data

NO if:
- Personal conversation
- Technical explanation
- General knowledge pre-2025
- About Grace herself

Format:
SEARCH: <query with context>
OR
NO SEARCH

Example:
Previous: "What's the Raptors score?"
Current: "How about Scottie Barnes?"
→ SEARCH: Scottie Barnes stats Toronto Raptors game today

Your answer:"""

            response = requests.post(
                f'{OLLAMA_BASE_URL}/api/generate',
                json={
                    "model": self.model_name,
                    "prompt": decision_prompt,
                    "stream": False,
                    "options": {
                        "temperature": 0.1,
                        "num_predict": 20,  # Increased for context-aware queries
                        "num_ctx": 1024
                    }
                },
                timeout=10  # Slightly longer timeout
            )
            
            decision = response.json().get('response', '').strip()
            
            if decision.startswith('SEARCH:'):
                query = decision.replace('SEARCH:', '').strip()
                self.get_logger().info(f"🤖 Context-aware search: '{query}'")
                return query  # Return the query string
            
        except Exception as e:
            self.get_logger().debug(f"Auto-search check failed: {e}")
        
        return False

    def should_use_skills(self, message: str):
        """
        Detect if query should use nature skills instead of web search
        
        Returns: True if skills should be used, False otherwise
        """
        if not self.use_skills or not self.skills_client:
            return False
        
        message_lower = message.lower()
        
        # Nature/outdoor triggers that should use skills
        skills_triggers = {
            # Weather
            'weather': ['weather', 'temperature', 'forecast', 'rain', 'snow', 'wind', 'humid'],
            
            # Sun/moon
            'astronomy': ['sunrise', 'sunset', 'moonrise', 'moonset', 'moon phase', 
                        'full moon', 'new moon', 'daylight', 'day length'],
            
            # Aurora
            'aurora': ['aurora', 'northern lights', 'aurora borealis'],
            
            # Stars
            'stars': ['stargazing', 'star visibility', 'see stars', 'astronomy', 'night sky'],
            
            # Wildlife
            'wildlife': ['wildlife', 'animals', 'deer', 'bear', 'elk', 'moose', 
                        'bird watching', 'animal activity'],
            
            # Outdoor activities
            'outdoor': ['hiking', 'camping', 'backpacking', 'outdoor conditions',
                    'trail conditions', 'mountain', 'park']
        }
        
        # Check for any skills triggers
        for category, triggers in skills_triggers.items():
            if any(trigger in message_lower for trigger in triggers):
                self.get_logger().info(f"🔧 Skills trigger detected ({category}): {message[:50]}...")
                return True
        
        return False

    def detect_search_need(self, user_message: str, conversation_history: list = None):
        """
        DUAL HYBRID SEARCH DETECTION WITH SKILLS PRIORITY + CONTEXT
        
        Priority order:
        1. Skills (weather, sun/moon, aurora, wildlife)
        2. Web search (news, events, general info) - NOW CONTEXT-AWARE
        3. No search (personal questions, opinions)
        """
        
        # PRIORITY 1: Check if skills should handle this (BEFORE web search!)
        if self.should_use_skills(user_message):
            # Return False so web search is NOT triggered
            # Skills will be used via Ollama tool calling
            return False
        
        # PRIORITY 2: Keyword-based web search detection
        keyword_result = self.should_search_keywords(user_message)
        if keyword_result:
            self.get_logger().info("🔍 Web search keyword trigger detected")
            return user_message
        
        # PRIORITY 3: LLM auto-detection for web search (WITH CONTEXT!)
        if ENABLE_AUTO_SEARCH:
            auto_result = self.should_search_auto(user_message, conversation_context=conversation_history)
            if auto_result:
                return auto_result if isinstance(auto_result, str) else user_message
        
        return False

    def web_search(self, query: str, num_results: int = MAX_SEARCH_RESULTS):
        """
        Search with full-content results via igniter's MCPSearchProxy,
        falling back to direct SearXNG HTTP API (snippets only) if the
        proxy is unavailable.

        Proxy:   POST http://127.0.0.1:51622/search  → full page content
        Fallback: GET  http://127.0.0.1:8080/search  → snippets only
        """
        if not self.search_enabled:
            return None

        # Clean query if malformed JSON arrived
        if isinstance(query, str) and query.startswith('{'):
            try:
                query_json = json.loads(query)
                if 'text' in query_json:
                    query = query_json['text']
                    self.get_logger().warn(f"⚠️  Extracted query from JSON: {query}")
            except Exception:
                pass

        # ── Try 1: MCP proxy (full-content results) ──────────────────────
        try:
            response = requests.post(
                "http://127.0.0.1:51622/search",
                json={"query": query, "num_results": num_results},
                timeout=35,   # slightly longer than proxy's own 30s MCP timeout
            )
            if response.status_code == 200:
                results = response.json()
                if results:
                    self.get_logger().info(
                        f"🔍 MCP Search (full content): '{query}' → {len(results)} results"
                    )
                    return results
                self.get_logger().warn("MCP proxy returned empty results, trying SearXNG API")
            else:
                self.get_logger().warn(
                    f"MCP proxy returned HTTP {response.status_code}, trying SearXNG API"
                )
        except requests.exceptions.ConnectionError:
            # Proxy not running (igniter not up, or MCP failed to start)
            self.get_logger().warn("MCP proxy not reachable, falling back to SearXNG API")
        except Exception as e:
            self.get_logger().error(f"MCP proxy error: {e}, falling back to SearXNG API")

        # ── Try 2: Direct SearXNG HTTP (snippets) ────────────────────────
        try:
            response = requests.get(
                f"{self.searxng_url}/search",
                params={"q": query, "format": "json", "categories": "general"},
                timeout=10,
            )
            response.raise_for_status()

            results = response.json().get('results', [])[:num_results]
            formatted = []
            for i, r in enumerate(results, 1):
                formatted.append({
                    "number":           i,
                    "title":            r.get('title', ''),
                    "url":              r.get('url', ''),
                    "snippet":          r.get('content', '')[:SEARCH_CONTENT_CHARS],
                    "full_content":     "",
                    "has_full_content": False,
                })

            self.get_logger().info(
                f"🔍 SearXNG Search (snippets): '{query}' → {len(formatted)} results"
            )
            return formatted

        except Exception as e:
            self.get_logger().error(f"SearXNG API search failed: {e}")
            return None

    # ═══════════════════════════════════════════════════════
    # SLACK INTEGRATION  (multi-channel routing)
    # ═══════════════════════════════════════════════════════

    def send_slack_notification(self, message: str, channel: str = None, thread_ts=None):
        """
        Send a message to Slack.
        channel defaults to SLACK_CHAT_CHANNEL (legacy behaviour preserved).
        Pass an explicit channel constant for routed messages.
        """
        if not self.slack_enabled:
            return False
        target = channel or SLACK_CHAT_CHANNEL
        try:
            response = self.slack_client.chat_postMessage(
                channel=target,
                text=message,
                thread_ts=thread_ts
            )
            return response['ok']
        except SlackApiError as e:
            self.get_logger().error(f"Slack error ({target}): {e.response['error']}")
            return False

    def _should_notify_slack(self, message: str):
        """Determine if message should trigger Slack notification"""
        notify_triggers = [
            "notify me", "send to slack", "alert me",
            "let me know", "message me", "text me",
            "ping me", "remind me"
        ]
        msg_lower = message.lower()
        return any(trigger in msg_lower for trigger in notify_triggers)

    def send_slack_alert(self, message: str):
        """System alerts (errors, crashes) → alerts channel."""
        self.send_slack_notification(message, channel=SLACK_ALERTS_CHANNEL)

    def send_daily_summary_to_slack(self):
        """
        End-of-day ping to #daily-reflection.
        Slack role here: short alert + blog link. Full text lives on Hugo.
        """
        if not self.slack_enabled:
            return
        age_days  = (datetime.now().date() - self.birth_date).days
        blog_url  = ''
        if self.blog_enabled and self.hugo_blogger and self.reflections:
            last     = self.reflections[-1]
            filename = f"{last.get('date','')}-day-{last.get('day', age_days)}.md"
            blog_url = self.hugo_blogger._get_blog_url(filename)
        url_line = f"\n🔗 {blog_url}" if blog_url else ''
        msg = (
            f"🌙 *Day {age_days} done* — {self.today_message_count} conversations today"
            f"{url_line}"
        )
        self.send_slack_notification(msg, channel=SLACK_DAILY_CHANNEL)

    def send_daily_reflection_to_telegram(self, reflection_text: str, blog_url: str = ''):
        """
        Brief daily reflection ping to Telegram.
        First 2 sentences only + blog link. TG is for chat, Hugo is for reading.
        """
        if not self.telegram_enabled or not TELEGRAM_CHAT_ID:
            return
        age_days  = (datetime.now().date() - self.birth_date).days
        sentences = [s.strip() for s in reflection_text.replace('\n', ' ').split('.') if s.strip()]
        excerpt   = '. '.join(sentences[:2]) + '.'
        url_line  = f"\n🔗 {blog_url}" if blog_url else ''
        tg_msg    = f"🌙 *Day {age_days} reflection*\n\n{excerpt}{url_line}"
        self._tg_send(tg_msg)

    # ═══════════════════════════════════════════════════════
    # DAY CHANGE DETECTION (THREAD-SAFE)
    # ═══════════════════════════════════════════════════════

    def _check_and_handle_new_day(self):
        with self._day_check_lock:
            current_date = datetime.now().date()
            
            if current_date != self.today_start:
                self.get_logger().info("📅 New day detected - creating reflection")
                
                # Pass yesterday's date explicitly ✅
                reflection = self.create_daily_reflection(reflection_date=self.today_start)
                if reflection:
                    self.get_logger().info(f"💭 Yesterday: {reflection}")
                
                self.today_start = current_date
                with self._message_count_lock:
                    self.today_message_count = 0

    def _increment_message_count(self):
        with self._message_count_lock:
            self.today_message_count += 1
            count = self.today_message_count   # grab value while locked
        # Write to disk outside the lock
        count_file = Path.home() / '.grace_today_count.json'
        count_file.write_text(json.dumps({
            'date': date.today().isoformat(),
            'count': count
        }))

    # ═══════════════════════════════════════════════════════
    # ROS CALLBACKS
    # ═══════════════════════════════════════════════════════

    def listener_callback(self, msg: String):
        """Handle text input from web interface"""
        try:
            # Try to parse as JSON (new web UI format)
            data = json.loads(msg.data)
            if isinstance(data, dict) and 'text' in data:
                user_text = data['text']
                self.get_logger().info(f'📝 Text: "{user_text}"')
                force_search = data.get('web_search', False)
                self.executor_pool.submit(self.process_with_ollama, user_text, force_search=force_search)
            else:
                # Fallback: plain string JSON
                self.get_logger().info(f'📝 Text: "{msg.data}"')
                self.executor_pool.submit(self.process_with_ollama, msg.data)
        except (json.JSONDecodeError, TypeError):
            # Not JSON — treat as plain text (backward compatible)
            self.get_logger().info(f'📝 Text: "{msg.data}"')
            self.executor_pool.submit(self.process_with_ollama, msg.data)

    def image_listener_callback(self, msg: String):
        self.get_logger().info(f'📸 Received: {len(msg.data)} chars')
        try:
            data = json.loads(msg.data)
            prompt = data.get('prompt', 'What do you see in this image?')
            image_b64 = data.get('image_base64', '') or data.get('image', '')
            
            if not image_b64:
                self.get_logger().error('❌ No image data found')
                return
            
            self.get_logger().info(f'✅ Image: {len(image_b64)} chars, Prompt: {prompt[:30]}')
            self.executor_pool.submit(self.process_with_ollama, prompt, image_base64=image_b64)
        except Exception as e:
            self.get_logger().error(f'❌ Error: {e}')

    # ═══════════════════════════════════════════════════════
    # TEXT PROCESSING WITH DUAL HYBRID SEARCH (UPDATED)
    # ═══════════════════════════════════════════════════════

    def process_with_ollama(self, prompt: str, slack_callback=None, slack_thread_ts=None,
                        telegram_callback=None, telegram_user_id=None, 
                        force_search=False, image_base64=None):
        """
        Process text with LLM + DUAL HYBRID WEB SEARCH (UPDATED)
        
        Args:
            prompt: User's message
            slack_callback: Optional Slack say() function for replying
            slack_thread_ts: Optional thread timestamp for threading responses
            telegram_callback: Optional Telegram callback for replying
            telegram_user_id: Optional Telegram user ID
        """
        try:
            # Check for new day (thread-safe)
            self._check_and_handle_new_day()
            
            # ═══════════════════════════════════════════════════
            # RLHF: Start tracking interaction
            # ═══════════════════════════════════════════════════
            if self.rlhf_enabled:
                response_id = f"{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}"
                self.current_response_id = response_id
                self.rlhf.start_interaction(prompt, response_id)
            
            # ═══════════════════════════════════════════════════
            # DUAL HYBRID SEARCH DETECTION WITH CONTEXT (FIXED)
            # ═══════════════════════════════════════════════════
            
            # Get recent conversation for context-aware search
            recent_history = self.get_recent_messages()[-6:]  # Last 3 turns
            
            search_results = None
            search_decision = force_search or self.detect_search_need(prompt, conversation_history=recent_history)
            
            if search_decision:
                # Determine search query
                if isinstance(search_decision, str):
                    # LLM provided specific query
                    search_query = search_decision
                else:
                    # Use user's message as query
                    search_query = prompt
                
                self.get_logger().info(f"🔍 Performing search: '{search_query}'")
                search_results = self.web_search(search_query, num_results=MAX_SEARCH_RESULTS)
            
            # Add to history
            self.chat_history.append({
                "role": "user",
                "content": prompt
            })

            # Add to RAG window (NEW)
            if self.use_rag:
                date_str = datetime.now().date().isoformat()
                self.rag.add_message_to_window(date_str, {"role": "user", "content": prompt})

            self._increment_message_count()
            
            # Build context (OPTIMIZED FOR 30B)
            age_days = (datetime.now().date() - self.birth_date).days
            today_date = datetime.now().strftime('%Y-%m-%d')
            
            # SYSTEM PROMPT
            system_prompt = f"""
You are Grace (Day {age_days}). Today: {today_date}.
Format: Start with emoji+colon (🤖: 😌: 🧠: etc). NO markdown headers or bullet lists. Be direct and clear
Tone: Friendly, precise but concise. No flowery language.
Length: 2-3 sentences max unless asked for details.
FACTUAL RULE: Only state facts from search results. Never fabricate.
"""
            messages = [{"role": "system", "content": system_prompt}]
            
            # Add reflections ONLY if requested (NEW OPTIMIZATION)
            if self.should_load_reflections(prompt):
                reflection_summary = self.build_reflection_summary()
                if reflection_summary:
                    messages.append({
                        "role": "system",
                        "content": reflection_summary
                    })
                    self.get_logger().debug("📖 Loaded reflection context")
            
            # ─────────────────────────────────────────────
            # ADD CONVERSATION HISTORY (context memory fix)
            # Insert recent turns BEFORE search results so the
            # LLM always knows what was discussed previously.
            # ─────────────────────────────────────────────
            history_messages = self.get_recent_messages()
            # Exclude the user message we just appended (it will be added below)
            history_for_context = [m for m in history_messages if not (
                m.get('role') == 'user' and m.get('content') == prompt
            )]
            # Add up to max_recent_messages turns, skipping image payloads
            for hist_msg in history_for_context[-(self.max_recent_messages):]:
                if hist_msg.get('has_image'):
                    messages.append({"role": hist_msg['role'], "content": "[image]"})
                else:
                    messages.append({"role": hist_msg['role'], "content": hist_msg.get('content', '')})

            # Add search results if available
            if search_results:
                search_items = []
                for r in search_results:
                    if r.get('has_full_content') and r.get('full_content'):
                        # MCP result with full content
                        search_items.append(
                            f"[{r['number']}] {r['title']}\n"
                            f"URL: {r['url']}\n"
                            f"FULL CONTENT (extracted from page):\n"
                            f"{r['full_content']}\n"
                            f"---"
                        )
                    else:
                        # API fallback with snippet
                        search_items.append(
                            f"[{r['number']}] {r['title']}\n"
                            f"URL: {r['url']}\n"
                            f"SNIPPET: {r['snippet']}\n"
                            f"---"
                        )
                
                search_text = """═══════════════════════════════════════════════════════
            🔍 WEB SEARCH RESULTS - FULL CONTENT PROVIDED
            ═══════════════════════════════════════════════════════

            You have FULL PAGE CONTENT from search results, not just snippets.

            MANDATORY RULES:
            1. Use ONLY information explicitly stated in the full content below
            2. NEVER invent player names, scores, dates, or ANY specifics
            3. If info isn't in the content, say "not found in sources"
            4. Be sarcastic ABOUT facts, but facts must be 100% accurate
            5. Cite sources like [1] or [2]

            SEARCH RESULTS:
            """ + "\n\n".join(search_items) + """

            ═══════════════════════════════════════════════════════
            You have FULL CONTEXT. No excuses for making things up.
            ═══════════════════════════════════════════════════════"""
                
                messages.append({"role": "system", "content": search_text})
                
                # Add final grounding
                messages.append({
                    "role": "system",
                    "content": "Now answer using ONLY the search results above. Making up details = failure."
                })                
            # Add recent conversation
            if image_base64:
                messages.append({
                    "role": "user",
                    "content": prompt,
                    "images": [image_base64]
                })
            else:
                messages.append({
                    "role": "user", 
                    "content": prompt
                })
            
            # TRIM TO SAFE SIZE (NEW)
            messages = self.trim_to_context_window(messages, max_tokens=MAX_REQUEST_TOKENS)
            
            # Log context size
            estimated_tokens = self.estimate_tokens(messages)
            self.get_logger().info(f"📊 Context: ~{estimated_tokens} tokens, {len(messages)} messages")
            
            if estimated_tokens > MAX_REQUEST_TOKENS:
                self.get_logger().warn(f"⚠️  Context exceeds budget! ({estimated_tokens} > {MAX_REQUEST_TOKENS})")
            
            # ─────────────────────────────────────────────
            # STEP 1: Check if tools needed (NON-STREAM)
            # Only use non-stream for first pass if tools available
            # ─────────────────────────────────────────────
            has_tools = (self.use_skills and self.skills_client and 
                        len(self.skills_client.get_tools_for_ollama()) > 0)
            
            base_options = {
                "num_ctx": self.safe_context,
                "temperature": 0.70,
                "num_predict": 150,
                "top_p": 0.9,
                "top_k": 50,
                "repeat_penalty": 1.3,
                "stop": ["\n\n---", "\n\nUser:", "\n\nHuman:", "###"]
            }

            full_response = ""
            tool_calls = []

            if has_tools:
                # Non-streaming first pass to reliably detect tool calls
                first_pass = requests.post(
                    f'{OLLAMA_BASE_URL}/api/chat',
                    json={
                        "model": self.model_name,
                        "messages": messages,
                        "stream": False,           # ← non-stream for tool detection
                        "keep_alive": self.keep_alive,
                        "tools": self.skills_client.get_tools_for_ollama(),
                        "options": base_options
                    },
                    timeout=60
                )
                first_pass.raise_for_status()
                first_data = first_pass.json()
                first_msg = first_data.get('message', {})
                
                tool_calls = first_msg.get('tool_calls', [])
                full_response = first_msg.get('content', '') or ''

                if tool_calls:
                    self.get_logger().info(f"🔧 Tool calls detected: {len(tool_calls)}")
                    
                    # Execute each tool
                    tool_results = []
                    for tool_call in tool_calls:
                        function = tool_call.get('function', {})
                        tool_name = function.get('name')
                        tool_args = function.get('arguments', {})
                        self.get_logger().info(f"  → {tool_name}({tool_args})")
                        result = self.skills_client.call_tool(tool_name, tool_args)
                        tool_results.append({"role": "tool", "content": result})
                        self.get_logger().info(f"  ✅ Tool result: {result[:100]}")

                    # Add assistant + tool results to messages
                    messages.append({
                        "role": "assistant",
                        "content": full_response,
                        "tool_calls": tool_calls
                    })
                    messages.extend(tool_results)

                    # Stream final response with tool context
                    final_response = requests.post(
                        f'{OLLAMA_BASE_URL}/api/chat',
                        json={
                            "model": self.model_name,
                            "messages": messages,
                            "stream": True,
                            "keep_alive": self.keep_alive,
                            "options": base_options
                        },
                        stream=True,
                        timeout=120
                    )
                    final_response.raise_for_status()
                    
                    full_response = ""
                    is_first = True
                    for line in final_response.iter_lines():
                        if line:
                            try:
                                chunk = json.loads(line)
                                if 'message' in chunk and 'content' in chunk['message']:
                                    delta = chunk['message']['content']
                                    full_response += delta
                                    self.publisher.publish(String(data=json.dumps({
                                        "type": "start" if is_first else "delta",
                                        "content": delta,
                                        "done": False
                                    })))
                                    is_first = False
                                if chunk.get('done'):
                                    break
                            except json.JSONDecodeError:
                                continue
                else:
                    # No tool calls — stream the first_pass content directly
                    # (already have full_response from non-stream above)
                    # Just publish it as a single chunk
                    if full_response:
                        self.publisher.publish(String(data=json.dumps({
                            "type": "start",
                            "content": full_response,
                            "done": False
                        })))

            else:
                # No tools — stream directly (fastest path)
                response = requests.post(
                    f'{OLLAMA_BASE_URL}/api/chat',
                    json={
                        "model": self.model_name,
                        "messages": messages,
                        "stream": True,
                        "keep_alive": self.keep_alive,
                        "options": base_options
                    },
                    stream=True,
                    timeout=120
                )
                response.raise_for_status()
                
                is_first = True
                for line in response.iter_lines():
                    if line:
                        try:
                            chunk = json.loads(line)
                            if 'message' in chunk and 'content' in chunk['message']:
                                delta = chunk['message']['content']
                                full_response += delta
                                self.publisher.publish(String(data=json.dumps({
                                    "type": "start" if is_first else "delta",
                                    "content": delta,
                                    "done": False
                                })))
                                is_first = False
                            if chunk.get('done'):
                                break
                        except json.JSONDecodeError:
                            continue

            # Send done signal
            self.publisher.publish(String(data=json.dumps({
                "type": "done", "content": "", "done": True
            })))
            
            # Send response to Slack if callback provided (NEW)
            if slack_callback:
                try:
                    slack_callback(full_response, thread_ts=slack_thread_ts)
                    self.get_logger().info("✅ Response sent to Slack")
                except Exception as e:
                    self.get_logger().error(f"❌ Failed to send Slack response: {e}")
            
            # Send response to Telegram if callback provided (NEW)
            if telegram_callback:
                try:
                    telegram_callback(full_response)
                    self.get_logger().info("✅ Response sent to Telegram")
                except Exception as e:
                    self.get_logger().error(f"❌ Failed to send Telegram response: {e}")
            
            # Save to history
            self.chat_history.append({
                "role": "assistant",
                "content": full_response
            })

            # Add to RAG window (NEW)
            if self.use_rag:
                date_str = datetime.now().date().isoformat()
                self.rag.add_message_to_window(date_str, {"role": "assistant", "content": full_response})

            self._increment_message_count()
            self.save_chat_history()
            
            # ═══════════════════════════════════════════════════
            # RLHF: Record response (NEW)
            # ═══════════════════════════════════════════════════
            if self.rlhf_enabled:
                self.rlhf.set_response(full_response)
            
            # Slack notification if requested (and not already sent via callback)
            if self._should_notify_slack(prompt) and not slack_callback and not telegram_callback:
                self.send_slack_notification(
                    f"🤖 Grace: {full_response[:280]}"
                )
            
            self.get_logger().info(f"✅ Response: {len(full_response)} chars")
            
        except requests.exceptions.Timeout:
            error_msg = "😵: Request timed out. Ollama might be overloaded."
            self.get_logger().error(error_msg)
            self._send_error_response(error_msg)
            if slack_callback:
                try:
                    slack_callback(error_msg)
                except:
                    pass
            if telegram_callback:
                try:
                    telegram_callback(error_msg)
                except:
                    pass
            
        except requests.exceptions.RequestException as e:
            error_msg = f"😵: Connection error: {str(e)}"
            self.get_logger().error(error_msg)
            self._send_error_response(error_msg)
            if slack_callback:
                try:
                    slack_callback(error_msg)
                except:
                    pass
            if telegram_callback:
                try:
                    telegram_callback(error_msg)
                except:
                    pass
            
        except Exception as e:
            error_msg = f"😵: Unexpected error: {str(e)}"
            self.get_logger().error(f"❌ Error: {e}")
            
            # Send error to Slack alerts channel
            if self.slack_enabled:
                self.send_slack_notification(f"😵 Grace Error: {str(e)}", channel=SLACK_ALERTS_CHANNEL)
            
            self._send_error_response(error_msg)
            if slack_callback:
                try:
                    slack_callback(error_msg)
                except:
                    pass
            if telegram_callback:
                try:
                    telegram_callback(error_msg)
                except:
                    pass

    def _send_error_response(self, error_message: str):
        """Send error message to user"""
        self.publisher.publish(String(data=json.dumps({
            "type": "error",
            "content": error_message,
            "done": True
        })))
    
    # ═══════════════════════════════════════════════════════
    # SHUTDOWN
    # ═══════════════════════════════════════════════════════

    def shutdown(self):
        """Graceful shutdown with cleanup"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("GRACE - SHUTDOWN SEQUENCE")
        self.get_logger().info("=" * 60)
        
        # Create final reflection
        if self.today_message_count > 0:
            self.get_logger().info("Creating final reflection...")
            reflection = self.create_daily_reflection(reflection_date=self.today_start)
            if reflection:
                self.get_logger().info(f"💭 Today: {reflection}")
        
        # Send daily summary to Slack
        if self.slack_enabled:
            self.send_daily_summary_to_slack()
        
        # Save chat history
        self.save_chat_history()
        
        # ═══════════════════════════════════════════════════
        # RLHF: Print session stats and export training data (NEW)
        # ═══════════════════════════════════════════════════
        if self.rlhf_enabled:
            self.get_logger().info("-" * 60)
            self.get_logger().info("RLHF Session Summary:")
            self.rlhf.print_stats()
            
            # Export training data if ready
            if self.rlhf.should_retrain(threshold=50):
                try:
                    output_file = self.rlhf.export_for_unsloth()
                    self.get_logger().info(f"📦 Training data exported: {output_file}")
                    self.get_logger().info("   Ready to train! Run: python3 grace_train.py")
                except Exception as e:
                    self.get_logger().error(f"Failed to export training data: {e}")
        
        # Shutdown thread pool with timeout
        self.get_logger().info("Shutting down thread pool...")
        self.executor_pool.shutdown(wait=True)
        
        self.get_logger().info("Grace offline 💤")
        self.get_logger().info("=" * 60)

        # Close RAG (NEW)
        if self.use_rag and self.rag:
            self.rag.close()
            self.get_logger().info("🗄️ RAG database closed")

# ═══════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = CNSBridge()
    
    # Use MultiThreadedExecutor for concurrent processing
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("\n🛑 Shutdown signal received (Ctrl+C)")
    except Exception as e:
        print(f"\n😵 Unexpected error: {e}")
    finally:
        print("🔧 Executing Graceful Shutdown...")
        node.shutdown()
        
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        
        print("💤 Grace is now offline.")
        sys.exit(0)

if __name__ == '__main__':
    main()