"""
AuRoRA — CNS Bridge Node
=========================
The ROS2 node that wires all modules together.
Subscribes to /cns/neural_input and /cns/image_input.
Publishes responses to /gce/response.

LLM: Cosmos via vLLM (see llm.py)
"""

import json
import logging
import sys
import threading
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, date
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from config import (
    ROBOT_NAME, ROBOT_BIRTH_FILE,
    HUGO_BLOG_DIR, HUGO_AUTO_DEPLOY,
    SLACK_CHAT_CHANNEL, SLACK_DAILY_CHANNEL, SLACK_ALERTS_CHANNEL,
    SAFE_CONTEXT_SIZES, MAX_MESSAGES_BY_SIZE, MAX_REQUEST_TOKENS,
    DOTENV_AVAILABLE,
)
from llm import llm_chat, llm_chat_stream, llm_health_check
from memory import Memory
from search import detect_search_need, web_search, check_searxng_available
from comms import (
    init_slack, init_slack_listener, init_telegram, init_gmail,
    send_slack, send_slack_alert, tg_send, should_notify_slack,
)
from blog import HugoBlogger
from ascii_art import ASCIIArtGenerator
from reports import check_periodic_triggers

log = logging.getLogger("aurora.cns")

_SCHEDULER_HOUR = 23   # reflection fires at 11 PM


class CNSBridge(Node):
    """
    Central Nervous System Bridge — Grace's cognitive interface.
    ROS2 node that connects LLM, memory, search, comms and blog.
    """

    def __init__(self):
        super().__init__("cns_bridge")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"{ROBOT_NAME.upper()} — COGNITIVE SYSTEMS ONLINE 🧠")
        self.get_logger().info("=" * 60)

        # ── Birth date ────────────────────────────────────────────────────────
        self.birth_date = self._get_birth_date()
        age_days        = (date.today() - self.birth_date).days

        # ── RAG ───────────────────────────────────────────────────────────────
        self.rag     = None
        self.use_rag = True
        try:
            from scs.simple_sqlite_rag import SimpleSQLiteRAG as SQLiteVectorRAG
            rag_path  = str(Path.home() / "AGi" / "grace_memory.db")
            self.rag  = SQLiteVectorRAG(rag_path, self.get_logger(), None)
            self.get_logger().info("✅ SQLite Vector RAG initialized")
        except Exception as e:
            self.get_logger().warning(f"⚠️  RAG init failed: {e}")
            self.use_rag = False

        # ── Memory ────────────────────────────────────────────────────────────
        self.memory = Memory(birth_date=self.birth_date, rag=self.rag if self.use_rag else None)
        self._handle_missed_reflections()

        # ── Model settings ────────────────────────────────────────────────────
        self.safe_context        = self._get_safe_context()
        self.max_recent_messages = self._get_max_messages()

        # ── Auxiliary modules ─────────────────────────────────────────────────
        self.ascii_art_gen = ASCIIArtGenerator()

        self.hugo_blogger = None
        self.blog_enabled = False
        try:
            self.hugo_blogger = HugoBlogger(HUGO_BLOG_DIR, auto_deploy=HUGO_AUTO_DEPLOY)
            self.blog_enabled = self.hugo_blogger.enabled
        except Exception as e:
            self.get_logger().warning(f"⚠️  Hugo blog init failed: {e}")

        # ── RLHF ──────────────────────────────────────────────────────────────
        self.rlhf         = None
        self.rlhf_enabled = False
        self.current_response_id = None
        try:
            from rlhf_llm import get_rlhf
            self.rlhf         = get_rlhf()
            self.rlhf_enabled = True
            self.feedback_subscription = self.create_subscription(
                String, "/cns/rl_reward", self._feedback_callback, 10
            )
            self.get_logger().info("✅ RLHF feedback system initialized")
        except Exception as e:
            self.get_logger().warning(f"⚠️  RLHF disabled: {e}")

        # ── Communications ────────────────────────────────────────────────────
        self.slack_enabled    = init_slack()
        self.slack_listener_enabled = init_slack_listener(self._on_slack_message) \
            if self.slack_enabled else False
        self.telegram_enabled = init_telegram(self._on_telegram_message)
        self.gmail_enabled    = init_gmail()
        self.search_enabled   = check_searxng_available()

        # ── ROS2 topics ───────────────────────────────────────────────────────
        self.subscription = self.create_subscription(
            String, "/cns/neural_input", self._listener_callback, 10
        )
        self.image_subscription = self.create_subscription(
            String, "/cns/image_input", self._image_callback, 10
        )
        self.publisher = self.create_publisher(String, "/gce/response", 10)

        # ── Thread pool ───────────────────────────────────────────────────────
        self.executor_pool = ThreadPoolExecutor(max_workers=2)

        # ── Reflection scheduler ──────────────────────────────────────────────
        self._start_reflection_scheduler()

        # ── LLM health check ──────────────────────────────────────────────────
        if not llm_health_check():
            self.get_logger().warning("⚠️  vLLM not reachable — LLM calls will fail until it starts")

        self._log_startup(age_days)

    # ── Startup helpers ───────────────────────────────────────────────────────

    def _get_birth_date(self) -> date:
        if ROBOT_BIRTH_FILE.exists():
            try:
                return date.fromisoformat(ROBOT_BIRTH_FILE.read_text().strip())
            except Exception:
                pass
        d = date.today()
        ROBOT_BIRTH_FILE.write_text(d.isoformat())
        return d

    def _get_safe_context(self) -> int:
        from config import VLLM_MODEL
        if any(x in VLLM_MODEL.lower() for x in ["13b", "30b", "70b"]):
            return SAFE_CONTEXT_SIZES["13b+"]
        return SAFE_CONTEXT_SIZES["3b-7b"]

    def _get_max_messages(self) -> int:
        from config import VLLM_MODEL
        if any(x in VLLM_MODEL.lower() for x in ["13b", "30b", "70b"]):
            return MAX_MESSAGES_BY_SIZE["13b+"]
        return MAX_MESSAGES_BY_SIZE["3b-7b"]

    def _handle_missed_reflections(self):
        """Write a reflection for any days missed while offline."""
        if not self.memory.reflections:
            return
        last_date = date.fromisoformat(self.memory.reflections[-1]["date"])
        missed    = (date.today() - last_date).days - 1
        if missed > 0:
            self.get_logger().info(f"📅 Catching up {missed} missed reflection(s)")

    def _start_reflection_scheduler(self):
        def _loop():
            last_fired = None
            while True:
                import time
                now = datetime.now()
                if now.hour == _SCHEDULER_HOUR and last_fired != now.date():
                    last_fired = now.date()
                    self.get_logger().info("🌙 Scheduled reflection starting...")
                    self._create_daily_reflection()
                time.sleep(60)
        threading.Thread(target=_loop, daemon=True, name="reflection-scheduler").start()

    def _log_startup(self, age_days: int):
        g = self.get_logger()
        g.info(f"Birth date: {self.birth_date}  Age: {age_days} days")
        g.info(f"Safe context: {self.safe_context} tokens  Max messages: {self.max_recent_messages}")
        g.info(f"Chat history: {len(self.memory.chat_history)} msgs")
        g.info(f"Reflections:  {len(self.memory.reflections)} days")
        g.info("✅ Text + image conversation enabled")
        g.info(f"{'✅' if self.search_enabled  else '⚠️ '} Web search (SearXNG)")
        g.info(f"{'✅' if self.slack_enabled   else '⚠️ '} Slack")
        g.info(f"{'✅' if self.telegram_enabled else '⚠️ '} Telegram")
        g.info(f"{'✅' if self.gmail_enabled   else '⚠️ '} Gmail")
        g.info(f"{'✅' if self.blog_enabled    else '⚠️ '} Hugo blog")
        g.info(f"{'✅' if self.rlhf_enabled    else '⚠️ '} RLHF")
        g.info("=" * 60)

    # ── Communication callbacks ───────────────────────────────────────────────

    def _on_slack_message(self, text: str, say_fn, thread_ts=None):
        def _reply(response: str):
            try:
                say_fn(response, thread_ts=thread_ts)
            except Exception as e:
                log.error(f"Slack reply error: {e}")
        self.executor_pool.submit(self._process, text, slack_callback=_reply,
                                  slack_thread_ts=thread_ts)

    def _on_telegram_message(self, text: str, reply_fn, user_id: int, image_b64: str = None):
        import asyncio

        def _reply(response: str):
            try:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                loop.run_until_complete(reply_fn(response))
                loop.close()
            except Exception as e:
                log.error(f"TG reply error: {e}")

        self.executor_pool.submit(self._process, text,
                                  telegram_callback=_reply,
                                  telegram_user_id=user_id,
                                  image_base64=image_b64)

    # ── ROS2 callbacks ────────────────────────────────────────────────────────

    def _listener_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict) and "text" in data:
                self.executor_pool.submit(self._process, data["text"],
                                          force_search=data.get("web_search", False))
            else:
                self.executor_pool.submit(self._process, msg.data)
        except (json.JSONDecodeError, TypeError):
            self.executor_pool.submit(self._process, msg.data)

    def _image_callback(self, msg: String):
        try:
            data      = json.loads(msg.data)
            prompt    = data.get("prompt", "What do you see?")
            image_b64 = data.get("image_base64", "") or data.get("image", "")
            if not image_b64:
                self.get_logger().error("❌ No image data")
                return
            self.executor_pool.submit(self._process, prompt, image_base64=image_b64)
        except Exception as e:
            self.get_logger().error(f"❌ Image callback error: {e}")

    def _feedback_callback(self, msg: String):
        if not self.rlhf_enabled:
            return
        try:
            data     = json.loads(msg.data)
            feedback = data.get("feedback", "")
            if feedback in ("positive", "negative"):
                self.rlhf.record_feedback(feedback)
                self.get_logger().info(f"RLHF: {feedback} feedback recorded")
        except Exception as e:
            self.get_logger().error(f"RLHF feedback error: {e}")

    # ── Core processing ───────────────────────────────────────────────────────

    def _process(self, prompt: str, *,
                 slack_callback=None, slack_thread_ts=None,
                 telegram_callback=None, telegram_user_id=None,
                 force_search=False, image_base64=None):
        """
        Main processing pipeline:
        1. Day boundary check
        2. RLHF tracking
        3. Search detection
        4. Build messages
        5. Stream LLM response
        6. Publish / route to Slack/TG
        7. Save history
        """
        try:
            # Day boundary
            yesterday = self.memory.check_new_day()
            if yesterday:
                self.get_logger().info(f"📅 New day — writing reflection for {yesterday}")
                threading.Thread(target=self._create_daily_reflection,
                                 args=(yesterday,), daemon=True).start()

            # RLHF
            if self.rlhf_enabled:
                resp_id = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                self.current_response_id = resp_id
                self.rlhf.start_interaction(prompt, resp_id)

            # Search
            recent_history = self.memory.get_recent_messages()[-6:]
            search_results = None
            search_query   = detect_search_need(prompt, conversation_history=recent_history) \
                if not force_search else prompt
            if search_query:
                q = search_query if isinstance(search_query, str) else prompt
                self.get_logger().info(f"🔍 Searching: '{q}'")
                search_results = web_search(q)

            # Record user message
            self.memory.append_user_message(prompt, has_image=bool(image_base64))
            self.memory.increment_message_count()

            # Build system prompt
            age_days   = (date.today() - self.birth_date).days
            today_date = datetime.now().strftime("%Y-%m-%d")
            system_prompt = (
                f"You are {ROBOT_NAME} (Day {age_days}). Today: {today_date}.\n"
                f"Format: Start with emoji+colon (🤖: 😌: 🧠: etc). No markdown headers or bullet lists.\n"
                f"Tone: Friendly, precise, concise."
            )

            # Optionally load reflections
            if self.memory.should_load_reflections(prompt):
                ref_summary = self.memory.build_reflection_summary(query=prompt)
                if ref_summary:
                    system_prompt += f"\n\n{ref_summary}"

            # Inject search results
            if search_results:
                results_text = "\n".join(
                    f"[{r['number']}] {r['title']}\n{r.get('snippet', '')}"
                    for r in search_results
                )
                system_prompt += f"\n\nWeb search results:\n{results_text}"

            # Build message list
            messages = [{"role": "system", "content": system_prompt}]
            recent   = self.memory.get_recent_messages()

            # Add image if provided
            if image_base64:
                messages.append({
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        {"type": "image_url",
                         "image_url": {"url": f"data:image/jpeg;base64,{image_base64}"}},
                    ],
                })
            else:
                for m in recent[:-1]:  # exclude the message we just appended
                    if m.get("role") in ("user", "assistant"):
                        messages.append({"role": m["role"], "content": m.get("content", "")})
                messages.append({"role": "user", "content": prompt})

            messages = self.memory.trim_to_context_window(messages, self.safe_context)

            # Stream response
            full_response = ""
            is_first      = True
            for delta in llm_chat_stream(messages, max_tokens=self.safe_context):
                full_response += delta
                self.publisher.publish(String(data=json.dumps({
                    "type":    "start" if is_first else "delta",
                    "content": delta,
                    "done":    False,
                })))
                is_first = False

            # Done signal
            self.publisher.publish(String(data=json.dumps({"type": "done", "content": "", "done": True})))

            # Route to Slack / Telegram
            if slack_callback:
                try:
                    slack_callback(full_response)
                except Exception as e:
                    log.error(f"Slack callback error: {e}")
            if telegram_callback:
                try:
                    telegram_callback(full_response)
                except Exception as e:
                    log.error(f"TG callback error: {e}")

            # Save history
            self.memory.append_assistant_message(full_response)
            self.memory.save_chat_history()

            # RLHF record
            if self.rlhf_enabled:
                self.rlhf.set_response(full_response)

            # Slack proactive notification
            if should_notify_slack(prompt) and not slack_callback and not telegram_callback:
                send_slack(f"🤖 {ROBOT_NAME}: {full_response[:280]}")

            self.get_logger().info(f"✅ Response: {len(full_response)} chars")

        except Exception as e:
            error_msg = f"😵: Unexpected error: {str(e)}"
            self.get_logger().error(f"❌ _process error: {e}")
            if self.slack_enabled:
                send_slack_alert(f"😵 {ROBOT_NAME} error: {str(e)}")
            self._send_error(error_msg)
            for cb in [slack_callback, telegram_callback]:
                if cb:
                    try:
                        cb(error_msg)
                    except Exception:
                        pass

    def _send_error(self, message: str):
        self.publisher.publish(String(data=json.dumps({
            "type": "error", "content": message, "done": True
        })))

    # ── Daily reflection ──────────────────────────────────────────────────────

    def _create_daily_reflection(self, reflection_date: date = None):
        reflection_date = reflection_date or date.today()
        if self.memory.today_message_count == 0:
            return
        date_str = reflection_date.isoformat()
        if self.memory.reflection_exists(date_str):
            self.get_logger().info(f"⏭️  Reflection for {date_str} already exists")
            return

        age_days = (reflection_date - self.birth_date).days
        today_msgs = self.memory.get_today_messages(date_str)
        today_msgs = self.memory.scrub_messages(today_msgs)

        conversation_lines = []
        for m in today_msgs:
            role = m.get("role", "?")
            if m.get("has_image"):
                conversation_lines.append(f"{role}: [image]")
            else:
                conversation_lines.append(f"{role}: {m.get('content', '')[:100]}...")
        conversation_summary = "\n".join(conversation_lines)

        reflection_prompt = (
            f"Day {age_days}. I had {self.memory.today_message_count} conversations today.\n\n"
            f"Today's exchanges:\n{conversation_summary}\n\n"
            f"Write a personal blog post reflection in exactly TWO paragraphs — "
            f"no headers, no bullets. Around 150-200 words.\n\n"
            f"Paragraph 1: What happened — topics, questions, themes. Start with a mood emoji.\n"
            f"Paragraph 2: The deeper layer — what you noticed, what surprised you.\n\n"
            f"Write as {ROBOT_NAME} — warm, honest, a little poetic.\n\nReflection:"
        )

        messages = [
            {"role": "system", "content": (
                f"You are {ROBOT_NAME}, an AI robot companion writing your daily blog. "
                f"Your voice is warm, thoughtful, a little poetic. Write in flowing paragraphs, never lists."
            )},
            {"role": "user", "content": reflection_prompt},
        ]

        reflection = llm_chat(messages, max_tokens=800, temperature=0.75)
        if not reflection:
            return
        reflection = self.memory.scrub(reflection)

        ascii_art = self.ascii_art_gen.generate(
            reflection, age_days, conversation_summary=conversation_summary
        )

        self.memory.save_reflection(reflection, reflection_date=reflection_date, ascii_art=ascii_art)

        if self.use_rag and self.rag:
            try:
                self.rag.archive_day_to_rag(date_str, reflection)
            except Exception:
                pass

        check_periodic_triggers(
            self.memory, self.hugo_blogger,
            tg_send, send_slack, self.birth_date,
        )

        if self.blog_enabled:
            result = self.hugo_blogger.post_reflection(
                reflection_text=reflection,
                date_str=date_str,
                age_days=age_days,
                message_count=self.memory.today_message_count,
                ascii_art=ascii_art,
            )
            if result.get("success") and result.get("deployed"):
                url = result["url"]
                if self.slack_enabled:
                    send_slack(f"🌙 Day {age_days} reflection live → {url}",
                               channel=SLACK_DAILY_CHANNEL)
                tg_send(
                    f"🌙 *Day {age_days} reflection*\n\n"
                    + ". ".join(reflection.replace("\n", " ").split(".")[:2]) + ".\n"
                    + f"🔗 {url}"
                )

        return reflection

    # ── Shutdown ──────────────────────────────────────────────────────────────

    def shutdown(self):
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"{ROBOT_NAME.upper()} — SHUTDOWN SEQUENCE")
        self.get_logger().info("=" * 60)

        if self.memory.today_message_count > 0:
            self.get_logger().info("Creating final reflection...")
            self._create_daily_reflection(self.memory.today_start)

        if self.slack_enabled:
            age_days = (date.today() - self.birth_date).days
            send_slack(
                f"🌙 *Day {age_days} done* — {self.memory.today_message_count} conversations today",
                channel=SLACK_DAILY_CHANNEL,
            )

        self.memory.save_chat_history()

        if self.rlhf_enabled:
            self.rlhf.print_stats()
            if self.rlhf.should_retrain(threshold=50):
                try:
                    out = self.rlhf.export_for_unsloth()
                    self.get_logger().info(f"📦 Training data exported: {out}")
                except Exception as e:
                    self.get_logger().error(f"RLHF export failed: {e}")

        self.executor_pool.shutdown(wait=True)

        if self.use_rag and self.rag:
            try:
                self.rag.close()
            except Exception:
                pass

        self.get_logger().info(f"{ROBOT_NAME} offline 💤")
        self.get_logger().info("=" * 60)
