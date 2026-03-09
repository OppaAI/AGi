"""
AuRoRA — Communications
========================
Slack (one-way notifications + two-way Socket Mode listener)
Telegram (two-way bot — text + photos)
Gmail (polling monitor → TG/Slack alerts)

Each section has an init_*() function that returns True/False.
The CNSNode holds the enabled flags and calls notify_* helpers.
"""

import asyncio
import imaplib
import json
import logging
import re
import threading
import time
import email as email_lib
from email.header import decode_header as _decode_header
from pathlib import Path
from typing import Callable, Optional

import requests

from config import (
    SLACK_BOT_TOKEN, SLACK_APP_TOKEN,
    SLACK_CHAT_CHANNEL, SLACK_DAILY_CHANNEL,
    SLACK_PROGRESS_CHANNEL, SLACK_ALERTS_CHANNEL,
    TELEGRAM_BOT_TOKEN, TELEGRAM_CHAT_ID,
    GMAIL_ADDRESS, GMAIL_APP_PASS, GMAIL_POLL_SECS,
    VLLM_BASE_URL,
)

log = logging.getLogger("aurora.comms")

# ═══════════════════════════════════════════════════════════════════════════════
# SLACK
# ═══════════════════════════════════════════════════════════════════════════════

_slack_client = None
_slack_app    = None


def init_slack() -> bool:
    global _slack_client
    if not SLACK_BOT_TOKEN:
        log.warning("⚠️  Slack disabled — no SLACK_BOT_TOKEN")
        return False
    try:
        from slack_sdk import WebClient
        _slack_client = WebClient(token=SLACK_BOT_TOKEN)
        _slack_client.auth_test()
        log.info("✅ Slack notifications enabled")
        return True
    except Exception as e:
        log.error(f"❌ Slack init failed: {e}")
        return False


def init_slack_listener(on_message: Callable[[str, Callable, Optional[str]], None]) -> bool:
    """
    Start two-way Slack Socket Mode listener.
    on_message(text, reply_fn, thread_ts) called for every @mention / DM.
    """
    global _slack_app
    if not SLACK_APP_TOKEN or not SLACK_BOT_TOKEN:
        log.warning("⚠️  Slack listener disabled — need both SLACK_BOT_TOKEN and SLACK_APP_TOKEN")
        return False
    try:
        from slack_bolt import App
        from slack_bolt.adapter.socket_mode import SocketModeHandler

        _slack_app = App(token=SLACK_BOT_TOKEN)

        @_slack_app.event("app_mention")
        def handle_mention(event, say):
            text = re.sub(r"<@[A-Z0-9]+>", "", event.get("text", "")).strip()
            ts   = event.get("thread_ts") or event.get("ts")
            log.info(f"📱 Slack mention: {text[:60]}")
            on_message(text, say, ts)

        @_slack_app.event("message")
        def handle_dm(event, say):
            if event.get("channel_type") != "im":
                return
            text = event.get("text", "").strip()
            ts   = event.get("ts")
            log.info(f"📱 Slack DM: {text[:60]}")
            on_message(text, say, ts)

        def _run():
            SocketModeHandler(_slack_app, SLACK_APP_TOKEN).start()

        threading.Thread(target=_run, daemon=True, name="slack-listener").start()
        log.info("✅ Slack listener enabled (two-way) 📱↔️🤖")
        return True
    except Exception as e:
        log.error(f"❌ Slack listener init failed: {e}")
        return False


def send_slack(message: str, channel: str = None, thread_ts: str = None) -> bool:
    if _slack_client is None:
        return False
    target = channel or SLACK_CHAT_CHANNEL
    try:
        _slack_client.chat_postMessage(channel=target, text=message, thread_ts=thread_ts)
        return True
    except Exception as e:
        log.error(f"Slack error ({target}): {e}")
        return False


def send_slack_alert(message: str):
    send_slack(message, channel=SLACK_ALERTS_CHANNEL)


def should_notify_slack(message: str) -> bool:
    triggers = ["notify me", "send to slack", "alert me", "let me know",
                "message me", "text me", "ping me", "remind me"]
    lower = message.lower()
    return any(t in lower for t in triggers)


# ═══════════════════════════════════════════════════════════════════════════════
# TELEGRAM
# ═══════════════════════════════════════════════════════════════════════════════

_telegram_app = None


def init_telegram(on_message: Callable[[str, Callable, int], None]) -> bool:
    """
    Start two-way Telegram bot.
    on_message(text, reply_fn, user_id) called for every text/photo message.
    """
    global _telegram_app
    if not TELEGRAM_BOT_TOKEN:
        log.warning("⚠️  Telegram disabled — no TELEGRAM_BOT_TOKEN")
        return False
    try:
        from telegram import Update
        from telegram.ext import (
            Application, CommandHandler, MessageHandler,
            filters, ContextTypes,
        )

        _telegram_app = Application.builder().token(TELEGRAM_BOT_TOKEN).build()

        async def _start(update: Update, ctx: ContextTypes.DEFAULT_TYPE):
            await update.message.reply_text(f"🤖 Aurora online. Chat ID: {update.effective_chat.id}")

        async def _text(update: Update, ctx: ContextTypes.DEFAULT_TYPE):
            text    = update.message.text or ""
            user_id = update.effective_user.id
            log.info(f"💬 TG text from {user_id}: {text[:60]}")

            async def _reply(response: str):
                try:
                    await update.message.reply_text(response[:4096])
                except Exception as err:
                    log.error(f"TG reply error: {err}")

            on_message(text, _reply, user_id)

        async def _photo(update: Update, ctx: ContextTypes.DEFAULT_TYPE):
            try:
                photo   = update.message.photo[-1]
                caption = update.message.caption or "What do you see?"
                f       = await ctx.bot.get_file(photo.file_id)
                import base64, io
                from PIL import Image
                img_bytes = await f.download_as_bytearray()
                img = Image.open(io.BytesIO(img_bytes)).convert("RGB")
                buf = io.BytesIO()
                img.save(buf, format="JPEG")
                b64 = base64.b64encode(buf.getvalue()).decode()

                async def _reply(response: str):
                    await update.message.reply_text(response[:4096])

                on_message(caption, _reply, update.effective_user.id, image_b64=b64)
            except Exception as e:
                log.error(f"TG photo error: {e}")

        _telegram_app.add_handler(CommandHandler("start", _start))
        _telegram_app.add_handler(MessageHandler(filters.TEXT & ~filters.COMMAND, _text))
        _telegram_app.add_handler(MessageHandler(filters.PHOTO, _photo))

        def _run():
            asyncio.run(_telegram_app.run_polling(drop_pending_updates=True))

        threading.Thread(target=_run, daemon=True, name="telegram-bot").start()
        log.info("✅ Telegram bot enabled (two-way) 💬↔️🤖📸")
        return True
    except Exception as e:
        log.error(f"❌ Telegram init failed: {e}")
        return False


def tg_send(text: str):
    """Fire-and-forget Telegram message to configured TELEGRAM_CHAT_ID."""
    if not _telegram_app or not TELEGRAM_CHAT_ID:
        return
    try:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(
            _telegram_app.bot.send_message(
                chat_id=TELEGRAM_CHAT_ID,
                text=text[:4096],
                parse_mode="Markdown",
            )
        )
        loop.close()
    except Exception as e:
        log.error(f"❌ tg_send failed: {e}")


# ═══════════════════════════════════════════════════════════════════════════════
# GMAIL
# ═══════════════════════════════════════════════════════════════════════════════

_gmail_seen_file: Path = Path.home() / ".grace_gmail_seen.json"
_gmail_seen_uids: set  = set()


def init_gmail() -> bool:
    global _gmail_seen_uids
    if not GMAIL_ADDRESS or not GMAIL_APP_PASS:
        log.warning("⚠️  Gmail disabled — set GRACE_GMAIL_ADDRESS + GRACE_GMAIL_APP_PASSWORD")
        return False
    try:
        mail = imaplib.IMAP4_SSL("imap.gmail.com")
        mail.login(GMAIL_ADDRESS, GMAIL_APP_PASS)
        mail.select("inbox")
        mail.logout()
    except Exception as e:
        log.error(f"❌ Gmail connection failed: {e}")
        return False

    if _gmail_seen_file.exists():
        try:
            _gmail_seen_uids = set(json.loads(_gmail_seen_file.read_text()))
        except Exception:
            pass

    threading.Thread(target=_gmail_poll_loop, daemon=True, name="gmail-poll").start()
    log.info(f"✅ Gmail polling every {GMAIL_POLL_SECS}s — {GMAIL_ADDRESS}")
    return True


def _gmail_poll_loop():
    global _gmail_seen_uids
    first_run = True
    while True:
        try:
            mail = imaplib.IMAP4_SSL("imap.gmail.com")
            mail.login(GMAIL_ADDRESS, GMAIL_APP_PASS)
            mail.select("inbox")
            _, data    = mail.search(None, "ALL")
            all_uids   = set(data[0].split())
            if first_run:
                _gmail_seen_uids = {u.decode() for u in all_uids}
                _save_gmail_seen()
                first_run = False
                log.info(f"📬 Gmail: {len(all_uids)} existing emails marked seen")
            else:
                new_uids = all_uids - {u.encode() for u in _gmail_seen_uids}
                for uid in new_uids:
                    _process_gmail_message(mail, uid)
                    _gmail_seen_uids.add(uid.decode())
                if new_uids:
                    _save_gmail_seen()
            mail.logout()
        except Exception as e:
            log.error(f"❌ Gmail poll error: {e}")
        time.sleep(GMAIL_POLL_SECS)


def _save_gmail_seen():
    try:
        _gmail_seen_file.write_text(json.dumps(list(_gmail_seen_uids)))
    except Exception:
        pass


def _decode_header_val(value: str) -> str:
    parts = _decode_header(value)
    out   = []
    for part, enc in parts:
        if isinstance(part, bytes):
            out.append(part.decode(enc or "utf-8", errors="replace"))
        else:
            out.append(str(part))
    return " ".join(out)


def _get_email_body(msg) -> str:
    body = ""
    if msg.is_multipart():
        for part in msg.walk():
            if part.get_content_type() == "text/plain" and not part.get("Content-Disposition"):
                charset = part.get_content_charset() or "utf-8"
                try:
                    body = part.get_payload(decode=True).decode(charset, errors="replace")
                    break
                except Exception:
                    pass
    else:
        charset = msg.get_content_charset() or "utf-8"
        try:
            body = msg.get_payload(decode=True).decode(charset, errors="replace")
        except Exception:
            pass
    return body[:3000]


def _classify_email(subject: str, sender: str, body: str) -> dict:
    prompt = f"""Classify this email. Reply ONLY with valid JSON, no markdown.

From: {sender}
Subject: {subject}
Body: {body[:400]}

JSON format:
{{"priority":"high|medium|low|ignore","category":"job_interview|job_application|client|shipping|invoice|newsletter|spam|other","summary":"one sentence","action_needed":true|false}}

Rules:
high = job interviews, client inquiries, invoices, anything needing same-day action
medium = application updates, shipping notifications
low = newsletters, account pings
ignore = spam, promotions"""

    try:
        from llm import llm_generate
        raw  = llm_generate(prompt, max_tokens=120, temperature=0.05)
        raw  = re.sub(r"```(?:json)?", "", raw).strip("`").strip()
        return json.loads(raw)
    except Exception:
        return {"priority": "low", "category": "other", "summary": subject, "action_needed": False}


def _process_gmail_message(mail, uid: bytes):
    try:
        _, msg_data = mail.fetch(uid, "(RFC822)")
        msg      = email_lib.message_from_bytes(msg_data[0][1])
        subject  = _decode_header_val(msg.get("Subject", "(no subject)"))
        sender   = _decode_header_val(msg.get("From", ""))
        body     = _get_email_body(msg)
        clf      = _classify_email(subject, sender, body)
        priority = clf.get("priority", "low")
        category = clf.get("category", "other")
        summary  = clf.get("summary", subject)
        action   = clf.get("action_needed", False)

        log.info(f"📧 [{priority}] {category}: {subject[:60]}")

        if priority not in ("high", "medium"):
            return

        emoji_map  = {"job_interview": "🎯", "job_application": "📋",
                      "client": "🤝", "shipping": "📦", "invoice": "💰"}
        emoji      = emoji_map.get(category, "📧")
        action_tag = "\n⚡ *Action needed*" if action else ""
        tg_alert   = (
            f"{emoji} *{priority.upper()} email*\n"
            f"📂 {category.replace('_', ' ').title()}\n"
            f"👤 `{sender[:60]}`\n"
            f"📌 {subject[:80]}\n"
            f"💬 {summary}{action_tag}"
        )
        tg_send(tg_alert)

        if priority == "high":
            send_slack(
                f"{emoji} *High-priority email*\nFrom: {sender[:60]}\n"
                f"Subject: {subject[:80]}\n{summary}",
                channel=SLACK_ALERTS_CHANNEL,
            )
    except Exception as e:
        log.error(f"❌ Email processing error: {e}")
