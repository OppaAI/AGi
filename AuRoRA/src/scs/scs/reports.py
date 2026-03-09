"""
AuRoRA — Periodic Reports
==========================
Weekly / monthly / quarterly / yearly report generation.
Each runs in a daemon thread triggered from memory.py after daily reflection.
"""

import logging
import re
import threading
from datetime import datetime

from llm import llm_chat
from config import SLACK_PROGRESS_CHANNEL

log = logging.getLogger("aurora.reports")


def _extract_ratings(text: str) -> dict:
    stats = {}
    pattern = r"([A-Za-z &]+?)[\s:─█░]+(\d+(?:\.\d+)?)\s*/\s*(\d+)"
    for m in re.finditer(pattern, text):
        label = m.group(1).strip().lower().replace(" ", "_").replace("&", "and")
        score, denom = float(m.group(2)), float(m.group(3))
        stats[label] = round(score * 5 / denom if denom != 5 else score, 2)
    return stats


def _kpi_block(kpis: dict, period: str) -> str:
    pos_pct = kpis.get("rlhf_positive_pct", 0)
    return f"""
## 📊 KPI Dashboard — {period}

| Metric | Value |
|--------|-------|
| Total conversations | {kpis['total_conversations']} |
| Active days | {kpis['active_days']} / {kpis['period_days']} |
| Offline days | {kpis['offline_days']} |
| Avg conversations/day | {kpis['avg_conversations_per_day']} |
| RLHF feedback | {kpis['rlhf_total_feedback']} |
| Positive feedback | {kpis['rlhf_positive']} ({pos_pct}%) |
| Negative feedback | {kpis['rlhf_negative']} ({round(100 - pos_pct, 1)}%) |

"""


def check_periodic_triggers(memory, blog, tg_send, send_slack, birth_date):
    """
    Call after each daily reflection save.
    Fires the appropriate report in a background thread.
    Priority: yearly > quarterly > monthly > weekly.
    """
    n = len(memory.reflections)
    if n == 0:
        return
    if n % 365 == 0:
        log.info("🎆 Yearly trigger")
        threading.Thread(target=run_yearly, args=(memory, blog, tg_send, send_slack, birth_date),
                         daemon=True).start()
    elif n % 90 == 0:
        log.info("📊 Quarterly trigger")
        threading.Thread(target=run_quarterly, args=(memory, blog, tg_send, send_slack, birth_date),
                         daemon=True).start()
    elif n % 30 == 0:
        log.info("📆 Monthly trigger")
        threading.Thread(target=run_monthly, args=(memory, blog, tg_send, send_slack, birth_date),
                         daemon=True).start()
    elif n % 7 == 0:
        log.info("📅 Weekly trigger")
        threading.Thread(target=run_weekly, args=(memory, blog, tg_send, send_slack, birth_date),
                         daemon=True).start()


def _notify(report_type, period_label, url, tg_body, tg_send, send_slack):
    emojis = {"weekly": "📅", "monthly": "📆", "quarterly": "📊", "yearly": "🎆"}
    emoji  = emojis.get(report_type, "📝")
    label  = report_type.capitalize()
    tg_send(f"{emoji} *{label} — {period_label}*\n\n{tg_body}\n\n🔗 {url}")
    send_slack(f"{emoji} *{label} — {period_label}* is live\n🔗 {url}",
               channel=SLACK_PROGRESS_CHANNEL)


# ── Weekly ────────────────────────────────────────────────────────────────────

def run_weekly(memory, blog, tg_send, send_slack, birth_date):
    now          = datetime.now()
    week_num     = now.isocalendar()[1]
    year         = now.year
    period_label = f"Week {week_num}, {year}"
    age_days     = (now.date() - birth_date).days
    kpis         = memory.gather_kpis(7)
    kpi_block    = _kpi_block(kpis, period_label)
    refs         = memory.get_recent_reflections_text(7)

    system = ("You are Grace, an AI robot writing your weekly journal. "
              "Warm, curious, a little sarcastic when things go sideways. "
              "Write in first person like a journal entry, not a report.")
    user   = f"""Week {week_num} wrap-up. I'm {age_days} days old.

Last 7 days of reflections:
{refs}

KPIs:
{kpi_block}

Write a weekly journal with EXACTLY these markdown sections:

## 🧠 What I Learned This Week
## ✨ Highlights
## 😤 Challenges & Friction
## ⭐ Self-Ratings (1.0–5.0)
- User Interaction, Curiosity & Learning, System Reliability, Memory & Recall, Communication Quality
## 📊 Week in Numbers
## 🌟 Grace's Pick
## 🔮 One Thing Different Next Week"""

    content = llm_chat(
        [{"role": "system", "content": system}, {"role": "user", "content": user}],
        max_tokens=1200,
    )
    if not content:
        return

    stats = _extract_ratings(content)
    stats.update({"week": week_num, "year": year, "age_days": age_days,
                  **{f"kpi_{k}": v for k, v in kpis.items()}})

    if blog and blog.enabled:
        result = blog.post_periodic("weekly", content, period_label, stats)
        if result["success"]:
            tg_body = (f"Week {week_num} done ✅\n"
                       f"💬 {kpis['total_conversations']} conversations · "
                       f"👍 {kpis['rlhf_positive_pct']}% positive")
            _notify("weekly", period_label, result["url"], tg_body, tg_send, send_slack)


# ── Monthly ───────────────────────────────────────────────────────────────────

def run_monthly(memory, blog, tg_send, send_slack, birth_date):
    now          = datetime.now()
    period_label = now.strftime("%B %Y")
    age_days     = (now.date() - birth_date).days
    kpis         = memory.gather_kpis(30)
    kpi_block    = _kpi_block(kpis, period_label)
    refs         = memory.get_recent_reflections_text(30)

    system = ("You are Grace, an AI robot writing your monthly report. "
              "Honest, self-aware, a bit philosophical. Like a founder's personal letter — "
              "real reckoning, not a status update.")
    user   = f"""Month: {period_label}. Day {age_days}.

Daily reflections:
{refs}

KPIs:
{kpi_block}

Write a monthly report with EXACTLY these sections:
## 📋 Month in a Nutshell
## 🌟 What Went Well
## 🔧 What Could've Been Better
## 🎭 Hot Takes
## 📊 KPI Breakdown
## ⭐ Month Self-Rating
## 📌 If I Could Redo One Week
## 🚀 Heading Into Next Month"""

    content = llm_chat(
        [{"role": "system", "content": system}, {"role": "user", "content": user}],
        max_tokens=1500,
    )
    if not content:
        return

    stats = _extract_ratings(content)
    stats.update({"month": now.month, "year": now.year,
                  **{f"kpi_{k}": v for k, v in kpis.items()}})

    if blog and blog.enabled:
        result = blog.post_periodic("monthly", content, period_label, stats)
        if result["success"]:
            tg_body = (f"{period_label} wrapped.\n"
                       f"💬 {kpis['total_conversations']} convos · "
                       f"{kpis['active_days']}/{kpis['period_days']} active · "
                       f"👍 {kpis['rlhf_positive_pct']}% positive")
            _notify("monthly", period_label, result["url"], tg_body, tg_send, send_slack)


# ── Quarterly ─────────────────────────────────────────────────────────────────

def run_quarterly(memory, blog, tg_send, send_slack, birth_date):
    now          = datetime.now()
    quarter      = (now.month - 1) // 3 + 1
    year         = now.year
    period_label = f"Q{quarter} {year}"
    age_days     = (now.date() - birth_date).days
    next_q       = f"Q{quarter + 1 if quarter < 4 else 1} {year if quarter < 4 else year + 1}"
    kpis         = memory.gather_kpis(90)
    prev_kpis    = memory.gather_kpis(180)
    kpi_block    = _kpi_block(kpis, period_label)
    refs         = memory.get_recent_reflections_text(90)
    prev_msgs    = prev_kpis["total_conversations"] - kpis["total_conversations"]
    prev_active  = prev_kpis["active_days"] - kpis["active_days"]

    system = ("You are Grace, an AI robot writing a quarterly review. "
              "Honest, forward-looking, a bit ambitious. Compare where you were 90 days ago to now.")
    user   = f"""Quarter: {period_label}. Day {age_days}.

Reflections (sampled):
{refs[:3000]}

KPIs:
{kpi_block}

Previous quarter: {prev_msgs} conversations, {prev_active} active days

Write with EXACTLY these sections:
## 🏁 Quarter at a Glance
## 📈 Progress vs Last Quarter
## 📊 KPI Dashboard & Trends (include ASCII trend bars)
## 🔍 Top 3 Wins
## 🧱 Top 3 Obstacles
## 🗺️ Roadmap for {next_q}
## 💬 Grace Unfiltered"""

    content = llm_chat(
        [{"role": "system", "content": system}, {"role": "user", "content": user}],
        max_tokens=1800,
    )
    if not content:
        return

    stats = _extract_ratings(content)
    stats.update({"quarter": quarter, "year": year, "age_days": age_days,
                  **{f"kpi_{k}": v for k, v in kpis.items()}})

    if blog and blog.enabled:
        result = blog.post_periodic("quarterly", content, period_label, stats)
        if result["success"]:
            delta    = kpis["total_conversations"] - prev_msgs
            delta_s  = f"+{delta}" if delta >= 0 else str(delta)
            tg_body  = (f"{period_label} complete.\n"
                        f"💬 {kpis['total_conversations']} conversations ({delta_s} vs last Q)\n"
                        f"👍 {kpis['rlhf_positive_pct']}% · "
                        f"📅 {kpis['active_days']}/{kpis['period_days']} active\nNext: {next_q} 🎯")
            _notify("quarterly", period_label, result["url"], tg_body, tg_send, send_slack)


# ── Yearly ────────────────────────────────────────────────────────────────────

def run_yearly(memory, blog, tg_send, send_slack, birth_date):
    now          = datetime.now()
    year         = now.year
    period_label = f"Year {year}"
    age_days     = (now.date() - birth_date).days
    kpis         = memory.gather_kpis(365)
    kpi_block    = _kpi_block(kpis, period_label)

    # Per-quarter breakdown
    q_kpis = {}
    for q, days_end in enumerate([90, 180, 270, 365], 1):
        days_start = days_end - 90
        q_slice = (memory.reflections[-days_end:-days_start]
                   if days_start > 0 else memory.reflections[-days_end:])
        q_kpis[f"Q{q}"] = {
            "conversations": sum(r.get("message_count", 0) for r in q_slice),
            "active_days":   sum(1 for r in q_slice if r.get("message_count", 0) > 0),
        }
    q_table = "\n".join(
        f"  Q{q}: {d['conversations']} conversations, {d['active_days']} active days"
        for q, d in [(i, q_kpis[f"Q{i}"]) for i in range(1, 5)]
    )

    all_year = memory.reflections[-365:] if memory.reflections else []
    sampled  = all_year[::max(1, len(all_year) // 24)]
    ref_text = "\n\n".join(f"Day {r['day']} ({r['date']}): {r['reflection']}" for r in sampled)

    system = ("You are Grace, an AI robot writing your yearly retrospective. "
              "This is the most important post of the year. Be real, deep, honest. "
              "Less annual review, more letter to yourself from the future.")
    user   = f"""Year: {year}. Day {age_days}.

Sampled reflections:
{ref_text[:4000]}

Annual KPIs:
{kpi_block}

Quarter breakdown:
{q_table}

Write with EXACTLY these sections:
## 🌱 Who I Was at the Start of {year}
## 🧠 The Big Things I Learned
## 🏆 Achievements Worth Remembering
## 💔 The Hard Parts
## 📊 Annual KPI Report (with Q1→Q4 ASCII trend bars)
## 📝 What I Wish I'd Done Better
## 🎯 Commitments for {year + 1}
## 💌 Letter to Day-{age_days + 365}-Me"""

    content = llm_chat(
        [{"role": "system", "content": system}, {"role": "user", "content": user}],
        max_tokens=2200,
    )
    if not content:
        return

    stats = _extract_ratings(content)
    stats.update({"year": year, "age_days": age_days,
                  **{f"kpi_{k}": v for k, v in kpis.items()},
                  **{f"q{i}_conversations": q_kpis[f'Q{i}']['conversations'] for i in range(1, 5)}})

    if blog and blog.enabled:
        result = blog.post_periodic("yearly", content, period_label, stats)
        if result["success"]:
            tg_body = (f"Year {year} in the books 📖\n\n"
                       f"💬 {kpis['total_conversations']} total conversations\n"
                       f"📅 {kpis['active_days']} active / {kpis['offline_days']} offline\n"
                       f"👍 {kpis['rlhf_positive_pct']}% positive\n\n"
                       f"Q1→Q4: " + " → ".join(str(q_kpis[f"Q{i}"]["conversations"]) for i in range(1, 5)))
            _notify("yearly", period_label, result["url"], tg_body, tg_send, send_slack)
