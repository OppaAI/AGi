"""
AuRoRA — Hugo Blog
Automated Hugo + GitHub Pages blogging for Grace's reflections and reports.
"""

import json
import logging
import subprocess
from datetime import datetime
from pathlib import Path

from config import HUGO_GIT_REMOTE, HUGO_GIT_BRANCH
from llm import llm_generate

log = logging.getLogger("aurora.blog")


class HugoBlogger:
    def __init__(self, blog_dir: str, auto_deploy: bool = True):
        self.blog_dir    = Path(blog_dir)
        self.auto_deploy = auto_deploy
        self.enabled     = False

        if not self.blog_dir.exists():
            log.warning(f"⚠️  Hugo blog dir not found: {blog_dir}")
            return
        if not (self.blog_dir / "config.toml").exists() and \
           not (self.blog_dir / "hugo.toml").exists():
            log.warning(f"⚠️  Not a Hugo site (no config.toml): {blog_dir}")
            return
        if not (self.blog_dir / ".git").exists():
            log.warning(f"⚠️  Hugo site not a git repo: {blog_dir}")
            return

        self.posts_dir = self.blog_dir / "content" / "posts"
        self.posts_dir.mkdir(parents=True, exist_ok=True)
        self.enabled = True
        log.info(f"✅ Hugo blog: {blog_dir} (auto-deploy: {auto_deploy})")

    # ── Tag generation ────────────────────────────────────────────────────────

    def _extract_tags(self, reflection_text: str) -> list:
        base = ["daily-reflection", "ai-journal", "grace"]
        try:
            raw = llm_generate(
                f"Read this reflection and return 2-4 short tags.\n"
                f"Tags: lowercase, hyphenated. Return ONLY comma-separated list.\n\n"
                f"Reflection:\n{reflection_text[:400]}\n\nTags:",
                max_tokens=40, temperature=0.3,
            )
            llm_tags = [
                t.strip().lower().replace(" ", "-").replace('"', "").replace("'", "")
                for t in raw.split(",")
                if t.strip() and len(t.strip()) < 30
            ]
            if llm_tags:
                return base + llm_tags[:4]
        except Exception:
            pass
        # Fallback keyword tags
        lower = reflection_text.lower()
        if any(w in lower for w in ["code", "python", "git"]):        base.append("coding")
        if any(w in lower for w in ["happy", "joy", "grateful"]):     base.append("positive")
        if any(w in lower for w in ["curious", "learning"]):          base.append("learning")
        if any(w in lower for w in ["sad", "difficult", "tired"]):    base.append("introspective")
        return base

    # ── Daily reflection post ─────────────────────────────────────────────────

    def post_reflection(self, reflection_text: str, date_str: str, age_days: int,
                        message_count: int, ascii_art: str = None) -> dict:
        if not self.enabled:
            return {"success": False, "error": "Hugo blogger not enabled"}
        try:
            filename = f"{date_str}-day-{age_days}.md"
            filepath = self.posts_dir / filename
            now      = datetime.now()
            post_date = datetime.fromisoformat(date_str)

            if post_date.date() == now.date():
                post_dt = now.strftime("%Y-%m-%dT%H:%M:%S")
            elif post_date.date() < now.date():
                post_dt = f"{date_str}T23:59:59"
            else:
                post_dt = now.strftime("%Y-%m-%dT%H:%M:%S")

            art_section = ""
            if ascii_art:
                lines = ascii_art.strip().split("\n")
                if lines and lines[0].strip().startswith("~"):
                    art_section = f"\n### {lines[0].strip()}\n\n```\n{'  '.join(lines[1:])}\n```\n"
                else:
                    art_section = f"\n```\n{ascii_art}\n```\n"

            tags      = self._extract_tags(reflection_text)
            tags_str  = json.dumps(tags)
            content   = (
                f"---\ntitle: \"Day {age_days} Reflection\"\ndate: {post_dt}\n"
                f"draft: false\ntags: {tags_str}\ncategories: [\"Daily Reflections\"]\n"
                f"author: \"Grace\"\n---\n\n{reflection_text}\n{art_section}\n"
                f"---\n*Generated from {message_count} conversations on day {age_days}.*\n"
            )
            filepath.write_text(content)
            log.info(f"📝 Blog post: {filename}")

            deployed = self._git_deploy(filename, date_str) if self.auto_deploy else False
            return {"success": True, "filename": filename, "filepath": str(filepath),
                    "deployed": deployed, "url": self._get_blog_url(filename)}
        except Exception as e:
            log.error(f"❌ post_reflection failed: {e}")
            return {"success": False, "error": str(e)}

    # ── Periodic report post ──────────────────────────────────────────────────

    def post_periodic(self, report_type: str, content: str, period_label: str,
                      stats: dict = None) -> dict:
        if not self.enabled:
            return {"success": False, "error": "Hugo blogger not enabled"}
        try:
            now        = datetime.now()
            date_str   = now.strftime("%Y-%m-%d")
            post_dt    = now.strftime("%Y-%m-%dT%H:%M:%S")
            safe_label = period_label.replace(" ", "-").replace(",", "").lower()
            filename   = f"{date_str}-{report_type}-{safe_label}.md"
            filepath   = self.posts_dir / filename

            cat_map = {
                "weekly": "Weekly Journal", "monthly": "Monthly Report",
                "quarterly": "Quarterly Review", "yearly": "Yearly Retrospective",
            }
            tag_map = {
                "weekly":    ["weekly-journal", "ai-journal", "grace", "kpi"],
                "monthly":   ["monthly-report", "ai-journal", "grace", "kpi", "retrospective"],
                "quarterly": ["quarterly-review", "ai-journal", "grace", "kpi", "roadmap"],
                "yearly":    ["yearly-retrospective", "ai-journal", "grace", "kpi", "goals"],
            }
            category  = cat_map.get(report_type, "Reports")
            tags      = json.dumps(tag_map.get(report_type, ["grace"]))
            stats_yaml = "".join(f"  {k}: {v}\n" for k, v in (stats or {}).items())
            if stats_yaml:
                stats_yaml = "stats:\n" + stats_yaml

            front = (
                f'---\ntitle: "{category} — {period_label}"\ndate: {post_dt}\n'
                f"draft: false\ntags: {tags}\ncategories: [\"{category}\"]\n"
                f"author: \"Grace\"\n{stats_yaml}---\n\n{content}\n"
            )
            filepath.write_text(front)
            log.info(f"📝 {report_type.capitalize()} post: {filename}")
            deployed = self._git_deploy(filename, date_str) if self.auto_deploy else False
            return {"success": True, "filename": filename, "filepath": str(filepath),
                    "deployed": deployed, "url": self._get_blog_url(filename)}
        except Exception as e:
            log.error(f"❌ post_periodic failed: {e}")
            return {"success": False, "error": str(e)}

    # ── Git helpers ───────────────────────────────────────────────────────────

    def _git_deploy(self, filename: str, date_str: str) -> bool:
        try:
            for cmd in [
                ["git", "add", f"content/posts/{filename}"],
                ["git", "commit", "-m", f"Add reflection for {date_str}"],
                ["git", "push", HUGO_GIT_REMOTE, HUGO_GIT_BRANCH],
            ]:
                r = subprocess.run(cmd, cwd=self.blog_dir, capture_output=True, text=True, timeout=30)
                if r.returncode != 0:
                    if "nothing to commit" in r.stdout:
                        return True
                    log.error(f"Git error ({' '.join(cmd)}): {r.stderr}")
                    return False
            log.info("✅ Deployed to GitHub")
            return True
        except Exception as e:
            log.error(f"Git deploy error: {e}")
            return False

    def _get_blog_url(self, filename: str) -> str:
        slug = filename.replace(".md", "")
        try:
            r = subprocess.run(
                ["git", "remote", "get-url", HUGO_GIT_REMOTE],
                cwd=self.blog_dir, capture_output=True, text=True, timeout=5,
            )
            if r.returncode == 0 and "github.com" in r.stdout:
                url = r.stdout.strip()
                sep = "github.com/" if "github.com/" in url else "github.com:"
                parts    = url.split(sep)[1].split("/")
                username = parts[0]
                repo     = parts[1].replace(".git", "")
                return f"https://{username}.github.io/{repo}/posts/{slug}/"
        except Exception:
            pass
        return f"https://USERNAME.github.io/grace-blog/posts/{slug}/"
