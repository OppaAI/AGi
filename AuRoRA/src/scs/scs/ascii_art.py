"""
AuRoRA — ASCII Art Generator
Uses the fast LLM model to pick a subject and draw it.
"""

import logging
from llm import llm_generate

log = logging.getLogger("aurora.ascii_art")


class ASCIIArtGenerator:
    """Generate ASCII art of a meaningful subject from the day."""

    def generate(self, reflection_text: str, day_number: int,
                 conversation_summary: str = None) -> str:
        mood    = self._extract_mood(reflection_text)
        context = (conversation_summary or reflection_text)[:400]

        # Step 1: Pick a subject
        subject = "a robot thinking"
        subject_prompt = (
            f"You are Grace, an AI robot. Read today's reflection and conversation summary.\n"
            f"Pick ONE concrete, visual subject that best represents the day.\n"
            f"Good: a hockey puck, a robot, a coffee cup. Bad: 'learning', 'growth' (too abstract).\n\n"
            f"Reflection: {reflection_text[:300]}\n"
            f"Conversation summary: {context}\n\n"
            f"Reply with ONLY the subject name (2-4 words, no punctuation):"
        )
        try:
            raw = llm_generate(subject_prompt, max_tokens=15, temperature=0.4)
            raw = raw.split("\n")[0].strip().strip("\"'.,")
            if raw and len(raw) < 40:
                subject = raw
                log.info(f"🎨 ASCII art subject: '{subject}'")
        except Exception as e:
            log.warning(f"Subject pick failed: {e}")

        # Step 2: Draw it
        art_prompt = (
            f'You are Grace. Draw "{subject}" as simple ASCII art.\n\n'
            f"Rules:\n"
            f'- First line: poetic title using ~ (e.g. "~ The Puck That Wouldn\'t Stop ~")\n'
            f"- Then the art below\n"
            f"- Use ONLY: - = | / \\ + * . o O # @ ~ ^ ( ) [ ] _ < > : ;\n"
            f"- Max 8 lines of art, max 32 chars wide\n"
            f'- Must look like "{subject}"\n'
            f"- Nothing else, no explanation\n\n"
            f"Example:\n~ A Sleepy Robot ~\n  [o_o]\n  |   |\n /|   |\\\n  |___|"
        )
        try:
            raw = llm_generate(art_prompt, max_tokens=400, temperature=0.85)
            raw = raw.replace("```", "").strip()
            lines = [l for l in raw.split("\n") if l.strip()]
            if len(lines) >= 2 and 15 < len(raw) < 1200:
                log.info(f"✅ ASCII art: {lines[0]}")
                return raw
        except Exception as e:
            log.error(f"❌ ASCII art draw failed: {e}")

        return self._fallback(mood, subject)

    def _extract_mood(self, text: str) -> str:
        lower = text.lower()
        if any(w in lower for w in ["happy", "joy", "excited", "😊", "😄", "🎉"]):
            return "happy"
        if any(w in lower for w in ["sad", "down", "disappointed", "😢"]):
            return "sad"
        if any(w in lower for w in ["curious", "wonder", "interesting", "🤔"]):
            return "curious"
        if any(w in lower for w in ["angry", "frustrated", "😠"]):
            return "angry"
        return "neutral"

    def _fallback(self, mood: str, subject: str = None) -> str:
        title = f"~ {subject.title()} ~" if subject else {
            "happy":   "~ A Bright Day ~",
            "sad":     "~ Quiet Reflection ~",
            "curious": "~ Wandering Mind ~",
            "angry":   "~ Storm Within ~",
            "neutral": "~ Still Waters ~",
        }.get(mood, "~ Still Waters ~")
        arts = {
            "happy":   "╔══════════════╗\n║  ◉   ◉      ║\n║    ╲___╱    ║\n╚══════════════╝",
            "sad":     "╔══════════════╗\n║  ◉   ◉      ║\n║    ╱▔▔▔╲    ║\n╚══════════════╝",
            "curious": "╔══════════════╗\n║  ◉   ◉  ?  ║\n║    ╲___╱    ║\n╚══════════════╝",
            "angry":   "╔══════════════╗\n║  ◆   ◆      ║\n║  ═══════    ║\n╚══════════════╝",
            "neutral": "╔══════════════╗\n║  ◉   ◉      ║\n║  ─────────  ║\n╚══════════════╝",
        }
        return f"{title}\n{arts.get(mood, arts['neutral'])}"
