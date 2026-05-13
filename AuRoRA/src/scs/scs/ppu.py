"""
PPU — Personal Progression Unit
================================
AuRoRA · Semantic Cognitive System (SCS)

Loads and maintains Grace's active persona and assembles the session system prompt.
Called once by CNC during boot — before the cognitive cycle begins.

Responsibilities:
    - Load Grace's active persona from persona.yaml
    - Assemble the session system prompt from persona and user context
    - Expose the assembled system prompt to CNC for injection into the cognitive cycle
    - Future: self-learning, persona evolution, knowledge acquisition

Architecture:
    Stateless loader — reads YAML, assembles system prompt, exposes to CNC.
    No ROS2 node — plain Python class, instantiated by CNC at init.
    All paths resolved from HRS constants — no magic strings.
    Depends on IRU for user context — IRU must be recognized before PPU provisions.

Lifecycle:
    CNC.__init__() → PPU(logger) → ppu.provision(user_prefs)
                   → AGi.SCS.GCE.SYSTEM_PROMPT hydrated
                   → ppu.system_prompt available to CNC

Terminology:
    Persona       — Grace's active identity, personality, and system prompt template
    Provision     — loading Grace's persona and assembling the session system prompt
    System Prompt — assembled prompt injecting Grace's persona and user context into the cognitive cycle

Public interface:
    ppu.provision(user_prefs) → None
    ppu.system_prompt         → str
"""

# System libraries
import yaml                             # YAML parsing — loads persona profile from persona.yaml

# AGi libraries
from hrs.hrm import AGi                 # manifest constants — SYSTEM_PROMPT hydrated from persona.yaml
from hrs.hru import GatewayMap          # gateway paths — resolves persona.yaml location

class PersonalProgressionUnit:
    """
    Personal Progression Unit — AuRoRA's persona loader and active cognition assembler.
    Retrieve AuRoRA's active persona and assemble the active cognition.
    """
    def __init__(self, logger) -> None:
        """
        Initialize the Personal Progression Unit.

        Args:
            logger: Logger instance forwarded from caller
        """
        self._logger        = logger        # logger forwarded from caller — all PPU methods emit through this handle
        self._system_prompt : str = ""      # assembled active cognition — populated by provision()

    def provision(self, user_prefs: dict) -> None:
        """
        Load AuRoRA's active persona and assemble the session system prompt.
        Called once by CNC at init — after IRU recognition, before cognitive cycle begins.

        Args:
            user_profiles (dict): User profiles from IRU — injected into active cognition
        """
        self._logger.info("─" * 60)                                            # visual separator
        self._logger.info("🧬 PPU — Personal Progression Unit activating…")    # log the activation of IRU 
        self._retrieve_active_persona()                                        # retrieve the active persona
        self._assemble_active_cognition(user_profiles)                         # assmble active cognition from the active persona and active user profile
        self._logger.info("✅ PPU — Progression complete")                     # log the completion of activation
        self._logger.info("─" * 60)                                            # visual separator

    @property
    def assemble_active_cognition(self) -> str:
        """Expose the assembled active cognition — available to CNC after provision()."""
        return self._active_cognition

    def _load_persona(self) -> None:
        """
        Load Grace's active persona from persona.yaml into AGi manifest.
        Falls back to HRM default system prompt if persona file is missing or malformed.
        """
        path = GatewayMap().active_persona

        if not path.exists():                                                           # no persona file — fall back to HRM default
            self._logger.warning(f"⚠️  No persona file at {path} — using HRM default")
            return

        try:
            data = yaml.safe_load(path.read_text())                                     # load persona.yaml — Grace's active identity
            if data and data.get("system_prompt"):
                AGi.SCS.GCE.SYSTEM_PROMPT = data["system_prompt"]                       # hydrate manifest — overrides HRM default
                self._logger.info("✅ Persona loaded")
            else:
                self._logger.warning("⚠️  Persona file missing system_prompt — using HRM default")
        except Exception as e:
            self._logger.error(f"❌ Failed to load persona: {e}")

    def _assemble_active_cognition(self, user_prefs: dict) -> None:
        """
        Assemble the active cognition from AuRoRA's persona and user context.
        Date excluded — injected per-turn by CNC.

        Args:
            user_prefs (dict): User preferences from IRU — name, location, seed injected here
        """
        user_name = user_prefs.get("known_as") or user_prefs.get("name", "unknown")    # preferred name — falls back to full name
        location  = user_prefs.get("location", "unknown")                              # user location — injected into persona template
        seed      = user_prefs.get("seed", "")                                         # personal context notes — injected into persona template

        self._system_prompt = AGi.SCS.GCE.SYSTEM_PROMPT.format(
            user_name    = user_name,
            user_location= location,
            user_context = seed,                                                        # seed only — location already in template
            date         = "{date}",                                                    # leave intact — CNC fills per-turn
        )
