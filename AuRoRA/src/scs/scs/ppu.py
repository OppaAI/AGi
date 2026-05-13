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
    CNC.__init__() → PPU(logger) → ppu.provision(user_profile)
                   → AGi.SCS.GCE.ACTIVE_COGNITION hydrated
                   → ppu.active_cognition available to CNC

Terminology:
    Persona       — Grace's active identity, personality, and system prompt template
    Provision     — loading Grace's persona and assembling the session system prompt
    System Prompt — assembled prompt injecting Grace's persona and user context into the cognitive cycle

Public interface:
    ppu.provision(user_profile) → None
    ppu.active_cognition        → str
"""

# System libraries
import yaml                             # YAML parsing — loads persona profile from persona.yaml

# AGi libraries
from hrs.hrm import AGi                 # manifest constants — ACTIVE_COGNITION hydrated from persona.yaml
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
        self._logger                    = logger            # logger forwarded from caller — all PPU methods emit through this handle
        self._active_cognition : str    = ""                # assembled active cognition — populated by provision()

    def provision(self, user_profile: dict) -> None:
        """
        Load AuRoRA's active persona and assemble the session system prompt.
        Called once by CNC at init — after IRU recognition, before cognitive cycle begins.

        Args:
            user_profiles (dict): User profiles from IRU — injected into active cognition
        """
        self._logger.info("─" * 60)                                            # visual separator
        self._logger.info("🧬 PPU — Personal Progression Unit activating…")    # log the activation of PPU 
        self._retrieve_active_persona()                                        # retrieve the active persona
        self._assemble_active_cognition(user_profiles)                         # assemble active cognition from the active persona and active user profile
        self._logger.info("✅ PPU — Progression complete")                     # log the completion of activation
        self._logger.info("─" * 60)                                            # visual separator

    @property
    def active_cognition(self) -> str:
        """Expose the assembled active cognition — available to CNC after provision()."""
        return self._active_cognition                                          # make active cognition public to whole system

    def _retrieve_active_persona(self) -> None:
        """
        Retrieves AuRoRA's active persona and applies to the HRS manifest..
        Falls back to HRS default active cognition if persona is missing or malformed.
        """
        gateway = GatewayMap().user_profiles                                                            # retrieve the path to users.yaml file

        if not gateway.exists():                                                                        # no persona file — fall back to HRS default
            self._logger.warning(f"⚠️  No persona file at {gateway} — using HRM default")               # log missing user profile at gateway path
            return                                                                                      # user not in yaml — load hardcoded demo directly

        try:                                                                                            # attempt to load persona
            active_persona = yaml.safe_load(gateway.read_text())                                        # parse persona.yaml — AuRoRA's active identity
            if active_persona and gateway.get("active_cognition"):                                      # if perosna exists,
                AGi.SCS.GCE.ACTIVE_COGNITION = active_persona["active_cognition"]                       # hydrate manifest — overrides HRM default
                self._logger.info("✅ Persona retrieved")                                               # log succesful retrieval of persona
            else:                                                                                       # if active persona not found,
                self._logger.warning("⚠️  Persona file missing system prompt — using HRM default")      # log the warning that active user not found in persona.yaml
        except Exception as e:                                                                          # if error occurs during loading of persona.yaml
            self._logger.error(f"❌ Failed to load persona: {e}")                                       # log the error that persona failed to load

    def _assemble_active_cognition(self, user_profile: dict) -> None:
        """
        Assemble the active cognition from AuRoRA's persona and user context.
        Date excluded — injected per-turn by CNC.

        Args:
            user_profile (dict): User preferences from IRU — name, location, seed injected here
        """
        user_name = user_profile.get("known_as") or user_profile.get("name", "unknown")# preferred name — falls back to full name
        location  = user_profile.get("location", "unknown")                            # user location — for context-aware responses
        seed      = user_profile.get("seed", "")                                       # relational seed — cold-start context, fades as EMC accumulates

        self._active_cognition = AGi.SCS.GCE.ACTIVE_COGNITION.format(                  # assemble active cognition from persona template
            user_name     = user_name,                                                 # bind user identity
            user_location = location,                                                  # bind resolved location
            user_context  = seed,                                                      # seed only — location already in template
            date          = "{date}",                                                  # deferred — CNC fills per-turn
        )
