"""
PPU — Personal Provisioning Unit
=================================
AuRoRA · Semantic Cognitive System (SCS)

Provisions AuRoRA's active identity and user context at session init.
Called once by CNC during boot — before the cognitive cycle begins.

Responsibilities:
    - Load active persona system prompt from persona YAML
    - Load active user profile and apply extrinsic preferences
    - Expose provisioned state to CNC for injection into cognitive cycle

Architecture:
    Stateless loader — reads YAML, mutates AGi constants, returns user prefs.
    No ROS2 node — plain Python class, instantiated by CNC at init.
    All paths resolved from HRS constants — no magic strings.

Lifecycle:
    CNC.__init__() → PPU(logger) → ppu.provision(user_id)
                   → AGi.SCS.GCE.SYSTEM_PROMPT hydrated
                   → AGi.SCS.EMC.RELEVANCE_THRESHOLD adjusted
                   → ppu.system_prompt available to CNC

Terminology:
    Persona     — AuRoRA's active identity and system prompt
    User Prefs  — extrinsic per-user preferences shaping AuRoRA's behaviour
    Provision   — the act of loading and applying identity and user context
"""

# System libraries
from pathlib import Path                # path manipulation and resolution
import yaml                             # YAML parsing for persona and user profiles

# AGi libraries
from hrs.hrm import AGi, RRR            # Homeostatic Regulation Manifest constants and RRR constants


class PersonalProvisioningUnit:
    """
    Personal Provisioning Unit — session identity and user context loader.
    Instantiated by CNC at init. Not a ROS2 node.
    """

    def __init__(self, logger) -> None:
        self._logger     = logger                   # logger forwarded from caller — all PPU methods emit through this handle
        self._user_prefs : dict = {}                # internal user profile — populated by _load_user()
        self._system_prompt: str = ""               # Assembled system prompt — populated by _assemble_system_prompt()

    # ── Public Interface ──────────────────────────────────────────────────────

    def provision(self, user_id: str) -> None:
        """
        Provision AuRoRA's active identity and user context.
        Called once by caller at init — before cognitive cycle begins.

        Args:
            user_id:    Active user identifier
        """
        self._logger.info("─" * 60)
        self._logger.info("🪪  PPU — Personal Provisioning Unit activating…")
        self._load_user(user_id)
        self._load_persona()
        self._assemble_system_prompt()
        self._logger.info("✅ PPU — provisioning complete")
        self._logger.info("─" * 60)

    @property
    def system_prompt(self) -> str:
        """Assembled system prompt — available to caller after provision()."""
        return self._system_prompt

    def _load_user(self, user_id: str) -> None:
        """
        Load active user profile and apply extrinsic preferences.
        Mutates AGi.SCS.EMC.RELEVANCE_THRESHOLD if salience bias is set.

        Args:
            user_id: User identifier matching a key in users.yaml
        """
        path = (
            Path.home()
            / AGi.ENTITY_GATEWAY
            / RRR.RETICULAR_ACTIVATING_COMPARTMENT
            / AGi.SCS.USER_PROFILES
        )
        if not path.exists():
            self._logger.warning(f"⚠️  No users file at {path} — using HRS defaults")
            return
        try:
            data = yaml.safe_load(path.read_text())
            user = (data or {}).get("users", {}).get(user_id)
            if user:
                self._user_prefs = user
                self._logger.info(f"✅ User profile loaded — {user_id}")
                bias = user.get("memory_salience_bias")
                if bias is not None:
                    AGi.SCS.EMC.RELEVANCE_THRESHOLD -= bias
                    self._logger.info(f"✅ Memory salience bias applied — {user_id}")
            else:
                self._logger.warning(f"⚠️  User '{user_id}' not found in users.yaml — using HRS defaults")
        except Exception as e:
            self._logger.error(f"❌ Failed to load user profile: {e}")

    def _load_persona(self) -> None:
        """
        Load active persona system prompt from persona YAML.
        Mutates AGi.SCS.GCE.SYSTEM_PROMPT in place.
        """
        path = (
            Path.home()
            / AGi.ENTITY_GATEWAY
            / RRR.SEMANTIC_COGNITIVE_SYSTEM
            / RRR.GENERATIVE_COGNITIVE_ENGINE
            / AGi.SCS.PERSONA_PROFILES
        )
        if not path.exists():
            self._logger.warning(f"⚠️  No persona file at {path} — using HRS default")
            return
        try:
            data   = yaml.safe_load(path.read_text())
            prompt = (data or {}).get("gce", {}).get("system_prompt")
            if prompt:
                AGi.SCS.GCE.SYSTEM_PROMPT = prompt
                self._logger.info(f"✅ GCE Persona loaded")
            else:
                self._logger.warning("⚠️  GCE Persona missing gce.system_prompt — using HRS default")
        except Exception as e:
            self._logger.error(f"❌ Failed to load persona: {e}")

    def _assemble_system_prompt(self) -> None:
        """
        Inject static user context into the system prompt after both
        persona and user profile are loaded.
        Date is excluded — injected per-turn by caller.
        """

        user_name     = self._user_prefs.get("known_as") or self._user_prefs.get("name", "unknown")
        location      = self._user_prefs.get("location", "unknown")
        seed          = self._user_prefs.get("seed", "")

        self._system_prompt = AGi.SCS.GCE.SYSTEM_PROMPT.format(
            user_name=user_name,
            user_location=location,
            user_context=seed,              # seed only — location already in template
            date="{date}",                  # leave {date} intact — caller fills this per-turn
        )
