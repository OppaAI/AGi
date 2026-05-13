"""
IRU — Identity Recognition Unit
=================================
AuRoRA · Semantic Cognitive System (SCS)

Recognizes the active user and loads their relational context for the session.
Called once by CNC during boot — before the cognitive cycle begins.

Responsibilities:
    - Load active user profile from users.yaml
    - Apply extrinsic user preferences to AGi manifest
    - Expose user identity and relational context to CNC and PPU

Architecture:
    Stateless loader — reads YAML, mutates AGi constants, returns user context.
    No ROS2 node — plain Python class, instantiated by CNC at init.
    All paths resolved from HRS constants — no magic strings.

Lifecycle:
    CNC.__init__() → IRU(logger) → iru.recognize(user_id)
                   → AGi.SCS.EMC.RELEVANCE_THRESHOLD adjusted
                   → iru.user_prefs available to PPU
                   → iru.user_type available to CNC

Terminology:
    Recognition     — the act of identifying the active user and loading their relational context
    User Prefs      — extrinsic per-user preferences shaping Grace's behaviour
    UserType        — access classification governing memory recall scope

Public interface:
    iru.recognize(user_id) → None
    iru.user_prefs         → dict
    iru.user_type          → UserType
"""

# System libraries
import yaml                             # YAML parsing for user profiles

# AGi libraries
from hrs.hrm import AGi                 # manifest constants — RELEVANCE_THRESHOLD mutated on salience bias
from hrs.hru import GatewayMap          # gateway paths — resolves users.yaml location
from hrs.hru import UserAccessLevel     # access classification — admin sees all memories, guest sees own only
_gateway = GatewayMap()

class IdentityRecognitionUnit:
    """
    Identity Recognition Unit — user identity and relational context loader.
    Instantiated by CNC at init. Not a ROS2 node.
    Recognizes the active user and exposes their profile and access type to the session.
    """

    def __init__(self, logger) -> None:
        """
        Initialize the Identity Recognition Unit.

        Args:
            logger: Logger instance forwarded from CNC
        """
        self._logger     = logger                                   # logger forwarded from caller — all IRU methods emit through this handle
        self._user_prefs : dict     = {}                            # user profile — populated by recognize()
        self._user_type  : UserAccessLevel = UserAccessLevel.GUEST  # access type — defaults to guest until recognized

    # ── Public Interface ──────────────────────────────────────────────────────

    def recognize(self, user_id: str) -> None:
        """
        Recognize the active user and load their relational context.
        Called once by CNC at init — before cognitive cycle begins.

        Args:
            user_id (str): Active user identifier matching a key in users.yaml
        """
        self._logger.info("─" * 60)
        self._logger.info("🪪  IRU — Identity Recognition Unit activating…")
        self._load_user(user_id)
        self._logger.info("✅ IRU — recognition complete")
        self._logger.info("─" * 60)

    @property
    def user_prefs(self) -> dict:
        """User preferences — available to PPU after recognize()."""
        return self._user_prefs

    @property
    def user_type(self) -> UserAccessLevel:
        """Access classification — available to CNC after recognize()."""
        return self._user_type

    # ── Private Methods ───────────────────────────────────────────────────────

    def _load_user(self, user_id: str) -> None:
        """
        Load active user profile and apply extrinsic preferences.
        Mutates AGi.SCS.EMC.RELEVANCE_THRESHOLD if salience bias is set.

        Args:
            user_id (str): User identifier matching a key in users.yaml
        """
        path = _gateway.user_profiles

        if not path.exists():                                                               # no users file — fall back to HRS defaults
            self._logger.warning(f"⚠️  No users file at {path} — using HRS defaults")
            return

        try:
            data = yaml.safe_load(path.read_text())                                         # load users.yaml — full user registry
            user = (data or {}).get("users", {}).get(user_id)                               # extract profile for active user
            if user:
                self._user_prefs = user                                                     # store user profile for PPU
                self._user_type  = UserAccessLevel(user.get("access_level", "guest"))       # load access type — defaults to guest if not set
                self._logger.info(f"✅ User recognized — {user_id} ({self._user_type.value})")
                bias = user.get("memory_salience_bias")
                if bias is not None:                                                        # salience bias present — adjust recall threshold
                    AGi.SCS.EMC.RELEVANCE_THRESHOLD -= bias                                 # lower threshold — surfaces more memories for this user
                    self._logger.info(f"✅ Memory salience bias applied — {user_id}")
            else:
                self._logger.warning(f"⚠️  User '{user_id}' not found in users.yaml — using HRS defaults")
        except Exception as e:
            self._logger.error(f"❌ Failed to recognize user: {e}")
