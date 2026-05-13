"""
IRU — Identity Recognition Unit
=================================
AuRoRA · Semantic Cognitive System (SCS)

Recognizes the active user and loads their relational context for the interaction.
Called once by CNC during ignition — before the cognitive cycle begins.

Responsibilities:
    - Load active user profile from users.yaml
    - Apply extrinsic user preferences to HRS manifest
    - Expose user identity and relational context to CNC and PPU

Architecture:
    Stateless loader — reads YAML, mutates HRS manifest constants, returns user context.
    No ROS2 node — plain Python class, instantiated by CNC at init.
    All paths resolved from HRS manifest constants — no magic strings.

Lifecycle:
    CNC.__init__() → IRU(logger) → iru.recognize_user(user_id)
                   → AGi.SCS.EMC.RELEVANCE_THRESHOLD adjusted
                   → iru.user_profile available to CNC and PPU

Terminology:
    Recognition         — identifying the active user and loading their relational context
    User Profile        — extrinsic per-user preferences shaping AuRoRA's behaviour
    User Access Level   — access classification governing memory recall scope and permissions

Public interface:
    iru.recognize_user(user_id) → None
    iru.user_profile            → UserProfile
"""

# System libraries
import yaml                                         # YAML parsing — loads user profiles from users.yaml

# AGi libraries
from hrs.hrm import AGi                             # manifest constants — RELEVANCE_THRESHOLD mutated on salience bias
from hrs.hru import GatewayMap                      # gateway paths — resolves users.yaml location
from hrs.hru import UserProfile, UserAccessLevel    # user identity types — shared across whole system

_gateway = GatewayMap()                             # module-level gateway — resolves all IRU filesystem paths

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
        self._user_profile: UserProfile | None = None               # populated by recognize_user() — None until user is loaded

    # ── Public Interface ──────────────────────────────────────────────────────

    def recognize_user(self, user_id: str) -> None:
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
    def user_profile(self) -> UserProfile | None:
        """Recognized user profile — available to CNC and PPU after recognize()."""
        return self._user_profile                                    # make recognized user profile public to whole system

    def _load_user(self, user_id: str) -> None:
        """
        Load active user profile and apply extrinsic preferences.
        Mutates AGi.SCS.EMC.RELEVANCE_THRESHOLD if salience bias is set.
    
        Args:
            user_id (str): User identifier matching a key in users.yaml
        """
        path = _gateway.user_profiles
    
        if not path.exists():                                                           # no users file — fall back to HRS defaults
            self._logger.warning(f"⚠️  No users file at {path} — using HRS defaults")
            return
    
        try:
            data = yaml.safe_load(path.read_text())                                     # load users.yaml — full user registry
            user = (data or {}).get("users", {}).get(user_id)                           # extract profile for active user
    
            if user:
                self._user_profile = UserProfile(
                    user_id = user_id,                                                  # inject key as user_id — not stored inside the YAML block
                    **{k: v for k, v in user.items()
                       if k in UserProfile.__dataclass_fields__}                        # filter unknown YAML keys — future fields won't break construction
                )
                self._logger.info(f"✅ User recognized — {user_id} ({self._user_profile.access_level.value})")
    
                if self._user_profile.memory_salience_bias:                             # salience bias present — adjust recall threshold
                    AGi.SCS.EMC.RELEVANCE_THRESHOLD -= self._user_profile.memory_salience_bias  # lower threshold — surfaces more memories for this user
                    self._logger.info(f"✅ Memory salience bias applied — {user_id}")
            else:
                self._logger.warning(f"⚠️  User '{user_id}' not found in users.yaml — using HRS defaults")
    
        except Exception as e:
            self._logger.error(f"❌ Failed to recognize user: {e}")
