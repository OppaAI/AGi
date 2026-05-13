"""
IRU — Identity Recognition Unit
=================================
AuRoRA · Semantic Cognitive System (SCS)

Recognizes the active user and retrieves their relational context for the interaction.
Called once by CNC during ignition — before the cognitive cycle begins.

Responsibilities:
    - Retrieve active user profile from users.yaml
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
    Recognition         — identifying the active user and retrieving their relational context
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

class IdentityRecognitionUnit:
    """
    Identity Recognition Unit — user identity and relational context loader.
    Recognizes the active user and exposes their profile and access type to whole system.
    """

    def __init__(self, logger) -> None:
        """
        Initialize the Identity Recognition Unit.

        Args:
            logger: Logger instance forwarded from caller
        """
        self._logger                           = logger             # logger forwarded from caller — all IRU methods emit through this handle
        self._user_profile: UserProfile | None = None               # populated by recognize_user() — None until user is loaded

    def recognize_user(self, user_id: str) -> None:
        """
        Recognize the active user and retrieve their relational context.
        Called once by CNC at init — before cognitive cycle begins.

        Args:
            user_id (str): Active user identifier matching a key in users.yaml
        """
        self._logger.info("─" * 60)                                            # visual separator — stdout and /rosout
        self._logger.info("🪪  IRU — Identity Recognition Unit activating…")   # log the activation of IRU  
        self._retrieve_user_profile(user_id)                                   # retrieve user profile of the given user_id
        self._logger.info("✅ IRU — recognition complete")                     # log the completion of activation
        self._logger.info("─" * 60)                                            # visual separator — stdout and /rosout

    @property
    def user_profile(self) -> UserProfile | None:
        """Recognized user profile — available to CNC and PPU after recognize()."""
        return self._user_profile                                              # make recognized user profile public to whole system

    def _retrieve_user_profile(self, user_id: str) -> None:
        """
        Retrieves the profile of active user and apply extrinsic preferences to the HRS manifest.
        Mutates AGi.SCS.EMC.RELEVANCE_THRESHOLD if salience bias is set.
    
        Args:
            user_id (str): User identifier matching a key in users.yaml
        """
        gateway = GatewayMap().user_profiles                                                     # retrieve the path to users.yaml file
        
        if not gateway.exists():                                                                 # no users profile — fall back to HRS defaults
            self._logger.warning(f"⚠️ User profile missing at {gateway} — loading hardcoded demo")  # log the missing of user profile
            self._user_profile = DEMO_USER                                                       # no yaml file — last resort fallback
            return
    
        try:
            user_profiles: dict = yaml.safe_load(gateway.read_text())                            # load users.yaml — full user registry
            active_user: dict | None = (user_profiles or {}).get("users", {}).get(user_id)       # extract profile for active user
    
            if active_user:                                                                      # if active user exists in the users.yaml
                self._user_profile = UserProfile(                                                # retrieve user profile of the active user
                    user_id = user_id,                                                           # inject key as user_id — not stored inside the YAML block
                    **{key: value for key, value in active_user.items()
                       if key in UserProfile.__dataclass_fields__}                               # filter unknown YAML keys — future fields won't break construction
                )
                self._logger.info(f"✅ User recognized — {user_id} ({self._user_profile.access_level.value})")    # log the successful retrieval of the active user profile
    
                if self._user_profile.memory_salience_bias:                                      # salience bias present — adjust recall threshold
                    AGi.SCS.EMC.RELEVANCE_THRESHOLD -= self._user_profile.memory_salience_bias   # lower threshold — surfaces more memories for this user
                    self._logger.info(f"✅ Memory salience bias applied — {user_id}")            # log the succesful adjustment of memory saliecne for this user
            else:                                                                                # if active user not found,
                self._logger.warning(f"⚠️  User '{user_id}' not found in users.yaml — using HRS defaults")  # log the warning that active user not found in users.yaml
                self._user_profile = DEMO_USER                                                   # user not in yaml — load hardcoded demo directly
                
        except Exception as e:                                                                   # if error occurs duirng loading of users.yaml
            self._logger.error(f"❌ Failed to recognize user: {e}")                              # log the error that user not recognized
