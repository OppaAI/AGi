"""
RAC — Reticular Activation Compartment
========================================
AuRoRA · Semantic Cognitive System (SCS)

Plain Python bootloader — called once by CNC at startup.
Not a ROS2 node. No spin, no topics, no lifecycle.

Responsibilities:
    1. Hydrate AGi from YAML — aurora.yaml, persona.yaml, users.yaml
    2. Recompute derived constants via _derive()
    3. Spawn downstream ROS2 nodes in dependency order
    4. Return AuroraConfig to CNC — persona + user context for the session

CNC delegates boot here and takes over once ignite() returns:

    class CNC(Node):
        def __init__(self):
            super().__init__("cnc")
            self.config = RAC(self).ignite()    # delegate boot — CNC takes over after

Design contract:
    - HRM is the static schema and fallback defaults.
    - RAC loads YAML and overrides AGi class attributes in place.
    - AGi is the single hydrated constant registry for this process.
    - All other modules import AGi after RAC has run — values are always hydrated.
    - Read AGi constants inside __init__ or methods, never at module level.

    Correct (read inside method — RAC has run by this point):
        def __init__(self):
            self.limit = AGi.SCS.EMC.RECALL_SURFACE_LIMIT   # ✅ hydrated

    Incorrect (read at module level — RAC has not run yet):
        LIMIT = AGi.SCS.EMC.RECALL_SURFACE_LIMIT             # ❌ unhydrated default

Path convention:
    ~/.agi/
        aurora.yaml             ← robot-wide settings
        personas/
            persona.yaml        ← active persona    (mutable)
            generic.yaml        ← default reset target (read-only)
        users.yaml              ← all user profiles

TODO:
    — spawn EEC, HRS nodes when those systems mature
    — GenomeEncryption.decrypt() before yaml.safe_load() in _read_yaml()
    — GenomeEncryption.validate_mirror() for backup strand integrity
"""

import subprocess
import time
import yaml

from dataclasses import dataclass, field
from pathlib     import Path
from rclpy.node  import Node
from std_msgs.msg import Bool

import hrs.hrm                            # HRM — static schema + fallback defaults

# ─── Path Convention ──────────────────────────────────────────────────────────

AGI_DIR     = Path.home() / hrs.hrm.ENTITY_GATEWAY / hrs.hrm.SEMANTIC_COGNITIVE_SYSTEM / hrs.hrm.RETICULAR_ACTIVATING_COMPARTMENT
AURORA_CFG  = AGI_DIR / hrs.hrm.SCS.AURORA_SETPOINTS
PERSONA_DIR = AGI_DIR / "personas"
USERS_CFG   = AGI_DIR / hrs.hrm.SCS.USER_PROFILES

# ─── Config Dataclasses ───────────────────────────────────────────────────────
#
#   YAML deserialization layer only — pure structure, no logic.
#   Defaults driven from HRM constants — one source of truth.
#   RAC populates these from YAML and returns them to CNC as AuroraConfig.
#
#   PersonaConfig and UserProfile are runtime objects carried on AuroraConfig
#   for direct node access (e.g. CNC injecting user context into prompts).
#   They are NOT hydrated into AGi — persona inference params go via AGi.SCS.GCE.

@dataclass
class PersonaConfig:
    """Active persona snapshot — built from hydrated AGi.SCS.GCE after persona.yaml loads."""
    system_prompt         : str   = AGi.SCS.GCE.SYSTEM_PROMPT
    cognitive_engine      : str   = AGi.SCS.GCE.COGNITIVE_ENGINE
    response_depth        : int   = AGi.SCS.GCE.RESPONSE_DEPTH
    context_window        : int   = AGi.SCS.GCE.CONTEXT_WINDOW
    temperature           : float = AGi.SCS.GCE.TEMPERATURE
    probability_threshold : float = AGi.SCS.GCE.PROBABILITY_THRESHOLD
    candidate_threshold   : int   = AGi.SCS.GCE.CANDIDATE_THRESHOLD
    perseveration_damping : float = AGi.SCS.GCE.PERSEVERATION_DAMPING
    habituation_damping   : float = AGi.SCS.GCE.HABITUATION_DAMPING
    novelty_bias          : float = AGi.SCS.GCE.NOVELTY_BIAS

@dataclass
class PreferencesSettings:
    """Per-user behavioural preferences — loaded from users.yaml."""
    response_verbosity   : str   = "concise"
    formality            : str   = "casual"
    preferred_language   : str   = "en"
    emotional_tone       : str   = "warm"
    memory_salience_bias : float = 0.0

@dataclass
class UserProfile:
    """User identity and preferences — loaded from users.yaml by id."""
    id          : str                 = "unknown"
    name        : str                 = "unknown"
    known_as    : str                 = "unknown"
    location    : str                 = "unknown"
    notes       : list[str]           = field(default_factory=list)
    preferences : PreferencesSettings = field(default_factory=PreferencesSettings)

@dataclass
class AuroraConfig:
    """
    Runtime config object — returned by RAC.ignite(), carried by CNC for the session.

    AGi class tree is the hydrated constant registry — nodes read from AGi.
    AuroraConfig carries persona and user context not represented in AGi.
    """
    persona : PersonaConfig = field(default_factory=PersonaConfig)
    user    : UserProfile   = field(default_factory=UserProfile)

# ─── Reticular Activation Compartment ────────────────────────────────────────

class RAC:
    """
    Reticular Activation Compartment — plain Python bootloader.

    Called once from CNC.__init__. Not a ROS2 node.
    Hydrates AGi, spawns downstream nodes, returns AuroraConfig to CNC.

    Args:
        cnc (Node): CNC node reference — used for logging and node spawning.
    """

    def __init__(self, cnc: Node):
        self._cnc = cnc                             # CNC node — logger + spawner
        self._log = cnc.get_logger()                # ROS2 logger — stdout + /rosout

    # ─── Public Interface ─────────────────────────────────────────────────────

    def ignite(self) -> AuroraConfig:
        """
        Execute full boot sequence — hydrate AGi and spawn downstream nodes.

        Boot sequence:
            1. Load aurora.yaml  → override AGi static defaults
            2. Load persona.yaml → override AGi.SCS.GCE inference constants
            3. Load users.yaml   → build UserProfile
            4. _derive()         → recompute derived constants
            5. Spawn nodes       → dependency order, each blocks until ready

        Returns:
            AuroraConfig: Hydrated persona snapshot + active user profile.
        """
        self._log.info("=" * 60)
        self._log.info("⚡ RAC — Reticular Activation Compartment igniting…")
        self._log.info("=" * 60)

        self._load_aurora()
        self._load_persona()
        config = self._load_user()
        self._derive()
        self._spawn_nodes()

        self._log.info("✅ RAC boot complete — AGi hydrated, nodes spawned")
        return config

    # ─── Hydration ────────────────────────────────────────────────────────────

    def _load_aurora(self) -> None:
        """
        Load aurora.yaml and override AGi class attributes in place.
        Missing keys fall back to HRM static defaults already set in AGi.
        Required sections must be present — missing section raises RuntimeError.
        """
        raw = self._read_yaml(AURORA_CFG)

        self._require_section(raw, AGi.SEMANTIC_COGNITIVE_SYSTEM,     AGi.AURORA_SETPOINTS)
        self._require_section(raw, AGi.HOMEOSTATIC_REGULATION_SYSTEM, AGi.AURORA_SETPOINTS)

        # ── Boot settings ─────────────────────────────────────────────────────
        # boot_timeout and active_persona/user live as instance attrs — not in AGi
        boot              = raw.get("boot", {})
        self._boot_timeout   = boot.get("boot_timeout",   10.0)
        self._active_persona = self._require_key(boot, "active_persona", "boot")
        self._active_user    = self._require_key(boot, "active_user",    "boot")

        # ── SCS ───────────────────────────────────────────────────────────────
        scs = raw[AGi.SEMANTIC_COGNITIVE_SYSTEM]
        AGi.SCS.CORTICAL_CAPACITY = scs.get("cortical_capacity", AGi.SCS.CORTICAL_CAPACITY)
        AGi.SCS.COGNITIVE_RESERVE = scs.get("cognitive_reserve", AGi.SCS.COGNITIVE_RESERVE)

        # ── EMC ───────────────────────────────────────────────────────────────
        emc = scs.get(AGi.EPISODIC_MEMORY_CORTEX, {})
        AGi.SCS.EMC.BINDING_STREAM_LIMIT     = emc.get("binding_stream_limit",     AGi.SCS.EMC.BINDING_STREAM_LIMIT)
        AGi.SCS.EMC.ENCODING_CYCLE_TIMEOUT   = emc.get("encoding_cycle_timeout",   AGi.SCS.EMC.ENCODING_CYCLE_TIMEOUT)
        AGi.SCS.EMC.ENCODING_PRIME_CAPACITY  = emc.get("encoding_prime_capacity",  AGi.SCS.EMC.ENCODING_PRIME_CAPACITY)
        AGi.SCS.EMC.ENCODING_PRIME_KEY_LIMIT = emc.get("encoding_prime_key_limit", AGi.SCS.EMC.ENCODING_PRIME_KEY_LIMIT)
        AGi.SCS.EMC.EPISODE_CONTENT_LIMIT    = emc.get("episode_content_limit",    AGi.SCS.EMC.EPISODE_CONTENT_LIMIT)
        AGi.SCS.EMC.THETA_INTERVAL           = emc.get("theta_interval",           AGi.SCS.EMC.THETA_INTERVAL)
        AGi.SCS.EMC.THETA_BATCH_LIMIT        = emc.get("theta_batch_limit",        AGi.SCS.EMC.THETA_BATCH_LIMIT)
        AGi.SCS.EMC.RECALL_RESERVE           = emc.get("recall_reserve",           AGi.SCS.EMC.RECALL_RESERVE)
        AGi.SCS.EMC.RECALL_SURFACE_LIMIT     = emc.get("recall_surface_limit",     AGi.SCS.EMC.RECALL_SURFACE_LIMIT)
        AGi.SCS.EMC.RECALL_POOL              = emc.get("recall_pool",              AGi.SCS.EMC.RECALL_POOL)
        AGi.SCS.EMC.RECALL_TIMEOUT           = emc.get("recall_timeout",           AGi.SCS.EMC.RECALL_TIMEOUT)
        AGi.SCS.EMC.RECOVERY_BATCH_SIZE      = emc.get("recovery_batch_size",      AGi.SCS.EMC.RECOVERY_BATCH_SIZE)
        AGi.SCS.EMC.RELEVANCE_THRESHOLD      = emc.get("relevance_threshold",      AGi.SCS.EMC.RELEVANCE_THRESHOLD)

        # ── WMC ───────────────────────────────────────────────────────────────
        wmc = scs.get(AGi.WORKING_MEMORY_CORTEX, {})
        AGi.SCS.WMC.PMT_SLOT_LIMIT  = wmc.get("pmt_slot_limit",  AGi.SCS.WMC.PMT_SLOT_LIMIT)
        AGi.SCS.WMC.PMT_SLOT_BUFFER = wmc.get("pmt_slot_buffer", AGi.SCS.WMC.PMT_SLOT_BUFFER)

        # ── HRS (stub) ────────────────────────────────────────────────────────
        # AGi.HRS.* — expand when HRS matures

        # ── SDS (stub) ────────────────────────────────────────────────────────
        # AGi.SDS.* — expand when SDS matures

        self._log.info("✅ aurora.yaml loaded")

    def _load_persona(self) -> None:
        """
        Load active persona from ~/.agi/personas/<stem>.yaml.
        Overrides AGi.SCS.GCE inference constants in place.
        Raises RuntimeError if file or any required key is missing.
        generic.yaml is a read-only reset target — never loaded at runtime.
        """
        path = PERSONA_DIR / f"{self._active_persona}.yaml"
        raw  = self._read_yaml(path)

        AGi.SCS.GCE.COGNITIVE_ENGINE      = self._require_key(raw, "cognitive_engine",      path.name)
        AGi.SCS.GCE.RESPONSE_DEPTH        = self._require_key(raw, "response_depth",        path.name)
        AGi.SCS.GCE.CONTEXT_WINDOW        = self._require_key(raw, "context_window",        path.name)
        AGi.SCS.GCE.TEMPERATURE           = self._require_key(raw, "temperature",           path.name)
        AGi.SCS.GCE.PROBABILITY_THRESHOLD = self._require_key(raw, "probability_threshold", path.name)
        AGi.SCS.GCE.CANDIDATE_THRESHOLD   = self._require_key(raw, "candidate_threshold",   path.name)
        AGi.SCS.GCE.PERSEVERATION_DAMPING = self._require_key(raw, "perseveration_damping", path.name)
        AGi.SCS.GCE.HABITUATION_DAMPING   = self._require_key(raw, "habituation_damping",   path.name)
        AGi.SCS.GCE.NOVELTY_BIAS          = self._require_key(raw, "novelty_bias",          path.name)
        AGi.SCS.GCE.SYSTEM_PROMPT         = self._require_key(raw, "system_prompt",         path.name)

        self._log.info(
            f"✅ Persona loaded — {self._active_persona} "
            f"| engine: {AGi.SCS.GCE.COGNITIVE_ENGINE}"
        )

    def _load_user(self) -> AuroraConfig:
        """
        Load user profile + preferences from ~/.agi/users.yaml by id.
        Returns AuroraConfig carrying persona snapshot + user profile.
        Raises RuntimeError if file is missing or user_id is not found.
        """
        raw   = self._read_yaml(USERS_CFG)
        users = raw.get("users", [])
        entry = next((u for u in users if u.get("id") == self._active_user), None)

        if not entry:
            raise RuntimeError(
                f"❌ RAC boot failed — user '{self._active_user}' not found in {AGi.USER_PROFILES}"
            )

        prefs = self._require_key(entry, "preferences", f"user:{self._active_user}")

        user = UserProfile(
            id       = self._require_key(entry, "id",       f"user:{self._active_user}"),
            name     = self._require_key(entry, "name",     f"user:{self._active_user}"),
            known_as = self._require_key(entry, "known_as", f"user:{self._active_user}"),
            location = self._require_key(entry, "location", f"user:{self._active_user}"),
            notes    = entry.get("notes", []),
            preferences = PreferencesSettings(
                response_verbosity   = self._require_key(prefs, "response_verbosity",   f"user:{self._active_user}.preferences"),
                formality            = self._require_key(prefs, "formality",            f"user:{self._active_user}.preferences"),
                preferred_language   = self._require_key(prefs, "preferred_language",   f"user:{self._active_user}.preferences"),
                emotional_tone       = self._require_key(prefs, "emotional_tone",       f"user:{self._active_user}.preferences"),
                memory_salience_bias = self._require_key(prefs, "memory_salience_bias", f"user:{self._active_user}.preferences"),
            ),
        )

        # Persona snapshot — built from now-hydrated AGi.SCS.GCE values
        persona = PersonaConfig(
            system_prompt         = AGi.SCS.GCE.SYSTEM_PROMPT,
            cognitive_engine      = AGi.SCS.GCE.COGNITIVE_ENGINE,
            response_depth        = AGi.SCS.GCE.RESPONSE_DEPTH,
            context_window        = AGi.SCS.GCE.CONTEXT_WINDOW,
            temperature           = AGi.SCS.GCE.TEMPERATURE,
            probability_threshold = AGi.SCS.GCE.PROBABILITY_THRESHOLD,
            candidate_threshold   = AGi.SCS.GCE.CANDIDATE_THRESHOLD,
            perseveration_damping = AGi.SCS.GCE.PERSEVERATION_DAMPING,
            habituation_damping   = AGi.SCS.GCE.HABITUATION_DAMPING,
            novelty_bias          = AGi.SCS.GCE.NOVELTY_BIAS,
        )

        self._log.info(
            f"✅ User loaded — {user.known_as} ({user.id}) | {user.location}"
        )

        return AuroraConfig(persona=persona, user=user)

    def _derive(self) -> None:
        """
        Recompute derived constants after hydration.
        Metaclass properties (GLOBAL_CHUNK_LIMIT, RECALL_DEPTH) self-compute —
        nothing to do here yet. Expand as derived constants grow.
        """
        self._log.info(
            f"✅ AGi hydrated "
            f"| cortical capacity: {AGi.SCS.CORTICAL_CAPACITY} tokens "
            f"| recall depth: {AGi.SCS.EMC.RECALL_DEPTH} candidates"
        )

    # ─── Node Spawning ────────────────────────────────────────────────────────

    def _spawn_nodes(self) -> None:
        """
        Spawn downstream ROS2 nodes in dependency order.
        Each stage blocks until the node signals ready before proceeding.
        CNC itself is already running — only downstream nodes spawned here.

        Boot order:
            Stage 1 — Infrastructure : EEC  (stub — commented until ready)
            Stage 2 — Regulatory     : HRS  (stub — commented until ready)
            Stage 3 — Cognition      : (CNC is the caller — already running)
        """
        # Stage 1 — Infrastructure
        # self._spawn(AGi.EMERGENCY_EXCEPTION_CORE)
        # self._wait_ready(f"/{AGi.EMERGENCY_EXCEPTION_CORE}/ready", self._boot_timeout)

        # Stage 2 — Regulatory
        # self._spawn(AGi.HOMEOSTATIC_REGULATION_SYSTEM)
        # self._wait_ready(f"/{AGi.HOMEOSTATIC_REGULATION_SYSTEM}/ready", self._boot_timeout)

        pass                                        # no downstream nodes active yet — expand per milestone

    def _spawn(self, node: str) -> None:
        """Spawn a ROS2 node as a subprocess."""
        self._log.info(f"⚡ Spawning {node.upper()}…")
        subprocess.Popen(
            ["ros2", "run", "aurora", node],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def _wait_ready(self, topic: str, timeout: float) -> None:
        """
        Block until a ready signal arrives on the given topic.
        Logs a warning and continues if timeout is exceeded —
        non-critical nodes must not stall the full boot sequence.
        """
        import rclpy
        self._log.info(f"⏳ Waiting for {topic}…")
        ready    = False
        deadline = time.time() + timeout

        def _cb(msg: Bool):
            nonlocal ready
            if msg.data:
                ready = True

        sub = self._cnc.create_subscription(Bool, topic, _cb, 1)
        while not ready and time.time() < deadline:
            rclpy.spin_once(self._cnc, timeout_sec=0.1)

        self._cnc.destroy_subscription(sub)

        if ready:
            self._log.info(f"✅ {topic} ready")
        else:
            self._log.warning(f"⚠️  {topic} timed out after {timeout}s — continuing")

    # ─── Validation Helpers ───────────────────────────────────────────────────

    def _require_section(self, raw: dict, key: str, source: str) -> None:
        """Raise RuntimeError if a required top-level section is missing from YAML."""
        if not raw or key not in raw:
            raise RuntimeError(
                f"❌ RAC boot failed — required section '{key}' missing from {source}"
            )

    def _require_key(self, raw: dict, key: str, source: str):
        """Raise RuntimeError if a required key is missing from a YAML section."""
        if raw is None or key not in raw:
            raise RuntimeError(
                f"❌ RAC boot failed — required key '{key}' missing from [{source}]"
            )
        return raw[key]

    # ─── File I/O ─────────────────────────────────────────────────────────────

    def _read_yaml(self, path: Path) -> dict:
        """
        Read and parse a YAML file.
        Raises RuntimeError if the file does not exist or cannot be parsed.
        TODO: call GenomeEncryption.decrypt() before yaml.safe_load()
        TODO: call GenomeEncryption.validate_mirror() for backup strand
        """
        if not path.exists():
            raise RuntimeError(
                f"❌ RAC boot failed — required config file not found: {path}"
            )
        try:
            with open(path, "r") as f:
                return yaml.safe_load(f)
        except Exception as e:
            raise RuntimeError(
                f"❌ RAC boot failed — could not parse {path.name}: {e}"
            )