"""
ARC — Arousal Reaction Core
========================================
AuRoRA · Reticular Activating System (RAS)

Bootloader and central config registry for the AuRoRA CNS.
Loads aurora.yaml + active persona/user at startup, hydrates the AGi
class tree, spawns all cognitive and regulatory nodes in dependency
order, and publishes a ready signal before any other node begins
processing.

Boot order:
    ARC → EEC → HRS → CNC

Design contract:
    - HRP is the static schema and fallback defaults.
    - ARC loads YAML and overrides AGi class attributes in place.
    - AGi is re-exported from arc so all other modules do:
          from arc import AGi
      and get fully hydrated runtime values — one import change, nothing else.
    - No node spawns until ARC has fully loaded, validated, and hydrated AGi.
    - Boot order guarantees all child processes import arc after hydration.
    - Read AGi constants inside __init__ or methods, never at module level.

Usage (all other modules):
    from arc import AGi

    EMC          = AGi.SCS.EMC
    recall_depth = AGi.SCS.EMC.RECALL_DEPTH   # hydrated runtime value
    recall_depth = EMC.RECALL_DEPTH            # same, shorthand alias

TODO: DNA genome encryption — dual-helix primary + mirror backup
"""

import rclpy
import subprocess
import time
import yaml

from rclpy.node     import Node
from std_msgs.msg   import Bool
from pathlib        import Path
from dataclasses    import dataclass, field

from hrp            import AGi                  # ARC owns HRP — hydrates and re-exports it

# ─── Path Convention ──────────────────────────────────────────────────────────
#
#   ~/.agi/
#       aurora.yaml             ← robot-wide settings         (AGi.AURORA_SETPOINTS)
#       personas/
#           persona.yaml        ← active persona              (AGi.PERSONA_ACTIVE)
#           generic.yaml        ← default reset target        (AGi.PERSONA_GENERIC)
#       users.yaml              ← all user profiles           (AGi.USER_PROFILES)
#       scs/
#           emc/
#               engram_complex.db

AGI_DIR     = Path.home() / AGi.ENTITY_GATEWAY
AURORA_CFG  = AGI_DIR / AGi.AURORA_SETPOINTS
PERSONA_DIR = AGI_DIR / "personas"
USERS_CFG   = AGI_DIR / AGi.USER_PROFILES

# ─── Dataclasses ──────────────────────────────────────────────────────────────
#
#   YAML deserialization layer only — pure structure, no logic.
#   Defaults are driven from HRP constants so there is one source of truth.
#   ARC populates these from YAML, then writes final values back into AGi.
#
#   PersonaConfig and UserProfile are runtime objects carried on AuroraConfig
#   for direct access by nodes that need persona/user context (e.g. CNC).
#   They are NOT hydrated into AGi — persona inference params go via AGi.SCS.GCE.

@dataclass
class PersonaConfig:
    """Active persona — loaded from ~/.agi/personas/<stem>.yaml."""
    system_prompt:         str   = AGi.SCS.GCE.SYSTEM_PROMPT
    cognitive_engine:      str   = AGi.SCS.GCE.COGNITIVE_ENGINE
    response_depth:        int   = AGi.SCS.GCE.RESPONSE_DEPTH
    context_window:        int   = AGi.SCS.GCE.CONTEXT_WINDOW
    temperature:           float = AGi.SCS.GCE.TEMPERATURE
    probability_threshold: float = AGi.SCS.GCE.PROBABILITY_THRESHOLD
    candidate_threshold:   int   = AGi.SCS.GCE.CANDIDATE_THRESHOLD
    perseveration_damping: float = AGi.SCS.GCE.PERSEVERATION_DAMPING
    habituation_damping:   float = AGi.SCS.GCE.HABITUATION_DAMPING
    novelty_bias:          float = AGi.SCS.GCE.NOVELTY_BIAS

@dataclass
class PreferencesSettings:
    """Per-user behavioural preferences — loaded from users.yaml."""
    response_verbosity:   str   = "concise"
    formality:            str   = "casual"
    preferred_language:   str   = "en"
    emotional_tone:       str   = "warm"
    memory_salience_bias: float = 0.0

@dataclass
class UserProfile:
    """User identity and preferences — loaded from users.yaml by id."""
    id:          str                 = "unknown"
    name:        str                 = "unknown"
    known_as:    str                 = "unknown"
    location:    str                 = "unknown"
    notes:       list[str]           = field(default_factory=list)
    preferences: PreferencesSettings = field(default_factory=PreferencesSettings)

@dataclass
class AuroraConfig:
    """
    Runtime config object — ARC loads once, carried for direct node access.
    AGi class tree is the hydrated constant registry — nodes read from AGi.
    AuroraConfig carries persona and user context not represented in AGi.
    """
    persona: PersonaConfig = field(default_factory=PersonaConfig)
    user:    UserProfile   = field(default_factory=UserProfile)

# ─── Encryption Interface (stub) ──────────────────────────────────────────────

class GenomeEncryption:
    """
    DNA genome encryption — stub for future implementation.

    Architecture intent:
        Primary helix  : encrypted config payload
        Mirror helix   : complementary backup/validation strand
        Decryption key : derived from device identity or hardware token

    When implemented, _read_yaml() calls decrypt() before yaml.safe_load()
    and validate_mirror() confirms backup strand integrity.
    Files on disk will be opaque — only ARC can read them.
    """

    @staticmethod
    def encrypt(data: str) -> bytes:
        raise NotImplementedError

    @staticmethod
    def decrypt(data: bytes) -> str:
        raise NotImplementedError

    @staticmethod
    def validate_mirror(primary: bytes, mirror: bytes) -> bool:
        raise NotImplementedError

# ─── ARC Node ─────────────────────────────────────────────────────────────────

class ArousedReactionCore(Node):
    """
    ARC — Arousal Reaction Core

    Bootloader and config registry for the AuRoRA CNS.

    Boot sequence:
        1. Load aurora.yaml  → override AGi static defaults
        2. Load persona.yaml → override AGi.SCS.GCE constants
        3. Load users.yaml   → populate AuroraConfig.user
        4. _derive()         → recompute derived constants in AGi
        5. AGi is now fully hydrated — all child processes will import it correctly
        6. Spawn nodes in dependency order

    All other modules:
        from arc import AGi     ← one line change from: from hrp import AGi
        AGi.SCS.EMC.RECALL_DEPTH  ← hydrated runtime value

    Topic: /ras/ready — published True when full boot sequence completes.
    """

    def __init__(self):
        super().__init__(AGi.RETICULAR_ACTIVATING_SYSTEM)

        # ── Boot sequence ─────────────────────────────────────────────────────
        # Order matters — hydrate AGi completely before spawning any node
        self._load_aurora()
        self._load_persona()
        self.config = self._load_user()
        self._derive()
        self._boot()

    # ─── Config Loading ───────────────────────────────────────────────────────

    def _load_aurora(self) -> None:
        """
        Load aurora.yaml and override AGi class attributes.
        Missing keys fall back to HRP static defaults already set in AGi.
        Required sections must be present — missing section raises RuntimeError.
        """
        raw = self._read_yaml(AURORA_CFG)

        self._require_section(raw, AGi.RETICULAR_ACTIVATING_SYSTEM,   AGi.AURORA_SETPOINTS)
        self._require_section(raw, AGi.SEMANTIC_COGNITIVE_SYSTEM,      AGi.AURORA_SETPOINTS)
        self._require_section(raw, AGi.HOMEOSTATIC_REGULATION_SYSTEM,  AGi.AURORA_SETPOINTS)
        self._require_section(raw, AGi.VULNERABILITY_DETECTION_SYSTEM, AGi.AURORA_SETPOINTS)

        ras = raw[AGi.RETICULAR_ACTIVATING_SYSTEM]
        scs = raw[AGi.SEMANTIC_COGNITIVE_SYSTEM]
        emc = scs.get(AGi.EPISODIC_MEMORY_CORTEX, {})
        wmc = scs.get(AGi.WORKING_MEMORY_CORTEX,  {})

        # ── RAS ───────────────────────────────────────────────────────────────
        # boot_timeout, active_persona, active_user carried as instance attrs
        # (not in AGi — RAS has no HRP class equivalent yet)
        self._boot_timeout   = ras.get("boot_timeout",   10.0)
        self._active_persona = self._require_key(ras, "active_persona", AGi.RETICULAR_ACTIVATING_SYSTEM)
        self._active_user    = self._require_key(ras, "active_user",    AGi.RETICULAR_ACTIVATING_SYSTEM)

        # ── SCS ───────────────────────────────────────────────────────────────
        AGi.SCS.CORTICAL_CAPACITY = scs.get("cortical_capacity", AGi.SCS.CORTICAL_CAPACITY)
        AGi.SCS.COGNITIVE_RESERVE = scs.get("cognitive_reserve", AGi.SCS.COGNITIVE_RESERVE)

        # ── EMC ───────────────────────────────────────────────────────────────
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
        AGi.SCS.WMC.PMT_SLOT_LIMIT  = wmc.get("pmt_slot_limit",  AGi.SCS.WMC.PMT_SLOT_LIMIT)
        AGi.SCS.WMC.PMT_SLOT_BUFFER = wmc.get("pmt_slot_buffer", AGi.SCS.WMC.PMT_SLOT_BUFFER)

        # ── HRS (stub) ────────────────────────────────────────────────────────
        # AGi.HRS.* — expand when HRS matures

        # ── VDS (stub) ────────────────────────────────────────────────────────
        # AGi.VDS.* — expand when VDS matures

        self.get_logger().info("✅ aurora.yaml loaded")

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

        self.get_logger().info(
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
                f"❌ ARC boot failed — user '{self._active_user}' not found in {AGi.USER_PROFILES}"
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

        self.get_logger().info(
            f"✅ User loaded — {user.known_as} ({user.id}) | {user.location}"
        )

        return AuroraConfig(persona=persona, user=user)

    def _derive(self) -> None:
        """
        Recompute [DERIVED] AGi constants from hydrated intrinsic values.
        Called after all YAML is loaded so all inputs are final.
        """
        AGi.SCS.EMC.RECALL_DEPTH = (
            AGi.SCS.EMC.RECALL_SURFACE_LIMIT
            * AGi.SCS.EMC.RECALL_POOL
        )
        AGi.SCS.WMC.GLOBAL_CHUNK_LIMIT = (
            AGi.SCS.CORTICAL_CAPACITY
            - AGi.SCS.COGNITIVE_RESERVE
            - AGi.SCS.EMC.RECALL_RESERVE
        )
        self.get_logger().info(
            f"✅ AGi hydrated — "
            f"RECALL_DEPTH={AGi.SCS.EMC.RECALL_DEPTH} | "
            f"GLOBAL_CHUNK_LIMIT={AGi.SCS.WMC.GLOBAL_CHUNK_LIMIT}"
        )

    # ─── Boot Sequence ────────────────────────────────────────────────────────

    def _boot(self) -> None:
        """
        Spawn cognitive nodes in dependency order.
        AGi is fully hydrated before any node is spawned — boot order is the guarantee.
        Each stage blocks until the node signals ready before proceeding.
        """
        ready_pub = self.create_publisher(
            Bool,
            f"/{AGi.RETICULAR_ACTIVATING_SYSTEM}/ready",
            1,
        )

        # Stage 1 — Infrastructure
        # self._spawn(AGi.EMERGENCY_EXCEPTION_CORE)
        # self._wait_ready(f"/{AGi.EMERGENCY_EXCEPTION_CORE}/ready", self._boot_timeout)

        # Stage 2 — Regulatory
        # self._spawn(AGi.HOMEOSTATIC_REGULATION_SYSTEM)
        # self._wait_ready(f"/{AGi.HOMEOSTATIC_REGULATION_SYSTEM}/ready", self._boot_timeout)

        # Stage 3 — Cognition
        self._spawn(AGi.CENTRAL_NERVOUS_CORE)
        self._wait_ready(f"/{AGi.CENTRAL_NERVOUS_CORE}/ready", self._boot_timeout)

        msg      = Bool()
        msg.data = True
        ready_pub.publish(msg)
        self.get_logger().info("🧠 AuRoRA CNS online")

    def _spawn(self, node: str) -> None:
        """Spawn a ROS node as a subprocess."""
        self.get_logger().info(f"⚡ Spawning {node.upper()}...")
        subprocess.Popen(
            ["ros2", "run", "aurora", node],
            stdout = subprocess.DEVNULL,
            stderr = subprocess.DEVNULL,
        )

    def _wait_ready(self, topic: str, timeout: float) -> None:
        """
        Block until a ready signal arrives on the given topic.
        Logs a warning and continues if timeout is exceeded —
        non-critical nodes should not stall the full boot sequence.
        """
        self.get_logger().info(f"⏳ Waiting for {topic}...")
        ready    = False
        deadline = time.time() + timeout

        def _cb(msg: Bool):
            nonlocal ready
            if msg.data:
                ready = True

        sub = self.create_subscription(Bool, topic, _cb, 1)
        while not ready and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.destroy_subscription(sub)

        if ready:
            self.get_logger().info(f"✅ {topic} ready")
        else:
            self.get_logger().warning(f"⚠️  {topic} timed out after {timeout}s — continuing")

    # ─── Validation Helpers ───────────────────────────────────────────────────

    def _require_section(self, raw: dict, key: str, source: str) -> None:
        """Raise RuntimeError if a required top-level section is missing from YAML."""
        if not raw or key not in raw:
            raise RuntimeError(
                f"❌ ARC boot failed — required section '{key}' missing from {source}"
            )

    def _require_key(self, raw: dict, key: str, source: str):
        """Raise RuntimeError if a required key is missing from a YAML section."""
        if raw is None or key not in raw:
            raise RuntimeError(
                f"❌ ARC boot failed — required key '{key}' missing from [{source}]"
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
                f"❌ ARC boot failed — required config file not found: {path}"
            )
        try:
            with open(path, "r") as f:
                return yaml.safe_load(f)
        except Exception as e:
            raise RuntimeError(
                f"❌ ARC boot failed — could not parse {path.name}: {e}"
            )

# ─── Entry Point ──────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ArousedReactionCore()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
