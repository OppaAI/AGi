"""
ARC — Arousal Reaction Core
========================================
AuRoRA · Reticular Activating System (RAS)

Bootloader and central config registry for the AuRoRA CNS.
Loads aurora.yaml + active persona/user at startup, spawns all
cognitive and regulatory nodes in dependency order, and publishes
a ready signal before any other node begins processing.

Boot order:
    ARC → EEC → HRS → EMC → MCC → CNC

Design contract:
    - HRP is the single source of truth for all constants and structure.
    - ARC owns YAML deserialization only — no defaults live here.
    - If any required config is missing or unreadable, ARC raises and halts.
    - No node spawns until ARC has fully loaded, validated, and hydrated config.
    - Fail loud, fail early, fail at ARC — never silently downstream.

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

from hrp            import AGi

# ─── Path Convention ──────────────────────────────────────────────────────────
#
#   ~/.agi/
#       aurora.yaml             ← robot-wide settings  (AGi.AURORA_SETPOINTS)
#       personas/
#           persona.yaml        ← active persona + inference params  (AGi.PERSONA_ACTIVE)
#           generic.yaml        ← default persona reset target       (AGi.PERSONA_GENERIC)
#       users.yaml              ← all user profiles + per-user extrinsic settings  (AGi.USER_PROFILES)
#       scs/
#           emc/
#               engram_complex.db

AGI_DIR     = Path.home() / AGi.ENTITY_GATEWAY
AURORA_CFG  = AGI_DIR / AGi.AURORA_SETPOINTS
PERSONA_DIR = AGI_DIR / "personas"
USERS_CFG   = AGI_DIR / AGi.USER_PROFILES

# ─── Dataclasses ──────────────────────────────────────────────────────────────
#
#   Pure structure — no defaults. Every field must be explicitly populated
#   by ARC during config load. Missing values are config errors, not
#   opportunities for silent substitution.
#
#   Type annotations are documentation + static analysis only.
#   HRP is the authoritative source for what values are valid.

@dataclass
class PersonaConfig:
    system_prompt:         str
    cognitive_engine:      str
    response_depth:        int
    context_window:        int
    temperature:           float
    probability_threshold: float
    candidate_threshold:   int
    perseveration_damping: float
    habituation_damping:   float
    novelty_bias:          float

@dataclass
class ExtrinsicSettings:
    """Per-user behavioural preferences — loaded from users.yaml alongside the user profile."""
    response_verbosity:   str
    formality:            str
    preferred_language:   str
    emotional_tone:       str
    memory_salience_bias: float

@dataclass
class UserProfile:
    id:        str
    name:      str
    known_as:  str
    location:  str
    notes:     list[str]
    extrinsic: ExtrinsicSettings

@dataclass
class EMCSettings:
    binding_stream_limit:    int
    encoding_cycle_timeout:  float
    encoding_prime_capacity: int
    encoding_prime_key_limit: int
    episode_content_limit:   int
    theta_interval:          float
    theta_batch_limit:       int
    recall_reserve:          int
    recall_surface_limit:    int
    recall_pool:             int
    recall_depth:            int   # [DERIVED] — computed by _derive(), not read from YAML
    recall_timeout:          float
    recovery_batch_size:     int
    relevance_threshold:     float

@dataclass
class WMCSettings:
    pmt_slot_limit:     int
    pmt_slot_buffer:    int
    global_chunk_limit: int   # [DERIVED] — computed by _derive(), not read from YAML

@dataclass
class GCESettings:
    cognitive_engine:      str
    response_depth:        int
    context_window:        int
    temperature:           float
    probability_threshold: float
    candidate_threshold:   int
    perseveration_damping: float
    habituation_damping:   float
    novelty_bias:          float

@dataclass
class SCSSettings:
    cortical_capacity:   int
    cognitive_reserve:   int
    induction_threshold: float   # placeholder — M1.5
    eviction_threshold:  float   # placeholder — M1.5
    salience_hard_gate:  float   # placeholder — M1.5
    boundary_threshold:  float   # placeholder — M1.5
    emc: EMCSettings
    wmc: WMCSettings
    gce: GCESettings

@dataclass
class HRSSettings:
    """Stub — expand as HRS matures."""
    watchdog_interval: float
    thermal_limit:     float
    battery_critical:  float

@dataclass
class SDSSettings:
    """Stub — expand as SDS matures."""
    min_obstacle_distance: float
    emergency_stop_dist:   float

@dataclass
class RASSettings:
    boot_timeout:   float
    active_persona: str   # filename stem — resolves to AGi.PERSONA_ACTIVE
    active_user:    str   # user id — loaded from AGi.USER_PROFILES

@dataclass
class AuroraConfig:
    """Single robot-wide config object — ARC loads once, all nodes read from it."""
    ras:     RASSettings
    scs:     SCSSettings
    hrs:     HRSSettings
    sds:     SDSSettings
    persona: PersonaConfig
    user:    UserProfile

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
    Loads aurora.yaml + active persona and user at startup.
    Spawns all nodes in dependency order, waits for each ready signal
    before proceeding to the next stage.

    Exposes a single AuroraConfig object — all other nodes import
    ARC and read config from it. No node does its own file I/O.

    Contract:
        _load_config() must succeed completely before _hydrate() or _boot() run.
        Any missing required key raises ConfigError — robot does not boot.

    Topic: /ras/ready — published True when full boot sequence completes.
    """

    def __init__(self):
        super().__init__(AGi.RETICULAR_ACTIVATING_SYSTEM)

        # ── Boot sequence — config must be fully loaded before anything spawns ─
        self.config: AuroraConfig = self._load_config()
        self._hydrate()
        self._boot()

    # ─── Config Loading ───────────────────────────────────────────────────────

    def _load_config(self) -> AuroraConfig:
        """
        Load and validate the complete robot config.
        Raises RuntimeError if any required file or key is missing.
        Returns a fully populated AuroraConfig — no partial states.
        """
        raw = self._read_yaml(AURORA_CFG)
        ras, scs, hrs, sds = self._parse_aurora(raw)

        # Derive computed constants from parsed intrinsic values
        scs = self._derive(scs)

        # Persona and user loaded after RAS so active_persona/active_user are known
        persona = self._load_persona(ras.active_persona)
        user    = self._load_user(ras.active_user)

        # Sync persona inference params into scs.gce so _hydrate() pushes them into AGi.SCS.GCE
        gce = GCESettings(
            cognitive_engine      = persona.cognitive_engine,
            response_depth        = persona.response_depth,
            context_window        = persona.context_window,
            temperature           = persona.temperature,
            probability_threshold = persona.probability_threshold,
            candidate_threshold   = persona.candidate_threshold,
            perseveration_damping = persona.perseveration_damping,
            habituation_damping   = persona.habituation_damping,
            novelty_bias          = persona.novelty_bias,
        )

        # Replace stub GCE with persona-sourced GCE
        scs = SCSSettings(
            cortical_capacity   = scs.cortical_capacity,
            cognitive_reserve   = scs.cognitive_reserve,
            induction_threshold = scs.induction_threshold,
            eviction_threshold  = scs.eviction_threshold,
            salience_hard_gate  = scs.salience_hard_gate,
            boundary_threshold  = scs.boundary_threshold,
            emc                 = scs.emc,
            wmc                 = scs.wmc,
            gce                 = gce,
        )

        self.get_logger().info(
            f"✅ Config loaded — persona: {ras.active_persona} "
            f"| user: {ras.active_user}"
        )

        return AuroraConfig(
            ras     = ras,
            scs     = scs,
            hrs     = hrs,
            sds     = sds,
            persona = persona,
            user    = user,
        )

    def _derive(self, scs: SCSSettings) -> SCSSettings:
        """
        Recompute [DERIVED] constants from parsed intrinsic values.
        Called after _parse_aurora so all inputs are already loaded from YAML.
        Mirrors arc._derive() contract in hrp.py — keep in sync.
        """
        scs.emc.recall_depth = (
            scs.emc.recall_surface_limit
            * scs.emc.recall_pool
        )
        scs.wmc.global_chunk_limit = (
            scs.cortical_capacity
            - scs.cognitive_reserve
            - scs.emc.recall_reserve
        )
        return scs

    def _hydrate(self) -> None:
        """
        Hydrate AGi.HRP class tree from loaded AuroraConfig.
        Maps nested config dataclass attributes to AGi class constants by name.
        Only UPPER_CASE attributes are written — private and dunder skipped.
        """
        section_map = {
            AGi.SEMANTIC_COGNITIVE_SYSTEM   : (self.config.scs,     AGi.SCS),
            AGi.EPISODIC_MEMORY_CORTEX      : (self.config.scs.emc, AGi.SCS.EMC),
            AGi.WORKING_MEMORY_CORTEX       : (self.config.scs.wmc, AGi.SCS.WMC),
            AGi.GENERATIVE_COGNITIVE_ENGINE : (self.config.scs.gce, AGi.SCS.GCE),
        }

        for section_key, (config_section, target_class) in section_map.items():
            for attr_name in vars(config_section):
                if attr_name.startswith("_"):
                    continue
                hrp_key = attr_name.upper()
                if hasattr(target_class, hrp_key):
                    setattr(target_class, hrp_key, getattr(config_section, attr_name))
                    self.get_logger().debug(
                        f"Hydrated {section_key}.{hrp_key} = {getattr(config_section, attr_name)}"
                    )

    def _parse_aurora(self, raw: dict) -> tuple[RASSettings, SCSSettings, HRSSettings, SDSSettings]:
        """
        Parse aurora.yaml into config dataclasses.
        Raises RuntimeError on any missing required section or key.
        No defaults — every value must be present in YAML.
        """
        self._require_section(raw, AGi.RETICULAR_ACTIVATING_SYSTEM, AGi.AURORA_SETPOINTS)
        self._require_section(raw, AGi.SEMANTIC_COGNITIVE_SYSTEM,    AGi.AURORA_SETPOINTS)
        self._require_section(raw, AGi.HOMEOSTATIC_REGULATION_SYSTEM, AGi.AURORA_SETPOINTS)
        self._require_section(raw, AGi.SELF_DEFENSE_SYSTEM, AGi.AURORA_SETPOINTS)

        ras_raw = raw[AGi.RETICULAR_ACTIVATING_SYSTEM]
        scs_raw = raw[AGi.SEMANTIC_COGNITIVE_SYSTEM]
        emc_raw = scs_raw.get(AGi.EPISODIC_MEMORY_CORTEX, {})
        wmc_raw = scs_raw.get(AGi.WORKING_MEMORY_CORTEX,  {})
        hrs_raw = raw[AGi.HOMEOSTATIC_REGULATION_SYSTEM]
        sds_raw = raw[AGi.SELF_DEFENSE_SYSTEM]

        ras = RASSettings(
            boot_timeout   = self._require_key(ras_raw, "boot_timeout",   AGi.RETICULAR_ACTIVATING_SYSTEM),
            active_persona = self._require_key(ras_raw, "active_persona", AGi.RETICULAR_ACTIVATING_SYSTEM),
            active_user    = self._require_key(ras_raw, "active_user",    AGi.RETICULAR_ACTIVATING_SYSTEM),
        )

        emc = EMCSettings(
            binding_stream_limit     = self._require_key(emc_raw, "binding_stream_limit",     AGi.EPISODIC_MEMORY_CORTEX),
            encoding_cycle_timeout   = self._require_key(emc_raw, "encoding_cycle_timeout",   AGi.EPISODIC_MEMORY_CORTEX),
            encoding_prime_capacity  = self._require_key(emc_raw, "encoding_prime_capacity",  AGi.EPISODIC_MEMORY_CORTEX),
            encoding_prime_key_limit = self._require_key(emc_raw, "encoding_prime_key_limit", AGi.EPISODIC_MEMORY_CORTEX),
            episode_content_limit    = self._require_key(emc_raw, "episode_content_limit",    AGi.EPISODIC_MEMORY_CORTEX),
            theta_interval           = self._require_key(emc_raw, "theta_interval",           AGi.EPISODIC_MEMORY_CORTEX),
            theta_batch_limit        = self._require_key(emc_raw, "theta_batch_limit",        AGi.EPISODIC_MEMORY_CORTEX),
            recall_reserve           = self._require_key(emc_raw, "recall_reserve",           AGi.EPISODIC_MEMORY_CORTEX),
            recall_surface_limit     = self._require_key(emc_raw, "recall_surface_limit",     AGi.EPISODIC_MEMORY_CORTEX),
            recall_pool              = self._require_key(emc_raw, "recall_pool",              AGi.EPISODIC_MEMORY_CORTEX),
            recall_depth             = 0,    # [DERIVED] — populated by _derive()
            recall_timeout           = self._require_key(emc_raw, "recall_timeout",           AGi.EPISODIC_MEMORY_CORTEX),
            recovery_batch_size      = self._require_key(emc_raw, "recovery_batch_size",      AGi.EPISODIC_MEMORY_CORTEX),
            relevance_threshold      = self._require_key(emc_raw, "relevance_threshold",      AGi.EPISODIC_MEMORY_CORTEX),
        )

        wmc = WMCSettings(
            pmt_slot_limit     = self._require_key(wmc_raw, "pmt_slot_limit",  AGi.WORKING_MEMORY_CORTEX),
            pmt_slot_buffer    = self._require_key(wmc_raw, "pmt_slot_buffer", AGi.WORKING_MEMORY_CORTEX),
            global_chunk_limit = 0,    # [DERIVED] — populated by _derive()
        )

        # GCE stub — real values injected from persona in _load_config()
        gce = GCESettings(
            cognitive_engine      = "",
            response_depth        = 0,
            context_window        = 0,
            temperature           = 0.0,
            probability_threshold = 0.0,
            candidate_threshold   = 0,
            perseveration_damping = 0.0,
            habituation_damping   = 0.0,
            novelty_bias          = 0.0,
        )

        scs = SCSSettings(
            cortical_capacity   = self._require_key(scs_raw, "cortical_capacity",   AGi.SEMANTIC_COGNITIVE_SYSTEM),
            cognitive_reserve   = self._require_key(scs_raw, "cognitive_reserve",   AGi.SEMANTIC_COGNITIVE_SYSTEM),
            induction_threshold = self._require_key(scs_raw, "induction_threshold", AGi.SEMANTIC_COGNITIVE_SYSTEM),
            eviction_threshold  = self._require_key(scs_raw, "eviction_threshold",  AGi.SEMANTIC_COGNITIVE_SYSTEM),
            salience_hard_gate  = self._require_key(scs_raw, "salience_hard_gate",  AGi.SEMANTIC_COGNITIVE_SYSTEM),
            boundary_threshold  = self._require_key(scs_raw, "boundary_threshold",  AGi.SEMANTIC_COGNITIVE_SYSTEM),
            emc = emc,
            wmc = wmc,
            gce = gce,
        )

        hrs = HRSSettings(
            watchdog_interval = self._require_key(hrs_raw, "watchdog_interval", AGi.HOMEOSTATIC_REGULATION_SYSTEM),
            thermal_limit     = self._require_key(hrs_raw, "thermal_limit",     AGi.HOMEOSTATIC_REGULATION_SYSTEM),
            battery_critical  = self._require_key(hrs_raw, "battery_critical",  AGi.HOMEOSTATIC_REGULATION_SYSTEM),
        )

        sds = SDSSettings(
            min_obstacle_distance = self._require_key(sds_raw, "min_obstacle_distance", AGi.SELF_DEFENSE_SYSTEM),
            emergency_stop_dist   = self._require_key(sds_raw, "emergency_stop_dist",   AGi.SELF_DEFENSE_SYSTEM),
        )

        return ras, scs, hrs, sds

    def _load_persona(self, persona_stem: str) -> PersonaConfig:
        """
        Load active persona from ~/.agi/personas/<stem>.yaml.
        Flat YAML structure — all keys at root level.
        Raises RuntimeError if file is missing or any required key is absent.
        generic.yaml is a read-only reset target and is never loaded at runtime.
        """
        path = PERSONA_DIR / f"{persona_stem}.yaml"
        raw  = self._read_yaml(path)

        return PersonaConfig(
            system_prompt         = self._require_key(raw, "system_prompt",         path.name),
            cognitive_engine      = self._require_key(raw, "cognitive_engine",      path.name),
            response_depth        = self._require_key(raw, "response_depth",        path.name),
            context_window        = self._require_key(raw, "context_window",        path.name),
            temperature           = self._require_key(raw, "temperature",           path.name),
            probability_threshold = self._require_key(raw, "probability_threshold", path.name),
            candidate_threshold   = self._require_key(raw, "candidate_threshold",   path.name),
            perseveration_damping = self._require_key(raw, "perseveration_damping", path.name),
            habituation_damping   = self._require_key(raw, "habituation_damping",   path.name),
            novelty_bias          = self._require_key(raw, "novelty_bias",          path.name),
        )

    def _load_user(self, user_id: str) -> UserProfile:
        """
        Load user profile + extrinsic settings from ~/.agi/users.yaml by id.
        Raises RuntimeError if file is missing or user_id is not found.
        """
        raw   = self._read_yaml(USERS_CFG)
        users = raw.get("users", [])
        entry = next((u for u in users if u.get("id") == user_id), None)

        if not entry:
            raise RuntimeError(
                f"❌ ARC boot failed — user '{user_id}' not found in {AGi.USER_PROFILES}"
            )

        ext = self._require_key(entry, "extrinsic", f"user:{user_id}")

        return UserProfile(
            id       = self._require_key(entry, "id",       f"user:{user_id}"),
            name     = self._require_key(entry, "name",     f"user:{user_id}"),
            known_as = self._require_key(entry, "known_as", f"user:{user_id}"),
            location = self._require_key(entry, "location", f"user:{user_id}"),
            notes    = entry.get("notes", []),   # notes are optional — empty list is valid
            extrinsic = ExtrinsicSettings(
                response_verbosity   = self._require_key(ext, "response_verbosity",   f"user:{user_id}.extrinsic"),
                formality            = self._require_key(ext, "formality",            f"user:{user_id}.extrinsic"),
                preferred_language   = self._require_key(ext, "preferred_language",   f"user:{user_id}.extrinsic"),
                emotional_tone       = self._require_key(ext, "emotional_tone",       f"user:{user_id}.extrinsic"),
                memory_salience_bias = self._require_key(ext, "memory_salience_bias", f"user:{user_id}.extrinsic"),
            ),
        )

    # ─── Boot Sequence ────────────────────────────────────────────────────────

    def _boot(self) -> None:
        """
        Spawn cognitive nodes in dependency order.
        Each stage waits for the previous node's ready signal before proceeding.
        ARC does not publish its own ready signal until all stages complete.
        """
        ready_pub = self.create_publisher(
            Bool,
            f"/{AGi.RETICULAR_ACTIVATING_SYSTEM}/ready",
            1,
        )

        timeout = self.config.ras.boot_timeout

        # Future stages uncommented as each system is built

        # Stage 1 — Infrastructure
        # self._spawn(AGi.EMERGENCY_EXCEPTION_CORE)
        # self._wait_ready(f"/{AGi.EMERGENCY_EXCEPTION_CORE}/ready", timeout)

        # Stage 2 — Regulatory
        # self._spawn(AGi.HOMEOSTATIC_REGULATION_SYSTEM)
        # self._wait_ready(f"/{AGi.HOMEOSTATIC_REGULATION_SYSTEM}/ready", timeout)

        # Stage 3 — Cognition (CNC owns MCC, WMC, EMC internally)
        self._spawn(AGi.CENTRAL_NERVOUS_CORE)
        self._wait_ready(f"/{AGi.CENTRAL_NERVOUS_CORE}/ready", timeout)

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
        if key not in raw:
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
