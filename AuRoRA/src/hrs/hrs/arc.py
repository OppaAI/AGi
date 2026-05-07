"""
ARC — Arousal Reaction Core
========================================
AuRoRA · Reticular Activating System (RAS)

Bootloader and central config registry for the AuRoRA CNS.
Loads aurora.yaml + active persona/user at startup, spawns all
cognitive and regulatory nodes in dependency order, and publishes
a ready signal before any other node begins processing.

Boot order:
    ARC → EEE → HRS → EMC → MCC → CNC

TODO: DNA genome encryption — dual-helix primary + mirror backup
"""

import rclpy
import subprocess
import time
import yaml

from rclpy.node         import Node
from std_msgs.msg       import Bool
from pathlib            import Path
from dataclasses        import dataclass, field

from hrp                import AGi

# ─── Path Convention ──────────────────────────────────────────────────────────
#
#   ~/.agi/
#       aurora.yaml             ← single robot-wide settings file
#       personas/
#           persona.yaml        ← active persona + inference params
#           generic.yaml        ← default persona reset target (read-only)
#       users/
#           oppaai.yaml         ← swappable user profile
#       scs/
#           emc/
#               engram_complex.db

AGI_DIR      = Path.home() / AGi.ENTITY_GATEWAY
AURORA_CFG   = AGI_DIR / "aurora.yaml"
PERSONA_DIR  = AGI_DIR / "personas"
USER_DIR     = AGI_DIR / "users"

# ─── Dataclasses ──────────────────────────────────────────────────────────────

@dataclass
class InferenceParams:
    model:       str   = "mag-mell-r1:12b"
    temperature: float = 0.7
    top_p:       float = 0.9
    max_tokens:  int   = 512
    stream:      bool  = True

@dataclass
class PersonaConfig:
    name:          str             = "GRACE"
    system_prompt: str             = ""
    params:        InferenceParams = field(default_factory=InferenceParams)

@dataclass
class UserProfile:
    name:     str       = "unknown"
    known_as: str       = "unknown"
    location: str       = "unknown"
    notes:    list[str] = field(default_factory=list)

@dataclass
class EMCSettings:
    binding_stream_limit:  int   = 512
    recall_surface_limit:  int   = 3
    recall_pool:           int   = 15
    recall_depth:          int   = 45
    relevance_threshold:   float = 0.45
    recall_timeout:        float = 2.0
    recovery_batch_size:   int   = 50
    episode_content_limit: int   = 3000
    theta_interval:        float = 2.0
    theta_batch_limit:     int   = 32
    recall_reserve:        int   = 2048

@dataclass
class WMCSettings:
    pmt_slot_limit:    int = 7
    pmt_slot_buffer:   int = 2
    global_chunk_limit: int = 11264

@dataclass
class GCESettings:
    model:               str   = "huihui_ai/granite4.1-abliterated:8b-q8_0"
    temperature:         float = 0.75
    top_p:               float = 0.88
    top_k:               int   = 50
    max_tokens:          int   = 512
    context_window:      int   = 32768
    repetition_penalty:  float = 1.25
    frequency_penalty:   float = 0.15
    presence_penalty:    float = 0.05

@dataclass
class SCSSettings:
    cortical_capacity:  int   = 16384
    cognitive_reserve:  int   = 2048
    induction_threshold: float = 0.4
    eviction_threshold:  float = 0.3
    salience_hard_gate:  float = 0.8
    boundary_threshold:  float = 0.35
    emc: EMCSettings    = field(default_factory=EMCSettings)
    wmc: WMCSettings    = field(default_factory=WMCSettings)
    gce: GCESettings    = field(default_factory=GCESettings)

@dataclass
class HRSSettings:
    """Stub — expand as HRS matures."""
    watchdog_interval: float = 5.0
    thermal_limit:     float = 85.0
    battery_critical:  float = 15.0

@dataclass
class SDSSettings:
    """Stub — expand as SDS matures."""
    min_obstacle_distance: float = 0.4
    emergency_stop_dist:   float = 0.2

@dataclass
class RASSettings:
    boot_timeout:   float = 10.0
    active_persona: str   = "persona"
    active_user:    str   = "oppaai"

@dataclass
class AuroraConfig:
    """Single robot-wide config object — ARC loads once, all nodes read from it."""
    ras:     RASSettings   = field(default_factory=RASSettings)
    scs:     SCSSettings   = field(default_factory=SCSSettings)
    hrs:     HRSSettings   = field(default_factory=HRSSettings)
    sds:     SDSSettings   = field(default_factory=SDSSettings)
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
    Loads aurora.yaml + active persona/user at startup.
    Spawns all nodes in dependency order, waits for each ready signal
    before proceeding to the next stage.

    Exposes a single AuroraConfig object — all other nodes import
    ARC and read config from it. No node does its own file I/O.

    Topic: /ras/ready — published True when full boot sequence completes.
    """

    def __init__(self):
        super().__init__(AGi.RETICULAR_ACTIVATING_SYSTEM)

        # ── Public config — all nodes read from here ──────────────────────────
        self.config: AuroraConfig = AuroraConfig()

        # ── Ready publisher ───────────────────────────────────────────────────
        self._ready_pub = self.create_publisher(
            Bool,
            f"/{AGi.RETICULAR_ACTIVATING_SYSTEM}/ready",
            1,
        )

        # ── Boot sequence ─────────────────────────────────────────────────────
        self._load_config()
        self._hydrate()
        self._boot()

    # ─── Config Loading ───────────────────────────────────────────────────────

    def _load_config(self) -> None:
        """Load aurora.yaml then active persona and user profile."""
        raw = self._read_yaml(AURORA_CFG)
        if not raw:
            self.get_logger().warning("⚠️  aurora.yaml not found — using defaults")
        else:
            self._parse_aurora(raw)

        # Persona and user loaded after RAS so active_persona/active_user are known
        self.config.persona = self._load_persona(self.config.ras.active_persona)
        self.config.user    = self._load_user(self.config.ras.active_user)

        self.get_logger().info(
            f"✅ Config loaded — persona: {self.config.ras.active_persona} "
            f"| user: {self.config.ras.active_user}"
        )

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
                    self.get_logger().debug(f"Hydrated {section_key}.{hrp_key} = {getattr(config_section, attr_name)}")

    def _parse_aurora(self, raw: dict) -> None:
        """Parse aurora.yaml into AuroraConfig dataclasses."""
        try:
            ras = raw.get(AGi.RETICULAR_ACTIVATING_SYSTEM, {})
            self.config.ras = RASSettings(
                boot_timeout   = ras.get("boot_timeout",   10.0),
                active_persona = ras.get("active_persona", "persona"),
                active_user    = ras.get("active_user",    "oppaai"),
            )
        except Exception as e:
            self.get_logger().error(f"❌ RAS settings parse error: {e}")

        try:
            scs = raw.get(AGi.SEMANTIC_COGNITIVE_SYSTEM, {})
            emc = scs.get(AGi.EPISODIC_MEMORY_CORTEX, {})
            wmc = scs.get(AGi.WORKING_MEMORY_CORTEX, {})
            gce = scs.get(AGi.GENERATIVE_COGNITIVE_ENGINE, {})
            self.config.scs = SCSSettings(
                cortical_capacity   = scs.get("cortical_capacity",   16384),
                cognitive_reserve   = scs.get("cognitive_reserve",   2048),
                induction_threshold = scs.get("induction_threshold", 0.4),
                eviction_threshold  = scs.get("eviction_threshold",  0.3),
                salience_hard_gate  = scs.get("salience_hard_gate",  0.8),
                boundary_threshold  = scs.get("boundary_threshold",  0.35),
                emc = EMCSettings(
                    binding_stream_limit  = emc.get("binding_stream_limit",  512),
                    recall_surface_limit  = emc.get("recall_surface_limit",  3),
                    recall_pool           = emc.get("recall_pool",           15),
                    recall_depth          = emc.get("recall_depth",          45),
                    relevance_threshold   = emc.get("relevance_threshold",   0.45),
                    recall_timeout        = emc.get("recall_timeout",        2.0),
                    recovery_batch_size   = emc.get("recovery_batch_size",   50),
                    episode_content_limit = emc.get("episode_content_limit", 3000),
                    theta_interval        = emc.get("theta_interval",        2.0),
                    theta_batch_limit     = emc.get("theta_batch_limit",     32),
                    recall_reserve        = emc.get("recall_reserve",        2048),
                ),
                wmc = WMCSettings(
                    pmt_slot_limit     = wmc.get("pmt_slot_limit",     7),
                    pmt_slot_buffer    = wmc.get("pmt_slot_buffer",    2),
                    global_chunk_limit = wmc.get("global_chunk_limit", 11264),
                ),
                gce = GCESettings(
                    model              = gce.get("model",              "huihui_ai/granite4.1-abliterated:8b-q8_0"),
                    temperature        = gce.get("temperature",        0.75),
                    top_p              = gce.get("top_p",              0.88),
                    top_k              = gce.get("top_k",              50),
                    max_tokens         = gce.get("max_tokens",         512),
                    context_window     = gce.get("context_window",     32768),
                    repetition_penalty = gce.get("repetition_penalty", 1.25),
                    frequency_penalty  = gce.get("frequency_penalty",  0.15),
                    presence_penalty   = gce.get("presence_penalty",   0.05),
                ),
            )
        except Exception as e:
            self.get_logger().error(f"❌ SCS settings parse error: {e}")

        try:
            hrs = raw.get(AGi.HOMEOSTATIC_REGULATION_SYSTEM, {})
            self.config.hrs = HRSSettings(
                watchdog_interval = hrs.get("watchdog_interval", 5.0),
                thermal_limit     = hrs.get("thermal_limit",     85.0),
                battery_critical  = hrs.get("battery_critical",  15.0),
            )
        except Exception as e:
            self.get_logger().error(f"❌ HRS settings parse error: {e}")

        try:
            sds = raw.get(AGi.SPATIAL_DETECTION_SYSTEM, {})
            self.config.sds = SDSSettings(
                min_obstacle_distance = sds.get("min_obstacle_distance", 0.4),
                emergency_stop_dist   = sds.get("emergency_stop_dist",   0.2),
            )
        except Exception as e:
            self.get_logger().error(f"❌ SDS settings parse error: {e}")

    def _load_persona(self, name: str) -> PersonaConfig:
        """Load active persona from ~/.agi/personas/{name}.yaml"""
        raw = self._read_yaml(PERSONA_DIR / f"{name}.yaml")
        if not raw:
            self.get_logger().warning(f"⚠️  personas/{name}.yaml not found — using defaults")
            return PersonaConfig()
        try:
            p      = raw.get("persona", {})
            params = raw.get("params",  {})
            return PersonaConfig(
                name          = p.get("name",          "GRACE"),
                system_prompt = p.get("system_prompt", ""),
                params        = InferenceParams(
                    model       = params.get("model",       "mag-mell-r1:12b"),
                    temperature = params.get("temperature", 0.7),
                    top_p       = params.get("top_p",       0.9),
                    max_tokens  = params.get("max_tokens",  512),
                    stream      = params.get("stream",      True),
                )
            )
        except Exception as e:
            self.get_logger().error(f"❌ Persona parse error ({name}): {e}")
            return PersonaConfig()

    def _load_user(self, name: str) -> UserProfile:
        """Load active user profile from ~/.agi/users/{name}.yaml"""
        raw = self._read_yaml(USER_DIR / f"{name}.yaml")
        if not raw:
            self.get_logger().warning(f"⚠️  users/{name}.yaml not found — using defaults")
            return UserProfile()
        try:
            return UserProfile(
                name     = raw.get("name",     "unknown"),
                known_as = raw.get("known_as", "unknown"),
                location = raw.get("location", "unknown"),
                notes    = raw.get("notes",    []),
            )
        except Exception as e:
            self.get_logger().error(f"❌ User profile parse error ({name}): {e}")
            return UserProfile()

    # ─── Boot Sequence ────────────────────────────────────────────────────────

    def _boot(self) -> None:
        timeout = self.config.ras.boot_timeout

        # Future stages uncommented as each system is built

        # Stage 1 — Infrastructure
        # self._spawn(AGi.ELECTRO_ENCEPHALIC_ENGINE)
        # self._wait_ready(f"/{AGi.ELECTRO_ENCEPHALIC_ENGINE}/ready", timeout)

        # Stage 2 — Regulatory
        # self._spawn(AGi.HOMEOSTATIC_REGULATION_SYSTEM)
        # self._wait_ready(f"/{AGi.HOMEOSTATIC_REGULATION_SYSTEM}/ready", timeout)

        # Stage 3 — Cognition (CNC owns MCC, WMC, EMC internally)
        self._spawn(AGi.CENTRAL_NERVOUS_CONTROLLER)
        self._wait_ready(f"/{AGi.CENTRAL_NERVOUS_CONTROLLER}/ready", timeout)

        msg      = Bool()
        msg.data = True
        self._ready_pub.publish(msg)
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

    # ─── File I/O ─────────────────────────────────────────────────────────────

    def _read_yaml(self, path: Path) -> dict | None:
        """
        Read and parse a YAML file.
        TODO: call GenomeEncryption.decrypt() before yaml.safe_load()
        TODO: call GenomeEncryption.validate_mirror() for backup strand
        """
        if not path.exists():
            return None
        try:
            with open(path, "r") as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"❌ Failed to read {path.name}: {e}")
            return None

# ─── Entry Point ──────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ArousedReactionCore()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
