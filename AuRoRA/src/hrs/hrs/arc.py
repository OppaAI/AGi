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
import asyncio
import time
import yaml

from rclpy.node         import Node
from std_msgs.msg       import Bool
from pathlib            import Path
from dataclasses        import dataclass, field

# ─── Path Convention ──────────────────────────────────────────────────────────
#
#   ~/.agi/
#       aurora.yaml             ← single robot-wide settings file
#       personas/
#           grace.yaml          ← swappable persona + inference params
#       users/
#           oppaai.yaml         ← swappable user profile
#       scs/
#           emc/
#               engram_complex.db

AGI_DIR      = Path.home() / ".agi"
AURORA_CFG   = AGI_DIR / "aurora.yaml"
PERSONA_DIR  = AGI_DIR / "personas"
USER_DIR     = AGI_DIR / "users"

# ─── Dataclasses ─────────────────────────────────────────────────────────────

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
class SCSSettings:
    cortical_capacity:    int   = 8
    recall_depth:         int   = 10
    recall_surface_limit: int   = 5
    relevance_threshold:  float = 0.25
    recall_timeout:       float = 2.0
    units_per_chunk:      int   = 350
    induction_threshold:  float = 0.4
    eviction_threshold:   float = 0.3
    salience_hard_gate:   float = 0.8
    boundary_threshold:   float = 0.35

@dataclass
class HRSSettings:
    """Stub — expand as HRS matures."""
    watchdog_interval:  float = 5.0
    thermal_limit:      float = 85.0
    battery_critical:   float = 15.0

@dataclass
class SDSSettings:
    """Stub — expand as SDS matures."""
    min_obstacle_distance: float = 0.4
    emergency_stop_dist:   float = 0.2

@dataclass
class RASSettings:
    boot_timeout:   float = 10.0
    active_persona: str   = "grace"
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
        super().__init__("arc")

        # ── Public config — all nodes read from here ──────────────────────────
        self.config: AuroraConfig = AuroraConfig()

        # ── Ready publisher ───────────────────────────────────────────────────
        self._ready_pub = self.create_publisher(Bool, "/ras/ready", 1)

        # ── Boot sequence ─────────────────────────────────────────────────────
        self._load_config()
        self._boot()

    # ─── Config Loading ───────────────────────────────────────────────────────

    def _load_config(self) -> None:
        """Load aurora.yaml then active persona and user profile."""
        raw = self._read_yaml(AURORA_CFG)
        if not raw:
            self.get_logger().warning("⚠️  aurora.yaml not found — using defaults")
        else:
            self._parse_aurora(raw)

        # Persona and user loaded after RAS settings so active_persona/active_user are known
        self.config.persona = self._load_persona(self.config.ras.active_persona)
        self.config.user    = self._load_user(self.config.ras.active_user)

        self.get_logger().info(
            f"✅ Config loaded — persona: {self.config.ras.active_persona} "
            f"| user: {self.config.ras.active_user}"
        )

    def _parse_aurora(self, raw: dict) -> None:
        """Parse aurora.yaml into AuroraConfig dataclasses."""
        try:
            ras = raw.get("ras", {})
            self.config.ras = RASSettings(
                boot_timeout   = ras.get("boot_timeout",   10.0),
                active_persona = ras.get("active_persona", "grace"),
                active_user    = ras.get("active_user",    "oppaai"),
            )
        except Exception as e:
            self.get_logger().error(f"❌ RAS settings parse error: {e}")

        try:
            scs = raw.get("scs", {})
            self.config.scs = SCSSettings(
                cortical_capacity    = scs.get("cortical_capacity",    8),
                recall_depth         = scs.get("recall_depth",         10),
                recall_surface_limit = scs.get("recall_surface_limit", 5),
                relevance_threshold  = scs.get("relevance_threshold",  0.25),
                recall_timeout       = scs.get("recall_timeout",       2.0),
                units_per_chunk      = scs.get("units_per_chunk",      350),
                induction_threshold  = scs.get("induction_threshold",  0.4),
                eviction_threshold   = scs.get("eviction_threshold",   0.3),
                salience_hard_gate   = scs.get("salience_hard_gate",   0.8),
                boundary_threshold   = scs.get("boundary_threshold",   0.35),
            )
        except Exception as e:
            self.get_logger().error(f"❌ SCS settings parse error: {e}")

        try:
            hrs = raw.get("hrs", {})
            self.config.hrs = HRSSettings(
                watchdog_interval = hrs.get("watchdog_interval", 5.0),
                thermal_limit     = hrs.get("thermal_limit",     85.0),
                battery_critical  = hrs.get("battery_critical",  15.0),
            )
        except Exception as e:
            self.get_logger().error(f"❌ HRS settings parse error: {e}")

        try:
            sds = raw.get("sds", {})
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
    # Future stages uncommented as each system is built
    
    # Stage 1 — Infrastructure
    # self._spawn("eee")
    # self._wait_ready("/eee/ready", timeout)

    # Stage 2 — Regulatory  
    # self._spawn("hrs")
    # self._wait_ready("/hrs/ready", timeout)

    # Stage 3 — Cognition (CNC owns MCC, WMC, EMC internally)
    self._spawn("cnc")
    self._wait_ready("/cnc/ready", timeout)

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
