"""
ARC — Arousal Reaction Core
========================================
AuRoRA · Recticular Activating System (RAS)

Central config loader and system-wide constant registry for GRACE/AuRoRA CNS
Loads all YAML configs at startup, exposes clean objects to all other nodes
TODO Future: DNA genome encryption for config files (dual-helix + mirror backup)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from pathlib import Path
from dataclasses import dataclass, field
from typing import Any
import yaml
import json
import logging

# ─── Config Root ─────────────────────────────────────────────────────────────

CNS_CONFIG_DIR = Path.home() / ".agi" / "cns"

# ─── Schema Dataclasses ───────────────────────────────────────────────────────

@dataclass
class InferenceParams:
    model:       str   = "mag-mell-r1:12b"
    temperature: float = 0.7
    top_p:       float = 0.9
    max_tokens:  int   = 512
    stream:      bool  = True

@dataclass
class PersonaConfig:
    name:          str            = "GRACE"
    system_prompt: str            = ""
    params:        InferenceParams = field(default_factory=InferenceParams)

@dataclass
class UserProfile:
    name:     str       = "unknown"
    known_as: str       = "unknown"
    location: str       = "unknown"
    notes:    list[str] = field(default_factory=list)

@dataclass
class IntrinsicConfig:
    """
    GRACE's internal drives, values, and motivational priors.
    Loaded from intrinsic.yaml — defines who she is, not how she speaks.
    Stub for M2+ — expand as cognitive architecture matures.
    """
    values:      list[str] = field(default_factory=list)
    drives:      list[str] = field(default_factory=list)
    constraints: list[str] = field(default_factory=list)

@dataclass
class ExtrinsicConfig:
    """
    Environmental and mission parameters — what AuRoRA is doing and where.
    Loaded from extrinsic.yaml — defines the operational context.
    Stub for M2+ — expand as mission planning matures.
    """
    mission:     str       = "wildlife observation"
    environment: str       = "outdoor"
    location:    str       = "Vancouver, BC, Canada"
    notes:       list[str] = field(default_factory=list)

# ─── Encryption Interface (stub) ─────────────────────────────────────────────

class GenomeEncryption:
    """
    DNA genome encryption interface — stub for future implementation.
    
    Architecture intent:
    - Primary helix:  encrypted config payload
    - Mirror helix:   backup/validation strand (complementary encoding)
    - Decryption key: derived from device identity or hardware token
    
    When implemented, all loaders call decrypt() before YAML parsing.
    Files on disk are opaque — only HRS can read them.
    """

    @staticmethod
    def encrypt(data: str) -> bytes:
        raise NotImplementedError("GenomeEncryption not yet implemented")

    @staticmethod
    def decrypt(data: bytes) -> str:
        raise NotImplementedError("GenomeEncryption not yet implemented")

    @staticmethod
    def validate_mirror(primary: bytes, mirror: bytes) -> bool:
        """Validate primary helix against mirror backup strand."""
        raise NotImplementedError("GenomeEncryption not yet implemented")

# ─── HRS Node ─────────────────────────────────────────────────────────────────

class HippocampalRelayStation(Node):
    """
    HRS — Hippocampal Relay Station
    
    Central config loader and constant registry for the GRACE CNS.
    Loads all YAML configs at startup, validates them, and publishes
    a ready signal before any other cognitive node begins processing.
    
    Startup order: HRS → (ready) → EMC → MCC → CNC
    
    Config files (all in ~/.agi/cns/):
        persona.yaml    — GRACE identity, system prompt, inference params
        user_profile.yaml — user facts always injected into context
        intrinsic.yaml  — GRACE's internal drives and values (M2+ stub)
        extrinsic.yaml  — mission and environment context (M2+ stub)
    
    Future: GenomeEncryption decrypts all files before parsing.
    Dual-helix format: primary payload + mirror backup strand.
    """

    def __init__(self):
        super().__init__("hrs")

        # ── Public config objects — read by all other nodes ──────────────────
        self.persona:   PersonaConfig  = PersonaConfig()
        self.user:      UserProfile    = UserProfile()
        self.intrinsic: IntrinsicConfig = IntrinsicConfig()
        self.extrinsic: ExtrinsicConfig = ExtrinsicConfig()

        # ── Ready signal publisher ────────────────────────────────────────────
        self._ready_pub = self.create_publisher(Bool, "/hrs/ready", 1)

        # ── Load all configs ──────────────────────────────────────────────────
        self._load_all()

        # ── Publish ready ─────────────────────────────────────────────────────
        msg = Bool()
        msg.data = True
        self._ready_pub.publish(msg)
        self.get_logger().info("✅ HRS ready — all configs loaded")

    # ─── Master Loader ────────────────────────────────────────────────────────

    def _load_all(self) -> None:
        self.persona   = self._load_persona()
        self.user      = self._load_user_profile()
        self.intrinsic = self._load_intrinsic()
        self.extrinsic = self._load_extrinsic()

    # ─── Individual Loaders ───────────────────────────────────────────────────

    def _load_persona(self) -> PersonaConfig:
        raw = self._read_yaml("persona.yaml")
        if not raw:
            self.get_logger().warning("⚠️  persona.yaml not found — using defaults")
            return PersonaConfig()
        try:
            p = raw.get("persona", {})
            params_raw = raw.get("params", {})
            return PersonaConfig(
                name          = p.get("name", "GRACE"),
                system_prompt = p.get("system_prompt", ""),
                params        = InferenceParams(
                    model       = params_raw.get("model", "mag-mell-r1:12b"),
                    temperature = params_raw.get("temperature", 0.7),
                    top_p       = params_raw.get("top_p", 0.9),
                    max_tokens  = params_raw.get("max_tokens", 512),
                    stream      = params_raw.get("stream", True),
                )
            )
        except Exception as e:
            self.get_logger().error(f"❌ persona.yaml parse error: {e}")
            return PersonaConfig()

    def _load_user_profile(self) -> UserProfile:
        raw = self._read_yaml("user_profile.yaml")
        if not raw:
            self.get_logger().warning("⚠️  user_profile.yaml not found — using defaults")
            return UserProfile()
        try:
            return UserProfile(
                name     = raw.get("name", "unknown"),
                known_as = raw.get("known_as", "unknown"),
                location = raw.get("location", "unknown"),
                notes    = raw.get("notes", []),
            )
        except Exception as e:
            self.get_logger().error(f"❌ user_profile.yaml parse error: {e}")
            return UserProfile()

    def _load_intrinsic(self) -> IntrinsicConfig:
        raw = self._read_yaml("intrinsic.yaml")
        if not raw:
            return IntrinsicConfig()  # silent — stub file optional
        try:
            return IntrinsicConfig(
                values      = raw.get("values", []),
                drives      = raw.get("drives", []),
                constraints = raw.get("constraints", []),
            )
        except Exception as e:
            self.get_logger().error(f"❌ intrinsic.yaml parse error: {e}")
            return IntrinsicConfig()

    def _load_extrinsic(self) -> ExtrinsicConfig:
        raw = self._read_yaml("extrinsic.yaml")
        if not raw:
            return ExtrinsicConfig()  # silent — stub file optional
        try:
            return ExtrinsicConfig(
                mission     = raw.get("mission", "wildlife observation"),
                environment = raw.get("environment", "outdoor"),
                location    = raw.get("location", "Vancouver, BC, Canada"),
                notes       = raw.get("notes", []),
            )
        except Exception as e:
            self.get_logger().error(f"❌ extrinsic.yaml parse error: {e}")
            return ExtrinsicConfig()

    # ─── File I/O ─────────────────────────────────────────────────────────────

    def _read_yaml(self, filename: str) -> dict | None:
        """
        Read and parse a YAML config file from CNS_CONFIG_DIR.
        Future: call GenomeEncryption.decrypt() before yaml.safe_load().
        Mirror helix validation will also happen here.
        """
        path = CNS_CONFIG_DIR / filename
        if not path.exists():
            return None
        try:
            with open(path, "r") as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"❌ Failed to read {filename}: {e}")
            return None

# ─── Entry Point ──────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = HippocampalRelayStation()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
