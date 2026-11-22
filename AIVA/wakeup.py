#!/usr/bin/env python3
"""Resolve AIVA's identity-based filesystem tracts during startup.

The YAML file stores relative component locations and identity metadata. This
module hydrates those declarations into absolute paths; launch.sh and setup.sh
remain responsible for starting and installing the runtime.

Usage:
    python3 wakeup.py resolve shell_workspace
    python3 wakeup.py show
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import re
import sys
from typing import Any

try:
    import yaml
except ImportError as exc:  # pragma: no cover - environment-specific diagnostic
    raise SystemExit(
        "PyYAML is required to read spec.yaml. Install python3-yaml or "
        "PyYAML in the Python environment used by AIVA."
    ) from exc


DEFAULT_SPEC = Path(__file__).with_name("spec.yaml")

# Existing launcher/setup names map to the biological route names in spec.yaml.
PATH_FIELDS = {
    "agi_root": "agi_root",
    "shell_workspace": "shell_connect",
    "cypher_connect": "cypher_connect",
    "telepathy_connect": "telepathy_connect",
    "console_connect": "console_connect",
    "interactive_connect": "interactive_connect",
    "health_connect": "health_connect",
    "memory_connect": "memory_connect",
    "canvas_source": "canvas_connect",
    "imaging_source": "imaging_connect",
    "python_venv": "python_environment",
    "ros_setup": "ros_setup",
    "llama_cpp_dir": "llama_cpp_connect",
    "llama_router_ini": "llama_router_config",
    "moss_paint_weights_root": "moss_weights_connect",
}


class AIVAWakeup:
    """Load the identity manifest and resolve configured routes to paths."""

    def __init__(self, spec_file: Path = DEFAULT_SPEC,
                 agi_root_override: str | None = None) -> None:
        self.spec_file = spec_file.expanduser().resolve()
        self.agi_root_override = agi_root_override
        try:
            with self.spec_file.open("r", encoding="utf-8") as stream:
                document: Any = yaml.safe_load(stream)
        except OSError as exc:
            raise ValueError(f"Cannot read AIVA spec: {self.spec_file}: {exc}") from exc
        except yaml.YAMLError as exc:
            raise ValueError(f"Invalid YAML in AIVA spec {self.spec_file}: {exc}") from exc

        if not isinstance(document, dict):
            raise ValueError("AIVA spec must contain a YAML mapping")
        self.identity = self._mapping(document, "identity")
        self.tracts = self._mapping(document, "tracts")
        self._validate_identity()

    @staticmethod
    def _mapping(document: dict[str, Any], name: str) -> dict[str, Any]:
        value = document.get(name)
        if not isinstance(value, dict):
            raise ValueError(f"AIVA spec requires a '{name}:' mapping")
        return value

    def _validate_identity(self) -> None:
        for key in ("entity_identity", "entity_codename", "creator_identity"):
            value = self.identity.get(key)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"identity.{key} must be a non-empty string")
        if not re.fullmatch(r"[A-Za-z0-9_-]+", self.identity["entity_identity"]):
            raise ValueError("identity.entity_identity must be a single path-safe name")

    def resolve(self, name: str) -> Path:
        """Return one absolute path, resolving relative values from spec.yaml."""
        if name == "agi_home":
            # The persistent entity home is derived from its identity, e.g. AGi -> ~/.agi.
            return (Path.home() / f".{self.identity['entity_identity'].lower()}").resolve()

        if name == "agi_root" and self.agi_root_override:
            override = Path(self.agi_root_override).expanduser()
            if not override.is_absolute():
                override = self.spec_file.parent / override
            return override.resolve()

        field = PATH_FIELDS.get(name)
        if field is None:
            valid = ", ".join((*PATH_FIELDS, "agi_home"))
            raise ValueError(f"Unknown tract '{name}'. Choose one of: {valid}")

        value = self.tracts.get(field)
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"AIVA spec requires a non-empty tracts.{field}")
        path = Path(value).expanduser()
        if not path.is_absolute():
            base = self.spec_file.parent
            if name in ("canvas_source", "imaging_source"):
                base = self.resolve("agi_root")
            path = base / path
        return path.resolve()

    def resolved_paths(self) -> dict[str, Path]:
        """Return all launcher/setup paths, including the derived user home."""
        return {name: self.resolve(name) for name in (*PATH_FIELDS, "agi_home")}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", type=Path, default=DEFAULT_SPEC,
                        help="identity/tract manifest (default: AIVA/spec.yaml)")
    parser.add_argument("--agi-root", default=os.environ.get("AGI_ROOT_DIR"),
                        help="optional AGi repository root override")
    subparsers = parser.add_subparsers(dest="command", required=True)
    resolve_parser = subparsers.add_parser("resolve", help="print one resolved path")
    resolve_parser.add_argument("name", choices=(*PATH_FIELDS, "agi_home"))
    subparsers.add_parser("show", help="print every resolved path")
    args = parser.parse_args()

    try:
        wakeup = AIVAWakeup(args.spec, args.agi_root)
        if args.command == "resolve":
            print(wakeup.resolve(args.name))
        else:
            for name, path in wakeup.resolved_paths().items():
                print(f"{name}: {path}")
    except ValueError as exc:
        print(f"AIVA wakeup: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
