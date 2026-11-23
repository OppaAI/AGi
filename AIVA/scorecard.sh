#!/usr/bin/env bash
# Convenience entry point for Scorecard runs from the AIVA directory.
set -Eeuo pipefail

AIVA_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
AGI_ROOT="$(cd -- "$AIVA_DIR/.." && pwd)"
SCORECARD_DIR="$AGI_ROOT/diagnostic/scorecard"

usage() {
  cat <<'EOF'
Usage:
  AIVA/scorecard.sh [launch.sh options]
      Open the Scorecard launcher menu. Options are forwarded to launch.sh,
      including --model=ministral, --model=swift, or --model=both.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

exec "$SCORECARD_DIR/launch.sh" "$@"
