#!/usr/bin/env bash
# Bootstrap local component checkouts, the Python environment, and ROS packages.
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
START_DIR="$PWD"
ENV_FILE="${AIVA_ENV_FILE:-$SCRIPT_DIR/.env}"

if [[ ! -f "$ENV_FILE" ]]; then
  cp "$SCRIPT_DIR/.env.example" "$ENV_FILE"
  chmod 600 "$ENV_FILE"
  printf 'Created %s; review its local settings, then run setup again.\n' "$ENV_FILE"
  exit 0
fi
set -a
# shellcheck disable=SC1090
source "$ENV_FILE"
set +a

SPEC_FILE="${AIVA_SPEC_FILE:-$SCRIPT_DIR/spec.yaml}"
SPEC_DIR="$(cd -- "$(dirname -- "$SPEC_FILE")" && pwd)"
spec_path() {
  local key="$1" value
  value="$(awk -F: -v key="$key" '
    /^paths:[[:space:]]*$/ { in_paths=1; next }
    in_paths && /^[^[:space:]#]/ { in_paths=0 }
    in_paths && $1 ~ "^[[:space:]]*" key "[[:space:]]*$" {
      sub(/^[^:]*:[[:space:]]*/, ""); sub(/[[:space:]]+#.*$/, "")
      gsub(/^['\''"]|['\''"]$/, ""); print; exit
    }
  ' "$SPEC_FILE")"
  [[ -n "$value" ]] || { printf 'Missing paths.%s in %s\n' "$key" "$SPEC_FILE" >&2; exit 1; }
  if [[ "$value" == '~/'* ]]; then value="$HOME/${value:2}"; fi
  if [[ "$value" = /* ]]; then printf '%s\n' "$value"; else (cd -- "$SPEC_DIR/$value" 2>/dev/null && pwd) || printf '%s/%s\n' "$SPEC_DIR" "$value"; fi
}
AGI_ROOT_DIR="${AGI_ROOT_DIR:-$(spec_path agi_root)}"
ROS_SETUP="${ROS_SETUP:-$(spec_path ros_setup)}"
AGI_BRANCH="${AGI_BRANCH:-dev}"

fail() { printf '✗ %s\n' "$*" >&2; exit 1; }

clone_or_check() {
  local url="$1" destination="$2"
  if [[ -d "$destination/.git" || -f "$destination/.git" ]]; then
    return
  fi
  [[ ! -e "$destination" ]] || fail "$destination exists but is not a Git checkout; move it aside and retry."
  mkdir -p "$(dirname -- "$destination")"
  git clone --branch "$AGI_BRANCH" "$url" "$destination"
}

link_component() {
  local link="$1" target="$2" relative_target="$3"
  mkdir -p "$(dirname -- "$link")"
  if [[ -L "$link" ]]; then
    [[ "$(readlink -f "$link")" == "$(readlink -f "$target")" ]] \
      || fail "$link points somewhere else; inspect and fix it before setup."
  elif [[ -e "$link" ]]; then
    fail "$link exists and is not the expected symlink; inspect it before setup."
  else
    ln -s "$relative_target" "$link"
  fi
}

clone_or_check https://github.com/AGi-robot/shell.git "$SCRIPT_DIR/shell"
clone_or_check https://github.com/AGi-robot/cypher.git "$SCRIPT_DIR/cypher"
clone_or_check https://github.com/AGi-robot/telepathy.git "$SCRIPT_DIR/telepathy"
clone_or_check https://github.com/AGi-robot/console.git "$SCRIPT_DIR/console"
clone_or_check https://github.com/AGi-robot/interactive.git "$SCRIPT_DIR/interactive"
clone_or_check https://github.com/AGi-robot/health.git "$SCRIPT_DIR/health"
clone_or_check https://github.com/AGi-robot/memory.git "$SCRIPT_DIR/mind/memory"
clone_or_check https://github.com/AGi-robot/canvas.git "$AGI_ROOT_DIR/capability/canvas"
clone_or_check https://github.com/AGi-robot/imaging.git "$AGI_ROOT_DIR/capability/imaging"

link_component "$SCRIPT_DIR/shell/src/cryptic_signal_core" "$SCRIPT_DIR/cypher" "../../cypher"
link_component "$SCRIPT_DIR/shell/src/telepathy_receptor" "$SCRIPT_DIR/telepathy" "../../telepathy"
link_component "$SCRIPT_DIR/shell/src/telepathic_management" "$SCRIPT_DIR/console" "../../console"
link_component "$SCRIPT_DIR/shell/src/agi_logging" "$SCRIPT_DIR/health" "../../health"
for package_xml in \
  "$SCRIPT_DIR/shell/src/cryptic_signal_core/package.xml" \
  "$SCRIPT_DIR/shell/src/telepathy_receptor/package.xml" \
  "$SCRIPT_DIR/shell/src/telepathic_management/package.xml" \
  "$SCRIPT_DIR/shell/src/agi_logging/package.xml"; do
  [[ -f "$package_xml" ]] || fail "Component package is missing at $package_xml. Update the sibling repository to its populated dev branch."
done

[[ -f "$ROS_SETUP" ]] || fail "ROS setup not found: $ROS_SETUP. Install/source ROS 2 Jazzy first."
set +u
source "$ROS_SETUP"
set -u
command -v colcon >/dev/null 2>&1 || fail "colcon is missing. Install ROS 2 colcon extensions first."
command -v rosdep >/dev/null 2>&1 || fail "rosdep is missing. Install and initialize rosdep, then rerun setup."

PYTHON_BIN="${AIVA_PYTHON:-python3}"
command -v "$PYTHON_BIN" >/dev/null 2>&1 || fail "Python executable not found: $PYTHON_BIN"
VENV_DIR="${AIVA_VENV:-$(spec_path python_venv)}"
case "$VENV_DIR" in
  /*) ;;
  *) VENV_DIR="$SCRIPT_DIR/$VENV_DIR" ;;
esac
case "$VENV_DIR" in
  /*) ;;
  *) VENV_DIR="$START_DIR/$VENV_DIR" ;;
esac
if [[ ! -x "$VENV_DIR/bin/python" ]]; then
  "$PYTHON_BIN" -m venv --system-site-packages "$VENV_DIR"
fi
"$VENV_DIR/bin/python" -m pip install --upgrade pip
TEMP_REQUIREMENTS="$(mktemp "$SCRIPT_DIR/.requirements.XXXXXX")"
trap 'rm -f "$TEMP_REQUIREMENTS"' EXIT
sed \
  -e '\|^-e \.\./capability/canvas$|d' \
  -e '\|^-e \.\./capability/imaging$|d' \
  "$SCRIPT_DIR/requirements.txt" > "$TEMP_REQUIREMENTS"
(cd "$SCRIPT_DIR" && "$VENV_DIR/bin/python" -m pip install -r "$TEMP_REQUIREMENTS")
"$VENV_DIR/bin/python" -m pip install \
  -e "$AGI_ROOT_DIR/capability/canvas" \
  -e "$AGI_ROOT_DIR/capability/imaging"

(cd "$SCRIPT_DIR/shell" && rosdep install --from-paths src --ignore-src -r -y)
(cd "$SCRIPT_DIR/shell" && colcon build --cmake-args "-DPython3_EXECUTABLE=$VENV_DIR/bin/python")

printf '\nGrace setup completed. Start with: %q\n' "$SCRIPT_DIR/launch.sh"
printf 'Component branch: %s (existing checkouts were left unchanged).\n' "$AGI_BRANCH"
