#!/usr/bin/env bash
# Bootstrap local component checkouts, the Python environment, and ROS packages.
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
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

fail() { printf '✗ %s\n' "$*" >&2; exit 1; }

SPEC_FILE="${AIVA_SPEC_FILE:-$SCRIPT_DIR/spec.yaml}"
CONFIG_PYTHON="${AIVA_PYTHON:-python3}"
command -v "$CONFIG_PYTHON" >/dev/null 2>&1 || fail "Python executable not found: $CONFIG_PYTHON"
"$CONFIG_PYTHON" -c 'import yaml' >/dev/null 2>&1 \
  || fail "PyYAML is required by wakeup.py. Install it in the bootstrap Python environment."
spec_path() {
  "$CONFIG_PYTHON" "$SCRIPT_DIR/wakeup.py" --spec "$SPEC_FILE" resolve "$1"
}
AGI_ROOT_DIR="${AGI_ROOT_DIR:-$(spec_path agi_root)}"
export AGI_ROOT_DIR
ROS_SETUP="${ROS_SETUP:-$(spec_path ros_setup)}"
AGI_BRANCH="${AGI_BRANCH:-dev}"
SHELL_DIR="${AIVA_SHELL_DIR:-$(spec_path shell_workspace)}"
CYPHER_DIR="${AIVA_CYPHER_DIR:-$(spec_path cypher_connect)}"
TELEPATHY_DIR="${AIVA_TELEPATHY_DIR:-$(spec_path telepathy_connect)}"
CONSOLE_DIR="${AIVA_CONSOLE_DIR:-$(spec_path console_connect)}"
INTERACTIVE_DIR="${AIVA_INTERACTIVE_DIR:-$(spec_path interactive_connect)}"
HEALTH_DIR="${AIVA_HEALTH_DIR:-$(spec_path health_connect)}"
MEMORY_DIR="${AIVA_MEMORY_REPO_DIR:-$(spec_path memory_connect)}"
CANVAS_DIR="${AGI_CANVAS_SOURCE_DIR:-$(spec_path canvas_source)}"
IMAGING_DIR="${AGI_IMAGING_SOURCE_DIR:-$(spec_path imaging_source)}"

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
  local link="$1" target="$2" relative_target
  [[ -d "$target" ]] || fail "Component checkout is missing at $target"
  relative_target="$(realpath --relative-to="$(dirname -- "$link")" "$target")"
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

clone_or_check https://github.com/AGi-robot/shell.git "$SHELL_DIR"
clone_or_check https://github.com/AGi-robot/cypher.git "$CYPHER_DIR"
clone_or_check https://github.com/AGi-robot/telepathy.git "$TELEPATHY_DIR"
clone_or_check https://github.com/AGi-robot/console.git "$CONSOLE_DIR"
clone_or_check https://github.com/AGi-robot/interactive.git "$INTERACTIVE_DIR"
clone_or_check https://github.com/AGi-robot/health.git "$HEALTH_DIR"
clone_or_check https://github.com/AGi-robot/memory.git "$MEMORY_DIR"
clone_or_check https://github.com/AGi-robot/canvas.git "$CANVAS_DIR"
clone_or_check https://github.com/AGi-robot/imaging.git "$IMAGING_DIR"

link_component "$SHELL_DIR/src/cryptic_signal_core" "$CYPHER_DIR"
link_component "$SHELL_DIR/src/telepathy_receptor" "$TELEPATHY_DIR"
link_component "$SHELL_DIR/src/telepathic_management" "$CONSOLE_DIR"
link_component "$SHELL_DIR/src/agi_logging" "$HEALTH_DIR"
for package_xml in \
  "$SHELL_DIR/src/cryptic_signal_core/package.xml" \
  "$SHELL_DIR/src/telepathy_receptor/package.xml" \
  "$SHELL_DIR/src/telepathic_management/package.xml" \
  "$SHELL_DIR/src/agi_logging/package.xml"; do
  [[ -f "$package_xml" ]] || fail "Component package is missing at $package_xml. Update the sibling repository to its populated dev branch."
done

[[ -f "$ROS_SETUP" ]] || fail "ROS setup not found: $ROS_SETUP. Install/source ROS 2 Jazzy first."
set +u
source "$ROS_SETUP"
set -u
command -v colcon >/dev/null 2>&1 || fail "colcon is missing. Install ROS 2 colcon extensions first."
command -v rosdep >/dev/null 2>&1 || fail "rosdep is missing. Install and initialize rosdep, then rerun setup."

VENV_DIR="${AIVA_VENV:-$(spec_path python_venv)}"
case "$VENV_DIR" in
  /*) ;;
  *) VENV_DIR="$SCRIPT_DIR/$VENV_DIR" ;;
esac
if [[ ! -x "$VENV_DIR/bin/python" ]]; then
  "$CONFIG_PYTHON" -m venv --system-site-packages "$VENV_DIR"
fi
"$VENV_DIR/bin/python" -m pip install --upgrade pip
TEMP_REQUIREMENTS="$(mktemp "$SCRIPT_DIR/.requirements.XXXXXX")"
trap 'rm -f "$TEMP_REQUIREMENTS"' EXIT
sed \
  -e '\|^-e \.\./capability/canvas$|d' \
  -e '\|^-e \.\./capability/imaging$|d' \
  -e '\|^-e \./mind/memory$|d' \
  -e '\|^-e \./interactive$|d' \
  "$SCRIPT_DIR/requirements.txt" > "$TEMP_REQUIREMENTS"
(cd "$SCRIPT_DIR" && "$VENV_DIR/bin/python" -m pip install -r "$TEMP_REQUIREMENTS")
"$VENV_DIR/bin/python" -m pip install \
  -e "$CANVAS_DIR" \
  -e "$IMAGING_DIR" \
  -e "$MEMORY_DIR" \
  -e "$INTERACTIVE_DIR"

(cd "$SHELL_DIR" && rosdep install --from-paths src --ignore-src -r -y)
(cd "$SHELL_DIR" && colcon build --cmake-args "-DPython3_EXECUTABLE=$VENV_DIR/bin/python")

printf '\nGrace setup completed. Start with: %q\n' "$SCRIPT_DIR/launch.sh"
printf 'Component branch: %s (existing checkouts were left unchanged).\n' "$AGI_BRANCH"
