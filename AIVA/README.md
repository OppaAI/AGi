# AIVA — Amazing Grace (PC companion)

AIVA is Grace's PC-side runtime. The `launch.sh` orchestrator at this level
starts the llama.cpp router, the shell ROS 2 workspace services, and wires in
the shared capabilities.

## AIVA and AuRoRA configuration

`AIVA/launch.sh` runs AIVA's `telepathy_receptor` adapter, not
`AuRoRA/ras/ras.launch.py`. It does not start AuRoRA's CNC or TMS TIC, import
the HRM manifest into chat, or load `~/.agi/ras/aurora.yaml`. The shared
`/tms/text_response` topic name does not imply that AuRoRA's TMS is running.

AIVA chat uses its declared ROS parameters and defaults, environment variables
such as `AIVA_LLM_MODEL` and `AIVA_LLM_BASE_URL`, the Shell `persona/SOUL.md`,
and `~/.agi/profile/user.md`. Its conversation history defaults to
`~/.agi/profile/grace_memory.sqlite3`. See the Shell README for overrides.
The AuRoRA YAML is not required for this AIVA chat path.

AuRoRA has a separate launch path: `ras/ras.launch.py` supplies
`~/.agi/ras/aurora.yaml` to CNC, TIC, and its HTTP API. CNC and TIC declare
parameters using HRM defaults and apply matching ROS YAML overrides. HRM is a
Python manifest, not a running service. AuRoRA also reads its own user and
persona YAML files where configured. Starting AIVA alone does not start AuRoRA.

The shared console's intrinsic settings page targets AuRoRA's YAML, not AIVA's
chat parameters. Its settings API requires AuRoRA's `hrs` Python package;
when that package is absent from an AIVA-only environment, the settings API
fails. Even if it is available, saving those values does not configure AIVA
chat.

## Layout

| Path | Tracked in | Contents |
|---|---|---|
| `launch.sh` | **this repo** (`OppaAI/AGi`) | Grace's launcher: builds the shell workspace, starts `agi-llm-router`, rosbridge, chat, telepathy |
| `shell/` | [AGi-robot/shell](https://github.com/AGi-robot/shell) | ROS 2 workspace, component links and `aiva_presence` |
| `cypher/` | [AGi-robot/cypher](https://github.com/AGi-robot/cypher) | Shared ROS messages and codec (`cryptic_signal_core`) |
| `telepathy/` | [AGi-robot/telepathy](https://github.com/AGi-robot/telepathy) | Grace entity receptor (`telepathy_receptor`) |
| `console/` | [AGi-robot/console](https://github.com/AGi-robot/console) | External WebUI, encoding and rosbridge transport |
| `interactive/` | [AGi-robot/interactive](https://github.com/AGi-robot/interactive) | Text conversation engine used by the `telepathy_receptor` ROS adapter |
| `health/` | [AGi-robot/health](https://github.com/AGi-robot/health) | ROS logging and system-health components; `agi_logging` is linked into Shell |
| `mind/memory/` | [AGi-robot/memory](https://github.com/AGi-robot/memory) | Pure Python memory library installed into AIVA's venv |

The component repositories are intentionally separate from `OppaAI/AGi` and
gitignored here; commit inside each one. The ROS workspace links Cypher,
Telepathy, and Health under `shell/src/`; Interactive and Memory install into
AIVA's root venv as Python packages. Canvas, Imaging, Interactive, and Memory
install from their local checkouts through `AIVA/requirements.txt`.

## Checkout

Clone this development workspace and run `AIVA/setup.sh`; it fetches the
component repositories at `dev` when they are missing. Existing checkouts are
preserved on their current branch. The setup script also checks out Canvas and
Imaging under `capability/` and links ROS packages into Shell.

## First-time setup and local settings

After the coordinated Shell, Telepathy, and Console changes have landed on
their `dev` branches, run `AIVA/setup.sh`. On its first run it creates an
untracked `AIVA/.env` from `AIVA/.env.example` and exits so you can review local
settings. Run it again to clone missing sibling repositories at `dev`, create
the Shell component links, install the editable Python packages from
`requirements.txt`, install ROS dependencies with `rosdep`, and build the ROS
workspace. Existing Git checkouts are left on their current branch and are not
updated automatically.

`AIVA/launch.sh` and `AIVA/setup.sh` load `.env` before reading settings. The
example groups ROS/Python paths, model endpoints, local console and ROS bridge,
memory/log paths, and optional Canvas/Imaging settings. Values left blank use
the code defaults. Keep API keys and machine-specific values in `.env`; it is
ignored by Git. `.env` is sourced as a Bash file, so only use trusted local
content. Scorecard-only settings stay in Scorecard rather than Grace's runtime
environment.

The current Console Settings page is **read-only** for runtime values. Its theme
and other browser-local preferences apply immediately. Environment and
`launch.sh` settings (model endpoints, model selection, memory path, service
ports, ROS bridge, Canvas/Imaging options, and MOSS paths) are captured when
processes start, so changing `.env` requires restarting Grace's launcher.
There is no live ROS parameter callback for these settings yet. A future editor
should label each setting as immediate or restart-required and save through a
validated configuration interface rather than presenting the current status
page as an editor.

The setup script requires ROS 2 Jazzy, `colcon`, and initialized `rosdep`. It
may invoke the system package manager through `rosdep`; install missing ROS
dependencies with the platform's usual permissions, then rerun setup.
