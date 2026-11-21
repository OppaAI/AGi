# AIVA — Amazing Grace (PC companion)

AIVA is Grace's PC-side runtime. The `launch.sh` orchestrator at this level
starts the llama.cpp router, the shell ROS 2 workspace services, and wires in
the shared capabilities.

## AIVA and AuRoRA configuration

`AIVA/launch.sh` runs AIVA's `telepathy_receptor` adapter, not
`AuRoRA/ras/ras.launch.py`. It does not start AuRoRA's CNC or TMS TIC, import
the HRM manifest into chat, or load `~/.agi/ras/aurora.yaml`. Chat responses
use `/telepathic/response`; AuRoRA's TMS topic and launch path are separate.

AIVA chat uses its declared ROS parameters and defaults, environment variables
such as `AIVA_LLM_MODEL` and `AIVA_LLM_BASE_URL`, the Shell `persona/SOUL.md`,
and `$AGI_HOME/profile/user.md`. Its conversation history defaults to
`$AGI_HOME/profile/grace_memory.sqlite3`. `AGI_HOME` is defined by
[`spec.yaml`](spec.yaml) and can be overridden in `.env`. See the Shell README
for persona and profile overrides.
The AuRoRA YAML is not required for this AIVA chat path.

AuRoRA has a separate launch path: `ras/ras.launch.py` supplies
`~/.agi/ras/aurora.yaml` to CNC, TIC, and its HTTP API. CNC and TIC declare
parameters using HRM defaults and apply matching ROS YAML overrides. HRM is a
Python manifest, not a running service. AuRoRA also reads its own user and
persona YAML files where configured. Starting AIVA alone does not start AuRoRA.

The current Console settings page is read-only for runtime configuration. Its
browser-local appearance preferences apply immediately; AIVA process settings
are read when the launcher starts and require a restart to change. Console does
not edit AIVA's `.env` or AuRoRA's parameter YAML, and its standalone runtime
does not require AuRoRA's HRM, SCS, or TMS packages. See
[`docs/telepathy-flow.md`](docs/telepathy-flow.md) for current topic and process
details.

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

The coordinated Shell, Telepathy, and Console changes are merged on their
`dev` branches. Run `AIVA/setup.sh`. On its first run it creates an
untracked `AIVA/.env` from `AIVA/.env.example` and exits so you can review local
settings. Run it again to clone missing sibling repositories at `dev`, create
the Shell component links, install the editable Python packages from
`requirements.txt`, install ROS dependencies with `rosdep`, and build the ROS
workspace. Existing Git checkouts are left on their current branch and are not
updated automatically.

`AIVA/launch.sh` and `AIVA/setup.sh` load `AIVA/.env` before reading settings.
Workspace identity and component routes are listed in [`spec.yaml`](spec.yaml).
`wakeup.py` resolves the `tracts:` entries into absolute paths, derives
`AGI_HOME` from the entity identity, and reports missing or invalid settings
before setup or launch continues. Component paths are relative to the spec file;
Canvas and Imaging routes are relative to `agi_root`. `.env` remains for
non-secret, machine-specific runtime overrides. It is sourced as Bash, so only
use trusted local content. API keys are not part of the path resolver; moving
their current environment-variable consumers to an OS credential store remains
separate work.
Scorecard-only settings stay in Scorecard rather than Grace's runtime
environment. AuRoRA keeps its own separate configuration.

The current Console Settings page is **read-only** for runtime values. Its theme
and other browser-local preferences apply immediately. Environment and
`launch.sh` settings (model endpoints, model selection, memory path, service
ports, ROS bridge, Canvas/Imaging options, and MOSS paths) are captured when
processes start, so changing `.env` requires restarting Grace's launcher.
There is no live ROS parameter callback for these settings yet. A future editor
should label each setting as immediate or restart-required and save through a
validated configuration interface rather than presenting the current status
page as an editor.

The setup script requires the ROS installation configured by `tracts.ros_setup`
in [`spec.yaml`](spec.yaml), `colcon`, initialized `rosdep`, and PyYAML for the
bootstrap Python. It may invoke
the system package manager through `rosdep`; install missing ROS dependencies
with the platform's usual permissions, then rerun setup.
