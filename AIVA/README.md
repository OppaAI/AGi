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

From the AGi checkout root:

```sh
git clone --branch dev https://github.com/AGi-robot/shell.git AIVA/shell
git clone --branch dev https://github.com/AGi-robot/cypher.git     AIVA/cypher
git clone --branch dev https://github.com/AGi-robot/telepathy.git  AIVA/telepathy
git clone --branch dev https://github.com/AGi-robot/console.git AIVA/console
git clone --branch dev https://github.com/AGi-robot/interactive.git AIVA/interactive
git clone --branch dev https://github.com/AGi-robot/health.git      AIVA/health
git clone --branch dev https://github.com/AGi-robot/memory.git AIVA/mind/memory
ln -s ../../health AIVA/shell/src/agi_logging
```

The AIVA Shell adapter currently uses `memory.conversation.ConversationStore`,
which is available on Memory's `dev` branch. Use that branch until the feature
is promoted to Memory's `main` branch.

Then follow [`shell/README.md`](shell/README.md) for the venv, build, and
launch instructions.
