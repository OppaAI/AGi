# AIVA — Amazing Grace (PC companion)

AIVA is Grace's PC-side runtime. The `launch.sh` orchestrator at this level
starts the llama.cpp router, the shell ROS 2 workspace services, and wires in
the shared capabilities.

## Layout

| Path | Tracked in | Contents |
|---|---|---|
| `launch.sh` | **this repo** (`OppaAI/AGi`) | Grace's launcher: builds the shell workspace, starts `agi-llm-router`, rosbridge, chat, telepathy |
| `shell/` | [AGi-robot/shell](https://github.com/AGi-robot/shell) | ROS 2 workspace — `aiva_chat`, `agi_logging`, `aiva_presence` |
| `cypher/` | [AGi-robot/cypher](https://github.com/AGi-robot/cypher) | Shared ROS messages and codec (`cryptic_signal_core`) |
| `telepathy/` | [AGi-robot/telepathy](https://github.com/AGi-robot/telepathy) | Shared WebUI and rosbridge transport (`telepathic_management`) |
| `interactive/` | [AGi-robot/interactive](https://github.com/AGi-robot/interactive) | Text conversation engine used by the `aiva_chat` ROS adapter |
| `mind/memory/` | [AGi-robot/memory](https://github.com/AGi-robot/memory) | Pure Python memory library installed into AIVA's venv |

The component repositories are intentionally separate from `OppaAI/AGi` and
gitignored here; commit inside each one. The ROS workspace links Cypher and
Telepathy under `shell/src/`; Interactive and Memory install into AIVA's root
venv as Python packages. Canvas, Imaging, Interactive, and Memory install from
their local checkouts through `AIVA/requirements.txt`.

## Checkout

From the AGi checkout root:

```sh
git clone --branch dev https://github.com/AGi-robot/shell.git AIVA/shell
git clone https://github.com/AGi-robot/cypher.git     AIVA/cypher
git clone https://github.com/AGi-robot/telepathy.git  AIVA/telepathy
git clone https://github.com/AGi-robot/interactive.git AIVA/interactive
git clone --branch dev https://github.com/AGi-robot/memory.git AIVA/mind/memory
```

The AIVA Shell adapter currently uses `memory.conversation.ConversationStore`,
which is available on Memory's `dev` branch. Use that branch until the feature
is promoted to Memory's `main` branch.

Then follow [`shell/README.md`](shell/README.md) for the venv, build, and
launch instructions.
