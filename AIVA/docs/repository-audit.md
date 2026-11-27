# AIVA repository naming and defect audit

This audit covers all 16 repositories accessible in Project-AGi and OppaAI/AGi. It reviews current development sources, including the four existing lighting PR branches so their pending work does not reintroduce old technical naming.

All changes are proposed through pull requests. No pull request was merged, no release was published, and no model inference or paid API request was run.

## Repository changes and checks

| Repository | Pull request | Target | Local validation |
|---|---|---|---|
| Project-AGi/.github | [#4](https://github.com/Project-AGi/.github/pull/4) | dev | Documentation / link review |
| OppaAI/AGi | [#48](https://github.com/OppaAI/AGi/pull/48) | dev | 21 adapter tests |
| Project-AGi/agentic | [#5](https://github.com/Project-AGi/agentic/pull/5) | dev | 5 tests |
| Project-AGi/augmented-reality | [#4](https://github.com/Project-AGi/augmented-reality/pull/4) | dev | Static C# / asset review; Unity pending |
| Project-AGi/canvas | [#5](https://github.com/Project-AGi/canvas/pull/5) | dev | 41 tests |
| Project-AGi/console | [#6](https://github.com/Project-AGi/console/pull/6) | dev | 11 Python + 5 browser tests |
| Project-AGi/cypher | [#2](https://github.com/Project-AGi/cypher/pull/2) | dev | 10 tests |
| Project-AGi/health | [#2](https://github.com/Project-AGi/health/pull/2) | dev | Syntax / lint review |
| Project-AGi/imaging | [#3](https://github.com/Project-AGi/imaging/pull/3) | dev | Mock sidecar checks through Telepathy |
| Project-AGi/interactive | [#6](https://github.com/Project-AGi/interactive/pull/6) | dev | 11 tests |
| Project-AGi/memory | [#5](https://github.com/Project-AGi/memory/pull/5) | dev | 14 tests |
| Project-AGi/research | [#2](https://github.com/Project-AGi/research/pull/2) | dev | 2 tests |
| Project-AGi/scorecard | [#19](https://github.com/Project-AGi/scorecard/pull/19) | main | 40 tests; 5 optional fixtures skipped |
| Project-AGi/shell | [#11](https://github.com/Project-AGi/shell/pull/11) | dev | 6 tests |
| Project-AGi/telekinesis | [#3](https://github.com/Project-AGi/telekinesis/pull/3) | dev | 1 cleanup test |
| Project-AGi/telepathy | [#8](https://github.com/Project-AGi/telepathy/pull/8) | dev | 88 tests; live ROS module skipped |
| Project-AGi/toolbox | [#4](https://github.com/Project-AGi/toolbox/pull/4) | dev | 34 tests |

Scorecard currently has only `main`, so its PR targets that branch. All other PRs target `dev`. The four existing lighting PRs were updated; thirteen new PRs were opened.

## Naming and data boundaries

Code identifiers, commands, package metadata, documentation, asset filenames, and custom benchmark paths use AIVA. The personal identity stays in SOUL.md and YAML, and identity tests read configured values instead of hardcoding a personal name. Ordinary English words describing orderly shutdown or error handling are unaffected.

The existing conversation database filename is retained through Telepathy's packaged `config/storage.yaml`; the `AIVA_MEMORY_PATH` environment override and ROS `memory_path` parameter remain available. The naming cleanup does not relocate the user's database.

The benchmark persona was moved to `benchmarks/aiva_chatbot_eval/prompts/SOUL.md` without changing its bytes. Its SHA-256 is `6712d548c31b11406372b747564b53eeeb3730fffb3eb79511f1112817ccf379`.

Deployed model URLs stay in Scorecard's `endpoints.yaml`, preserving the real existing deployments. The archived EQ-Bench v2 question payload is retained byte-for-byte as JSON-compatible YAML, and the archival reader still reads the same JSON representation. Its SHA-256 is `b4e9bf11df7e88bc12381efab18565f8b0ff0d4743fe8708c4482df91c4e617a`. Upstream question text was not rewritten to rename fictional characters.

All 27 moved PNG/WebP/VRM assets retain identical Git blob hashes. Previously tracked render artifacts remain tracked after their folder move despite the bulk-output ignore rules. Existing measured scores were not regenerated or reinterpreted.

## Confirmed defects fixed

| Component | Defect | Result |
|---|---|---|
| Agentic | Tool calls could execute after a truncated response, or with ambiguous call IDs. | Require a complete tool-call finish and unique IDs before executing any tool. |
| Memory | Recording an action before the first exchange in a new conversation could violate a foreign key. | Create the conversation row before recording the event. |
| Memory | Empty event-recall queries produced an invalid SQLite ORDER BY ordinal. | Use recent-event order directly when no query terms exist. |
| Research | Fractional/boolean limits and malformed provider result arrays failed inside slicing. | Validate the request and provider structure explicitly. |
| Telekinesis | A browser-close exception during failed setup could prevent Playwright cleanup. | Always stop the Playwright runtime in the cleanup path. |
| Inference performance | A second function definition hid complete model/projector discovery. | Keep one implementation that inspects multiple workers and projection files; tolerate a missing alias value. |
| Benchmark launcher | Dry runs and guide requests required secrets and could probe model servers. | Return dry runs/guides before credential checks and preflight. |
| Chatbot evaluation | The configured API key was not forwarded to authenticated endpoints. | Forward the configured key in the request Authorization header. |
| Canvas evaluation | Importing the evaluator required an orchestrator checkout even with Canvas installed. | Support installed capabilities; browser rendering still requires an explicitly available Console source. |
| Console | A literal activity-path rename would collide with its presence route; the manifest also repeated a dependency. | Use `/api/activity` for activity and retain `/api/presence/aiva` for online presence; remove the duplicate dependency. |
| AuRoRA output | WebSocket server startup referenced an unimported module. | Import the module used by the server call. |
| AuRoRA tests | Retired endpoint/module tests and oversized stubs no longer matched the active API. | Remove the deleted-endpoint test and exercise the current TIC gates with canonical SSS blueprints and minimal ROS doubles. |
| Shell tests | Launcher lookup depended on an old checkout layout and did not copy bootstrap files. | Locate the sibling launcher or explicit checkout and copy its spec/bootstrap into the fixture. |
| Android AR | Camera input actions were not disposed; teardown assumed full initialization. | Dispose both actions and guard partially initialized reply components. |
| AGi CI | The Console checkout was pinned to its early implementation and lacked current identity dependencies. | Exercise the current development interface and include Telepathy and Shell signal sources. |

Unused imports, repeated imports, dead local assignments, and redundant f-string prefixes were also removed from owned code.

## Validation

- 284 Python tests passed across the checked suites, plus five browser tests.
- Six optional checks were skipped: five official benchmark fixtures without their configured upstream checkouts and one live ROS test module.
- Python syntax, shell syntax, JavaScript syntax, YAML/XML parsing, Ruff F checks, and Git whitespace checks passed for the reviewed owned sources.
- Pinned official scorer sources were preserved. Their three existing unused typing imports were excluded from cleanup to avoid changing scorer snapshots.
- Real Chromium rendering checks passed after supplying the browser runtime.
- CLI help and editable installation checks passed for the renamed commands and capability distributions.

These checks do not establish that every possible defect is absent. Unity/Android builds, live ROS Jazzy delivery, actual GPU image generation, physical OpenRGB output, and real TTS playback were not available for end-to-end validation in this workspace. Existing FastAPI and test-dependency deprecation warnings remain.

## Coordinated update

Install or rebuild the affected components together after review. The main changed interfaces are:

- Commands: `aiva-agent` and `aiva-lighting`.
- Capability distribution names: `aiva-canvas` and `aiva-imagegen`; Python imports remain `canvas` and `imagegen`.
- Adapter class: `AivaMemory`.
- Custom suites: `benchmarks/aiva_canvas_eval` and `benchmarks/aiva_chatbot_eval`.
- Console activity endpoint: `/api/activity`.
- Lighting service configuration: `AGI_LIGHTING_SERVICE_URL`, with its independent service on loopback port 8854.

Refresh the WebUI assets after rebuilding Console. TTS remains a prepared interface: response heartbeat is wired, and future playback-volume input has priority. Console still owns only the automatic activity fallback.
