# Organization audit — 2026-11-19

## Scope and method

Reviewed all 12 repositories returned by the AGi-robot organization inventory.
The baseline is each repository's `dev` branch where present, otherwise its
default branch. The in-flight Console/Receptor changes were reviewed separately
at their PR heads because the baseline Console branch contains only a license.

The baseline inventory contains 469 tracked regular files. Python syntax, shell
syntax, JavaScript syntax, JSON and YAML parse checks passed for the applicable
tracked files. Source review focused on startup dependencies, message contracts,
stream completion, persistence, process lifecycle, rendering adapters and Unity
reply/VRM lifecycle. Binary assets were inventoried, not semantically audited.
This is a code audit with focused runtime tests, not a proof that every behavior
is correct or a completed deployment/device validation.

## Repository coverage

| Repository | Baseline commit | Findings and evidence |
|---|---|---|
| [console](https://github.com/AGi-robot/console) | `c924fe33aa06` | Reviewed Console #1 implementation separately; its transport, presence and settings fixes are included there. Five Python tests and four Canvas JS tests passed. |
| [shell](https://github.com/AGi-robot/shell) | `55b21bd4f1da` | Reviewed Shell #10 migration. Two launcher lifecycle checks passed; scheme registry retained as a library. |
| [telepathy](https://github.com/AGi-robot/telepathy) | `74cdd16ae01d` | Reviewed Telepathy #5 receptor migration; 83 tests passed and one ROS-only module skipped. Removed unused test imports. |
| [health](https://github.com/AGi-robot/health) | `933964a0439f` | Source review plus isolated routing, JSON, broadcast and file-rotation smoke check using ROS doubles. No new source defect identified. |
| [scorecard](https://github.com/AGi-robot/scorecard) | `8819b0db2509` | Rescore used a hardcoded checkout path; align with evaluator discovery and remove dead imports. 15 tests and four subtests passed; four optional fixtures skipped. |
| [interactive](https://github.com/AGi-robot/interactive) | `5a284214ed2c` | Interrupted nonempty SSE streams could be saved as complete. Require a completion marker; three regression cases passed. |
| [memory](https://github.com/AGi-robot/memory) | `f21422966a4e` | Append after an unterminated record could lose the new memory; failed writes could appear committed in process memory. Ten tests passed with recovery regressions. |
| [imaging](https://github.com/AGi-robot/imaging) | `b74b12fff679` | Reviewed sidecar/client and bridge; exercised subprocess behavior via receptor tests. Added the missing root README. CUDA image generation not run. |
| [canvas](https://github.com/AGi-robot/canvas) | `e68293adda33` | Cross-suite registry mutation made a vector-only test order dependent. Isolated it; removed dead assignments/imports and an always-true assertion. Forty tests passed. |
| [cypher](https://github.com/AGi-robot/cypher) | `35b53ed6988c` | Reviewed codec/message contract and invalid-signal handling. Ten codec tests passed; the AGi ROS CI job also built the messages. |
| [.github](https://github.com/AGi-robot/.github) | `ab69e1ee4c84` | Repository map still named AIVA and assigned the WebUI to Telepathy. Updated ownership, links and missing component entries. |
| [augmented-reality](https://github.com/AGi-robot/augmented-reality) | `baf8b01f9630` | Old reply topic, null address handling, shared-stream image size cap and late VRM load cleanup. Static source/config review only; Unity build remains required. |

## Fix PRs

- [memory](https://github.com/AGi-robot/memory/pull/3)
- [interactive](https://github.com/AGi-robot/interactive/pull/3)
- [canvas](https://github.com/AGi-robot/canvas/pull/3)
- [scorecard](https://github.com/AGi-robot/scorecard/pull/12)
- [augmented-reality](https://github.com/AGi-robot/augmented-reality/pull/3)
- [.github](https://github.com/AGi-robot/.github/pull/2)
- [imaging](https://github.com/AGi-robot/imaging/pull/2)

The Console, Telepathy and Shell migration changes remain in Console #1,
Telepathy #5 and Shell #10. AGi #42 wires those components together. Preserve
inherited licenses and official scorer sources; unused imports inside vendored
IFEval scorers were deliberately left unchanged. The retired Shell adapter has
unused test imports on baseline dev; its package is removed by Shell #10 and
those imports were removed from the migrated receptor tests.

## AGi #42 review and CI fix

Both CI jobs previously checked out Console's empty `dev` branch. They now use
the same immutable populated revision `ce9af3a65df0ed8778e44aea974cd6a2ce28e35f`.
Checkout credentials are not persisted and workflow permissions are read-only.
The populated Console layout is checked explicitly. Both shared-console and
ROS Jazzy jobs passed on `d021a7e`; the CodeRabbit thread is resolved.
The pin makes CI independent of merge order; update it deliberately when the
Console contract changes. This does not merge any component PR.

## Remaining deployment checks

- Rebuild the matched component versions on Grace's PC and verify the actual
  receptor graph, a browser conversation, restart persistence and shutdown.
- A rosbridge connection/publication confirms transport submission, not that
  a receptor processed the request. Verify delivery and startup timing on ROS.
- Unity Android compilation, scene/VRM teardown, AR placement and reply activity
  require Unity and a phone. AR still mirrors text activity; it does not receive
  or synthesize WebUI speech audio.
- MossInfer/CUDA compatibility, VRAM usage and real image generation require
  the separately installed binary, weights and GPU.
- Optional pinned IFBench/LiveCodeBench scorer fixtures were not enabled;
  four such fixtures skipped. No model benchmark scores were produced.

Organization defaults currently point to `main`; the coordinated unreleased
migration targets `dev`. Development setup commands must select `dev` explicitly.
