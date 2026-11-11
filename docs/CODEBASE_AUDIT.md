# Codebase audit — November 2026

This is a first repository-wide static and regression-test pass, followed by
targeted runtime review. It is not a claim that every feature is bug-free.

## Fixed in this pass

- Canvas ellipses now use their declared width and height without rotating the
  painted geometry outside the retained object's bounds.
- Clicking the canvas no longer scrolls it under the pointer. Shapes and paths
  stay inside the artboard when moved; their saved geometry survives reload.
- Selection moves now use the same 50-entry undo limit as drawing operations.
- A stopped replay cannot report a stale failure against a later request.
- Legacy AuRoRA TOC uses its actual callback, connection, event loop, server,
  thread, exception types, and output-port manifest entry. Broadcast iteration
  tolerates a connection disappearing during an awaited send.
- Legacy TIC buffers and publishes the real SSS `content` field. Invalid echo
  envelopes and nonfinite/negative expiry values no longer escape the callback.
- Removed five duplicate unused ROS-launch imports from TIC.
- Manual model and SQLite diagnostics run only as scripts, so collecting tests
  cannot download/load a model or terminate the test runner.
- The console endpoint regression now exercises the current typed telepathic
  publisher instead of importing the removed binary encoder endpoint.

## Verification and boundaries

- Combined selected regression run: **47 passed, 1 skipped, 5 passing subtests**.
- All Python modules parsed, and Ruff F821/F823 checks pass across this repo.
- AIVA chat/drawing/memory/adapter and cryptic-core tests, current console/codec
  tests, new legacy-adapter regressions, and diagnostic import checks pass.
- Chromium verifies actual ellipse pixels and move/save/reload geometry.
- The real ROS graph smoke test remains opt-in (`AIVA_TEST_WITH_ROS=1`).
- The old `AuRoRA/tests/tms/test_tic_unit.py` suite still targets removed TIC
  class/method names and obsolete SSS fields. Migrate its cases to the current
  pipeline before treating an unrestricted AuRoRA test run as a release gate.

## Remaining review areas

1. Run the real ROS launch/shutdown/topic tests on both AIVA and AuRoRA.
2. Audit console settings persistence, authentication/access assumptions, and
   concurrent configuration writes beyond the canvas paths covered here.
3. Review memory lifecycle and cancellation under real model failures.
4. Measure Jetson 8 GB RAM and response latency with actual models and sensors.
5. Review Unity/VRM interaction and deployed hardware independently of the
   browser geometry tests. Vector rendering tests do not establish realistic
   image-generation quality.
