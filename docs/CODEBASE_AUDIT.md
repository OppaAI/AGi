# Codebase audit — November 2026

The cloud static/regression audit and follow-up fixes are complete for this
checkout. This is a verification boundary, not a claim that every feature is
bug-free or that deployed hardware has been exercised.

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

- Combined selected regression run: **94 passed, 1 skipped, 5 passing subtests**.
- All Python modules parsed, and Ruff F821/F823 checks pass across this repo.
- AIVA chat/drawing/memory/adapter and cryptic-core tests, current console/codec
  tests, new legacy-adapter regressions, and diagnostic import checks pass.
- Chromium verifies actual ellipse pixels and move/save/reload geometry.
- The real ROS graph smoke test remains opt-in (`AIVA_TEST_WITH_ROS=1`).
- Migrated the old AuRoRA input suite to current TIC methods and real SSS fields.
  Its ROS doubles are fixture-scoped and no longer replace unrelated libraries
  during test collection.
- Input reception now creates/preserves request IDs, rejects malformed sender
  fields, and stamps external WebSocket input as user role.
- Console settings preserve ROS selector boundaries, clean up failed temporary
  writes, validate stored numeric types, and support an explicit
  `AGI_AURORA_SETTINGS_PATH` on hosts without HRS. The default reports a useful
  503 when AuRoRA configuration is unavailable.
- Grace's SQLite transactions now close their connections on success and failure,
  with rollback and handle-lifetime regression coverage.


## Deployment checks still required

1. Run real ROS launch/shutdown/topic tests on AIVA and AuRoRA, including the
   ament lint hooks. This cloud image has no ROS/ament installation.
2. Validate the actual Tailscale ACL/Serve configuration. The console trusts its
   loopback/tailnet deployment boundary; test doubles do not verify that boundary.
   Settings writes are atomic but multiple independent server processes writing
   the same file are not supported as a shared configuration service.
3. Exercise inference failures and launch shutdown against a live model server.
   Already-running HTTP calls are bounded by their timeouts, not cancelled by
   executor shutdown.
4. Measure Jetson 8 GB RAM and response latency with actual models and sensors.
5. Review Unity/VRM interaction and deployed hardware independently of the
   browser geometry tests. Vector rendering tests do not establish realistic
   image-generation quality.
