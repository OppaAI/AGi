# AGi · AuRoRA · GRACE — Milestone 1 Review

**Reviewer:** Antigravity (Opus 4.6)
**Date:** 2026-05-04
**Tag reviewed:** `0.1.0` (commit `817de39`)
**Sources:** README.md M1 spec, SCS CHANGELOG.rst, HRS CHANGELOG.rst, GitHub v0.1.0 release notes, full source inspection of `cnc.py`, `mcc.py`, `wmc.py`, `emc.py`, `msb.py`, `hrp.py`, `AGi.html`, `grace_ui.py`

---

## 1. M1 Spec vs. Delivered — Line-by-Line

The M1 spec is defined in [README.md](file:///home/oppa-ai/AGi/README.md#L106-L120) under **"M1 — Chatbot + WMC + EMC"**. All 11 checklist items are marked `[x]` (complete). Below is verification against the tagged code.

| # | M1 Spec Item | Verdict | Evidence |
|---|---|---|---|
| 1 | PMT lifecycle with hybrid chunk/slot eviction (Miller's Law 7±2) | ✅ **Met** | [wmc.py:197–200](file:///home/oppa-ai/AGi/AuRoRA/src/scs/scs/wmc.py#L197-L200) — dual-guard eviction loop checks `global_chunk_limit` AND `pmt_slot_limit + pmt_slot_buffer`. [hrp.py:138–139](file:///home/oppa-ai/AGi/AuRoRA/src/hrs/hrs/hrp.py#L138-L139) — `PMT_SLOT_LIMIT=7`, `PMT_SLOT_BUFFER=2`. |
| 2 | Async embedding worker via BAAI/bge-base-en-v1.5 | ✅ **Met** | [msb.py:162–165](file:///home/oppa-ai/AGi/AuRoRA/src/scs/scs/msb.py#L162-L165) — `EncodingEngine` loads `SentenceTransformer(self.encoding_engine)`. [hrp.py:112](file:///home/oppa-ai/AGi/AuRoRA/src/hrs/hrs/hrp.py#L112) — `ENCODING_ENGINE = "BAAI/bge-base-en-v1.5"`. Async via [emc.py:289–294](file:///home/oppa-ai/AGi/AuRoRA/src/scs/scs/emc.py#L289-L294) background thread. |
| 3 | Semantic search with RRF fusion (semantic + lexical dual-path) | ✅ **Met** | [msb.py:597–613](file:///home/oppa-ai/AGi/AuRoRA/src/scs/scs/msb.py#L597-L613) — `recall_engram()` calls `_semantic_recall()` + `_lexical_recall()` → `_converge_memories()`. RRF formula at [msb.py:804–807](file:///home/oppa-ai/AGi/AuRoRA/src/scs/scs/msb.py#L804-L807). |
| 4 | SQLite WAL episodic storage — no expiry, 1TB NVMe | ✅ **Met** | [msb.py:309](file:///home/oppa-ai/AGi/AuRoRA/src/scs/scs/msb.py#L309) — `PRAGMA journal_mode=WAL`. No TTL/expiry logic anywhere in EMC. |
| 5 | Conflict/versioning columns in EMC schema (prep for M2b) | ⚠️ **Not in code** | README marks `[x]` for `conflict`, `superseded_by`, `valid_from`, `valid_until` in the EMC schema. **These columns do not exist** in `EMC_SCHEMA` ([emc.py:143–162](file:///home/oppa-ai/AGi/AuRoRA/src/scs/scs/emc.py#L143-L162)). Only `id`, `timestamp`, `date`, `content`, `encoding`, `created_at` are defined. See **Finding F1** below. |
| 6 | Importance columns in EMC schema (prep for M2a) | ⚠️ **Not in code** | README marks `[x]` for `memory_strength`, `last_recalled_at`, `recall_count`, `novelty_score`. **These columns do not exist** in `EMC_SCHEMA`. Same gap as item 5. See **Finding F1** below. |
| 7 | Register user turn before LLM stream — amnesia fix | ✅ **Met** | [cnc.py:207](file:///home/oppa-ai/AGi/AuRoRA/src/scs/scs/cnc.py#L207) — `register_memory("user", user_prompt)` precedes `_stream_gce()` at line 232. CHANGELOG line 25 documents the fix. |
| 8 | Double user message guard in WMC `_induced_pmt` | ✅ **Met** | [wmc.py:137–145](file:///home/oppa-ai/AGi/AuRoRA/src/scs/scs/wmc.py#L137-L145) — `if self._induced_pmt is not None:` appends to existing prompt rather than overwriting. |
| 9 | Schema versioning in MSB — `schema_meta` table | ✅ **Met** | [msb.py:368–407](file:///home/oppa-ai/AGi/AuRoRA/src/scs/scs/msb.py#L368-L407) — `schema_meta` table created, version checked on boot, `RuntimeError` on mismatch. Legacy DB guard at line 383. |
| 10 | Binding stream `maxlen` cap — OOM guard | ✅ **Met** | [emc.py:173–174](file:///home/oppa-ai/AGi/AuRoRA/src/scs/scs/emc.py#L173-L174) — `deque(maxlen=EMC.BINDING_STREAM_LIMIT)`. Warning at capacity: [emc.py:524–528](file:///home/oppa-ai/AGi/AuRoRA/src/scs/scs/emc.py#L524-L528). |
| 11 | Trivial PMT length filter at WMC→EMC eviction boundary | ✅ **Met** | [mcc.py:159–164](file:///home/oppa-ai/AGi/AuRoRA/src/scs/scs/mcc.py#L159-L164) — strips JSON overhead, discards content under 20 chars. |

### Summary: 9/11 spec items fully verified. 2 items (rows 5–6) are marked done in the README but absent from the codebase.

---

## 2. Findings

### F1 — Phantom Schema Columns (M2a/M2b prep)

> [!WARNING]
> The README M1 checklist marks two items as `[x]` complete, but the columns **do not exist** in the shipped `EMC_SCHEMA`:
>
> **Missing conflict/versioning columns:** `conflict`, `superseded_by`, `valid_from`, `valid_until`
> **Missing importance columns:** `memory_strength`, `last_recalled_at`, `recall_count`, `novelty_score`

The `EMC_SCHEMA` in [emc.py:143–162](file:///home/oppa-ai/AGi/AuRoRA/src/scs/scs/emc.py#L143-L162) defines only:
```
storage: id, timestamp, date, content, encoding, created_at
staging: id, timestamp, date, content
```

**Impact:** Low — these are prep-only columns not consumed by any M1 logic. However, the README checklist is misleading. Either:
- (a) The columns were present in an earlier schema and were intentionally removed during the `EngramComplex` refactor (conversation `6d7a8af2`), or
- (b) They were never added and the checklist was prematurely checked.

**Recommendation:** Uncheck items 5–6 in the README, or add the columns to `EMC_SCHEMA` before M1.5. Adding unused nullable columns now (with `essential=False`) is zero-cost and avoids a schema migration at M2a.

---

### F2 — v0.1.0 Release Notes Describe M1.5 Scope, Not M1

> [!IMPORTANT]
> The GitHub release notes for `0.1.0` include a "Next" section that describes **M1.5 work as if it were already shipped**:
>
> - ✍️ "Anchor vector semantic filtering (embeddinggemma)" — listed under "Next" but described as a capability
> - ✍️ "Session-end WMC flush"
> - ✍️ "Basic user profile store (user_profile.json)"
> - ✍️ "Episodic scaffold"
> - ✍️ "SharedChunkEstimator"
> - ✍️ "Anti-hallucination grounding"
> - ✍️ "NeuralTextInput typed schema"
> - ✍️ "`user_id` threaded through CNC → MCC → WMC"
> - ✍️ "`pack_vector` / `normalize_vector` migrated to `hrs.hrp` under `AGi.Math`"
> - ✍️ Agentic tools (weather, moon, aurora)

**None of these exist in the tagged `0.1.0` codebase:**
- No `scs/types.py` or `NeuralTextInput` anywhere
- No `user_profile.json` or user profile logic
- No `ChunkEstimator` class
- No `AGi.Math` namespace in [hrp.py](file:///home/oppa-ai/AGi/AuRoRA/src/hrs/hrs/hrp.py)
- `pack_vector` / `normalize_vector` remain in [msb.py:104–131](file:///home/oppa-ai/AGi/AuRoRA/src/scs/scs/msb.py#L104-L131)
- No agentic tool modules exist
- CNC still uses hardcoded `"user"` / `"assistant"` speaker literals — no `user_id` parameter

**Impact:** The release notes create confusion about what `0.1.0` actually ships. The "Next" section appears to be a forward-looking roadmap pasted into the release body without a clear "Not yet implemented" header.

**Recommendation:** Rewrite the v0.1.0 release notes to clearly separate "Shipped in 0.1.0" from "Planned for M1.5". Add a `---` divider and a `### Planned for M1.5` header.

---

### F3 — CLI Not Registered as Entry Point

> [!NOTE]
> The CLI (`grace_ui.py`) exists in `/tests/` but is not registered as a `console_scripts` entry point in [setup.py](file:///home/oppa-ai/AGi/AuRoRA/src/scs/setup.py#L25-L29). Only `cnc` is registered.

The CHANGELOG line 24 claims `grace_cli.py` was introduced, but the file is actually named `grace_ui.py` and lives under `/tests/`, not `/src/scs/scs/`. It is functional (publishes/subscribes on the correct ROS topics) but not installable via `ros2 run scs grace_cli`.

**Recommendation:** Either register it as an entry point or document the manual invocation path. Also resolve the naming discrepancy (`grace_cli` vs `grace_ui`).

---

### F4 — No Unit Tests for WMC or EMC

> [!NOTE]
> The test directories (`src/scs/test/` and `src/hrs/test/`) contain only boilerplate ROS2 lint tests (`test_copyright.py`, `test_flake8.py`, `test_pep257.py`). No functional unit tests exist for WMC eviction logic, EMC encoding/recall, or MSB RRF fusion.

The M1 spec does not require tests. The M1.5 spec explicitly calls for:
- WMC unit tests — dual-guard eviction logic
- EMC unit tests — RRF recall correctness with known episodes
- Manual integration test suite — document pass criteria for M2a entry

This is correctly deferred.

---

### F5 — `setup.py` Description Still `TODO`

> [!NOTE]
> [setup.py:18](file:///home/oppa-ai/AGi/AuRoRA/src/scs/setup.py#L18) — `description='TODO: Package description'`. Minor polish item.

---

## 3. M1 Goal Assessment

**M1 Goal from README:** *"Grace can remember across sessions."*

| Criterion | Status |
|---|---|
| End-to-end chatbot loop (user → CNC → GCE → response) | ✅ Working |
| Working memory with capacity management | ✅ Working |
| Episodic memory persistence across restarts | ✅ Working (SQLite WAL, `emc_staging` crash recovery) |
| Semantic + lexical recall with RRF fusion | ✅ Working |
| Background encoding with theta-rhythm cycle | ✅ Working |
| Schema versioning preventing silent corruption | ✅ Working |
| Parameter centralization in HRP | ✅ Working |
| Web GUI for interaction | ✅ Present (3188-line `AGi.html`) |
| CLI for interaction | ✅ Present (`grace_ui.py`, functional, not installable) |

**Overall M1 verdict: ✅ PASS — the core goal is met.** GRACE can hold conversations with working memory and episodic recall across sessions. The cognitive pipeline (CNC → MCC → WMC/EMC → GCE) is fully operational.

---

## 4. Gaps to Close Before M1.5

| Gap | Severity | Recommendation |
|---|---|---|
| **F1** — Phantom schema columns checked in README | 🟡 Low | Uncheck items 5–6, or add the 8 nullable columns to `EMC_SCHEMA`. Zero runtime impact either way. Closing before M1.5 avoids a schema version bump mid-milestone. |
| **F2** — Release notes mix shipped vs planned | 🟡 Low | Edit the v0.1.0 GitHub release to clearly delimit M1 deliverables from M1.5 roadmap. |
| **F3** — CLI naming/registration | 🟢 Trivial | Rename `grace_ui.py` → `grace_cli.py`, move to `src/scs/scs/`, register entry point. Or document the manual run path. |
| **F5** — `setup.py` TODO description | 🟢 Trivial | Fill in the package description. |

> [!TIP]
> None of these gaps are blockers. M1.5 can begin immediately. F1 is the only item worth resolving before M1.5 since adding schema columns later forces a `SCHEMA_VERSION` bump and migration path.

---

## 5. Items Correctly Deferred to M1.5+

These are items that appear as TODO comments in the code or in the M1.5 spec, and are **not** expected at M1:

| Item | Defer Target | Source |
|---|---|---|
| Anchor vector PMT filtering (embeddinggemma) | M1.5 | [mcc.py:152](file:///home/oppa-ai/AGi/AuRoRA/src/scs/scs/mcc.py#L152) |
| Session-end WMC flush to EMC | M1.5 | [mcc.py:56–58](file:///home/oppa-ai/AGi/AuRoRA/src/scs/scs/mcc.py#L56-L58) |
| `NeuralTextInput` typed schema | M1.5 | [cnc.py:50–52](file:///home/oppa-ai/AGi/AuRoRA/src/scs/scs/cnc.py#L50-L52) |
| Multi-user `user_id` | M1.5 | [cnc.py:53–54](file:///home/oppa-ai/AGi/AuRoRA/src/scs/scs/cnc.py#L53-L54) |
| `ChunkEstimator` with real tokenizer | M1.5 | README M1.5 spec |
| Busy queue (buffer vs drop) | M1.5 | [cnc.py:59](file:///home/oppa-ai/AGi/AuRoRA/src/scs/scs/cnc.py#L59) |
| Agentic tools (weather, moon, aurora) | M1.5 | README M1.5 spec |
| WMC/EMC unit tests | M1.5 | README M1.5 spec |
| `_strip_model_artifacts()` | M2 | [cnc.py:55–56](file:///home/oppa-ai/AGi/AuRoRA/src/scs/scs/cnc.py#L55-L56) |
| Salience gate on assistant response | M2 | [cnc.py:57–58](file:///home/oppa-ai/AGi/AuRoRA/src/scs/scs/cnc.py#L57-L58) |
| Dream Cycle (SMC distillation) | M2 | [emc.py:94](file:///home/oppa-ai/AGi/AuRoRA/src/scs/scs/emc.py#L94) |
| YAML config migration | M1.6 | README M1.6 spec |
| BioLogic Clock (SCN) | M1.6 | README M1.6 spec |
| `EpisodicScaffold` class | M1.5 → M2 | [emc.py:103–108](file:///home/oppa-ai/AGi/AuRoRA/src/scs/scs/emc.py#L103-L108), TODO header says M2 |

> [!NOTE]
> There's a discrepancy on `EpisodicScaffold`: the README M1.5 spec lists it as M1.5 work, but the `emc.py` TODO header labels it M2. Clarify ownership before M1.5 begins.

---

## 6. Quality Observations

- **Documentation quality is high.** Every module has a thorough docstring header with architecture notes, terminology, public interface, and TODO markers with milestone tags.
- **Neuroscience metaphor consistency is strong.** Naming is internally consistent (PMT, engram, theta rhythm, binding stream, etc.) across all 5 SCS modules and HRP.
- **Thread safety model is clear.** Each file documents its concurrency guarantees. The CNC attention gate is correctly TOCTOU-safe (closes before `run_coroutine_threadsafe`).
- **No dead code or orphan files** in the tagged release.
- **HRP three-tier hierarchy** (STATIC/INTRINSIC/EXTRINSIC) is well-designed but purely documentary at M1 — all values are hardcoded class attributes. YAML migration is correctly deferred to M1.6.
