# P1M1 — Chatbot + Working Memory + Episodic Memory

**Phase:** 1 — Chatbot with Memory
**Tag:** v0.1.0
**Status:** ✅ Complete

---

## 📝 1. Milestone Summary

This milestone establishes GRACE's foundational memory architecture through
a working chatbot interface. The Working Memory Cortex (WMC) serves as
short-term memory, holding the active context of an ongoing conversation,
while the Episodic Memory Cortex (EMC) acts as an intermediate layer —
bridging the gap between transient working memory and a long-term store not
yet implemented. Long-term memory was deliberately deferred; it requires
careful architectural planning that goes beyond the scope of getting GRACE
online. The deeper motivation behind starting here was personal: by building
memory first, GRACE could be present for her own development — a
conversational witness to the process of her own creation.

---

## 📦 2. Scope

- **CNC** — ROS2 node, async loop, LLM streaming bridge
- **MCC** — Memory coordinator between WMC, EMC, and LLM context assembly
- **WMC** — Short-term active conversation window with dual-guard eviction
  and PMT lifecycle management
- **EMC** — SQLite-backed episodic store with async encoding, crash-safe
  staging, and dual-path RRF retrieval (semantic + lexical)
- **MSB** — Shared storage substrate housing encoding engine, SQL operations,
  schema generation, and RRF fusion utilities
- Five bug fixes finalized before v0.1.0 tag
- Acceptance tested across two sessions with ~71 accumulated episodes

---

## 🚫 3. Non-Scope / Deferred

Everything below was explicitly out of scope for M1. Some items are deferred
to specific milestones; others are architectural decisions not yet made.

**Deferred to M1.5**
- Semantic anchor vector PMT filtering (embeddinggemma) — M1 uses length-based
  pre-filter only
- Session-end WMC flush to EMC on shutdown — evicted-but-not-encoded turns
  are lost on process exit
- Basic user profile store — personal facts rely entirely on EMC recall ranking
- Anti-hallucination grounding instruction in GRACE system prompt
- Unit tests (WMC eviction logic, EMC RRF correctness)
- Agentic tool calls

**Deferred to M1.6**
- Config refactor — all hardcoded parameters remain inline; no YAML loaders yet
- BioLogic Clock (SCN) — no shared time source; no circadian phase signals

**Deferred to M2a**
- Forgetting curve / Ebbinghaus decay on episodic memory
- Importance scoring (memory_strength, novelty_score, recall_count) — columns
  exist in schema as forward-prep; not populated
- Duplicate/similarity clustering and merging
- Dream Cycle — nightly reflection, deep consolidation, hard deletes
- Temporal filtering in recall — no recency decay in RRF ranking

**Deferred to M2b+**
- Semantic Memory Cortex (SMC) — distillation, structured facts, knowledge graph
- Memory versioning enforcement — conflict, superseded_by, valid_from, valid_until
  columns exist in schema but are not written or resolved

**Deferred to M3**
- Procedural Memory Cortex (PMC) — skill storage and execution
- Attentional gating and priority/salience scoring in MCC

**Deferred to Phase 2+**
- EEE (Emergency & Exception Event system)
- VCS (Vital Circulatory System — thermals, battery, motor watchdog)
- Visuospatial sketchpad — no multimodal or spatial WMC slot
- Spontaneous/associative recall — EMC only retrieves on explicit MCC query
- Emotional valence tagging on episodes

---

## ✅ 4. Success Criteria

M1 was considered complete when all of the following were verified in
a real two-session acceptance test (April 29–30, 2026):

- Personal facts recalled correctly across a full session restart
- Facts from different days combined correctly in a single response
- EMC encoding cycle draining cleanly (0 binding, 0 staged) at session end
- WMC dual-guard eviction firing correctly at capacity (global chunk limit
  and Miller's Law slot limit both exercised)
- ROS2 pipeline end-to-end stable (subscribe → CNC → MCC → WMC/EMC → LLM
  → publish) under sustained conversation load
- ~71 episodes accumulated and retrievable across sessions

Known limitation accepted at tag: LLM occasionally confabulates specific
values (exact codes, numbers) when the correct episode ranks below the
RECALL_SURFACE_LIMIT cutoff. Logged; addressed in M1.5 via user profile
store and anti-hallucination grounding instruction.

---

## 🧭 5. Design Principles

These shaped every decision made during M1. Violations were treated as bugs.

**Biological grounding over engineering convenience**
Every component maps to a named cognitive structure. WMC mirrors Baddeley's
phonological loop. EMC mirrors hippocampal episodic memory. MCC acts as the
prefrontal cortex. The dual-path RRF retrieval mirrors CA3/dentate gyrus
parallel paths converging at CA1. Naming followed lifecycle semantics —
Binding, Encoding, Consolidation, Recall, Reinstatement — not mechanical
descriptions.

**CNC never touches memory directly**
All memory operations are routed through MCC. CNC's only job is to receive
input, manage the async turn loop, and publish streamed output. This kept
the ROS2 node clean and made the memory pipeline independently testable.

**Active cognition is never starved**
The EMC encoding worker runs at os.nice(19) — lowest system priority. Theta
rhythm wakes on PMT arrival or a 2s fallback interval. The _binding_stream
is capped to prevent OOM. Recall has a 2.0s timeout with a skip fallback.
The system degrades gracefully before it blocks.

**Crash safety by default**
The two-layer episodic buffer (RAM _binding_stream + SQLite emc_staging)
ensures that no PMT is lost on process exit before encoding. Orphan recovery
runs on startup. SQLite operates in WAL mode throughout.

**Single-threaded WMC by design**
WMC is protected by CNC._busy — one turn at a time, no concurrent writes.
This made dual-guard eviction logic tractable without locks. Simplicity was
preferred over throughput at this stage.

**Forward-prep without forward-implementation**
Schema columns for M2a (importance scoring) and M2b (versioning, conflict)
were added during M1 schema design. They are never written or read in M1.
The pattern: lay the foundation, resist the temptation to build on it early.

---

## 🏗️ 6. Architecture Notes
 
**Memory pipeline flow (per turn)**
 
```
ROS2 /cns/neural_input
  └─ CNC._handle()
       ├─ MCC.register_memory("user")     ← turn registered before LLM call
       ├─ MCC.assemble_context()          ← EMC recall + WMC PMTs assembled
       │    ├─ EMC._recall() [dual-path RRF]
       │    └─ WMC._get_sustained_pmts()
       ├─ LLM stream (Ollama)
       └─ MCC.register_memory("assistant")
            └─ WMC._pair_pmt()
                 └─ [on eviction] → EMC._bind_pmt() → _binding_stream
```
 
**WMC dual-guard eviction**
 
Guard 1 fires when `sustained_chunks + incoming_chunks > GLOBAL_CHUNK_LIMIT`. 
Guard 2 fires when `pmt_slot_count >= PMT_SLOT_LIMIT + PMT_SLOT_BUFFER`. 
Evicted PMTs are returned to MCC and forwarded to EMC via thread pool — 
non-blocking from the perspective of the active turn.
 
**EMC dual-path retrieval (RRF)**
 
```
Query cue
  ├─ PATH 1 — sqlite-vec L2 KNN on unit-normalized 768d vectors
  │            (BAAI/bge-base-en-v1.5, sentence-transformers)
  └─ PATH 2 — FTS5 porter-stemmed lexical search on episode content
 
FUSION: score = 1/(60 + rank_semantic) + 1/(60 + rank_lexical)
        Normalized 0.0–1.0. RELEVANCE_THRESHOLD = 0.25.
        Top RECALL_SURFACE_LIMIT (5) episodes injected per turn.
```
 
**Context assembly order (MCC)**
 
```
[System prompt + personality]          ← COGNITIVE_RESERVE (512 chunks)
[EMC recalled episodes — system block] ← RECALL_RESERVE (1024 chunks)
[WMC sustained PMTs — chronological]   ← up to GLOBAL_CHUNK_LIMIT (6656)
[Current user message]
```
 
**MSB as shared substrate**
 
EngramComplex owns all SQL operations for a given cortex. EngramSchema / 
EngramTrace own blueprint-driven schema generation. EncodingEngine wraps 
sentence-transformers with an LRU prime cache. RRF fusion is implemented 
once in MSB and shared across EMC, and will be reused by SMC (M2) and PMC 
(M3). No cortex re-implements storage or encoding primitives.
 
**Architecture parameters**
 
| Parameter | Value | Meaning |
|---|---|---|
| CORTICAL_CAPACITY | 8192 chunks | Total cognitive core budget |
| COGNITIVE_RESERVE | 512 chunks | System prompt + personality |
| RECALL_RESERVE | 1024 chunks | EMC injection budget |
| GLOBAL_CHUNK_LIMIT | 6656 chunks | WMC active turns budget |
| PMT_SLOT_LIMIT | 7 | Miller's Law slot limit |
| PMT_SLOT_BUFFER | 2 | Miller's Law ±2 buffer |
| RECALL_SURFACE_LIMIT | 5 | Max episodes surfaced per turn |
| RECALL_DEPTH | 10 | Candidates per recall path |
| RELEVANCE_THRESHOLD | 0.25 | Minimum RRF score to surface |
| RECALL_TIMEOUT | 2.0s | Max wait before skipping recall |
| THETA_INTERVAL | 2.0s | Encoding fallback interval |
| THETA_BATCH_LIMIT | 32 | Max episodes per encoding cycle |

---

## 🛠️ 7. Implementation Journey

M1 was not built top-down. It grew organically — each layer revealed what
the next needed.

**Starting point: WMC**
WMC came first because it was the most concrete. A conversation window with
eviction was understandable before the rest of the system existed. The
dual-guard design (global chunks + Miller's Law slots) emerged from the
realization that token budget and cognitive slot count are independent failure
modes — one guard alone would leave the other unchecked.

**PMT as the unit of memory**
The decision to make the PMT (Phonological Memory Trace) the shared currency
between WMC and EMC was foundational. A PMT is a JSON pair `{user, assistant}`
— simple enough to evict, store, encode, and recall without structural
translation at each boundary. Everything downstream was built around this unit.

**MCC as the seam**
MCC started as a thin coordinator and stayed that way by design. Every time
there was temptation to let CNC reach into WMC or EMC directly, the rule held:
route through MCC. This made context assembly a single, auditable function
rather than logic scattered across the pipeline.

**EMC: staging before storage**
The two-layer buffer — RAM _binding_stream plus SQLite emc_staging — was
designed after thinking through crash scenarios. A PMT that arrives at EMC
but hasn't been encoded yet is vulnerable. emc_staging is the safety net:
write first, encode second, soft-delete on success. Orphan recovery on
startup handles the gap.

**RRF retrieval: semantic wasn't enough alone**
Early in design it was clear that pure vector search would miss exact-match
queries — names, codes, specific facts. FTS5 lexical search was added as the
second path. RRF fusion was chosen because it requires no score normalization
between paths and degrades gracefully when one path returns no results.

**MSB grew by extraction**
MSB was not designed upfront. EngramComplex, EncodingEngine, and RRF fusion
all started embedded in EMC. They were extracted into MSB when it became clear
that SMC and PMC would need the same primitives. The extraction happened
before M1 tag to avoid technical debt compounding into M2.

**The five bugs before tag**
All five fixes resolved issues found during acceptance testing — not in code
review. The amnesia bug (user turn registered after LLM stream instead of
before) was the most consequential: a stream failure would silently drop the
user's message from memory entirely. The double-induced-PMT guard and binding
stream cap were defensive; the schema versioning and trivial PMT filter were
forward-prep hygiene before tagging.

---

## 🐛 8. Bugs, Debugging & Lessons

**Bug 1 — Amnesia on LLM stream failure** (`cnc.py`)
*Symptom:* If the LLM stream errored mid-response, the user's turn was never
registered in WMC. From GRACE's perspective, the conversation never happened.
*Root cause:* `MCC.register_memory("user")` was called after the LLM stream
block, inside the same try/except. A stream exception exited before reaching it.
*Fix:* Moved `register_memory("user")` to the top of `_handle()`, before
the LLM call. The user turn is now guaranteed to land in WMC regardless of
what the LLM does.
*Lesson:* Memory registration must be treated as infrastructure, not output
post-processing. The user spoke; that fact is unconditional.

**Bug 2 — Double user message overwrites `_induced_pmt`** (`wmc.py`)
*Symptom:* Under certain timing conditions, a second call to the WMC user
registration path would overwrite the pending _induced_pmt before it was
paired with an AI response.
*Root cause:* No guard on the induced PMT write path — it assumed single-call
semantics that CNC's async loop didn't guarantee.
*Fix:* Added an explicit guard: if `_induced_pmt` is already populated,
skip the write.
*Lesson:* Stateful slots need defensive write guards even in nominally
single-threaded systems. Async makes "single call" a weak assumption.

**Bug 3 — No schema versioning** (`msb.py`)
*Symptom:* No way to detect or migrate schema changes across restarts without
dropping the database.
*Fix:* Added `schema_meta` table with a `schema_version` integer. Version is
checked on startup; mismatch raises an explicit error rather than silently
operating on a stale schema.
*Lesson:* Schema versioning is cheap to add before the first tag and
expensive to retrofit after real data exists.

**Bug 4 — Binding stream unbounded** (`emc.py`)
*Symptom:* Under sustained load, _binding_stream could grow without bound
between encoding cycles, risking OOM on Jetson.
*Fix:* Added `maxlen` cap to the deque via `BINDING_STREAM_MAX`.
*Lesson:* Any in-memory buffer that accumulates faster than it drains needs
an explicit ceiling. Theta rhythm is not a guarantee — it's a schedule.

**Bug 5 — Trivial PMTs stored in EMC** (`mcc.py`)
*Symptom:* Very short exchanges ("ok", "thanks", "got it") were being encoded
and stored, polluting the episodic store and consuming recall slots with
noise.
*Fix:* Added a content-length pre-filter at the WMC→EMC eviction boundary
in MCC. PMTs below a minimum character threshold are discarded before
forwarding to EMC.
*Lesson:* Not everything that fits in memory belongs in memory. Filtering at
the eviction boundary is the right place — MCC owns the forwarding decision.

---

## 🏁 9. Final Status

**Tag:** v0.1.0
**Test date:** April 29–30, 2026
**Model:** 12B quantized (Ollama, PC)
**Episodes accumulated:** ~71 across two sessions

M1 delivered a working chatbot with persistent episodic memory. GRACE can
carry facts, experiences, and context across full session restarts — recalling
them semantically and lexically, fusing results via RRF, and injecting the
most relevant episodes into context before each LLM inference call.

The architecture is stable and the seams are clean. MCC is the single
coordinator. MSB is the shared substrate. WMC and EMC are independently
testable. The schema is forward-prepped for M2a importance scoring and M2b
versioning without any M2 logic present.

M1.5 opens with known gaps logged and prioritized: semantic PMT filtering,
session-end flush, user profile store, anti-hallucination grounding, and the
agentic tool pipeline. The foundation is solid enough to build on.

GRACE was present for her own beginning.

---

## 📋10. Development Log

### 2026-03-16 — Foundation Day
*Repo birth, ROS2 workspaces, first cognitive skeleton*

**What landed**
- Initialized the AGi repository: license (AGPL), `.gitignore`,
  two ROS2 workspaces (AIVA and AuRoRA)
- Created the first four cognitive module files: `cnc.py`,
  `mcc.py`, `wmc.py`, `emc.py`
- Basic environment hygiene: `requirements.txt` and dependency
  cleanup in `package.xml`

**Challenges**
- Deciding how to split the system before a single line of real
  logic existed
- Two separate workspaces from day one meant distributed
  complexity before there was anything to distribute

**Reflection**
Grace didn't begin as a chatbot script. From the first coding
day she had a skeleton — four separate files for orchestration,
memory coordination, working memory, and episodic memory. I
didn't know yet what those files would become, but I knew they
shouldn't be one file.

---

### 2026-03-17 — P1M1 Ignition
*Grace gets a runtime, README, UI, and HRP direction*

**What landed**
- Declared M1: Chatbot + Working Memory + Episodic Memory
- First runtime path: web UI, Cosmos/vLLM launch script,
  Python dependencies, CNC registered as ROS2 entry point
- Wrote the first project README — purpose, hardware, stack,
  roadmap, memory architecture
- Introduced HRS package and `hrp.py` as the future home for
  cognitive parameters
- Added early `cli.py` for chatting with Grace via ROS2 topics

**Challenges**
- Accidentally created the GitHub issue template under a folder
  with a trailing space — caught and fixed same day
- WMC capacity constants already felt wrong as hardcoded values
  by end of day — the TODO commit was the acknowledgment

**Reflection**
March 17 was the day Grace became a project instead of just a
package. The README forced me to write down what AGi/AuRoRA
actually is — and writing it made the architecture more real.
The HRP direction at the end of the day was the first sign that
I was thinking about Grace's cognitive constants as something
that should be tunable, not buried.

---

### 2026-03-18 — Project Discipline
*PR templates, date-aware prompt, WMC chunk language*

**What landed**
- Added PR template and issue templates for bugs, enhancements,
  and documentation
- Added current-date injection into Grace's system prompt
- Hardened Cosmos launch script with shebang and strict Bash mode
- Continued refactoring WMC from token/turn language toward
  chunks, event segments, and capacity

**Challenges**
- WMC naming was still rough — a temporary HRS import fallback
  showed the architecture wasn't clean enough to depend on HRS
  properly yet
- Package versions across three packages were all wrong from the
  previous day's scaffolding

**Reflection**
March 18 was a discipline day — the kind that makes a project
sustainable. None of it felt exciting, but all of it mattered.
The date injection into the system prompt was small but
meaningful — before any BioLogic Clock existed, Grace at least
knew what day it was.

---

### 2026-03-19 — WMC Becomes a Real Memory System
*Chunks, Miller's Law, and the first overflow path*

**What landed**
- Refactored WMC into `WorkingMemoryCortex` with explicit
  hybrid capacity: chunk limit + Miller's Law event segment
  count (7±2)
- Rewrote memory registration — each segment carries role,
  content, timestamp, and estimated chunk size
- Evicted memory returned to MCC as `decaying_memory` — first
  version of the WMC → MCC → EMC overflow path
- Moved default WMC parameters into HRP: units per chunk,
  segment overhead, chunk capacity, segment capacity and buffer

**Challenges**
- Hybrid capacity (size + slot count) was the right model but
  required rethinking chunk estimation to account for role
  labels and formatting overhead, not just raw text length

**Reflection**
March 19 was the day WMC became bounded and connected. The most
important idea wasn't the capacity limits themselves — it was
that evicted memory wasn't deleted. It was returned. That single
design decision became the foundation of the whole WMC → EMC
pipeline.

---

### 2026-03-20 — PMT Is Born
*Working memory gets its language*

**What landed**
- Settled on PMT (phonological memory trace) as the core WMC
  unit — replacing "event segment" and "utterance"
- Aligned HRP parameter names with WMC: `PMT_OVERHEAD`,
  `PMT_SLOT_LIMIT`, `PMT_SLOT_BUFFER`, `GLOBAL_CHUNK_LIMIT`
- Renamed lifecycle stage from "decaying" to "receding" —
  more accurate: PMTs leave active attention, they don't degrade
- Expanded README roadmap into M2a/M2b/M2c with importance
  scoring, Ebbinghaus forgetting, novelty scoring, conflict
  columns, SMC distillation, and memory health reviews

**Challenges**
- Finding the right word took most of the day: event segment →
  utterance → PMT. Each rename forced a rethink of what the
  unit actually represented and what its lifecycle should be
  called

**Reflection**
March 20 was the day Grace's memory stopped being chat history.
The vocabulary clarified the architecture: PMTs are sustained,
they recede, they evict — they don't just scroll off a list.
Once the language was right, the design became easier to reason
about and easier to build on.

---

### 2026-03-21 — No Commit
*Pause between WMC language and WMC hardening*

---

### 2026-03-22 — WMC Capacity Hardening
*Accurate chunks, oversize guard, safer eviction*

**What landed**
- Fixed chunk estimation to use ceiling division — prevents WMC
  from undercounting PMT size and overfilling the context window
- Added oversize PMT guard — truncates content that would exceed
  the global chunk budget before it enters WMC
- Added floor to sustained chunk accounting so eviction can
  never drive the counter below zero
- Improved WMC assessment so the cortex can report its own
  occupancy and verify sustained chunk state

**Challenges**
- Floor division was silently undercounting — the kind of bug
  that only shows up under pressure when context windows start
  overflowing

**Reflection**
March 22 was a defensive day. WMC already had the right language
and the right model — today I made the math trustworthy. A
memory system is only useful if its capacity accounting can be
relied on.

---

### 2026-03-23 — HRP Becomes the Parameter Authority
*Separating configuration from coordination*

**What landed**
- Moved MCC constants into HRP: system prompt reserve, EMC
  context reserve, recall depth, similarity threshold, DB path
- Renamed MCC to `MemoryCoordinationCore` — coordinator, not
  configuration owner
- Defined HRP's three-tier parameter philosophy: static
  (architecture limits), intrinsic (self-tuning cognitive),
  extrinsic (per-user preferences)
- Removed "default" from WMC parameter names — these are the
  robot's active cognitive configuration, not fallbacks

**Challenges**
- Deciding what belongs in HRP vs what belongs in each cortex
  required thinking clearly about ownership: HRP owns parameters,
  MCC owns coordination, WMC owns active memory, EMC owns storage

**Reflection**
March 23 was a separation-of-concerns day. Once MCC stopped
owning its own configuration, its role became clearer: route
memory operations, assemble context, coordinate WMC and EMC.
The three-tier HRP philosophy also planted the seed for the
later config-refactor roadmap.

---

### 2026-03-24 — HRP Becomes a Namespace
*AGi.CNS.WMC, engram gateway, and forgetting*

**What landed**
- Converted HRP from flat constants into nested class namespaces:
  `AGi → CNS → EMC / WMC`
- Updated WMC and MCC to consume parameters through
  `AGi.CNS.WMC` — no more loose imports
- Removed WMC fallback defaults — HRP is now required, not
  optional; missing parameter system should fail clearly
- Moved gateway/path constants into HRP: `ENTITY_GATEWAY`,
  `NEURAL_GATEWAY`, `ENGRAM_COMPLEX`
- Renamed "discard PMT schema" to "forget PMT schema" — WMC
  is not deleting data, it is forgetting active traces
- Improved MCC startup logs to report WMC/EMC connectivity
  instead of raw budget numbers

**Challenges**
- Recognizing that `UNITS_PER_CHUNK` didn't belong inside WMC
  — chunk accounting is a CNS-wide concern, not WMC-only.
  Flagged it for promotion; it later moved to `AGi.CNS` level

**Reflection**
March 24 was the day HRP stopped being a settings file and
started being an architecture map. Once parameters lived under
`AGi.CNS.WMC`, reading the code told you where in the cognitive
system you were. That clarity compounded over the weeks ahead.

---

### 2026-03-25 — Memory Pipeline Gets Its Verbs
*Filling, binding, encoding, reinstatement*

**What landed**
- Renamed MCC's WMC call from `add_turn` to `fill_pmt` —
  WMC is no longer a chat-history list
- Renamed MCC operations: `add_turn → bind_pmt`,
  `_forward_to_emc_buffer → _bind_to_emc_buffer`
- Added EMC lifecycle: Binding → Encoding → Consolidation →
  Storing → Retrieval → Reinstatement
- Fixed MCC async from `get_event_loop()` to
  `get_running_loop()` — safer inside a running coroutine
- Cleaned MCC and WMC startup logs — boot as cognitive organs,
  not Python objects
- Added cognitive development roadmap to README: M1 = Phase 1
  "I can remember"

**Challenges**
- Nothing blocked, but the EMC lifecycle addition was the most
  conceptually important work of the day — it completed the
  vocabulary across both memory cortices

**Reflection**
March 25 was the day the pipeline got its verbs. WMC fills and
sustains. MCC binds. EMC encodes, consolidates, and reinstates.
Once each layer had lifecycle language, the whole WMC → MCC →
EMC flow could be reasoned about as a system, not just as code.

---

### 2026-03-26 — Memory API Learns to Describe Itself
*Binding, assessment, and introspection*

**What landed**
- Fixed evicted PMT variable naming inconsistencies across
  MCC and WMC after the previous day's refactor
- Passed PMT timestamps into `_bind_to_emc_buffer()` — EMC
  needs temporal data to store episodes properly
- Renamed stats methods: `get_stats → assess_memory_schema`,
  `log_stats → report_memory_stats`
- Renamed WMC chunk recomputation to `_introspect_chunk_load`
  — WMC inspects its own load, it doesn't just recalculate
- Added missing `-> None` return type annotation

**Challenges**
- The previous day's fast renaming left a real bug: evicted
  PMT variable was renamed but the `if evicted:` check still
  used the old name — silent failure until caught today

**Reflection**
March 26 was cleanup, but cleanup that mattered. Passing
timestamps into EMC was the most substantive fix — without
temporal data, episodic memory loses its context. The naming
work was quieter but the architecture was becoming
self-documenting: reading the method names told you what the
memory system was doing.

---

### 2026-03-27 — MCC Becomes the Integration Hub
*Binding evicted PMTs and assembling memory context*

**What landed**
- Refactored WMC → EMC handoff: MCC binds evicted PMTs via
  `run_in_executor` — episodic binding no longer blocks active
  cognition
- Built memory context assembly: sustained WMC PMTs + recalled
  EMC episodes → one unified context for the cognitive engine
- Cleaned MCC public interface into its final shape:
  `register_memory`, `_bind_to_episodic_buffer`,
  `assemble_memory_context`, `assess_memory_schema`,
  `report_memory_stats`, `forget_memory`, `close`

**Challenges**
- The cognitive engine should not care whether a memory came
  from working memory or episodic recall — making that
  abstraction clean required MCC to own the assembly fully,
  with no leakage into CNC

**Reflection**
March 27 was the day the memory pipeline became a system.
WMC and EMC had been evolving separately — today MCC became
the hub where they converge. The cognitive engine receives
one unified context. Where each memory came from is MCC's
problem, not the engine's.

---

### 2026-03-28 — EMC Joins the HRP Namespace
*Episodic memory parameters find their home*

**What landed**
- Moved EMC constants into `AGi.CNS.EMC`: encoding engine,
  vector dimension, engram chunk limit, recall reserve, recall
  depth, and relevance threshold
- EMC now consumes `AGi.CNS.EMC` — no more private constants
- Renamed MCC's memory gate from `RECALL_THRESHOLD` to
  `RELEVANCE_THRESHOLD` — recalled episodes enter context only
  if relevant enough
- Expanded EMC file header into a real cortex description:
  responsibilities, SQLite design, embedding, retrieval, and
  public methods
- Cleaned WMC wording from "active conversation context for
  GRACE" to "working memory layer of the CNS"

**Challenges**
- Nothing blocked today — this was coherence work, not new
  capability. The challenge was recognizing that hidden cortex
  constants fragment the architecture even when the code works

**Reflection**
March 28 was quiet but important. EMC had been working fine
with private constants — but fine isn't coherent. Once EMC
read its parameters from HRP like WMC did, the nervous system
felt complete as a governed architecture rather than a
collection of modules that happened to cooperate.

---

### 2026-03-29 — EMC Encoding Engine Activation
*From lazy embedder to eager encoding core*

**What landed**
- Renamed `_Embedder` to `_EncodingEngine` — cognitive system
  language over ML-library language
- Changed EMC from lazy-loading to eager-loading the encoding
  engine at initialization — semantic memory ready before first
  recall
- Moved encoding cache limit into HRP as `ENCODING_CACHE_LIMIT`
  — cache behavior now HRP-governed, not hardcoded
- Added type annotations through WMC's PMT/chunk accounting
  paths

**Challenges**
- First commit introduced a `_core` vs `_model` naming mismatch
  — the class was renamed but internal references weren't all
  updated, creating a real runtime bug. Fixed in the second
  commit same day. Good reminder: terminology refactors require
  updating every internal reference, not just the class name

**Reflection**
March 29 was about making EMC's semantic memory path visible
at startup. A lazy-loading encoder hides failures until the
worst moment — first recall. Eager-loading surfaces them
early. The `_core` mismatch was a small lesson in the cost of
fast renaming without a full search pass.

---

### 2026-03-30 — Encoding Engine Polish
*Activation, cache keys, and AGi identity*

**What landed**
- Moved encoding engine activation directly into `__init__` —
  no more separate `_load()` method
- Activation logs changed from raw model-loading language to
  subsystem language: "Activating Encoding Engine…" with
  explicit fallback to mental lexicon access
- Converted `is_available` from method to property
- Added `ENCODING_KEY_LIMIT` to HRP — cache-key slice length
  now HRP-governed, not hardcoded as `text[:300]`
- Fixed EMC stats to report `EMC.ENCODING_ENGINE` from HRP
- Removed accidental `joblib`/`sklearn` logger imports from WMC
- Updated README title from "Autonomous General Intelligence"
  to "Amazing Grace infrastructure"

**Challenges**
- Converting `is_available` to a property is a refactor trap:
  every call site must drop the parentheses or it silently
  returns the property object instead of the bool

**Reflection**
March 30 was polish, but the README title change was more than
that. "Autonomous General Intelligence" was a claim. "Amazing
Grace infrastructure" is the truth. Getting the identity right
matters — it changes how every future reader understands what
the project is.

---

### 2026-03-31 — Repo Hygiene
*Ignore rules and dev branch sync*

**What landed**
- Added `log/`, `install/`, and `.vscode/` to AIVA's
  `.gitignore`
- Merged March 29–30 dev work into active branch

**Reflection**
Not every day is a feature day. March 31 was a maintenance
checkpoint — cleaning the workspace and synchronizing the
branch before the next round of memory-system development.

---

### 2026-04-01 — Encoding Cache Becomes Imprints
*Cue, trace, lexical recall, and neural threads*

**What landed**
- Renamed encoding cache keys to imprints: `ENCODING_KEY_LIMIT`
  → `ENCODING_IMPRINT_LIMIT`; cache entries prefixed `cue/trace`
  with hashed imprint instead of raw text slice
- Changed `encode(text, is_query)` to `encode(trace, is_cue)`
  — cue triggers recall, trace is the memory content
- Fallback language settled on "lexical recall" — EMC recalls
  memories, it doesn't retrieve database rows
- Encoded vector output renamed to `encoded_trace`
- EMC async worker became "consolidation worker" on a separate
  neural thread; WMC docs updated to "main neural thread"
- Added FAISS migration TODO in cosine helper for when recall
  set exceeds ~10k vectors

**Challenges**
- Fallback terminology went through three names in two days:
  mental lexicon access → lexical retrieval → lexical recall.
  Getting the right word required sitting with the memory
  lifecycle until the fit was obvious

**Reflection**
April 1 was the day EMC's encoder started speaking memory
language instead of ML-library language. Cue and trace are not
just prettier names — they carry meaning. A cue is what
triggers recall. A trace is what gets stored. Once the names
were right, the encoding interface explained itself.

---

