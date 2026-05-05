^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for AuRoRA · Semantic Cognitive System (SCS)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

* feat(M1): introduce ``cnc.py`` — ROS2 Central Neural Core; full perception-cognition-response pipeline with dedicated asyncio event loop on background thread; publishes streamed JSON chunks to ``/gce/response``
* feat(M1): introduce ``mcc.py`` — Memory Coordination Core; single memory interface for CNC coordinating WMC and EMC; assembles full memory context each turn
* feat(M1): introduce ``wmc.py`` — Working Memory Cortex; deque-based active conversation window with dual-guard eviction (chunk limit + Miller's Law 7±2 slot limit)
* feat(M1): introduce ``emc.py`` — Episodic Memory Cortex; hippocampal long-term episodic store with theta-rhythm driven background encoding cycle and dual-path RRF retrieval
* feat(M1): introduce ``msb.py`` — Memory Storage Bank; shared substrate for all memory cortices; owns encoding engine, vector math, SQLite schema generation, and RRF fusion
* feat(M1): implement dual-path episodic retrieval — semantic (sqlite-vec L2 KNN on unit-normalized vectors) + lexical (FTS5 porter-stemmed) fused via Reciprocal Rank Fusion (RRF)
* feat(M1): implement crash-safe episodic staging — unencoded PMTs survive restart via ``emc_staging`` table; recovered into binding stream on boot
* feat(M1): implement schema versioning in MSB — ``schema_meta`` table raises ``RuntimeError`` on version mismatch before touching schema
* feat(M1): implement PMT pairing — user prompt and AI response paired into one complete interaction before WMC fill and EMC encoding
* feat(M1): implement GCE VRAM keep-alive — Ollama model pinned in VRAM; guarded by ``None`` check for vLLM compatibility
* feat(M1): implement GCE priming at boot — fire-and-forget model preload via ``_prime_gce()``
* feat(M1): add ``num_ctx`` to GCE inference packet — overrides Ollama default 2048 context window
* feat(M1): add binding stream ``maxlen`` cap — OOM guard; logs warning at capacity
* feat(M1): add trivial PMT filter — discards turns under 20 chars at WMC→EMC eviction boundary
* feat(M1): add consecutive user message guard in WMC — appends to ``_induced_pmt`` rather than overwriting
* feat(M1): upgrade embedding model to ``BAAI/bge-base-en-v1.5`` with asymmetric cue prefixing for improved retrieval accuracy
* feat(M1): introduce web GUI (``AGi.html``) and CLI (``grace_cli.py``) — WebSocket bridge via rosbridge
* fix(M1): amnesia bug — ``register_memory("user")`` now called before ``_stream_gce()``
* fix(M1): attention gate TOCTOU — gate closed before coroutine scheduled
* fix(M1): ``stored_version`` type annotation corrected from ``str`` to ``int`` in MSB schema versioning
* fix(M1): redundant future cancellation removed from MCC
* fix(M1): user prompt correctly included in WMC content
* fix(M1): `#1 <https://github.com/OppaAI/AGi/issues/1>`_, `#3 <https://github.com/OppaAI/AGi/issues/3>`_, `#4 <https://github.com/OppaAI/AGi/issues/4>`_, `#5 <https://github.com/OppaAI/AGi/issues/5>`_, `#7 <https://github.com/OppaAI/AGi/issues/7>`_, `#8 <https://github.com/OppaAI/AGi/issues/8>`_, `#11 <https://github.com/OppaAI/AGi/issues/11>`_, `#15 <https://github.com/OppaAI/AGi/issues/15>`_ — initial SCS bug fixes resolved at M1 launch
* fix(M1): `#29 <https://github.com/OppaAI/AGi/issues/29>`_ — CNC command execution logic and error handling
* fix(M1): `#30 <https://github.com/OppaAI/AGi/issues/30>`_, `#31 <https://github.com/OppaAI/AGi/issues/31>`_, `#32 <https://github.com/OppaAI/AGi/issues/32>`_, `#33 <https://github.com/OppaAI/AGi/issues/33>`_ — additional bug fixes
* refactor(M1): all naming throughout CNC, MCC, WMC, EMC, MSB refactored to humanistic neuroscience terminology
* refactor(M1): temporal trace mapping in MSB — replaces hardcoded date columns with configurable schema field
* refactor(M1): ``EngramComplex`` replaces ``MemoryBank`` / ``EngramStorageBank`` — streamlined internal API
* refactor(M1): encoding cycle extracted into dedicated ``EncodingCycle`` class
* refactor(M1): episodic buffer redesigned as two-stream (binding + recall) with full thread-safety
* docs(M1): README with CNS module reference, full Phase 1–6 roadmap, architecture flowchart, conversation sequence diagram, and quick start
* docs(M1): M1.5 spec — memory bridges, agentic tools, integration test gate (11 criteria)
* docs(M1): roadmap updated with M1.5–M3 TODOs surfaced from code review; M1.X side quests milestone added
* Contributors: OppaAI