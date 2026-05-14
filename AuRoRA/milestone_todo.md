## TODO — AuRoRA Cognitive Architecture

### SCS
- [ ] **M1.x** `cnc.py` — WebUI operator dashboard: rosbridge_server WebSocket bridge for browser-based debug stream, memory stats, and teleop interface
- [ ] **M1.x** `cnc.py` — MEMORY_CONTEXT_GATEWAY: WebUI memory context debug stream
- [ ] **M1.x** `scs/types.py` — define and lock NeuralTextInput schema; standardize JSON contract across all input sources (CLI, web, voice); evaluate custom ROS2 msg type vs JSON bridge for non-ROS interfaces
- [ ] **M1.x** `cnc.py` `mcc.py` — multi-user identity: replace hardcoded "user" speaker literal with user_id from NeuralTextInput schema — required for ASR multi-speaker routing
- [ ] **M1.6** `cnc.py` — Add login/logout Service (UI/UX, Voice, both?) (see below)
- [ ] **M2?** `cnc.py` — remove M1 stub — replace with login sequence
- [ ] **M2** `emc.py` — standardize logger format across all EMC classes (EpisodicMemoryCortex, EncodingCycle, EpisodicBuffer); establish consistent prefix convention — e.g. [EMC], [EncodingCycle], [EpisodicBuffer]
- [ ] **M2** `cnc.py` — `_strip_model_artifacts()`: strip think blocks and roleplay artifacts from assistant response before handing to MCC for memory storage
- [ ] **M2** `cnc.py` — salience gate: score assistant response before registering in MCC — discard low-salience turns at the CNC boundary
- [ ] **M2** `cnc.py` — busy queue: buffer incoming inputs during processing rather than dropping

### MCC
- [ ] **M2** `mcc.py` — implement session-end consolidation: flush WMC PMTs to EMC on shutdown, gated on novelty/importance scoring — low-salience turns should be truly forgotten, not blindly bound
- [ ] **M2** `mcc.py` — salience gate at eviction boundary in `_bind_to_episodic_buffer()`: score evicted PMTs for novelty and importance before binding; discard low-salience turns, bind high-salience turns to EMC — WMC and EMC remain salience-agnostic, MCC owns this gate
- [ ] **M2** `mcc.py` — cleanup: move EncodingEngine to scs/eee.py — dedicated encoding layer between MCC and MSB.
- [ ] **M2** `mcc.py` — dynamic EMC capacity adjustment: if recalled engrams exceed EMC_RECALL_RESERVE, trim to fit rather than silently overrunning WMC's chunk limit
- [ ] **M2** `mcc.py` — WMC/EMC health check with capacity breach warnings
- [ ] **M2** `mcc.py` — add SMC, 11pm reflection trigger
- [ ] **M3** `mcc.py` — add PMC, procedural skill retrieval
- [ ] **M3** `mcc.py` — expand into detailed health check with warnings on capacity breaches, anomalous eviction rates, etc.
- [ ] **M3** `mcc.py` — GUI: expose via ROS2 topic for real-time memory visualisation

### WMC
- [ ] **M1.6** `wmc.py` — integrate HRS.BLC for biological clock timestamps
- [ ] **M3** `wmc.py` — add salience weighting to eviction policy

### EMC
- [ ] **M1.5** `emc.py` — EpisodicScaffold: implement scaffold as an explicit object in EMC owning engram anchoring at bind time, RECALL_RESERVE trimming, and chronological sequencing before reinstatement; extend EMC_SCHEMA with scaffold metadata fields: sequence_index (INTEGER), session_id (TEXT), salience (REAL), consolidation_state (TEXT DEFAULT 'raw') — required for Dream Cycle distillation tracking and scaffold-aware recall ordering
- [ ] **M2** `emc.py` — date-range filtering on recall interface and buffer entries
- [ ] **M2** `emc.py` — DiskANN ANN index when episodes exceed ~50k (currently exact KNN)
- [ ] **M2** `emc.py` — SMC distillation trigger at 11pm reflection (Dream Cycle)
- [ ] **M2** `emc.py` — add watchdog for theta rhythm (interval of one encoding cycle) during dreaming cycle
- [ ] **M2** `emc.py` — heartbeat logging during long idle periods
- [ ] **M2** `emc.py` — staging_id integrity check after Dream Cycle consolidation
- [ ] **M2** `emc.py` — graceful drain + optional timeout fallback for SWR trigger during close

### HRS / HRU / HRC
- [ ] **M2** `hrs.py` — build hrs.py to allow GRACE to update [INTRINSIC] constants at runtime and persist changes back to aurora.yaml
- [ ] **M2** `hrs.py` — add HRS startup/shutdown lifecycle management
- [ ] **M2** `hrs.py` — add recency parameter for identification of the most recent event segments in EMC
- [ ] **M2?** `hru.py` — decide whether to use database to store user profile instead of yaml
- [ ] **M2** `hrc.py` — HRC takes ownership of manifest hydration lifecycle
- [ ] **M2** `hrc.py` — add manifest diff and rollback for safe runtime parameter updates
- [ ] **M3** `hrc.py` — emotional state initialization
- [ ] **M3** `cnc.py` — multi-modal input: image and audio signal routing through CNC
- [ ] **M?** `cnc.py` — perception subsystems (visual, auditory)
- [ ] **M?** `cnc.py` — motor control interface

### MSB
- [ ] **M1.5** `msb.py` `hrs/hrp.py` — migrate pack_vector, normalize_vector to hrs.py if vector math is needed outside memory cortices
- [ ] **M2** `msb.py` — extend engram complex with SMC and PMC tables
- [ ] **M3** `msb.py` — backend swap: Qdrant, pgvector, or other vector DB — swap here only, cortex cognitive logic untouched
- [ ] **M2/3** `msb.py` — evaluate Qdrant migration (SMC/PMC scale) vs SQLite-vec; expand schema_meta (encoding_engine, vector_dim, robot_id, created_at) once storage backend is finalized

### Login
# TODO M1.6 — Login/Logout Service (after TSR/ASR UX decisions)
Current state: Single active user per session, set at startup via AGi.ACTIVE_USER
Architecture ready: user_id threading complete (CNC → MCC → WMC → EMC → MSB)
 
Deferred decision: How should users authenticate?
   - Voice command? ("Grace, login as Alice")
   - GUI button/form?
   - ROS2 service call from external UI?
   - Face recognition integration?

Implementation plan (once UX decided):
   1. Add ROS2 service: /scs/session/login (user_id) → (success, message)
   2. Add ROS2 service: /scs/session/logout () → (success, message)
   3. Service handler reloads PPU with new user_id:
      ```
      def _handle_login(self, request, response):
          user_id = request.user_id
          try:
              self._ppu.provision(user_id=user_id)
              self._active_user = user_id
              self._user_type = self._ppu.user_type
              response.success = True
              response.message = f"Logged in as {user_id}"
          except Exception as e:
              response.success = False
              response.message = f"Login failed: {e}"
          return response
      ```
   4. Optional: Add session timeout/auto-logout
   5. Optional: Multi-factor auth for admin users

 Dependencies:
   - M1.X TSR/ASR voice pipeline (voice command login)
   - UI/UX design decisions (GUI vs voice vs both)
   - Security policy (face recognition? PIN? passphrase?)

Current workaround: Edit AGi.ACTIVE_USER in HRM or restart with different user
