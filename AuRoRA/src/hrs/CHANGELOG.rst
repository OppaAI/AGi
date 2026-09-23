^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for AuRoRA · Homeostatic Regulation System (HRS)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2026-05-04)
------------------
 
* feat(M1): introduce ``hrp.py`` — three-tier constant hierarchy (STATIC / INTRINSIC / EXTRINSIC) as system-wide parameter registry under ``AGi`` namespace
* feat(M1): add ``AGi.CNS`` namespace — cortical capacity, cognitive reserve, neural gateway, engram complex path, units-per-chunk, and text input gateway
* feat(M1): add ``AGi.CNS.GCE`` namespace — full GCE inference parameter set including model, endpoint, sampling parameters, keep-alive, and streaming state constants
* feat(M1): add ``AGi.CNS.GenericGrace`` namespace — factory-default GCE parameters separate from personalised instance
* feat(M1): add ``AGi.CNS.EMC`` namespace — encoding engine, binding stream limit, theta rhythm, recall pool (increased from 2 to 10), recall depth, recall timeout, relevance threshold, and episode content limit
* feat(M1): add ``AGi.CNS.WMC`` namespace — PMT overhead, slot limit, slot buffer, and derived global chunk limit
* feat(M1): add ``AGi.CNS.SMC`` namespace — encoding engine and encoding dimension
* feat(M1): move GCE model config, LLM parameters, and GRACE system prompt from ``cnc.py`` into HRP
* feat(M1): move EMC, MCC, and WMC parameters from source files into HRP
* feat(M1): add streaming state constants — ``STREAM_LEADING``, ``STREAM_PROPAGATING``, ``STREAM_TRAILING``, ``STREAM_ANOMALY`` — marked static
* refactor(M1): rename all constants to humanistic neuroscience terminology throughout
* refactor(M1): convert all constants to nested class-type hierarchy
* Contributors: OppaAI
