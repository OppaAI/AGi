"""
SIU — Sensory Input Unit
========================
AuRoRA · Semantic Cognitive System (SCS)

Typed input schema layer for all incoming neural stimuli at the CNC boundary.
All input sources (CLI, webUI, voice, messaging bridges, etc) conform to these schemas.
CNC parses against these types — no ad-hoc dict access at the boundary.

Responsibilities:
    - Define typed input schema for all incoming neural stimuli
    - Enumerate all valid input modalities
    - Enforce schema at the CNC boundary — malformed stimuli rejected before reaching the cognitive cycle

Architecture:
    Stateless type definitions — no ROS2 node, no persistent state.
    Imported by CNC at the input boundary.
    New input modalities declared here — routing logic keys off InputChannel enum.

Terminology:
    Neural Input Channel  — the modality through which a stimulus enters the cognitive boundary
    Neural Stimulus       — a single discrete input event carrying identity, content, and source

Public interface:
    NeuralInputChannel  — CLI | WEBUI | VOICE | TELEGRAM | DISCORD | GMAIL
    NeuralStimulus      — source, user_id, text, audio, image
"""

