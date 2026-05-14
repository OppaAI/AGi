"""
NIU — Neural Input Unit
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

# System components
from dataclasses import dataclass, field    # for NeuralStimulus typed schema
from enum import Enum                       # for NeuralInputChannel modality enumeration

class NeuralInputChannel(Enum):
    """
    Input modality enumeration for all incoming neural stimuli.
    New modalities declared here — CNC routing logic keys off this enum.
    All input adapters must supply one of these values — no plain strings at the boundary.
    """
    CLI      = "cli"                        # keyboard-driven terminal input
    WEBUI    = "webui"                      # web interface input
    VOICE    = "voice"                      # voice pipeline — (TODO: M1.X-b)
    TELEGRAM = "telegram"                   # Telegram messaging bridge — (TODO: M1.X-c)
    DISCORD  = "discord"                    # Discord messaging bridge — (TODO: M1.X-c)
    GMAIL    = "gmail"                      # Gmail email bridge — (TODO: M1.X-c)

@dataclass
class NeuralStimulus:
    """
    Typed input schema for all incoming neural stimuli at the CNC boundary.
    All input sources must conform — malformed stimuli rejected at the CNC boundary.

    Fields:
        source   (NeuralInputChannel) : Input modality — required, every adapter must supply
        user_id  (str)                : Speaker identity — defaults to 'demo' if omitted
        text     (str)                : Text content — CLI, webUI, messaging bridges
        audio    (bytes)              : Raw audio payload — voice pipeline (TODO: M1.X-b)
        image    (bytes)              : Raw image payload — vision pipeline (TODO: M2+)
    """
    source  : NeuralInputChannel                        # input modality — required, every adapter must supply
    user_id : str   = field(default="demo")             # speaker identity — defaults to demo for minimum functionality
    text    : str   = field(default="")                 # text content — empty if modality is not text
    audio   : bytes = field(default=b"")                # audio payload — empty until voice pipeline lands
    image   : bytes = field(default=b"")                # image payload — empty until vision pipeline lands
