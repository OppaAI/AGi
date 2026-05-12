"""
NIU — Neural Input Unit
========================
AuRoRA · Semantic Cognitive System (SCS)

Typed input contract layer for all incoming stimuli at the CNC boundary.
All input sources (CLI, web, voice, messaging bridges) conform to these contracts.
CNC parses against these types — no ad-hoc dict access at the boundary.

Responsibilities:
    - Define typed input schema for all incoming text stimuli
    - Enumerate all valid input modalities
    - Enforce contract at parse time — malformed payloads rejected before reaching CNC

Architecture:
    Stateless type definitions — no ROS2 node, no persistent state.
    Imported by CNC at the input boundary.
    New input modalities declared here — routing logic keys off InputChannel enum.

Terminology:
    NeuralTextInput — typed contract for all incoming text stimuli
    InputChannel    — enum of all valid input modalities

Public interface:
    InputChannel    — CLI | WEB | VOICE | TELEGRAM
    NeuralTextInput — text, user_id, source
"""

# System libraries
from dataclasses import dataclass, field    # for typed input schema
from enum import Enum                       # for input channel enumeration

class InputChannel(Enum):
    """
    Valid input modalities for incoming text stimuli.
    New modalities declared here — CNC routing logic keys off this enum.
    """
    CLI      = "cli"        # keyboard-driven terminal input
    WEB      = "web"        # web interface input
    VOICE    = "voice"      # voice pipeline input — M1.X-b
    TELEGRAM = "telegram"   # Telegram messaging bridge — M1.X-c

@dataclass
class NeuralTextInput:
    """
    Typed contract for all incoming text stimuli at the CNC boundary.
    All input sources must conform — malformed payloads rejected at parse time.

    Fields:
        text     (str)          : User message content
        user_id  (str)          : Speaker identity — defaults to 'demo' if omitted
        source   (InputChannel) : Input modality — routes correctly through voice and messaging bridges
    """
    text    : str
    user_id : str          = field(default="demo")              # speaker identity — defaults to demo if omitted
    source  : InputChannel = field(default=InputChannel.CLI)    # input modality — defaults to CLI if omitted
