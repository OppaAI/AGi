"""
HRU — Homeostatic Regulation Unit
===================================
AuRoRA · Homeostatic Regulation System (HRS)

Runtime utility layer of the HRS — propagates and manages the homeostatic 
regulation manifest within and across AuRoRA systems.

Responsibilities:
    - Hydrate AGi manifest constants from AuRoRA parameter server at node init
    - Estimate and truncate cognitive context chunks for budget management
    - Normalize and pack encoding vectors for semantic memory storage

Architecture:
    Stateless utilities — no ROS2 node, no persistent state.
    Imported by any AuRoRA node that needs manifest hydration or chunk sampling.
    HRM supplies the constants — HRU supplies the runtime tools that operate on them.
    HRC (TODO: M2) will orchestrate HRU at the system level for adaptive self-regulation.

Terminology:
    Manifest    — the AGi class tree of cognitive architecture constants
    Hydration   — binding ROS2 parameter server values into AGi manifest constants
    Chunk       — model-agnostic unit of context length; maps to a token when
                  the router's /tokenize endpoint is reachable, or a
                  character-division approximation otherwise

Public interface:
    hydrate_manifest(core, system) → None
    
    normalize_vector(vector) → list[float]
    pack_vector(vector) → bytes
    
    ChunkSampler:
        probe(content, overhead?) → int
        truncate(content, limit)  → str
"""
# System components
import json                                 # JSON encoding for /tokenize request bodies
import urllib.request                       # sync HTTP POST — chunk counting against the llama.cpp router
import urllib.error                         # HTTP error handling for tokenize requests
from dataclasses import dataclass, field    # for GatewayMap frozen dataclass
from enum import Enum                       # enum base for UserType identity classification
from pathlib import Path                    # filesystem path abstraction — GatewayMap builds all AuRoRA paths from this

# ROS2 components
from rclpy.node import Node                 # for node type hinting (Node) and core node

# AGi components
from hrs.hrm import AGi, RRR                # manifest constants; RRR — filesystem path segments

class UserAccessLevel(Enum):
    """
    Classify the active user's access scope for recall and permission gating.
    """
    ADMIN = "admin"                         # full system access — unrestricted recall, admin actions
    GUEST = "guest"                         # restricted system access — limited recall, user-scoped actions only
    
@dataclass
class UserProfile:
    user_id:              str                # unique identifier — matches the key in users.yaml
    name:                 str                # full legal name — used in formal contexts
    known_as:             str                # preferred address — how AIVA speaks to the user
    location:             str                # physical location — timezone and locale context
    access_level:         UserAccessLevel    # permission scope — admin unrestricted, guest user-scoped only
    seed:                 str   = ""         # relational seed — injected into system prompt for context
    memory_salience_bias: float = 0.0        # recall sensitivity — lowers threshold to surface more memories

    def __post_init__(self):
        if isinstance(self.access_level, str):                      # YAML delivers raw strings — coerce to enum if needed
            self.access_level = UserAccessLevel(self.access_level)  # coerce string → enum — stored profile always delivers raw strings

DEMO_USER = UserProfile(                                            # user profile missing — last resort fallback
    user_id              = "demo",
    name                 = "Demo User",
    known_as             = "user",
    location             = "Unknown",
    access_level         = UserAccessLevel.GUEST,
    seed                 = "This is a demo session. No user has been identified.",
    memory_salience_bias = 0.0
)

def hydrate_manifest(core: Node, system: str) -> None:
    """
    Hydrate the system manifest with parameters declared by AuRoRA or admin under the given system.

    Args:
        core        : Core node instance receiving the hydrated parameters
        system (str): System name identifying the manifest to hydrate (e.g. "scs")
    """
    manifest: type | None = _find_manifest(AGi, system)                        # recurse class tree — match nested subclass by system name
    if manifest is None:                                                       # if no match subclass is found,
        raise RuntimeError(f"❌ No manifest matching system name '{system}'")  # abort — unrecognized system name cannot be safely hydrated
    _hydrate_system(core, manifest, system)                                    # walk manifest tree — declare and bind all parameters to core

def _find_manifest(tree: type, system: str) -> type | None:
    """
    Recursively locate the manifest subclass matching the given system name in the given class tree.

    Args:
        tree (type) : AuRoRA class tree to search through
        system (str): System name to match against nested subsystem names

    Returns:
        type | None : matched manifest subclass, or None if not found
    """
    for system_name in vars(tree):                                                # iterate attribute names of the current class level
        subsystem: object = getattr(tree, system_name)                            # retrieve the attribute value for inspection
        if not isinstance(subsystem, type) or system_name.startswith("_"):        # if non-class attributes and private members,
            continue                                                              # skip — only walk public nested classes
        if system_name.lower() == system:                                         # case-insensitive match,
            return subsystem                                                      # return manifest subclass if system name matches
        matched_manifest: type | None = _find_manifest(subsystem, system)         # recurse into nested subclass — walk deeper into the system tree
        if matched_manifest:                                                      # if manifest match the subclass
            return matched_manifest                                               # propagate match up the call stack
    return None
    
def _hydrate_system(core: Node, manifest: type, system: str) -> None:
    """
    Recursively walk the manifest subclass and bind each parameter to the AuRoRA parameter server.

    Args:
        core (Node)         : Core node instance receiving the hydrated parameters
        manifest (type)     : Manifest subclass to walk through and hydrate
        system (str)        : System name identifying the manifest to hydrate
    """
    for param_name in vars(manifest):                                                # iterate attribute names of the manifest subclass
        if param_name.startswith('_'):                                               # if private members,
            continue                                                                 # skip — internal constants not exposed to parameter server
        param_value: type | int | float | bool | str = getattr(manifest, param_name) # retrieve the attribute value for inspection
        param_key: str = f"{system}.{param_name.lower()}"                            # build fully qualified parameter key — dot-separated namespace path

        if isinstance(param_value, type):                                            # nested class — recurse deeper into the manifest tree
            _hydrate_system(core, param_value, param_key)                            # recurse into nested subclass — depth-first walk
        elif isinstance(param_value, (int, float, bool, str)):                       # if parameter value is integer, float, boolean, string,
            core.declare_parameter(param_key, param_value)                           # declare parameter with default value on the core node
            setattr(manifest, param_name, core.get_parameter(param_key).value)       # write server value back into manifest — AuRoRA overrides the default

@dataclass(frozen=True)
class GatewayMap:
    """
    Declarative registry of all AuRoRA filesystem gateway paths.

    Usage:
        gm = GatewayMap()                        # initialize gateway map
        db = gm.engram_complex                   # gateway to engram complex
        gm.connect_gateway(gm.engram_complex)    # ensure gateway properly connected
    """
    home: Path = field(default_factory=Path.home)                                # OS home directory — default injectable for testing

    @property
    def entity_root(self) -> Path:
        """Root for all AGi core system state."""
        return self.home / AGi.ENTITY_GATEWAY                                    # root directory of the filesystem (ie. ~/agi)

    # RAS Gateway
    @property
    def ras_gateway(self) -> Path:
        """Open the gateway to the components of Reticular Activating System (RAS)."""
        return self.entity_root / RRR.RETICULAR_ACTIVATING_SYSTEM           # path to access files related to RAS (ie. ~/.agi/ras)

    @property
    def aurora_setpoints(self) -> Path:
        """Locate the AuRoRA setpoints — intrinsic parameters self-regulated by RAS at ignition."""
        return self.ras_gateway / AGi.SCS.AURORA_SETPOINTS                       # path to access the self-adjusted parameters (ie. ~/.agi/ras/aurora.yaml)

    # SCS Gateway
    @property
    def scs_gateway(self) -> Path:
        """Open the gateway to the components of Semantic Cognitive System (SCS)."""
        return self.entity_root / RRR.SEMANTIC_COGNITIVE_SYSTEM                  # path to access files related to SCS (ie. ~/.agi/scs)

    @property
    def user_profiles(self) -> Path:
        """Locate the user profiles — per-user extrinsic settings retrieved by CNC after login. (TODO: may change to database instead of yaml)"""
        return self.scs_gateway / AGi.SCS.USER_PROFILES                          # path to access user profile (ie. ~/.agi/scs/users.yaml)

    # GCE Gateway
    @property
    def gce_gateway(self) -> Path:
        """Open the gateway to the components of Generative Cognitive Engine (GCE)."""
        return self.scs_gateway / RRR.GENERATIVE_COGNITIVE_ENGINE                # path to access files related to GCE (ie. ~/.agi/scs/gce)

    @property
    def active_persona(self) -> Path:
        """Locate the active persona — retrieved by CNC at initialization."""
        return self.gce_gateway / AGi.SCS.PERSONA_PROFILES                       # path to access LLM persona (ie. ~/.agi/scs/gce/persona.yaml)

    # MCC Gateway
    @property
    def mcc_gateway(self) -> Path:
        """Open the gateway to the components of Memory Coordination Cortex (MCC)."""
        return self.scs_gateway / AGi.SCS.MEMORY_GATEWAY                         # path to access files related to MCC (ie. ~/.agi/scs/mcc)

    @property
    def engram_complex(self) -> Path:
        """Locate the engram complex — Storage of episodic memory, semantic memory and procedural memory across users. (TODO: may upgrade to Qdrant later on)"""
        return self.mcc_gateway / AGi.SCS.ENGRAM_COMPLEX                         # path to access the memory database (ie. ~/.agi/scs/mcc/engram_complex.db)

    # Helpers
    def connect_gateway(self, gateway: Path) -> Path:
        """
        Ensure the gateway's parent directory chain exists, creating it if absent.
        
        Args:
            gateway (Path): target gateway to connect to

        Returns:
            Path: the gateway itself — for inline chaining
        """
        gateway.parent.mkdir(parents=True, exist_ok=True)                         # create parent dirs — idempotent, safe to call repeatedly
        return gateway                                                            # return gateway for inline chaining: f.open(gm.connect_gateway(gm.engram_complex))

class ChunkSampler:
    """
    Estimate and truncate cognitive context chunks for cognitive context management.

    Counts tokens through the llama.cpp router's /tokenize endpoint — the exact
    tokenizer of the serving model — with a char-division fallback when the
    endpoint is unreachable.
    """
    def __init__(self, logger) -> None:
        """
        Initialize the chunk sampler — no model load; the router serves tokenization.

        Args:
            logger : logger instance for runtime diagnostics passed from the caller
        """        
        self.logger           = logger                                    # runtime diagnostics interface
        self._tokenize_url    = f"{AGi.SCS.GCE.NEURAL_ENDPOINT}/tokenize"     # llama.cpp router tokenize endpoint — token-accurate counting
        self._detokenize_url  = f"{AGi.SCS.GCE.NEURAL_ENDPOINT}/detokenize"   # llama.cpp router detokenize endpoint — used by truncate to rebuild text
        self._endpoint_ok     = None                                          # tri-state — None until the first endpoint probe, then sticky True/False
        self._units_per_chunk = AGi.SCS.UNITS_PER_CHUNK                       # chars-per-token constant — used in fallback approximation

    def _tokenize(self, content: str) -> list[int] | None:
        """
        Tokenize content through the llama.cpp router's /tokenize endpoint.

        Args:
            content (str): text to tokenize

        Returns:
            list[int] | None: token IDs, or None when the endpoint is unavailable — caller falls back
        """
        if self._endpoint_ok is False:                                                 # sticky failure — skip the request entirely
            return None                                                                # signal fallback to char-division

        try:                                                                           # attempt to reach the router
            request  = urllib.request.Request(                                         # build the tokenize request
                self._tokenize_url,
                data    = json.dumps({"content": content, "add_special": False}).encode(),  # exclude structural special tokens — matches prior tokenizer behaviour
                headers = {"Content-Type": "application/json"},
                method  = "POST",
            )
            with urllib.request.urlopen(request, timeout=10.0) as response:            # generous timeout — long contexts tokenize slowly on CPU
                tokens: list[int] = json.loads(response.read()).get("tokens", [])      # parse the token ID list from the reply
            self._endpoint_ok = True                                                   # endpoint confirmed — sticky success, skip probing on later calls
            return tokens                                                              # return the token IDs for counting

        except (urllib.error.URLError, TimeoutError, OSError, json.JSONDecodeError) as e:  # broad catch — any failure is non-fatal
            if self._endpoint_ok is not True:                                          # log only on first failure — avoid spamming every probe
                self.logger.debug(f"⚠️ Chunk sampler endpoint unavailable, falling back to chunk-division: {e}")
            self._endpoint_ok = False                                                  # sticky failure — later probes skip the request
            return None                                                                # signal fallback to char-division

    def _detokenize(self, tokens: list[int]) -> str | None:
        """
        Detokenize token IDs back into text through the router's /detokenize endpoint.

        Args:
            tokens (list[int]): token IDs to rebuild into text

        Returns:
            str | None: rebuilt text, or None when the endpoint is unavailable — caller falls back
        """
        try:                                                                           # attempt to reach the router
            request  = urllib.request.Request(                                         # build the detokenize request
                self._detokenize_url,
                data    = json.dumps({"tokens": tokens}).encode(),                     # token ID list to rebuild
                headers = {"Content-Type": "application/json"},
                method  = "POST",
            )
            with urllib.request.urlopen(request, timeout=10.0) as response:            # same timeout policy as tokenize
                return json.loads(response.read()).get("content", "")                  # return the rebuilt text

        except (urllib.error.URLError, TimeoutError, OSError, json.JSONDecodeError):   # broad catch — any failure is non-fatal
            return None                                                                # signal fallback to char-division

    def probe(self, content: str, overhead: int = 0) -> int:
        """
        Probe content and return the estimated chunk count.
    
        Args:
            content (str) : content to probe for chunk estimation
            overhead (int): structural chunk overhead to add — e.g. WMC.PMT_OVERHEAD; default 0
    
        Returns:
            int: estimated chunk count including overhead
        """
        if not content:                                                                                # guard — empty content yields zero tokens
            return 0                                                                                   # return zero tokens
            
        tokens: list[int] | None = self._tokenize(content)                                             # tokenize through the router — exact serving-model tokenizer
        if tokens is not None:                                                                         # endpoint reachable — token-accurate count
            return len(tokens) + overhead                                                              # return token count plus structural overhead
        # Fallback: character-division approximation
        return max(1, (len(content) + self._units_per_chunk - 1) // self._units_per_chunk) + overhead  # fallback — ceiling divide by chars-per-token constant, minimum 1

    def truncate(self, content: str, limit: int) -> str:
        """Truncate content to fit within the given chunk limit.
    
        Args:
            content : text content to truncate
            limit   : maximum number of chunks to retain
    
        Returns:
            str : truncated content fitting within the chunk limit
        """
        if not content or limit <= 0:                                                       # guard — empty content or zero limit yields empty string
            return ""                                                                       # return empty string

        tokens: list[int] | None = self._tokenize(content)                                  # tokenize through the router — exact serving-model tokenizer
        if tokens is not None:                                                              # endpoint reachable — token-accurate truncation
            if len(tokens) <= limit:                                                        # content already fits — no truncation needed
                return content                                                              # content fits within limit — no truncation needed
            rebuilt: str | None = self._detokenize(tokens[:limit])                          # rebuild text from the retained token prefix — exact chunk boundary
            if rebuilt is not None:                                                         # detokenize succeeded
                return rebuilt                                                              # return the truncated text
            return content[:limit * self._units_per_chunk]                                  # detokenize failed — fall back to char-division slicing
    
        return content[:limit * self._units_per_chunk]                                      # fallback — ceiling char-division approximation
