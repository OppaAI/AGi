"""
HRU — Homeostatic Regulation Unit
===================================
AuRoRA · Homeostatic Regulation System (HRS)

Runtime utility layer of the HRS — propagates and manages the homeostatic 
regulation manifest within and across AuRoRA systems.

Responsibilities:
    - Hydrate AGi manifest constants from AuRoRA parameter server at node init
    - Estimate and truncate cognitive context chunks for budget management

Architecture:
    Stateless utilities — no ROS2 node, no persistent state.
    Imported by any AuRoRA node that needs manifest hydration or chunk sampling.
    HRM supplies the constants — HRU supplies the runtime tools that operate on them.
    HRC (M2) will orchestrate HRU at the system level for adaptive self-regulation.

Terminology:
    Manifest    — the AGi class tree of cognitive architecture constants
    Hydration   — binding ROS2 parameter server values into AGi manifest constants
    Chunk       — unit of cognitive engine context window size

Public interface:
    hydrate_manifest(core, system) → None
    
    ChunkSampler:
        probe(content, overhead?) → int
        truncate(content, limit)  → str
"""
# AGi libraries
from hrs.hrm import AGi                # manifest constants — operated on by hydration functions

def hydrate_manifest(core, system: str) -> None:
    """Hydrate the single source of truth manifest from parameters declared by AuRoRA or admin under the given system.

    Args:
        core        : Core node instance receiving the hydrated parameters
        system (str): System name identifying the manifest to hydrate (e.g. "scs")
    """
    manifest: type | None = _find_manifest(AGi, system)                        # recurse class tree — match nested subclass by system name
    if manifest is None:                                                       # if no match subclass is found,
        raise RuntimeError(f"❌ No manifest matching system name '{system}'")  # hard fail — system name must map to a known subclass
    _hydrate_system(core, manifest, system)                                    # walk manifest tree — declare and bind all parameters to core

def _find_manifest(tree: type, system: str) -> type | None:
    """
    Recursively search the AuRoRA class tree to locate the manifest subclass matching the given system name.

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
        matched_manifest: type | None = _find_manifest(subsystem, system)         # recurse into nested subclass — depth-first search
        if matched_manifest:                                                      # if manifest match the subclass
            return matched_manifest                                               # propagate match up the call stack
    return None
    
def _hydrate_system(core, manifest: type, system: str) -> None:
    """
    Recursively search the AuRoRA class tree and hydrate parameters declared by AuRoRA — skips static manifest parameters.

    Args:
        core            : Core node instance receiving the hydrated parameters
        manifest (type) : Manifest subclass to search through and hydrate
        system (str)    : System name identifying the manifest to hydrate
    """
    for param_name in vars(manifest):                                                # iterate attribute names of the manifest subclass
        if param_name.startswith('_'):                                               # if private members,
            continue                                                                 # skip — internal constants not exposed to parameter server
        param_value: type | int | float | bool | str = getattr(manifest, param_name) # retrieve the attribute value for inspection
        param_key: str = f"{system}.{param_name.lower()}"                            # build fully qualified parameter key — dot-separated namespace path

        if isinstance(param_value, type):                                        # if nested subclass is found,
            _hydrate_system(core, param_value, param_key)                        # recurse into nested subclass — depth-first walk
        elif isinstance(param_value, (int, float, bool, str)):                   # if parameter value is integer, float, boolean, string,
            core.declare_parameter(param_key, param_value)                       # declare parameter with default value on the core node
            setattr(manifest, param_name, core.get_parameter(param_key).value)   # write AuRoRA-declared value back to manifest — overrides default

@dataclass(frozen=True)
class GatewayMap:
    """
    Declarative registry of all AuRoRA gateway paths to access filesystem.
    Constructs absolute gateways from HRS manifest.

    Usage:
        gm = GatewayMap()                    # initialize gateway map
        db = gm.engram_db                    # gateway to engram complex
        gm.inspect_gateway(gm.engram_db)     # ensure gateway properly connected
    """
    home: Path = field(default_factory=Path.home)                                # [STATIC] OS home — injectable for testing

    @property
    def entity_root(self) -> Path:
        """Root for all AGi core system state."""
        return self.home / AGi.ENTITY_GATEWAY                                    # root directory of the filesystem (ie. ~/agi)

    # RAW Gateway
    @property
    def ras_gateway(self) -> Path:
        """Gateway to access the components of Reticular Activating System (RAS)."""
        return self.entity_root / RRR.RETICULAR_ACTIVATING_COMPARTMENT           # path to access files related to RAS (ie. ~/.agi/ras)

    @property
    def aurora_setpoints(self) -> Path:
        """AuRoRA setpoints — robot-wide AuRoRA self-regulated intrinsic parameters by RAS during ignition."""
        return self.ras_gateway / AGi.SCS.AURORA_SETPOINTS                       # path to access the self-adjusted parameters (ie. ~/.agi/ras/aurora.yaml)

    # SCS Gateway
    @property
    def scs_gateway(self) -> Path:
        """Gateway to access the components of Semantic Cognitive System (SCS)."""
        return self.entity_root / RRR.SEMANTIC_COGNITIVE_SYSTEM                  # path to access files related to SCS (ie. ~/.agi/scs)

    @property
    def user_profiles(self) -> Path:
        """User profiles — per-user extrinsic settings retrieved by CNC after login. (TODO: may change to database instead of yaml)"""
        return self.scs_gateway / AGi.SCS.USER_PROFILES                          # path to access user profile (ie. ~/.agi/scs/users.yaml)

    # GCE Gateway
    @property
    def gce_gateway(self) -> Path:
        """Gateway to access the components of Generative Cognitive Engine (GCE)."""
        return self.scs_gateway / RRR.GENERATIVE_COGNITIVE_ENGINE                # path to access files related to GCE (ie. ~/.agi/scs/gce)

    @property
    def active_persona(self) -> Path:
        """Active Persona — active persona retrieved by CNC at initialization."""
        return self.gce_gateway / AGi.SCS.PERSONA_PROFILES                       # path to access LLM persona (ie. ~/.agi/scs/gce/persona.yaml)

    # MCC Gateway
    @property
    def mcc_gateway(self) -> Path:
        """Gateway to access the components of Memory Coordination Cortex (MCC)."""
        return self.scs_gateway / AGi.SCS.MEMORY_GATEWAY                         # path to access files related to MCC (ie. ~/.agi/scs/mcc)

    @property
    def engram_complex(self) -> Path:
        """Engram complex — Storage of episodic memory, semantic memory and procedural memory across users. (TODO: may upgrade to Qdrant later on)"""
        return self.mcc_gateway / AGi.SCS.ENGRAM_COMPLEX                         # path to access the memory database (ie. ~/.agi/scs/mcc/engram_complex.db)

    # Helpers
    def inspect_gateway(self, gateway: Path) -> Path:
        """
        Inspect the gateway to determine if connections to parent connector are properly established.
        Generate the gateway chain if not established.

        Args:
            gateway (Path): target gateway to inspected

        Returns:
            Path: the gateway itself — for inline chaining
        """
        gateway.parent.mkdir(parents=True, exist_ok=True)                         # create parent dirs — idempotent, safe to call repeatedly
        return gateway                                                            # return gateway for inline chaining: f.open(gm.inspectgateway(gm.engram_db))

class ChunkSampler:
    """
    Sample the context and estimate chunk count for cognitive context management.
    Attempts probing with base cognitive engine first — falls back to chunk-division approximation.
    """
    def __init__(self, logger) -> None:
        """
        Initialize the ChunkSampler with a logger and attempt to load the base cognitive engine chunk sampler.
    
        Args:
            logger : logger instance for runtime diagnostics passed from the caller
        """        
        self.logger           = logger                                    # runtime diagnostics interface
        self._chunk_slicer    = None                                      # tokenizer — loaded from base model on init
        self._gce_base        = AGi.SCS.GCE.BASE_COGNITIVE_ENGINE         # base model name — used to load tokenizer
        self._units_per_chunk = AGi.SCS.UNITS_PER_CHUNK                   # chars-per-token constant — used in fallback approximation
        
        try:                                                                                          # attempt to load the tokenizer
            from transformers import AutoTokenizer                                                    # lazy import — only load HuggingFace tokenizer if available
            self.logger.info(f"⏳ Activating chunk sampler ({self._gce_base})…")                      # log the activating of the tokenizer
            self._chunk_slicer = AutoTokenizer.from_pretrained(self._gce_base)                        # load base model tokenizer — slices context into model-accurate chunks
            self.logger.info("✅ Chunk sampler activated")                                            # log the activation successful
        except Exception as e:                                                                        # if the tokenizer fails to load,
            self.logger.debug(f"Chunk sampler unavailable, falling back to chunk-division: {e}")      # soft fail — chunk-division approximation takes over
    
    def probe(self, content: str, overhead: int = 0) -> int:
        """
        Probe the content and return the estimated chunk count.
    
        Args:
            content (str) : content of the context to probe for chunk estimation
            overhead (int): structural token overhead to add —
                            e.g. WMC.PMT_OVERHEAD for WMC PMTs; Default 0 for pre-formatted content.
    
        Returns:
            int : estimated number of chunks in the content
        """
        if not content:                                                                                # guard — empty content yields zero tokens
            return 0                                                                                   # return zero tokens
            
        if self._chunk_slicer:                                                                         # if tokenizer loaded successfully,
            return len(self._chunk_slicer.encode(content, add_special_tokens=False)) + overhead        # encode content into tokens — return count excluding structural special tokens
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
    
        if self._chunk_slicer:                                                              # if chunk slicer loaded successfully,
            chunks = self._chunk_slicer.encode(content, add_special_tokens=False)           # encode content into chunk IDs — exclude structural special tokens
            if len(chunks) <= limit:                                                        # content already fits — no truncation needed
                return content                                                              # content fits within limit — no truncation needed
            return self._chunk_slicer.decode(chunks[:limit])                                # decode truncated tokens back to text — exact chunk boundary
    
        return content[:limit * self._units_per_chunk]                                      # fallback — ceiling char-division approximation
