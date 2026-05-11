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

TODO:
    M2 — HRC takes ownership of manifest hydration lifecycle
    M2 — add manifest diff and rollback for safe runtime parameter updates
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
