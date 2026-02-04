# Copyright 2016-2019 Dirk Thomas
# Licensed under the Apache License, Version 2.0

import argparse
from collections import OrderedDict
import os
from pathlib import Path
import sys


FORMAT_STR_COMMENT_LINE = '# {comment}'
FORMAT_STR_SET_ENV_VAR = 'export {name}="{value}"'
FORMAT_STR_USE_ENV_VAR = '${name}'
FORMAT_STR_INVOKE_SCRIPT = 'COLCON_CURRENT_PREFIX="{prefix}" _colcon_prefix_sh_source_script "{script_path}"'  # noqa: E501
FORMAT_STR_REMOVE_LEADING_SEPARATOR = 'if [ "$(echo -n ${name} | head -c 1)" = ":" ]; then export {name}=${{{name}#?}} ; fi'  # noqa: E501
FORMAT_STR_REMOVE_TRAILING_SEPARATOR = 'if [ "$(echo -n ${name} | tail -c 1)" = ":" ]; then export {name}=${{{name}%?}} ; fi'  # noqa: E501

DSV_TYPE_APPEND_NON_DUPLICATE = 'append-non-duplicate'
DSV_TYPE_PREPEND_NON_DUPLICATE = 'prepend-non-duplicate'
DSV_TYPE_PREPEND_NON_DUPLICATE_IF_EXISTS = 'prepend-non-duplicate-if-exists'
DSV_TYPE_SET = 'set'
DSV_TYPE_SET_IF_UNSET = 'set-if-unset'
DSV_TYPE_SOURCE = 'source'


def main(argv=sys.argv[1:]):  # noqa: D103
    """
    Emit shell commands that set up environment variables and source package scripts for packages
    ordered topologically by runtime dependencies.
    
    Parses command-line arguments from `argv`:
    - first positional: `primary_extension` — file extension of the primary shell script to invoke.
    - optional second positional: `additional_extension` — extra file extension to consider.
    - optional flag `--merged-install` — treat all install prefixes as merged into a single location.
    
    Parameters:
        argv (list[str]): Command-line arguments to parse (defaults to sys.argv[1:]).
    """
    parser = argparse.ArgumentParser(
        description='Output shell commands for the packages in topological '
                    'order')
    parser.add_argument(
        'primary_extension',
        help='The file extension of the primary shell')
    parser.add_argument(
        'additional_extension', nargs='?',
        help='The additional file extension to be considered')
    parser.add_argument(
        '--merged-install', action='store_true',
        help='All install prefixes are merged into a single location')
    args = parser.parse_args(argv)

    packages = get_packages(Path(__file__).parent, args.merged_install)

    ordered_packages = order_packages(packages)
    for pkg_name in ordered_packages:
        if _include_comments():
            print(
                FORMAT_STR_COMMENT_LINE.format_map(
                    {'comment': 'Package: ' + pkg_name}))
        prefix = os.path.abspath(os.path.dirname(__file__))
        if not args.merged_install:
            prefix = os.path.join(prefix, pkg_name)
        for line in get_commands(
            pkg_name, prefix, args.primary_extension,
            args.additional_extension
        ):
            print(line)

    for line in _remove_ending_separators():
        print(line)


def get_packages(prefix_path, merged_install):
    """
    Discover installed packages under the given install prefix and return each package's runtime dependency names.
    
    When `merged_install` is True this inspects files in `share/colcon-core/packages` under the prefix (one file per package). When False it looks for per-package files at `<prefix>/<pkg>/share/colcon-core/packages/<pkg>`. Dependencies listed in those files are filtered to only include packages that are present in the discovered set.
    
    Parameters:
        prefix_path (Path): Install prefix to search for package metadata.
        merged_install (bool): If True, packages are installed directly in the prefix (merged layout); if False, each package is installed in a subdirectory named after the package.
    
    Returns:
        dict: Mapping from package name (str) to a set of runtime dependency package names (set of str).
    """
    packages = {}
    # since importing colcon_core isn't feasible here the following constant
    # must match colcon_core.location.get_relative_package_index_path()
    subdirectory = 'share/colcon-core/packages'
    if merged_install:
        # return if workspace is empty
        if not (prefix_path / subdirectory).is_dir():
            return packages
        # find all files in the subdirectory
        for p in (prefix_path / subdirectory).iterdir():
            if not p.is_file():
                continue
            if p.name.startswith('.'):
                continue
            add_package_runtime_dependencies(p, packages)
    else:
        # for each subdirectory look for the package specific file
        for p in prefix_path.iterdir():
            if not p.is_dir():
                continue
            if p.name.startswith('.'):
                continue
            p = p / subdirectory / p.name
            if p.is_file():
                add_package_runtime_dependencies(p, packages)

    # remove unknown dependencies
    pkg_names = set(packages.keys())
    for k in packages.keys():
        packages[k] = {d for d in packages[k] if d in pkg_names}

    return packages


def add_package_runtime_dependencies(path, packages):
    """
    Read a DSV-like resource file and record its runtime dependencies in the provided mapping.
    
    The function reads the text content of `path`, splits it on `os.pathsep` into dependency names
    (or produces an empty set for empty content), and stores the resulting set under `path.name`
    in the `packages` mapping.
    
    Parameters:
        path (Path): Path to the resource file containing runtime dependency entries.
        packages (dict): Mapping of package name to a set of runtime dependency names; this mapping
            is modified in-place by assigning the parsed dependency set to `packages[path.name]`.
    """
    content = path.read_text()
    dependencies = set(content.split(os.pathsep) if content else [])
    packages[path.name] = dependencies


def order_packages(packages):
    """
    Produce a topologically ordered list of package names based on their runtime dependencies.
    
    The input mapping is mutated: dependency entries for packages that are selected are removed from other packages' dependency sets. If a circular dependency is detected, a RuntimeError is raised listing the packages involved.
    
    Parameters:
        packages (dict): Mapping from package name to a set of runtime dependency package names; this mapping is modified in-place.
    
    Returns:
        list: Package names in topological order.
    """
    # select packages with no dependencies in alphabetical order
    to_be_ordered = list(packages.keys())
    ordered = []
    while to_be_ordered:
        pkg_names_without_deps = [
            name for name in to_be_ordered if not packages[name]]
        if not pkg_names_without_deps:
            reduce_cycle_set(packages)
            raise RuntimeError(
                'Circular dependency between: ' + ', '.join(sorted(packages)))
        pkg_names_without_deps.sort()
        pkg_name = pkg_names_without_deps[0]
        to_be_ordered.remove(pkg_name)
        ordered.append(pkg_name)
        # remove item from dependency lists
        for k in list(packages.keys()):
            if pkg_name in packages[k]:
                packages[k].remove(pkg_name)
    return ordered


def reduce_cycle_set(packages):
    """
    Trim the input mapping to only the packages that participate in a circular dependency.
    
    This function mutates the provided mapping by removing any package that is not
    referenced as a dependency by another remaining package. It repeats this
    pruning until the set of dependent packages stabilizes.
    
    Parameters:
        packages (dict): Mapping from package name to a set of runtime dependency
            names; the mapping is modified in place.
    
    Returns:
        iterable or None: An iterable of package names representing the reduced
        cycle when the set stabilizes, or `None` if the mapping becomes empty.
    """
    last_depended = None
    while len(packages) > 0:
        # get all remaining dependencies
        depended = set()
        for pkg_name, dependencies in packages.items():
            depended = depended.union(dependencies)
        # remove all packages which are not dependent on
        for name in list(packages.keys()):
            if name not in depended:
                del packages[name]
        if last_depended:
            # if remaining packages haven't changed return them
            if last_depended == depended:
                return packages.keys()
        # otherwise reduce again
        last_depended = depended


def _include_comments():
    # skipping comment lines when COLCON_TRACE is not set speeds up the
    # processing especially on Windows
    """
    Determine whether to include comment lines based on the COLCON_TRACE environment variable.
    
    Returns:
        `True` if the COLCON_TRACE environment variable is set, `False` otherwise.
    """
    return bool(os.environ.get('COLCON_TRACE'))


def get_commands(pkg_name, prefix, primary_extension, additional_extension):
    """
    Collect shell commands for a package by reading its package.dsv under the package's share directory (if present).
    
    Parameters:
        pkg_name (str): Package name used to locate the package.dsv file.
        prefix (str): Install prefix path containing the package share directory.
        primary_extension (str | None): Primary script file extension to source (e.g., ".sh"); may be None.
        additional_extension (str | None): Additional script file extension to source alongside the primary; may be None.
    
    Returns:
        list[str]: Ordered list of shell command strings generated from the package.dsv file, or an empty list if no package.dsv exists.
    """
    commands = []
    package_dsv_path = os.path.join(prefix, 'share', pkg_name, 'package.dsv')
    if os.path.exists(package_dsv_path):
        commands += process_dsv_file(
            package_dsv_path, prefix, primary_extension, additional_extension)
    return commands


def process_dsv_file(
    dsv_path, prefix, primary_extension=None, additional_extension=None
):
    """
    Parse a DSV file and generate shell command lines for environment and sourcing actions.
    
    Parameters:
        dsv_path (str or Path): Path to the .dsv file to process.
        prefix (str): Installation prefix used to resolve relative paths referenced in the DSV.
        primary_extension (str | None): File extension considered primary for sourcing (e.g., 'sh'); if None, primary sourcing is disabled.
        additional_extension (str | None): Alternate file extension to source when primary is not selected; if None, additional sourcing is disabled.
    
    Returns:
        list[str]: Ordered shell command strings produced from the DSV entries, including commands returned by handlers and invoked script source commands.
    
    Raises:
        RuntimeError: If a DSV line is malformed (missing semicolon) or if processing a non-source line fails; error messages include the offending line number and file path.
    """
    commands = []
    if _include_comments():
        commands.append(FORMAT_STR_COMMENT_LINE.format_map({'comment': dsv_path}))
    with open(dsv_path, 'r') as h:
        content = h.read()
    lines = content.splitlines()

    basenames = OrderedDict()
    for i, line in enumerate(lines):
        # skip over empty or whitespace-only lines
        if not line.strip():
            continue
        # skip over comments
        if line.startswith('#'):
            continue
        try:
            type_, remainder = line.split(';', 1)
        except ValueError:
            raise RuntimeError(
                "Line %d in '%s' doesn't contain a semicolon separating the "
                'type from the arguments' % (i + 1, dsv_path))
        if type_ != DSV_TYPE_SOURCE:
            # handle non-source lines
            try:
                commands += handle_dsv_types_except_source(
                    type_, remainder, prefix)
            except RuntimeError as e:
                raise RuntimeError(
                    "Line %d in '%s' %s" % (i + 1, dsv_path, e)) from e
        else:
            # group remaining source lines by basename
            path_without_ext, ext = os.path.splitext(remainder)
            if path_without_ext not in basenames:
                basenames[path_without_ext] = set()
            assert ext.startswith('.')
            ext = ext[1:]
            if ext in (primary_extension, additional_extension):
                basenames[path_without_ext].add(ext)

    # add the dsv extension to each basename if the file exists
    for basename, extensions in basenames.items():
        if not os.path.isabs(basename):
            basename = os.path.join(prefix, basename)
        if os.path.exists(basename + '.dsv'):
            extensions.add('dsv')

    for basename, extensions in basenames.items():
        if not os.path.isabs(basename):
            basename = os.path.join(prefix, basename)
        if 'dsv' in extensions:
            # process dsv files recursively
            commands += process_dsv_file(
                basename + '.dsv', prefix, primary_extension=primary_extension,
                additional_extension=additional_extension)
        elif primary_extension in extensions and len(extensions) == 1:
            # source primary-only files
            commands += [
                FORMAT_STR_INVOKE_SCRIPT.format_map({
                    'prefix': prefix,
                    'script_path': basename + '.' + primary_extension})]
        elif additional_extension in extensions:
            # source non-primary files
            commands += [
                FORMAT_STR_INVOKE_SCRIPT.format_map({
                    'prefix': prefix,
                    'script_path': basename + '.' + additional_extension})]

    return commands


def handle_dsv_types_except_source(type_, remainder, prefix):
    """
    Process a single DSV entry that is not a `source` type and return the shell commands it produces.
    
    Parameters:
        type_ (str): DSV entry type (one of the DSV_TYPE_* constants, excluding `source`).
        remainder (str): The remainder of the DSV line after the type token; expected formats:
            - For `set` / `set-if-unset`: "ENV_NAME;value"
            - For append/prepend variants: "ENV_NAME;value1;value2;..."
        prefix (str or Path): Filesystem prefix used to resolve relative paths or empty values.
    
    Returns:
        list[str]: A list of shell command lines (strings) to apply the described environment changes.
    
    Raises:
        RuntimeError: If `remainder` is malformed (missing the required semicolon separator)
                      or if `type_` is not a recognized non-source DSV type.
    """
    commands = []
    if type_ in (DSV_TYPE_SET, DSV_TYPE_SET_IF_UNSET):
        try:
            env_name, value = remainder.split(';', 1)
        except ValueError:
            raise RuntimeError(
                "doesn't contain a semicolon separating the environment name "
                'from the value')
        try_prefixed_value = os.path.join(prefix, value) if value else prefix
        if os.path.exists(try_prefixed_value):
            value = try_prefixed_value
        if type_ == DSV_TYPE_SET:
            commands += _set(env_name, value)
        elif type_ == DSV_TYPE_SET_IF_UNSET:
            commands += _set_if_unset(env_name, value)
        else:
            assert False
    elif type_ in (
        DSV_TYPE_APPEND_NON_DUPLICATE,
        DSV_TYPE_PREPEND_NON_DUPLICATE,
        DSV_TYPE_PREPEND_NON_DUPLICATE_IF_EXISTS
    ):
        try:
            env_name_and_values = remainder.split(';')
        except ValueError:
            raise RuntimeError(
                "doesn't contain a semicolon separating the environment name "
                'from the values')
        env_name = env_name_and_values[0]
        values = env_name_and_values[1:]
        for value in values:
            if not value:
                value = prefix
            elif not os.path.isabs(value):
                value = os.path.join(prefix, value)
            if (
                type_ == DSV_TYPE_PREPEND_NON_DUPLICATE_IF_EXISTS and
                not os.path.exists(value)
            ):
                comment = f'skip extending {env_name} with not existing ' \
                    f'path: {value}'
                if _include_comments():
                    commands.append(
                        FORMAT_STR_COMMENT_LINE.format_map({'comment': comment}))
            elif type_ == DSV_TYPE_APPEND_NON_DUPLICATE:
                commands += _append_unique_value(env_name, value)
            else:
                commands += _prepend_unique_value(env_name, value)
    else:
        raise RuntimeError(
            'contains an unknown environment hook type: ' + type_)
    return commands


env_state = {}


def _append_unique_value(name, value):
    """
    Ensure a value is appended to an environment variable without creating duplicates and produce the shell command to do so.
    
    Updates the module-level env_state to record the appended value (initializing the entry from the current environment if necessary) and returns the shell command that appends the given value to the named environment variable. If the value is already recorded, returns an empty list unless COLCON_TRACE is enabled, in which case a commented command line is returned.
    
    Parameters:
        name (str): Environment variable name to modify.
        value (str): Value to append to the environment variable.
    
    Returns:
        list[str]: A single-element list containing the shell command to append the value, or an empty list (or a single commented command when comments are enabled) if the value was already present.
    """
    global env_state
    if name not in env_state:
        if os.environ.get(name):
            env_state[name] = set(os.environ[name].split(os.pathsep))
        else:
            env_state[name] = set()
    # append even if the variable has not been set yet, in case a shell script sets the
    # same variable without the knowledge of this Python script.
    # later _remove_ending_separators() will cleanup any unintentional leading separator
    extend = FORMAT_STR_USE_ENV_VAR.format_map({'name': name}) + os.pathsep
    line = FORMAT_STR_SET_ENV_VAR.format_map(
        {'name': name, 'value': extend + value})
    if value not in env_state[name]:
        env_state[name].add(value)
    else:
        if not _include_comments():
            return []
        line = FORMAT_STR_COMMENT_LINE.format_map({'comment': line})
    return [line]


def _prepend_unique_value(name, value):
    """
    Prepend a value to an environment variable for the generated shell commands and record the addition in the internal environment state.
    
    Parameters:
        name (str): Environment variable name to modify.
        value (str): Value to prepend to the variable.
    
    Returns:
        list[str]: One-element list containing the shell command that sets the environment variable (with the value prepended), or an empty list if no command should be emitted. The internal environment state is updated to include `value`; if the value was already present, a commented command line may be returned instead of an active command when comment emission is enabled.
    """
    global env_state
    if name not in env_state:
        if os.environ.get(name):
            env_state[name] = set(os.environ[name].split(os.pathsep))
        else:
            env_state[name] = set()
    # prepend even if the variable has not been set yet, in case a shell script sets the
    # same variable without the knowledge of this Python script.
    # later _remove_ending_separators() will cleanup any unintentional trailing separator
    extend = os.pathsep + FORMAT_STR_USE_ENV_VAR.format_map({'name': name})
    line = FORMAT_STR_SET_ENV_VAR.format_map(
        {'name': name, 'value': value + extend})
    if value not in env_state[name]:
        env_state[name].add(value)
    else:
        if not _include_comments():
            return []
        line = FORMAT_STR_COMMENT_LINE.format_map({'comment': line})
    return [line]


# generate commands for removing prepended underscores
def _remove_ending_separators():
    # do nothing if the shell extension does not implement the logic
    """
    Produce shell commands to remove leading and trailing separators for environment variables
    that were introduced by this script.
    
    If the platform-specific removal templates are not defined, returns an empty list.
    Variables that already existed in the environment before the script ran are skipped.
    
    Returns:
        list[str]: Shell command strings to remove leading and trailing separators for each
        environment variable managed by this script; returns an empty list if not supported.
    """
    if FORMAT_STR_REMOVE_TRAILING_SEPARATOR is None:
        return []

    global env_state
    commands = []
    for name in env_state:
        # skip variables that already had values before this script started prepending
        if name in os.environ:
            continue
        commands += [
            FORMAT_STR_REMOVE_LEADING_SEPARATOR.format_map({'name': name}),
            FORMAT_STR_REMOVE_TRAILING_SEPARATOR.format_map({'name': name})]
    return commands


def _set(name, value):
    """
    Set an environment variable in the module's internal state and produce the corresponding shell export command.
    
    Parameters:
        name (str): The environment variable name to set.
        value (str): The value to assign to the environment variable; this overwrites any previous value stored in the internal state.
    
    Returns:
        list[str]: A single-element list containing the shell command that exports the environment variable.
    """
    global env_state
    env_state[name] = value
    line = FORMAT_STR_SET_ENV_VAR.format_map(
        {'name': name, 'value': value})
    return [line]


def _set_if_unset(name, value):
    """
    Prepare a shell export command that sets an environment variable only if it is not already set.
    
    Parameters:
    	name (str): Environment variable name to set.
    	value (str): Value to assign to the environment variable.
    
    Returns:
    	list[str]: A single-element list containing the export command; if the variable is already present
    	in the current env state or os.environ, the command is returned as a commented line.
    """
    global env_state
    line = FORMAT_STR_SET_ENV_VAR.format_map(
        {'name': name, 'value': value})
    if env_state.get(name, os.environ.get(name)):
        line = FORMAT_STR_COMMENT_LINE.format_map({'comment': line})
    return [line]


if __name__ == '__main__':  # pragma: no cover
    try:
        rc = main()
    except RuntimeError as e:
        print(str(e), file=sys.stderr)
        rc = 1
    sys.exit(rc)