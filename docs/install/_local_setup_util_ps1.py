# Copyright 2016-2019 Dirk Thomas
# Licensed under the Apache License, Version 2.0

import argparse
from collections import OrderedDict
import os
from pathlib import Path
import sys


FORMAT_STR_COMMENT_LINE = '# {comment}'
FORMAT_STR_SET_ENV_VAR = 'Set-Item -Path "Env:{name}" -Value "{value}"'
FORMAT_STR_USE_ENV_VAR = '$env:{name}'
FORMAT_STR_INVOKE_SCRIPT = '_colcon_prefix_powershell_source_script "{script_path}"'  # noqa: E501
FORMAT_STR_REMOVE_LEADING_SEPARATOR = ''  # noqa: E501
FORMAT_STR_REMOVE_TRAILING_SEPARATOR = ''  # noqa: E501

DSV_TYPE_APPEND_NON_DUPLICATE = 'append-non-duplicate'
DSV_TYPE_PREPEND_NON_DUPLICATE = 'prepend-non-duplicate'
DSV_TYPE_PREPEND_NON_DUPLICATE_IF_EXISTS = 'prepend-non-duplicate-if-exists'
DSV_TYPE_SET = 'set'
DSV_TYPE_SET_IF_UNSET = 'set-if-unset'
DSV_TYPE_SOURCE = 'source'


def main(argv=sys.argv[1:]):  # noqa: D103
    """
    Print shell commands for discovered packages in topological order.
    
    Parameters:
        argv (List[str]): Command-line arguments. Expected values:
            - argv[0]: primary shell file extension (required).
            - argv[1]: additional file extension (optional).
            - '--merged-install' flag (optional) to treat all install prefixes as a single merged location.
    
    The function discovers packages from the script's install layout, computes a topological ordering based on package runtime dependencies, processes each package's DSV entries to generate shell commands, and writes those commands to standard output.
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
    Discover installed packages and their runtime dependencies from a Colcon install layout.
    
    Scans the install prefix for colcon package index files (share/colcon-core/packages) and builds a mapping from package name to the set of its runtime dependencies. Only dependencies that correspond to discovered packages are retained.
    
    Parameters:
        prefix_path (Path): Install prefix containing package layouts.
        merged_install (bool): If True, packages are installed directly under the prefix (use the shared index). If False, each package is installed in its own subdirectory named after the package.
    
    Returns:
        dict: Mapping of package name to a set of its runtime dependency package names.
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
    Record runtime dependencies from a dependency file into the provided packages mapping using the file's name as the package key.
    
    Parameters:
        path (Path): Path to a file whose contents are runtime dependencies separated by os.pathsep.
        packages (dict): Mapping from package name to a set of runtime dependency names; this function sets packages[path.name] to the parsed dependency set.
    """
    content = path.read_text()
    dependencies = set(content.split(os.pathsep) if content else [])
    packages[path.name] = dependencies


def order_packages(packages):
    """
    Produce a topological ordering of package names based on their runtime dependencies.
    
    Parameters:
        packages (dict): Mapping from package name to a set of package names it depends on.
    
    Returns:
        list: Package names ordered so that each package appears after any packages it depends on.
    
    Raises:
        RuntimeError: If a circular dependency between packages is detected.
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
    Reduce the given package dependency mapping to the subset involved in a circular dependency.
    
    This function mutates the provided `packages` dict in place by removing any package that is not depended on by another remaining package until the set stabilizes or becomes empty.
    
    Parameters:
        packages (dict): Mapping from package name to a set of runtime dependency names; this mapping is modified in place.
    
    Returns:
        sequence or None: A sequence (view) of package names that remain and are part of the circular dependency when the reduction stabilizes, or `None` if no packages remain.
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
    Determine whether generated output should include comment lines.
    
    Checks the environment for the COLCON_TRACE variable to decide if comment
    lines (trace information) should be included.
    
    Returns:
        True if the COLCON_TRACE environment variable is set, False otherwise.
    """
    return bool(os.environ.get('COLCON_TRACE'))


def get_commands(pkg_name, prefix, primary_extension, additional_extension):
    """
    Collect shell command lines for a package by processing its package.dsv file if present.
    
    Parameters:
        pkg_name (str): Name of the package whose package.dsv will be looked up under <prefix>/share/<pkg_name>/package.dsv.
        prefix (str): Root install prefix used to locate the package.dsv and to resolve relative source paths.
        primary_extension (str or None): Primary file extension to prefer when emitting source-invoke commands (e.g., 'ps1').
        additional_extension (str or None): Additional file extension to also consider when emitting source-invoke commands.
    
    Returns:
        list[str]: Ordered list of command lines generated from the package.dsv file, or an empty list if no package.dsv exists.
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
    Parse a .dsv file and generate a list of shell command lines described by its entries.
    
    Processes each non-empty, non-comment line in the .dsv file at `dsv_path`. Non-`source` lines are delegated to handle_dsv_types_except_source which may produce commands; `source` lines are grouped by basename and filtered by the provided `primary_extension` and `additional_extension`. If a corresponding `.dsv` file exists for a basename it is processed recursively. For qualifying source entries, emits invoke-script commands using FORMAT_STR_INVOKE_SCRIPT. If COLCON_TRACE is set, a comment line for the processed file is prepended.
    
    Parameters:
    	dsv_path (str): Path to the .dsv file to read.
    	prefix (str): Base directory to resolve relative source paths.
    	primary_extension (str|None): Primary source file extension to prefer (e.g., "ps1"); may be None.
    	additional_extension (str|None): Alternate source file extension to consider; may be None.
    
    Returns:
    	commands (list[str]): Ordered list of command lines generated from the .dsv file and any recursively processed .dsv files.
    
    Raises:
    	RuntimeError: If a line in the file is missing the required semicolon separator, or if an error from handling a non-source line occurs (the error message is annotated with the line number and file path).
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
    Build command lines for DSV directives other than the `source` type.
    
    Processes `set`, `set_if_unset`, `append_non_duplicate`, `prepend_non_duplicate`, and
    `prepend_non_duplicate_if_exists` directives described by `type_` and `remainder`,
    resolving relative paths against `prefix` as needed.
    
    Parameters:
        type_ (str): The DSV directive type token.
        remainder (str): The portion of the DSV line after the initial type token and semicolon.
        prefix (str): Base path used to resolve relative file system paths referenced in values.
    
    Returns:
        list: A list of command lines (strings) representing environment mutations or
        comment lines generated for the directive.
    
    Raises:
        RuntimeError: If `remainder` is malformed (missing expected semicolon-separated fields)
        or if `type_` is not a recognized non-`source` DSV type.
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
    Append a path-like value to the module's tracked environment variable state and produce the shell command(s) to apply the change.
    
    Parameters:
        name (str): The environment variable name to modify.
        value (str): The value to append to the environment variable (treated as a path component).
    
    Returns:
        list[str]: A list containing a single command string to set the environment variable.
            If the value is already present in the tracked state and comment output is enabled,
            returns a single commented command string. If the value is already present and comments
            are disabled, returns an empty list.
    
    Side effects:
        Updates the global env_state mapping to include `value` in env_state[name].
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
    Prepare a command string that prepends `value` to the environment variable `name` and update the internal `env_state`.
    
    If `name` is not already tracked, `env_state` is initialized from the current process environment (if present). The function records `value` in `env_state` and returns a list containing a single command string that prepends `value` to `name`. If `value` is already recorded for `name`, the function returns a commented command when comment output is enabled, or an empty list when it is disabled.
    
    Parameters:
        name (str): The environment variable name to modify.
        value (str): The value to prepend to the environment variable.
    
    Returns:
        list[str]: A single-item list with the command string to prepend the value, a single-item list with the command commented out when a duplicate is detected and comments are enabled, or an empty list when a duplicate is detected and comments are disabled.
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
    Generate command lines to remove leading and trailing path separators from environment variables added by this script.
    
    The function returns commands for each environment variable tracked in the module state that did not already exist in the process environment; if the configured formatter for removing trailing separators is None, or no variables require modification, an empty list is returned.
    
    Returns:
        list[str]: Shell command strings to remove leading and trailing separators, or an empty list.
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
    Set the generated environment variable value and return the corresponding set command line.
    
    Parameters:
        name (str): The environment variable name to set.
        value (str): The value to assign to the environment variable; stored in module state.
    
    Returns:
        list[str]: A single-item list containing the formatted command line that sets the environment variable.
    """
    global env_state
    env_state[name] = value
    line = FORMAT_STR_SET_ENV_VAR.format_map(
        {'name': name, 'value': value})
    return [line]


def _set_if_unset(name, value):
    """
    Produce a command line to set an environment variable, or a commented command if the variable is already set.
    
    Parameters:
        name (str): Environment variable name to set.
        value (str): Value to assign to the environment variable.
    
    Returns:
        list[str]: A single-item list containing the formatted set command, or a commented set command if the variable already exists in the current env_state or in os.environ.
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