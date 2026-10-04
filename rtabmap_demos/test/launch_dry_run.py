"""
Run a launch file through a real LaunchService without starting any process.

Every action is visited as it would be by `ros2 launch`: arguments are declared and
validated, conditions evaluated, OpaqueFunctions called, includes loaded, and every
substitution of every node resolved. Only the very last step -- spawning the process,
or asking a container to load a component -- is replaced by recording what would have
been started. What a demo breaks on is almost always visible at that point: a file
moved, an argument renamed in an included launch file, an RTAB-Map parameter renamed,
an executable that no longer exists.

Packages the machine does not have (simulators, robot descriptions) are replaced by
empty stubs instead of failing the run: their launch files include nothing and their
nodes are recorded unchecked. What the demo itself passes on is still checked. Packages
named rtabmap* are never stubbed, so a typo in one of ours still fails.

check_launch_file() is the whole check of one launch file, as the test_launch_files of
rtabmap_demos and rtabmap_examples run it. This file is installed with rtabmap_demos
(share/rtabmap_demos/test) so that other packages' tests can import it.
"""

import contextlib
import functools
import logging
import os
import subprocess
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

import ament_index_python
import ament_index_python.packages
import launch_ros.substitutions.find_package
import yaml
from ament_index_python.packages import (PackageNotFoundError, get_package_prefix,
                                         get_package_share_directory)
from launch import LaunchContext, LaunchService
from launch.actions import (DeclareLaunchArgument, ExecuteLocal, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import (AnyLaunchDescriptionSource,
                                               FrontendLaunchDescriptionSource,
                                               PythonLaunchDescriptionSource)
from launch.launch_description_sources.python_launch_file_utilities import \
    get_launch_description_from_python_launch_file
from launch.substitutions.substitution_failure import SubstitutionFailure
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import LifecycleNode, LoadComposableNodes, Node
from launch_ros.utilities import evaluate_parameters


@dataclass
class Started:
    """A process or component the launch file would have started."""

    kind: str                       # 'node', 'component' or 'process'
    package: Optional[str]
    executable: str                 # executable, component plugin, or cmd[0]
    stubbed: bool
    cmd: List[str] = field(default_factory=list)
    parameters: Dict[str, object] = field(default_factory=dict)

    def describe(self) -> str:
        if self.package:
            return f'{self.kind} {self.package}/{self.executable}'
        return f'{self.kind} {self.executable}'


@dataclass
class DryRunResult:
    started: List[Started] = field(default_factory=list)
    includes: List[str] = field(default_factory=list)
    stubbed_packages: Set[str] = field(default_factory=set)
    problems: List[str] = field(default_factory=list)


def _is_rtabmap_package(package: Optional[str]) -> bool:
    return bool(package) and package.startswith('rtabmap')


def _flatten(params: dict, prefix: str = '') -> Dict[str, object]:
    out = {}
    for key, value in params.items():
        name = f'{prefix}{key}'
        if isinstance(value, dict):
            out.update(_flatten(value, name + '.'))
        else:
            out[name] = value
    return out


class _ParamsLoader(yaml.SafeLoader):
    """launch_ros writes a list parameter given as a Python tuple with its Python tag."""


_ParamsLoader.add_constructor(
    'tag:yaml.org,2002:python/tuple',
    lambda loader, node: tuple(loader.construct_sequence(node)))


def _load_param_file(path: str) -> Dict[str, object]:
    """All parameters of a ROS 2 params file, every node block merged."""
    with open(path) as f:
        content = yaml.load(f, Loader=_ParamsLoader) or {}
    params = {}
    for block in content.values():
        if isinstance(block, dict) and isinstance(block.get('ros__parameters'), dict):
            params.update(_flatten(block['ros__parameters']))
    return params


class _DryRun:
    """The patched hooks. One instance per run; they all record into self.result."""

    def __init__(self, stub_root: str, known_parameters: Optional[Set[str]]):
        self.result = DryRunResult()
        self.stub_root = stub_root
        self.known_parameters = known_parameters
        self.rtabmap_shares = sorted(
            os.path.join(prefix, 'share', name)
            for name, prefix in ament_index_python.get_packages_with_prefixes().items()
            if _is_rtabmap_package(name))
        self._real_get_package_prefix = ament_index_python.packages.get_package_prefix

    # -- checks -------------------------------------------------------------------------

    def problem(self, msg: str):
        if msg not in self.result.problems:
            self.result.problems.append(msg)

    def is_stub_path(self, path: str) -> bool:
        return path.startswith(self.stub_root + os.sep)

    def in_rtabmap_share(self, path: str) -> bool:
        return any(path == share or path.startswith(share + os.sep)
                   for share in self.rtabmap_shares)

    def check_path(self, value, where: str):
        """A path into one of our packages must exist. Others are not ours to check."""
        if isinstance(value, str) and self.in_rtabmap_share(value) and not os.path.exists(value):
            self.problem(f'{where}: {value} does not exist')

    def check_parameters(self, started: Started):
        """RTAB-Map's own parameters ("Group/Name") must exist in the installed library."""
        if self.known_parameters is None or not _is_rtabmap_package(started.package):
            return
        for name, value in started.parameters.items():
            self.check_path(value, f'{started.describe()} parameter {name}')
            group, sep, rest = name.partition('/')
            if not sep or not group.isalnum() or '/' in rest or '.' in name:
                continue
            if name not in self.known_parameters:
                # Renamed parameters still work, with a warning asking to update the
                # launch file; the installed library no longer lists the old name.
                self.problem(f'{started.describe()}: "{name}" is not a parameter of the '
                             'installed RTAB-Map (renamed or removed)')

    def record(self, started: Started):
        self.check_parameters(started)
        self.result.started.append(started)

    # -- package lookup -----------------------------------------------------------------

    def get_package_prefix(self, package_name):
        try:
            return self._real_get_package_prefix(package_name)
        except PackageNotFoundError:
            if _is_rtabmap_package(package_name):
                raise
            prefix = os.path.join(self.stub_root, package_name)
            os.makedirs(os.path.join(prefix, 'share', package_name), exist_ok=True)
            self.result.stubbed_packages.add(package_name)
            return prefix

    def is_stubbed(self, package: Optional[str]) -> bool:
        return package in self.result.stubbed_packages

    # -- launch hooks -------------------------------------------------------------------

    def wrap_get_launch_description(self, original):
        def _get_launch_description(source, location):
            if self.is_stub_path(location):
                from launch import LaunchDescription
                return LaunchDescription([])
            return original(source, location)
        return _get_launch_description

    def wrap_include_execute(self, original):
        def execute(action: IncludeLaunchDescription, context: LaunchContext):
            entities = original(action, context)
            location = action.launch_description_source.location
            self.result.includes.append(location)
            passed = {
                perform_substitutions(context, normalize_to_list_of_substitutions(name)):
                perform_substitutions(context, normalize_to_list_of_substitutions(value))
                for name, value in action.launch_arguments}
            for name, value in passed.items():
                self.check_path(value, f'argument {name} passed to {os.path.basename(location)}')
            # Launch accepts any argument and silently ignores the ones the included file
            # does not declare, so a renamed argument breaks the demo without a word. For
            # our own files it is always a mistake; others may read undeclared ones.
            if any(location.startswith(share + os.sep) for share in self.rtabmap_shares):
                declared = {
                    arg.name for arg in
                    action.launch_description_source.get_launch_description(context)
                    .get_launch_arguments()}
                for name in sorted(set(passed) - declared):
                    self.problem(
                        f'argument "{name}" passed to {os.path.basename(location)}, '
                        'which does not declare it')
            return entities
        return execute

    def wrap_declare_execute(self, original):
        def execute(action: DeclareLaunchArgument, context: LaunchContext):
            entities = original(action, context)
            self.check_path(context.launch_configurations.get(action.name),
                            f'launch argument {action.name}')
            return entities
        return execute

    def execute_local(self, action: ExecuteLocal, context: LaunchContext):
        """Record the process instead of starting it."""
        package = None
        if isinstance(action, Node):
            package = perform_substitutions(
                context, normalize_to_list_of_substitutions(action.node_package))
        cmd = []
        for i, part in enumerate(action.cmd):
            try:
                cmd.append(perform_substitutions(context, part))
            except (SubstitutionFailure, PackageNotFoundError) as e:
                # The executable of a stubbed package cannot be found, and a plain
                # ExecuteProcess may call a tool (gz, xacro) this machine does not have.
                if i == 0 and (self.is_stubbed(package) or not isinstance(action, Node)):
                    cmd.append('<unresolved>')
                else:
                    self.problem(f'{package or "process"}: {e}')
                    return None

        if isinstance(action, Node):
            executable = perform_substitutions(
                context, normalize_to_list_of_substitutions(action.node_executable))
            started = Started('node', package, executable, self.is_stubbed(package), cmd)
        else:
            started = Started('process', None, os.path.basename(cmd[0]), False, cmd)

        for i, token in enumerate(cmd):
            self.check_path(token, started.describe())
            previous = cmd[i - 1] if i > 0 else None
            if previous == '--params-file':
                if not self.is_stub_path(token) and not os.path.isfile(token):
                    self.problem(f'{started.describe()}: params file {token} does not exist')
                    continue
                if not self.is_stub_path(token):
                    started.parameters.update(_load_param_file(token))
            elif previous == '-p' and ':=' in token:
                name, value = token.split(':=', 1)
                started.parameters[name] = value
            elif (previous == '-d' and os.path.isabs(token) and not self.is_stub_path(token)
                  and not self.in_rtabmap_share(token) and not os.path.exists(token)):
                # rviz2's config; rtabmap's own -d (delete the database) takes no value.
                self.problem(f'{started.describe()}: -d {token} does not exist')
        self.record(started)
        return None

    def wrap_load_composable_nodes_init(self, original):
        def __init__(action, *args, **kwargs):
            original(action, *args, **kwargs)
            descriptions = kwargs.get('composable_node_descriptions')
            if descriptions is None and args:
                descriptions = args[0]
            action._dry_run_descriptions = list(descriptions or [])
        return __init__

    def load_composable_nodes(self, action: LoadComposableNodes, context: LaunchContext):
        """Record the components instead of asking the container to load them."""
        for description in action._dry_run_descriptions:
            package = perform_substitutions(context, description.package)
            plugin = perform_substitutions(context, description.node_plugin)
            # A component's package is not looked up by launch, only by the container
            # loading it: look it up here, so a package not installed gets stubbed like
            # a node's would.
            self.get_package_prefix(package)
            stubbed = self.is_stubbed(package)
            started = Started('component', package, plugin, stubbed)
            if not stubbed:
                try:
                    content, _ = ament_index_python.get_resource('rclcpp_components', package)
                    plugins = [line.split(';')[0] for line in content.splitlines()]
                    if plugin not in plugins:
                        self.problem(f'component {plugin} is not registered by {package}')
                except LookupError:
                    self.problem(f'package {package} registers no components (wanted {plugin})')
            if description.parameters is not None:
                for evaluated in evaluate_parameters(context, description.parameters):
                    if isinstance(evaluated, dict):
                        started.parameters.update(_flatten(evaluated))
                    else:
                        path = str(evaluated)
                        if not os.path.isfile(path):
                            self.problem(f'{started.describe()}: params file {path} does not exist')
                        else:
                            started.parameters.update(_load_param_file(path))
            self.record(started)
        return None


class _ErrorCollector(logging.Handler):
    """LaunchService reports an exception by logging it and returning 1.

    launch_ros only warns about a parameter file that does not exist, and starts the
    node without it; for a demo that is as broken as an exception.
    """

    def __init__(self):
        super().__init__(logging.WARNING)
        self.messages = []

    def emit(self, record):
        message = record.getMessage()
        if record.levelno >= logging.ERROR or 'Parameter file path is not a file' in message:
            self.messages.append(message)


@contextlib.contextmanager
def _patched(session: _DryRun):
    patches = []

    def patch(owner, name, value):
        patches.append((owner, name, getattr(owner, name)))
        setattr(owner, name, value)

    # Launch files bind these by name when they are loaded -- which happens inside the
    # run, after the patch -- but launch_ros bound its copies at import.
    for module in (ament_index_python.packages, ament_index_python,
                   launch_ros.substitutions.find_package):
        patch(module, 'get_package_prefix', session.get_package_prefix)
    for cls in (PythonLaunchDescriptionSource, AnyLaunchDescriptionSource,
                FrontendLaunchDescriptionSource):
        patch(cls, '_get_launch_description',
              session.wrap_get_launch_description(cls._get_launch_description))
    patch(IncludeLaunchDescription, 'execute',
          session.wrap_include_execute(IncludeLaunchDescription.execute))
    patch(DeclareLaunchArgument, 'execute',
          session.wrap_declare_execute(DeclareLaunchArgument.execute))
    # Node and ExecuteProcess do all their substitutions, then hand over to
    # ExecuteLocal.execute to spawn. Stopping there keeps everything before it real.
    patch(ExecuteLocal, 'execute', lambda action, context: session.execute_local(action, context))
    # LifecycleNode subscribes to the node through rclpy before running it.
    patch(LifecycleNode, 'execute', lambda action, context: Node.execute(action, context))
    patch(LoadComposableNodes, '__init__',
          session.wrap_load_composable_nodes_init(LoadComposableNodes.__init__))
    patch(LoadComposableNodes, 'execute',
          lambda action, context: session.load_composable_nodes(action, context))
    # Timers only wait for a simulator that is not there; what they start is what matters.
    patch(TimerAction, 'execute', lambda action, context: list(action.actions))
    try:
        yield
    finally:
        for owner, name, value in reversed(patches):
            setattr(owner, name, value)


@contextlib.contextmanager
def _restored_environ():
    # On Humble SetEnvironmentVariable writes os.environ directly, and some demos set
    # os.environ themselves (TURTLEBOT3_MODEL). Keep each run from seeing the last one's.
    saved = os.environ.copy()
    try:
        yield
    finally:
        os.environ.clear()
        os.environ.update(saved)


@contextlib.contextmanager
def _session(known_parameters):
    with tempfile.TemporaryDirectory(prefix='rtabmap_demos_stubs_') as stub_root:
        session = _DryRun(os.path.realpath(stub_root), known_parameters)
        with _patched(session), _restored_environ():
            yield session


def declared_arguments(launch_file: str) -> List[DeclareLaunchArgument]:
    """The arguments a launch file declares itself, loaded with the same package stubs.

    Not get_launch_arguments(): it also returns those of every file it includes, and
    their switches are the business of those files.
    """
    with _session(None):
        description = get_launch_description_from_python_launch_file(launch_file)
        return [e for e in description.entities if isinstance(e, DeclareLaunchArgument)]


def dry_run(launch_file: str, launch_arguments: Dict[str, str],
            known_parameters: Optional[Set[str]] = None) -> DryRunResult:
    """Run launch_file with launch_arguments; see the module docstring."""
    with _session(known_parameters) as session:
        errors = _ErrorCollector()
        # Launch loggers do not propagate: listen on each one that reports something.
        loggers = [logging.getLogger('launch'), logging.getLogger('launch_ros.actions.node')]
        for logger in loggers:
            logger.addHandler(errors)
        try:
            service = LaunchService(noninteractive=True)
            service.include_launch_description(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file),
                launch_arguments=list(launch_arguments.items())))
            return_code = service.run()
        finally:
            for logger in loggers:
                logger.removeHandler(errors)
        for message in errors.messages:
            session.problem(message)
        if return_code != 0 and not errors.messages:
            session.problem(f'launch returned {return_code}')
        return session.result


@functools.lru_cache(maxsize=None)
def known_rtabmap_parameters() -> frozenset:
    """All RTAB-Map parameters of the installed library.

    The SLAM node leaves the odometry ones out of its --params listing, and the
    odometry node the SLAM ones, so it takes both.
    """
    names = set()
    for package, executable in (('rtabmap_slam', 'rtabmap'), ('rtabmap_odom', 'rgbd_odometry')):
        path = Path(get_package_prefix(package)) / 'lib' / package / executable
        output = subprocess.run([str(path), '--params'], capture_output=True, text=True,
                                timeout=60, check=True).stdout
        for line in output.splitlines():
            if line.startswith('Param: '):
                names.add(line[len('Param: '):].split(' = ', 1)[0].strip())
    if len(names) < 300:
        raise RuntimeError(f'could not read the parameter list ({len(names)} found)')
    return frozenset(names)


def variants(launch_file: str):
    """The default arguments, then each boolean flipped and each other choice, one at a time."""
    yield {}
    context = LaunchContext()
    for argument in declared_arguments(launch_file):
        if argument.choices:
            default = None
            if argument.default_value is not None:
                default = perform_substitutions(context, argument.default_value)
            for choice in argument.choices:
                if choice != default:
                    yield {argument.name: choice}
            continue
        if argument.default_value is None:
            continue
        try:
            default = perform_substitutions(context, argument.default_value)
        except Exception:
            continue  # default made of other arguments; not a switch
        if default.lower() in ('true', 'false'):
            yield {argument.name: 'false' if default.lower() == 'true' else 'true'}


def launch_files(source_dir: Path) -> List[str]:
    """The launch files under a package's launch directory, relative to it."""
    return sorted(str(p.relative_to(source_dir)) for p in source_dir.rglob('*.launch.py'))


def check_launch_file(package: str, launch_file: str) -> Tuple[List[str], Set[str]]:
    """Dry-run the installed share/<package>/launch/<launch_file> with each variant.

    Returns the failures, one per variant that has problems, and the packages that had
    to be stubbed.
    """
    installed = Path(get_package_share_directory(package)) / 'launch' / launch_file
    if not installed.is_file():
        return [f'{launch_file} is not installed'], set()
    failures = []
    stubbed = set()
    for arguments in variants(str(installed)):
        result = dry_run(str(installed), arguments, known_rtabmap_parameters())
        stubbed |= result.stubbed_packages
        problems = list(result.problems)
        # Every launch file here is there to run some part of RTAB-Map. A condition or
        # an include gone wrong can drop it without any error.
        if not arguments and not any(_is_rtabmap_package(s.package) for s in result.started):
            problems.append('starts nothing from an rtabmap package')
        if problems:
            shown = ' '.join(f'{k}:={v}' for k, v in arguments.items()) or '(defaults)'
            failures.append(f'{shown}\n    ' + '\n    '.join(problems))
    return failures, stubbed
