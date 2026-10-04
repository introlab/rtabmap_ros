"""
Dry-run every demo launch file, with its default arguments and with each of its
switches flipped, and check what it would start. See launch_dry_run.py.

A demo is only checked as far as this machine can resolve it: the packages of a
simulator that is not installed are stubbed, and the test says which.
"""

import subprocess
import warnings
from pathlib import Path

import pytest
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchContext
from launch.utilities import perform_substitutions

from launch_dry_run import _is_rtabmap_package, declared_arguments, dry_run

LAUNCH_DIR = Path(__file__).resolve().parent.parent / 'launch'
LAUNCH_FILES = sorted(str(p.relative_to(LAUNCH_DIR)) for p in LAUNCH_DIR.rglob('*.launch.py'))


@pytest.fixture(scope='session')
def known_parameters():
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
    assert len(names) > 300, f'could not read the parameter list ({len(names)} found)'
    return names


def variants(launch_file):
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


@pytest.mark.parametrize('launch_file', LAUNCH_FILES)
def test_launch_file(launch_file, known_parameters):
    installed = Path(get_package_share_directory('rtabmap_demos')) / 'launch' / launch_file
    assert installed.is_file(), f'{launch_file} is not installed'

    failures = []
    stubbed = set()
    for arguments in variants(str(installed)):
        result = dry_run(str(installed), arguments, known_parameters)
        stubbed |= result.stubbed_packages
        problems = list(result.problems)
        # Every demo is there to run some part of RTAB-Map. A condition or an include
        # gone wrong can drop it without any error.
        if not arguments and not any(_is_rtabmap_package(s.package) for s in result.started):
            problems.append('starts nothing from an rtabmap package')
        if problems:
            shown = ' '.join(f'{k}:={v}' for k, v in arguments.items()) or '(defaults)'
            failures.append(f'{shown}\n    ' + '\n    '.join(problems))

    if stubbed:
        warnings.warn(f'{launch_file}: not installed, stubbed: {", ".join(sorted(stubbed))}')
    assert not failures, f'{launch_file}:\n  ' + '\n  '.join(failures)
