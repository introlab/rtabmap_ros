"""
Dry-run every launch file of a package, with its default arguments and with each of its
switches flipped, and check what it would start. See launch_dry_run.py, next to this file.

Installed with rtabmap_launch, and run by each package whose launch files it checks,
which names itself in RTABMAP_LAUNCH_TEST_PACKAGE (see their CMakeLists.txt): the
installed launch files of that package are the ones checked.

A launch file is only checked as far as this machine can resolve it: the drivers or
simulators that are not installed are stubbed, and the test says which.
"""

import os
import warnings
from pathlib import Path

import pytest
from ament_index_python.packages import get_package_share_directory

from launch_dry_run import check_launch_file, declared_arguments, dry_run, launch_files

PACKAGE = os.environ.get('RTABMAP_LAUNCH_TEST_PACKAGE', 'rtabmap_launch')
LAUNCH_DIR = Path(get_package_share_directory(PACKAGE)) / 'launch'
LAUNCH_FILES = launch_files(LAUNCH_DIR)


@pytest.mark.parametrize('launch_file', LAUNCH_FILES)
def test_launch_file(launch_file):
    failures, stubbed = check_launch_file(PACKAGE, launch_file)
    if stubbed:
        warnings.warn(f'{launch_file}: not installed, stubbed: {", ".join(sorted(stubbed))}')
    assert not failures, f'{launch_file}:\n  ' + '\n  '.join(failures)


def _with_ground_truth_arguments(launch_file: str) -> bool:
    names = {a.name for a in declared_arguments(str(LAUNCH_DIR / launch_file))}
    return {'frame_id', 'ground_truth_frame_id', 'ground_truth_base_frame_id'} <= names


GROUND_TRUTH_LAUNCH_FILES = [f for f in LAUNCH_FILES if _with_ground_truth_arguments(f)]


# Where a launch file declares frame_id and the ground truth's frames, the robot frame of
# the ground truth defaults to frame_id + "_gt", as in the nodes; set empty, it reaches the
# nodes empty, which turns the ground truth off. Defined only where there is one: an empty
# parametrization would be reported as skipped.
if GROUND_TRUTH_LAUNCH_FILES:
    @pytest.mark.parametrize('launch_file', GROUND_TRUTH_LAUNCH_FILES)
    @pytest.mark.parametrize('arguments, expected', [
        ({}, 'base_footprint_gt'),
        ({'ground_truth_base_frame_id': 'tracker'}, 'tracker'),
        ({'ground_truth_base_frame_id': ''}, ''),
    ], ids=['default', 'set', 'empty'])
    def test_ground_truth_base_frame_id(launch_file, arguments, expected):
        arguments = dict(arguments, frame_id='base_footprint', ground_truth_frame_id='world')
        result = dry_run(str(LAUNCH_DIR / launch_file), arguments)
        assert not result.problems, '\n'.join(result.problems)
        given = {s.describe(): s.parameters.get('ground_truth_base_frame_id')
                 for s in result.started if 'ground_truth_frame_id' in s.parameters}
        assert given, f'{launch_file} starts no node with a ground truth'
        for node, value in given.items():
            assert value == expected, \
                f'{node}: ground_truth_base_frame_id={value!r}, expected {expected!r}'
