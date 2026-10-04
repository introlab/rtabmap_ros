"""
Dry-run every demo launch file, with its default arguments and with each of its
switches flipped, and check what it would start. The dry run is rtabmap_examples' (its
test/launch_dry_run.py, installed with it), which documents what is checked.

A demo is only checked as far as this machine can resolve it: the packages of a
simulator that is not installed are stubbed, and the test says which.
"""

import sys
import warnings
from pathlib import Path

import pytest
from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, str(Path(get_package_share_directory('rtabmap_examples')) / 'test'))
from launch_dry_run import check_launch_file, launch_files  # noqa: E402

LAUNCH_FILES = launch_files(Path(__file__).resolve().parent.parent / 'launch')


@pytest.mark.parametrize('launch_file', LAUNCH_FILES)
def test_launch_file(launch_file):
    failures, stubbed = check_launch_file('rtabmap_demos', launch_file)
    if stubbed:
        warnings.warn(f'{launch_file}: not installed, stubbed: {", ".join(sorted(stubbed))}')
    assert not failures, f'{launch_file}:\n  ' + '\n  '.join(failures)
