"""
Dry-run every example launch file, with its default arguments and with each of its
switches flipped, and check what it would start. The dry run is rtabmap_demos' (its
test/launch_dry_run.py, installed with it), which documents what is checked.

An example is only checked as far as this machine can resolve it: the drivers of a
camera or lidar that are not installed are stubbed, and the test says which.
"""

import sys
import warnings
from pathlib import Path

import pytest
from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, str(Path(get_package_share_directory('rtabmap_demos')) / 'test'))
from launch_dry_run import check_launch_file, launch_files  # noqa: E402

LAUNCH_FILES = launch_files(Path(__file__).resolve().parent.parent / 'launch')


@pytest.mark.parametrize('launch_file', LAUNCH_FILES)
def test_launch_file(launch_file):
    failures, stubbed = check_launch_file('rtabmap_examples', launch_file)
    if stubbed:
        warnings.warn(f'{launch_file}: not installed, stubbed: {", ".join(sorted(stubbed))}')
    assert not failures, f'{launch_file}:\n  ' + '\n  '.join(failures)
