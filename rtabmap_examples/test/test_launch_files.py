"""
Dry-run every example launch file, with its default arguments and with each of its
switches flipped, and check what it would start. See launch_dry_run.py.

An example is only checked as far as this machine can resolve it: the drivers of a
camera or lidar that are not installed are stubbed, and the test says which.
"""

import warnings
from pathlib import Path

import pytest

from launch_dry_run import check_launch_file, launch_files

LAUNCH_FILES = launch_files(Path(__file__).resolve().parent.parent / 'launch')


@pytest.mark.parametrize('launch_file', LAUNCH_FILES)
def test_launch_file(launch_file):
    failures, stubbed = check_launch_file('rtabmap_examples', launch_file)
    if stubbed:
        warnings.warn(f'{launch_file}: not installed, stubbed: {", ".join(sorted(stubbed))}')
    assert not failures, f'{launch_file}:\n  ' + '\n  '.join(failures)
