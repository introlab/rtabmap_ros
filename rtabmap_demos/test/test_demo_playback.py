"""
Replay a demo's bag through the demo's own launch file and compare the resulting
graph with a golden one.

The golden trajectory is replayed in TF beside the bag as ground truth, so rtabmap
computes the error of its map against it itself, as it would for any ground truth
(Gt/* statistics, in /info). The loop closures are counted from the graph.

The bag is played in lockstep with the pipeline (see bag_lockstep.py), so the result
does not depend on how loaded the machine is. Skipped when the bag has not been
fetched (test/fetch_test_data.sh): the bags are several GB.

Environment:
  RTABMAP_DEMOS_TEST_DATA     where the bags are (default: test/data)
  RTABMAP_DEMOS_TEST_RESULTS  where each run's graph, database and launch log are kept
                              (default: a pytest temporary directory)
  RTABMAP_DEMOS_UPDATE_GOLDEN set to 1 to write the run's graph as the new golden one
                              instead of comparing with it
"""

import ctypes
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List

import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import SingleThreadedExecutor
from rtabmap_msgs.msg import Info

from bag_lockstep import LockstepPlayer, ProcessTree
from graph_metrics import Graph, export_graph, load_tum

TEST_DIR = Path(__file__).resolve().parent
DATA_DIR = Path(os.environ.get('RTABMAP_DEMOS_TEST_DATA', TEST_DIR / 'data'))
GOLDEN_DIR = TEST_DIR / 'golden'
# The golden trajectory's frames in TF; unconnected to the robot's own tree.
GROUND_TRUTH_FRAME = 'golden_map'
GROUND_TRUTH_BASE_FRAME = 'golden_base'


@dataclass
class Scenario:
    name: str
    launch_file: str
    bag: str
    # Topics the pipeline must have subscribed to before the replay can start.
    required_topics: List[str]
    launch_arguments: Dict[str, str] = field(default_factory=dict)
    # ROS parameters set on every node the launch file starts (see _wrapper_launch()), for
    # settings the launch file has no argument for. A node's own value for one wins.
    parameters: Dict[str, object] = field(default_factory=dict)
    # How far a run may be from the golden graph.
    max_node_difference: float = 0.02      # relative
    # Loop closures, for each kind, may be fewer than the golden graph's by this ratio
    # of them, or by 2, whichever is more: from one run to the next, a closure more or
    # less is noise, however few there are.
    closure_slack: float = 0.2
    max_rmse: float = 0.05                 # m, Gt/Translational_rmse
    max_rotational_rmse: float = 1.0       # deg, Gt/Rotational_rmse

    def __str__(self):
        return self.name


SCENARIOS = [
    Scenario(
        name='robot_mapping',
        launch_file='robot_mapping_demo.launch.py',
        bag='demo_mapping_bag',
        required_topics=['/jn0/base_scan', '/data_throttled_image/compressed'],
        launch_arguments={'rtabmap_viz': 'false', 'rviz': 'false'},
        # Every image rgbd_sync makes reaches rtabmap: with a history of 1, the next one
        # replaces one rtabmap has not received yet when it is busy.
        parameters={'output_queue_size': 10},
        # How many frames rtabmap merges into the previous node (rehearsal) depends on
        # its build (its optional dependencies): 205 nodes in CI against 210 here. Loop
        # closures vary from run to run (11 to 13 global ones), and so the errors:
        # 0.046-0.053 m and up to 1.22 deg.
        max_node_difference=0.05,
        max_rmse=0.1,
        max_rotational_rmse=2.0),
    Scenario(
        name='stereo_outdoorA',
        launch_file='stereo_outdoor_demo.launch.py',
        bag='stereo_outdoorA_bag',
        required_topics=['/stereo_camera/left/image_raw_throttle/compressed',
                         '/stereo_camera/right/image_raw_throttle/compressed'],
        launch_arguments={'rtabmap_viz': 'false', 'rviz': 'false'},
        # As for netherdrone_lidar3d below: every pair stereo_sync makes reaches odometry
        # and is registered (all 3741 of them on an idle machine).
        parameters={'output_queue_size': 10,
                    'always_process_most_recent_frame': False},
        # As for robot_mapping: 161 nodes in CI against 185 here.
        max_node_difference=0.15,
        max_rmse=0.1,
        max_rotational_rmse=3.0),
    Scenario(
        name='netherdrone_lidar3d',
        launch_file='netherdrone_lidar3d_demo.launch.py',
        bag='netherdrone_ouster_vertige_bag_0',
        required_topics=['/os_cloud_node/points', '/imu/data_raw',
                         '/camera/image_raw/compressed', '/camera/camera_info'],
        # A node is half a turn of the lidar's mast, about 4.4 s apart; the odometry
        # poses in between are saved too (intermediate nodes), and compared as well.
        launch_arguments={'rtabmap_viz': 'false', 'rviz': 'false', 'intermediate_nodes': 'true'},
        # Every scan reaches odometry, and is registered. By default odometry drops those
        # arriving while it is busy (as when lidar_deskewing, waiting for a transform
        # asleep, looks idle to the lockstep player), and lidar_deskewing replaces a scan
        # odometry has not received yet with the next one: how many depends on the
        # machine. output_queue_size also reaches rgb_sync, for the camera. topic_queue_size
        # reaches only icp_odometry: rgb_sync and point_cloud_assembler already default
        # to 10, and rtabmap sets its own.
        parameters={'output_queue_size': 10,
                    'topic_queue_size': 10,
                    'always_process_most_recent_frame': False},
        max_rmse=0.1,
        max_rotational_rmse=2.0,
        max_node_difference=0.05),
]


def _wrapper_launch(scenario: Scenario, arguments: Dict[str, str], path: Path) -> Path:
    """Write a launch file that sets scenario.parameters on every node, then includes the
    demo's with `arguments`: launch_ros' SetParameter applies to the nodes launched after
    it, whatever launch file they come from."""
    demo = Path(get_package_share_directory('rtabmap_demos')) / 'launch' / scenario.launch_file
    set_parameters = ''.join(f'        SetParameter(name={name!r}, value={value!r}),\n'
                             for name, value in scenario.parameters.items())
    path.write_text(
        'from launch import LaunchDescription\n'
        'from launch.actions import IncludeLaunchDescription\n'
        'from launch.launch_description_sources import PythonLaunchDescriptionSource\n'
        'from launch_ros.actions import SetParameter\n'
        '\n'
        '\n'
        'def generate_launch_description():\n'
        '    return LaunchDescription([\n'
        f'{set_parameters}'
        f'        IncludeLaunchDescription(PythonLaunchDescriptionSource({str(demo)!r}),\n'
        f'                                 launch_arguments={list(arguments.items())!r}),\n'
        '    ])\n')
    return path


def _stop(process: subprocess.Popen) -> bool:
    """Stop `ros2 launch` the way Ctrl-C does, so rtabmap closes its database.

    Returns False if it had to be killed: rtabmap then did not close its database.
    """
    if process.poll() is not None:
        return True
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=60)
        return True
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()
        return False


# See bag_lockstep.py: publish() must have sent the message when it returns, in this
# process and in the demo's nodes alike. Read when each process initializes its RMW.
os.environ['RMW_FASTRTPS_PUBLICATION_MODE'] = 'SYNCHRONOUS'


def _die_with_parent():
    """In the launch process before exec: SIGINT it if the test process dies.

    The launch runs in a session of its own (so it can be stopped as a group); without
    this, a test killed by a timeout would leave the demo running.
    """
    PR_SET_PDEATHSIG = 1
    ctypes.CDLL('libc.so.6', use_errno=True).prctl(PR_SET_PDEATHSIG, signal.SIGINT)


def _replay(scenario: Scenario, bag: Path, results: Path, ground_truth=None):
    """Replay the bag; returns the statistics of rtabmap's last update.

    rtabmap is stopped when this returns, its database closed with the optimized graph.
    """
    arguments = dict(scenario.launch_arguments, database_path=str(results / 'rtabmap.db'))
    if ground_truth:
        arguments.update(ground_truth_frame_id=GROUND_TRUTH_FRAME,
                         ground_truth_base_frame_id=GROUND_TRUTH_BASE_FRAME)
    log = open(results / 'launch.log', 'w')
    launch = subprocess.Popen(
        ['ros2', 'launch', str(_wrapper_launch(scenario, arguments, results / 'demo.launch.py'))],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
        preexec_fn=_die_with_parent)
    context = rclpy.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('demo_playback_test', context=context)
    stats = {}

    def on_info(msg):
        stats.clear()
        stats.update(zip(msg.stats_keys, msg.stats_values))
    node.create_subscription(Info, '/info', on_info, 10)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        tree = ProcessTree(launch.pid)
        player = LockstepPlayer(node, str(bag), tree)
        if ground_truth:
            player.add_trajectory(GROUND_TRUTH_FRAME, GROUND_TRUTH_BASE_FRAME, ground_truth)
        player.connect(scenario.required_topics)
        nodes = {pid: Path(f'/proc/{pid}/comm').read_text().strip() for pid in tree.pids()}
        print(f'\n{scenario.name}: {len(nodes)} processes ({", ".join(nodes.values())}), '
              f'gating {", ".join(sorted(player.gated))}', flush=True)

        start = time.monotonic()
        last_report = [0.0]

        def progress(done, total):
            # A node that died leaves the others idle forever: the replay would go on
            # into nothing and only fail at the end. Stop at once instead.
            if done % 100 == 0:
                gone = [name for pid, name in nodes.items() if not Path(f'/proc/{pid}').exists()]
                if gone:
                    raise RuntimeError(f'{", ".join(gone)} exited during the replay '
                                       f'(see {results / "launch.log"})')
            now = time.monotonic()
            if now - last_report[0] > 30 or done == total:
                last_report[0] = now
                print(f'  {done}/{total} messages, {now - start:.0f} s', flush=True)

        player.play(progress=progress)
        print(f'  replayed in {time.monotonic() - start:.0f} s', flush=True)
        result = dict(stats)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
        spinner.join(timeout=10)
        stopped = _stop(launch)
        log.close()
    if not stopped:
        raise RuntimeError(f'{scenario.launch_file} did not stop within 60 s and was killed: '
                           f'rtabmap did not close its database (see {results / "launch.log"})')
    return result


@pytest.mark.parametrize('scenario', SCENARIOS, ids=str)
def test_demo_playback(scenario: Scenario, tmp_path):
    bag = DATA_DIR / scenario.bag
    if not bag.is_dir():
        pytest.skip(f'{bag} not found; fetch it with {TEST_DIR / "fetch_test_data.sh"}')

    results = Path(os.environ.get('RTABMAP_DEMOS_TEST_RESULTS', tmp_path)) / scenario.name
    results.mkdir(parents=True, exist_ok=True)
    golden_prefix = GOLDEN_DIR / scenario.name
    update = os.environ.get('RTABMAP_DEMOS_UPDATE_GOLDEN') == '1'
    if not update and not Path(str(golden_prefix) + '.g2o').is_file():
        pytest.fail(f'no golden graph {golden_prefix}.g2o; '
                    'run once with RTABMAP_DEMOS_UPDATE_GOLDEN=1 to create it')

    ground_truth = None if update else load_tum(str(golden_prefix) + '.tum')
    stats = _replay(scenario, bag, results, ground_truth)
    prefix = results / scenario.name
    export_graph(results / 'rtabmap.db', prefix)
    summary = Graph.load(str(prefix) + '.g2o').summary()
    print(f'  {summary}; graph exported to {prefix}.g2o', flush=True)

    if update:
        GOLDEN_DIR.mkdir(exist_ok=True)
        for suffix in ('.g2o', '.tum'):
            shutil.copyfile(str(prefix) + suffix, str(golden_prefix) + suffix)
        print(f'  golden graph updated: {golden_prefix}.g2o', flush=True)
        return

    expected = Graph.load(str(golden_prefix) + '.g2o').summary()
    rmse = stats.get('Gt/Translational_rmse/m')
    rotational_rmse = stats.get('Gt/Rotational_rmse/deg')

    problems = []
    if abs(summary['nodes'] - expected['nodes']) > scenario.max_node_difference * expected['nodes']:
        problems.append(f'nodes: {summary["nodes"]}, golden {expected["nodes"]}')
    for kind in ('global_closures', 'local_closures'):
        minimum = expected[kind] - max(2, scenario.closure_slack * expected[kind])
        if summary[kind] < minimum:
            problems.append(f'{kind}: {summary[kind]}, golden {expected[kind]} '
                            f'(at least {minimum:.0f} expected)')
    if rmse is None:
        problems.append('rtabmap reported no Gt/* statistics: the golden trajectory did not '
                        f'reach it as ground truth (see {results / "launch.log"})')
    else:
        if rmse > scenario.max_rmse:
            problems.append(f'Gt/Translational_rmse: {rmse:.3f} m (max {scenario.max_rmse} m)')
        if rotational_rmse > scenario.max_rotational_rmse:
            problems.append(f'Gt/Rotational_rmse: {rotational_rmse:.2f} deg '
                            f'(max {scenario.max_rotational_rmse} deg)')
        print(f'  vs golden {expected}: RMSE {rmse:.3f} m, {rotational_rmse:.2f} deg',
              flush=True)
    assert not problems, f'{scenario.name}:\n  ' + '\n  '.join(problems)


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-s', *sys.argv[1:]]))
