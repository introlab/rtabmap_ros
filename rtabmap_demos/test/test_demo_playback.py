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
  RTABMAP_DEMOS_REPLAY        lockstep (default): each sensor message once the pipeline is
                              idle; chunked: at the bag's pace, 10 s at a time, waiting
                              for rtabmap to catch up after each
  RTABMAP_DEMOS_RATE          with chunked: how many times the bag's pace (default 1)
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
from typing import Dict, List, Union

import pytest
import yaml
import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.executors import SingleThreadedExecutor
from rtabmap_msgs.msg import Info

from bag_lockstep import LockstepPlayer, ProcessTree
from graph_metrics import Graph, export_graph, load_tum

TEST_DIR = Path(__file__).resolve().parent
DATA_DIR = Path(os.environ.get('RTABMAP_DEMOS_TEST_DATA', TEST_DIR / 'data'))
# lockstep (default, see bag_lockstep.py) or chunked (LockstepPlayer.play_chunked()).
REPLAY = os.environ.get('RTABMAP_DEMOS_REPLAY', 'lockstep')
# chunked only: how many times the bag's own pace.
REPLAY_RATE = float(os.environ.get('RTABMAP_DEMOS_RATE', '1.0'))
GOLDEN_DIR = TEST_DIR / 'golden'
# The golden trajectory's frames in TF; unconnected to the robot's own tree.
GROUND_TRUTH_FRAME = 'golden_map'
GROUND_TRUTH_BASE_FRAME = 'golden_base'


@dataclass
class Scenario:
    name: str
    launch_file: str
    bag: Union[str, List[str]]  # several: played one after the other
    # Topics the pipeline must have subscribed to before the replay can start.
    required_topics: List[str]
    launch_arguments: Dict[str, str] = field(default_factory=dict)
    # ROS parameters set on every node the launch file starts (see _wrapper_launch()), for
    # settings the launch file has no argument for. A node's own value for one wins.
    parameters: Dict[str, object] = field(default_factory=dict)
    # ROS parameters for one node, by node name, for a parameter name other nodes use too.
    node_parameters: Dict[str, Dict[str, object]] = field(default_factory=dict)
    # Packages the launch file needs besides ours: the scenario is skipped without them
    # (not every ROS distro has them).
    packages: List[str] = field(default_factory=list)
    # With RTABMAP_DEMOS_REPLAY=chunked: how far rtabmap's last output may lag the last
    # sensor message published, as it processes the data (seconds).
    chunk_slack: float = 2.0
    # How far a run may be from the golden graph.
    max_node_difference: float = 0.02      # relative
    # Loop closures, global and local together, may be fewer than the golden graph's by
    # this ratio of them, or by 2, whichever is more: from one run to the next, a closure
    # more or less is noise, however few there are. Together, as a place revisited can be
    # closed by either: in stereo_outdoor, 13 global and 9 local closures in one run, 15
    # and 5 in another.
    closure_slack: float = 0.2
    # False when the scenario turns loop closure detection off, against a golden graph
    # made with it.
    compare_closures: bool = True
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
        name='stereo_outdoor',
        launch_file='stereo_outdoor_demo.launch.py',
        # Both parts of the run, played one after the other.
        bag=['stereo_outdoorA_bag', 'stereo_outdoorB_bag'],
        required_topics=['/stereo_camera/left/image_raw_throttle/compressed',
                         '/stereo_camera/right/image_raw_throttle/compressed'],
        launch_arguments={'rtabmap_viz': 'false', 'rviz': 'false'},
        # As for netherdrone_lidar3d below: every pair stereo_sync makes reaches odometry
        # and is registered (all 3741 of them on an idle machine).
        parameters={'output_queue_size': 10,
                    'always_process_most_recent_frame': False},
        # As for robot_mapping: with bag A alone, 161 nodes in CI against 185 here, and
        # up to 3.06 deg.
        max_node_difference=0.15,
        max_rmse=0.1,
        max_rotational_rmse=4.0),
    Scenario(
        name='netherdrone_lidar3d',
        launch_file='netherdrone_lidar3d_demo.launch.py',
        # The whole flight, its six parts played one after the other.
        bag=[f'netherdrone_ouster_vertige_bag_{i}' for i in range(6)],
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
        # A node every 4.3 s, when the assembled cloud of half a turn of the mast is ready.
        chunk_slack=5.0,
        # Local closures only (no global ones: the camera's bag-of-words finds none),
        # 25 in the golden graph, 19 in a Kilted CI run.
        closure_slack=0.3,
        max_rmse=0.1,
        max_rotational_rmse=2.0,
        max_node_difference=0.05),
    Scenario(
        name='find_object',
        launch_file='find_object_demo.launch.py',
        bag='demo_find_object_bag',
        required_topics=['/base_scan', '/camera/data_throttled_image/compressed'],
        launch_arguments={'rtabmap_viz': 'false', 'rviz': 'false', 'find_object_gui': 'false'},
        # As for robot_mapping, with the same pipeline (find_object_2d besides it).
        parameters={'output_queue_size': 10},
        # Without loop closures (neither from the images nor by proximity), only odometry
        # and the objects as landmarks constrain the map: how far it stays from the golden
        # one, made with all of them, is what the landmarks are worth. 0.124 m and 1.06
        # deg with the landmarks, against 0.708 m without them.
        node_parameters={'rtabmap': {'Kp/MaxFeatures': '-1',
                                     'RGBD/ProximityBySpace': 'false'}},
        compare_closures=False,
        packages=['find_object_2d'],
        max_node_difference=0.05,
        max_rmse=0.2,
        max_rotational_rmse=2.0),
]


# Every QoS parameter of rtabmap's nodes: reliable (1). Their default, the system's, is best
# effort for a subscriber with Fast DDS, which drops what arrives while the node is busy:
# the lockstep player cannot see that (see bag_lockstep.py).
RELIABLE_QOS = {name: 1 for name in (
    'qos', 'qos_camera_info', 'qos_env_sensor', 'qos_global_pose', 'qos_gps', 'qos_image',
    'qos_imu', 'qos_odom', 'qos_pub', 'qos_scan', 'qos_scan_cloud', 'qos_sensor_data',
    'qos_sub', 'qos_user_data')}


def _wrapper_launch(scenario: Scenario, rtabmap_parameters: Dict[str, object],
                    arguments: Dict[str, str], path: Path) -> Path:
    """Write a launch file that sets parameters on the demo's nodes, then includes the
    demo's launch file with `arguments`.

    The parameters go in a parameter file next to it, which launch_ros'
    SetParametersFromFile gives to every node launched after it, whatever launch file they
    come from: each node reads its own section. RELIABLE_QOS and scenario.parameters are
    for every node (/**), and `rtabmap_parameters` for the node named rtabmap only (/**/rtabmap): the
    ground truth frames, which the odometry nodes also read. A node's own value for a
    parameter, set by its launch file, wins over the file's.
    """
    demo = Path(get_package_share_directory('rtabmap_demos')) / 'launch' / scenario.launch_file
    parameter_file = path.with_suffix('.yaml')
    # The nodes' own sections first: after /**, rcl would not apply /** to the other nodes.
    sections = {f'/**/{node}': {'ros__parameters': dict(values)}
                for node, values in scenario.node_parameters.items()}
    rtabmap_section = sections.setdefault('/**/rtabmap', {'ros__parameters': {}})
    rtabmap_section['ros__parameters'].update(rtabmap_parameters)
    sections['/**'] = {'ros__parameters': dict(RELIABLE_QOS, **scenario.parameters)}
    parameter_file.write_text(yaml.safe_dump(sections, sort_keys=False))
    path.write_text(
        'from launch import LaunchDescription\n'
        'from launch.actions import IncludeLaunchDescription\n'
        'from launch.launch_description_sources import PythonLaunchDescriptionSource\n'
        'from launch_ros.actions import SetParametersFromFile\n'
        '\n'
        '\n'
        'def generate_launch_description():\n'
        '    return LaunchDescription([\n'
        f'        SetParametersFromFile({str(parameter_file)!r}),\n'
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


def _replay(scenario: Scenario, bag: List[Path], results: Path, ground_truth=None):
    """Replay the bag; returns the statistics of rtabmap's last update.

    rtabmap is stopped when this returns, its database closed with the optimized graph.
    """
    # The database and the ground truth are for rtabmap only: the demos have no launch
    # argument for them, they are set from the parameter file (see _wrapper_launch()).
    rtabmap_parameters = {'database_path': str(results / 'rtabmap.db')}
    if ground_truth:
        rtabmap_parameters.update(ground_truth_frame_id=GROUND_TRUTH_FRAME,
                                  ground_truth_base_frame_id=GROUND_TRUTH_BASE_FRAME)
    arguments = dict(scenario.launch_arguments)
    log = open(results / 'launch.log', 'w')
    launch = subprocess.Popen(
        ['ros2', 'launch', str(_wrapper_launch(scenario, rtabmap_parameters, arguments,
                                               results / 'demo.launch.py'))],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
        preexec_fn=_die_with_parent)
    context = rclpy.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('demo_playback_test', context=context)
    stats = {}
    info_stamp = [0.0]

    def on_info(msg):
        stats.clear()
        stats.update(zip(msg.stats_keys, msg.stats_values))
        info_stamp[0] = max(info_stamp[0], msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
    node.create_subscription(Info, '/info', on_info, 10)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        tree = ProcessTree(launch.pid)
        player = LockstepPlayer(node, [str(b) for b in bag], tree)
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

        if REPLAY == 'chunked':
            player.play_chunked(lambda: info_stamp[0], slack=scenario.chunk_slack,
                                rate=REPLAY_RATE, progress=progress)
        else:
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
    for package in scenario.packages:
        try:
            get_package_share_directory(package)
        except PackageNotFoundError:
            pytest.skip(f'{package} is not installed')
    bag = [DATA_DIR / name for name in
           ([scenario.bag] if isinstance(scenario.bag, str) else scenario.bag)]
    for b in bag:
        if not b.is_dir():
            pytest.skip(f'{b} not found; fetch it with {TEST_DIR / "fetch_test_data.sh"}')

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
    if scenario.compare_closures:
        closures, golden_closures = (
            g['global_closures'] + g['local_closures'] for g in (summary, expected))
        minimum = golden_closures - max(2, scenario.closure_slack * golden_closures)
        if closures < minimum:
            problems.append(f'loop closures (global and local): {closures}, golden '
                            f'{golden_closures} (at least {minimum:.0f} expected)')
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
