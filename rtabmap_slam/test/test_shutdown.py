"""
Ctrl-C on `ros2 launch` leaves rtabmap's database complete.

Ctrl-C sends SIGINT to the whole process group, and `ros2 launch` then sends its own
SIGINT to each node: rtabmap gets two. It saves what it kept in memory only -- the
optimized graph, the links added to nodes already in the database -- when it closes the
database, so it has to survive the second SIGINT until then.
"""

import os
import signal
import sqlite3
import subprocess
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
from rtabmap_msgs.msg import Info
from tf2_msgs.msg import TFMessage

LAUNCH_FILE = Path(__file__).parent / 'rtabmap_odom_only.launch.py'
NODES = 5


def _odometry(stamp: float, x: float):
    """odom -> base_link at @p x, as TF and as an odometry message, the way the C++
    tests' sendOdom() sends them."""
    tf = TransformStamped()
    tf.header.frame_id = 'odom'
    tf.header.stamp.sec = int(stamp)
    tf.child_frame_id = 'base_link'
    tf.transform.translation.x = x
    tf.transform.rotation.w = 1.0
    odom = Odometry()
    odom.header = tf.header
    odom.child_frame_id = 'base_link'
    odom.pose.pose.position.x = x
    odom.pose.pose.orientation.w = 1.0
    odom.pose.covariance = [0.001 if i % 7 == 0 else 0.0 for i in range(36)]
    odom.twist.covariance = odom.pose.covariance
    return TFMessage(transforms=[tf]), odom


def _spin_until(node, done, timeout: float) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if done():
            return True
        rclpy.spin_once(node, timeout_sec=0.05)
    return done()


def _map(node, launch):
    """Drives NODES odometry updates into rtabmap, waiting for each to be processed."""
    tf_pub = node.create_publisher(TFMessage, '/tf', QoSProfile(depth=100))
    odom_pub = node.create_publisher(Odometry, 'odom', 10)
    info = []
    node.create_subscription(Info, 'info', info.append, 10)
    started = lambda: (launch.poll() is not None or (  # noqa: E731
        odom_pub.get_subscription_count() > 0 and tf_pub.get_subscription_count() > 0
        and node.count_publishers('info') > 0))
    assert _spin_until(node, started, 60.0) and launch.poll() is None, 'rtabmap did not start'
    for i in range(NODES):
        tf, odom = _odometry(1.0 + i, 0.5 * i)
        tf_pub.publish(tf)
        odom_pub.publish(odom)
        assert _spin_until(node, lambda: len(info) > i, 10.0), f'update {i} was not processed'


def test_ctrl_c_saves_the_database(tmp_path):
    database = tmp_path / 'rtabmap.db'
    log_path = tmp_path / 'launch.log'
    with open(log_path, 'w') as log:
        launch = subprocess.Popen(
            ['ros2', 'launch', str(LAUNCH_FILE), f'database_path:={database}',
             f'working_directory:={tmp_path}'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    try:
        node = rclpy.create_node('rtabmap_slam_test_shutdown')
        try:
            _map(node, launch)
        finally:
            node.destroy_node()
        os.killpg(launch.pid, signal.SIGINT)  # Ctrl-C
        launch.wait(timeout=60)
    finally:
        rclpy.try_shutdown()
        if launch.poll() is None:
            os.killpg(launch.pid, signal.SIGKILL)
            launch.wait()
    log = log_path.read_text()

    assert 'process has finished cleanly' in log, log
    with sqlite3.connect(database) as db:
        optimized = db.execute('SELECT opt_poses FROM Admin').fetchone()[0]
        # Each neighbor link is in both directions; new -> old goes in with the new node,
        # old -> new is added to a node already saved, and is written at closing.
        forward, backward = (
            db.execute(f'SELECT COUNT(*) FROM Link WHERE type = 0 AND from_id {op} to_id')
            .fetchone()[0] for op in ('<', '>'))
    assert optimized, 'no optimized graph saved'
    assert (forward, backward) == (NODES - 1, NODES - 1)
