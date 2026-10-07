"""
Play a rosbag2 bag in lockstep with the nodes consuming it.

`ros2 bag play` publishes at the bag's pace whatever the consumers do; on a loaded
machine a node still busy with the previous frame drops the next one, and which
frames it drops changes from run to run. Here a sensor message is only published
once every process of the pipeline is idle again, so on a slower machine the replay
just takes longer, and each node sees every message it would on an idle one.

This needs every publisher of the pipeline, this player's included, to send the
message before publish() returns (RMW_FASTRTPS_PUBLICATION_MODE=SYNCHRONOUS with Fast
DDS, which otherwise sends large messages from a thread of its own; Cyclone DDS always
does). Else, right after a publish(), nothing has woken up yet and all looks idle.

"Idle" is read from the kernel, not from the nodes: a thread with work to do is
runnable (state R) -- also while it waits for a CPU, which is what makes this hold
under load -- and a node waiting for its next message has every thread sleeping. A
node passing a message on wakes the next one before going back to sleep, so a
pipeline is idle only once its last stage is done. That needs no knowledge of what
each node publishes, or of which frames the SLAM node keeps (those it skips because
of Rtabmap/DetectionRate publish nothing at all).

TF is not gated. It is published ahead of the sensor data by a lookahead, so a
transform interpolated at a frame's stamp -- or at the stamp of a lidar sweep's last
point, for deskewing -- is already in the listener's buffer when the frame arrives.

Transforms not in the bag can be added to its TF (add_trajectory), each sent a lead
time before its stamp: a ground truth trajectory for rtabmap, for instance.
"""

import heapq
import os
import time
from typing import Callable, Dict, List, Optional, Set

import numpy as np
import rosbag2_py
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation, Slerp
from tf2_msgs.msg import TFMessage

# Topics published without waiting: TF listeners keep a long queue and a buffer.
UNGATED_TOPICS = ('/tf', '/tf_static')


def _read_stat(path: str):
    with open(path) as f:
        stat = f.read()
    # The command name is in parentheses and may itself hold spaces or parentheses.
    name = stat[stat.index('(') + 1:stat.rindex(')')]
    return name, stat[stat.rindex(')') + 2:].split()


class ProcessTree:
    """The processes started under a root process (the `ros2 launch` one)."""

    def __init__(self, root_pid: int):
        self.root_pid = root_pid
        self._pids: List[int] = []
        self._refreshed = 0.0

    def pids(self) -> List[int]:
        # Nodes start in the first seconds and component containers may spawn later;
        # walking /proc is the expensive part, so it is not done on every poll.
        if time.monotonic() - self._refreshed > 1.0:
            children: Dict[int, List[int]] = {}
            for entry in os.listdir('/proc'):
                if entry.isdigit():
                    try:
                        _, fields = _read_stat(f'/proc/{entry}/stat')
                    except (OSError, ValueError):
                        continue
                    children.setdefault(int(fields[1]), []).append(int(entry))
            pids, todo = [], [self.root_pid]
            while todo:
                for child in children.get(todo.pop(), []):
                    pids.append(child)
                    todo.append(child)
            self._pids = pids
            self._refreshed = time.monotonic()
        return self._pids

    def busy_threads(self) -> List[str]:
        """Threads of the tree's processes (not of the root) that have work to do."""
        busy = []
        for pid in self.pids():
            try:
                tids = os.listdir(f'/proc/{pid}/task')
            except OSError:
                continue
            for tid in tids:
                try:
                    name, fields = _read_stat(f'/proc/{pid}/task/{tid}/stat')
                except (OSError, ValueError):
                    continue
                # R: running or runnable. D: in an uninterruptible wait, writing the
                # database for instance -- busy as well. S, I, T, Z: nothing to do.
                if fields[0] in ('R', 'D'):
                    busy.append(f'{pid}/{name}:{fields[0]}')
        return busy

    def wait_idle(self, idle_polls: int = 3, period: float = 0.002, timeout: float = 300.0):
        """Wait until idle_polls polls in a row see no busy thread.

        A single quiet poll is not enough: timers wake threads briefly, and a message
        in flight between two nodes leaves a gap of a few microseconds where both
        sleep. Raises TimeoutError, naming the busy threads, if never idle.
        """
        deadline = time.monotonic() + timeout
        quiet = 0
        busy: List[str] = []
        while quiet < idle_polls:
            if time.monotonic() > deadline:
                raise TimeoutError(f'still busy after {timeout:.0f} s: {", ".join(busy)}')
            busy = self.busy_threads()
            quiet = 0 if busy else quiet + 1
            time.sleep(period)


class LockstepPlayer:
    """Publish a bag's messages, each sensor message once the pipeline is idle."""

    def __init__(self, node: Node, bag_dir: str, tree: ProcessTree, lookahead: float = 0.2):
        self.node = node
        self.bag_dir = bag_dir
        self.tree = tree
        self.lookahead = lookahead
        self.gated: Set[str] = set()
        self._publishers = {}
        self._clock = node.create_publisher(Clock, '/clock', 10)
        self._extra = []  # (release time, serialized TFMessage)
        self._extra_tf = node.create_publisher(
            TFMessage, '/tf', QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST,
                                         reliability=ReliabilityPolicy.RELIABLE))

        reader = self._open()
        for meta in reader.get_all_topics_and_types():
            name = meta.name if meta.name.startswith('/') else '/' + meta.name
            if name == '/tf_static':
                qos = QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST,
                                 reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            else:
                qos = QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self._publishers[meta.name] = (
                name, node.create_publisher(get_message(meta.type), name, qos))

    def add_trajectory(self, frame_id: str, child_frame_id: str, trajectory,
                       step: float = 0.25):
        """Publish (stamp, (x y z qx qy qz qw)) poses as frame_id -> child_frame_id.

        The first and last poses are held to the bag's start and end, so that no lookup
        falls outside of it: one would make the node wait for the transform, asleep,
        looking idle. Poses are then added, interpolated as TF would (linearly, and by
        slerp for the rotation), so that none are more than `step` seconds apart: TF
        returns the same from them at any stamp.

        TF interpolates between two samples, so a lookup at a stamp needs the sample after
        it: each is sent 2 * step ahead of its stamp. A node may still look a stamp up
        late (rtabmap, an intermediate node when the next node with data arrives) and a
        TF buffer keeps 10 s: the trajectory's own gaps can be several seconds (a robot
        standing still adds no nodes), and a lead covering them would leave it no slack.
        """
        metadata = self._open().get_metadata()
        start = metadata.starting_time.nanoseconds / 1e9
        end = start + metadata.duration.nanoseconds / 1e9
        trajectory = sorted(trajectory)
        if not trajectory:
            return
        trajectory = [(start, trajectory[0][1])] + trajectory + [(end, trajectory[-1][1])]
        stamps = np.array([stamp for stamp, _ in trajectory])
        poses = np.array([pose for _, pose in trajectory])
        stamps, unique = np.unique(stamps, return_index=True)
        poses = poses[unique]
        dense = np.union1d(stamps, np.arange(stamps[0], stamps[-1], step))
        xyz = np.stack([np.interp(dense, stamps, poses[:, k]) for k in range(3)], axis=1)
        quaternions = Slerp(stamps, Rotation.from_quat(poses[:, 3:]))(dense).as_quat()
        lead = 2 * step
        for stamp, (x, y, z), (qx, qy, qz, qw) in zip(dense, xyz, quaternions):
            ns = int(round(stamp * 1e9))
            msg = TransformStamped()
            msg.header.stamp = _to_time(ns)
            msg.header.frame_id = frame_id
            msg.child_frame_id = child_frame_id
            t, r = msg.transform.translation, msg.transform.rotation
            t.x, t.y, t.z, r.x, r.y, r.z, r.w = x, y, z, qx, qy, qz, qw
            self._extra.append(
                (ns - int(lead * 1e9), serialize_message(TFMessage(transforms=[msg]))))

    def _open(self):
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=self.bag_dir, storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('cdr', 'cdr'))
        return reader

    def connect(self, required: List[str], timeout: float = 60.0):
        """Wait for the pipeline to subscribe, then choose which topics to gate.

        A reliable publisher still loses what it publishes before discovery has
        matched it with a subscriber. Only topics something subscribes to are gated:
        waiting after a message nobody reads would only slow the replay down.
        """
        deadline = time.monotonic() + timeout
        by_name = {name: pub for name, pub in self._publishers.values()}
        for topic in required:
            while by_name[topic].get_subscription_count() == 0:
                if time.monotonic() > deadline:
                    raise TimeoutError(f'nothing subscribed to {topic} after {timeout:.0f} s')
                time.sleep(0.1)
        # Subscribers to the other topics are created by the same nodes, in the same
        # constructors; leave discovery a moment to catch up with them too.
        time.sleep(2.0)

        def wait_matched(name, publisher):
            while publisher.get_subscription_count() < self.node.count_subscribers(name):
                if time.monotonic() > deadline:
                    # Usually a subscriber asking for a QoS this publisher cannot offer.
                    subscribers = '\n'.join(
                        f'  {info.node_namespace.rstrip("/")}/{info.node_name}: '
                        f'{info.qos_profile.reliability.name}, '
                        f'{info.qos_profile.durability.name}, '
                        f'{info.qos_profile.history.name} {info.qos_profile.depth}'
                        for info in self.node.get_subscriptions_info_by_topic(name))
                    raise TimeoutError(
                        f'{name} matched with {publisher.get_subscription_count()} of its '
                        f'{self.node.count_subscribers(name)} subscribers:\n{subscribers}')
                time.sleep(0.1)

        # Discovery may not have found every subscriber yet, and what is published before
        # a subscriber is matched never reaches it: the first transforms of the bag, say,
        # which the first frames need. Wait until the subscribers this node knows of, and
        # those each publisher is matched with, have not changed for a while.
        def counts():
            return tuple((self.node.count_subscribers(name), publisher.get_subscription_count())
                         for name, publisher in list(self._publishers.values()) +
                         [('/tf', self._extra_tf)])

        stable_since, last = time.monotonic(), counts()
        while time.monotonic() - stable_since < 2.0:
            if time.monotonic() > deadline:
                raise TimeoutError('discovery did not settle')
            time.sleep(0.1)
            current = counts()
            if current != last:
                stable_since, last = time.monotonic(), current

        for bag_topic, (name, publisher) in self._publishers.items():
            if self.node.count_subscribers(name) > 0:
                # TF too: the first frames' transforms would otherwise be lost, or not,
                # depending on when discovery got there.
                wait_matched(name, publisher)
                if name not in UNGATED_TOPICS:
                    self.gated.add(bag_topic)
        wait_matched('/tf', self._extra_tf)

    def play(self, progress: Optional[Callable[[int, int], None]] = None,
             idle_timeout: float = 300.0) -> int:
        """Publish the whole bag; returns the number of gated messages published."""
        reader = self._open()
        pending = []  # (release time, sequence, topic, data, bag time)
        sequence = 0
        for release_ns, data in self._extra:
            heapq.heappush(pending, (release_ns, sequence, None, data, release_ns))
            sequence += 1
        clock_ns = 0
        published = 0
        gated_total = self._count_gated()

        def release(until_ns):
            nonlocal clock_ns, published
            while pending and pending[0][0] <= until_ns:
                _, _, topic, data, t = heapq.heappop(pending)
                if topic is None:
                    self._extra_tf.publish(data)
                    continue
                _, publisher = self._publishers[topic]
                if topic in self.gated:
                    # Also lets the TF listeners take in the TF released so far, before
                    # the frame needing it arrives: a node waiting for a transform
                    # sleeps, which would look idle.
                    self.tree.wait_idle(timeout=idle_timeout)
                    if t > clock_ns:
                        clock_ns = t
                        self._clock.publish(Clock(clock=_to_time(clock_ns)))
                    publisher.publish(data)
                    published += 1
                    if progress:
                        progress(published, gated_total)
                else:
                    publisher.publish(data)

        lookahead_ns = int(self.lookahead * 1e9)
        while reader.has_next():
            topic, data, t = reader.read_next()
            # Sensor data is held back by the lookahead, so the TF of the next
            # lookahead seconds goes out before it.
            delay = lookahead_ns if topic in self.gated else 0
            heapq.heappush(pending, (t + delay, sequence, topic, data, t))
            sequence += 1
            release(t)
        release(float('inf'))
        self.tree.wait_idle(idle_polls=50, timeout=idle_timeout)
        return published

    def _count_gated(self) -> int:
        metadata = self._open().get_metadata()
        return sum(t.message_count for t in metadata.topics_with_message_count
                   if t.topic_metadata.name in self.gated)


def _to_time(ns: int):
    from builtin_interfaces.msg import Time
    return Time(sec=ns // 1000000000, nanosec=ns % 1000000000)
