#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TransformStamped
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

# Ground truth of the EuRoC MAV MH sequences: the Leica laser tracker's position of the MAV's
# prism (/leica/position), published as a TF frame with the orientation of the MAV's IMU (from
# imu_complementary_filter without magnetometer, on /imu0), so that the prism -> IMU lever arm
# can follow the MAV's rotation. The Leica gives no orientation: the IMU's is aligned in yaw to
# the Leica's frame, and the Leica's stamps (late by 0.13 to 0.22 s depending on the sequence)
# are corrected. Both are estimated online from the accelerations seen by the two sensors (the
# Leica position's second derivative against the IMU's):
#   * time offset: vertical accelerations (the IMU's |acceleration|, independent of its yaw),
#     over the windows with enough vertical motion, accumulated over the whole run;
#   * yaw: horizontal accelerations (the IMU's rotated by its orientation), over the windows
#     with enough horizontal motion, with older windows forgotten (the IMU's yaw drifts).
#     Within each window, the orientation is integrated from the gyro, from the IMU's
#     orientation at the window's middle: the orientation of a filter correcting its tilt with
#     the accelerometer is off while the body accelerates, which biases the yaw.
# Nothing is published until both are estimated (the yaw from the buffered data once the time
# offset is): the body should be moved up and down, and horizontally, at the start (e.g.,
# EuRoC MAV's MH sequences).


def savgol_kernel(n, order, deriv, dt):
    """Savitzky-Golay filter: convolving a signal with it fits a polynomial of that order over n
    samples around each one, and gives that polynomial's value (deriv 0: smoothing) or derivative
    (deriv 2: acceleration from positions) at the sample."""
    x = np.arange(n) - n // 2
    coeffs = np.linalg.pinv(np.vander(x, order + 1, increasing=True))[deriv]
    return coeffs[::-1] * math.factorial(deriv) / dt ** deriv


def smooth(signal, kernel):
    """Convolve each column of signal (samples x axes) with kernel (e.g., savgol_kernel()). The
    output has the input's length: its first and last half kernel are not valid (see pad)."""
    return np.apply_along_axis(lambda s: np.convolve(s, kernel, mode='same'), 0, signal)


# Quaternions as x, y, z, w (ROS's order), on arrays of them (last axis)
def qmul(a, b):
    """Product a * b: the rotation b, then a."""
    ax, ay, az, aw = np.moveaxis(a, -1, 0)
    bx, by, bz, bw = np.moveaxis(b, -1, 0)
    return np.stack([aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw,
                     aw * bw - ax * bx - ay * by - az * bz], -1)


def qrotate(q, v):
    """Vector(s) v rotated by q."""
    u = q[..., :3]
    t = 2.0 * np.cross(u, v)
    return v + q[..., 3:4] * t + np.cross(u, t)


def qexp(rotvec):
    """The quaternion of a rotation vector (axis * angle), e.g., gyro rate * dt."""
    angle = np.linalg.norm(rotvec, axis=-1, keepdims=True)
    axis = rotvec / np.maximum(angle, 1e-12)
    return np.concatenate([axis * np.sin(angle / 2), np.cos(angle / 2)], -1)


def qinterp(t, stamps, quats):
    """The orientations at times t, interpolated (normalized linear) between the stamped ones."""
    i = np.clip(np.searchsorted(stamps, t), 1, len(stamps) - 1)
    a, b = quats[i - 1], quats[i]
    b = np.where(np.sum(a * b, -1, keepdims=True) < 0, -b, b)
    w = np.clip((t - stamps[i - 1]) / np.maximum(stamps[i] - stamps[i - 1], 1e-9), 0, 1)[..., None]
    q = (1 - w) * a + w * b
    return q / np.linalg.norm(q, axis=-1, keepdims=True)


class PointImuAligner:
    """Online time offset and yaw between a tracked point and an IMU, from accelerations."""

    def __init__(self, yaw_forgetting=30.0):
        self.dt = 0.01                                     # s, grid the two sensors are resampled on
        self.offsets = np.arange(-0.4, 0.1 + 1e-9, 0.005)  # s, time offsets tried (EuRoC's: -0.13 to -0.22)
        self.block = 3.0                                   # s, windows the data are processed by
        self.min_samples = 50                              # usable grid samples for a window to count
        self.pad = 1.0                                     # s, data around a window, for the smoothing
        self.gap = 0.2                                     # s, point measurements further apart: a gap, not used with this margin
        self.min_vertical_blocks = 3                       # windows with vertical motion before the offset is estimated
        self.min_excitation = 0.3                          # m/s^2, point's acceleration std for a window to count
        self.min_coherence = 0.5                           # horizontal accelerations' correlation for a window's yaw to count
        self.yaw_forgetting = yaw_forgetting               # s, time constant older windows' yaw fades with
        self.buffer = 120.0                                # s, data kept (the yaw is redone over half once the offset is known)
        self.k_smooth = savgol_kernel(int(0.5 / self.dt) | 1, 2, 0, self.dt)  # IMU's accelerations, smoothed over 0.5 s
        self.k_acc = savgol_kernel(int(0.5 / self.dt) | 1, 2, 2, self.dt)     # point's acceleration, the same way
        self.imu_t, self.imu_f, self.imu_w, self.imu_q = [], [], [], []  # stamps, accelerations, gyro, orientations
        self.pt_t, self.pt_p = [], []                      # point's stamps and positions
        self.next_block = None                             # start of the next window to process
        self.sums = np.zeros((len(self.offsets), 6))       # per offset tried: n, sx, sy, sxx, syy, sxy (correlation)
        self.vertical_blocks = 0                           # windows with vertical motion so far
        self.offset = None                                 # s, estimated time offset (added to the point's stamps)
        self.yaw_sum = 0j                                  # horizontal accelerations' correlation, its angle the yaw
        self.yaw = None                                    # rad, estimated yaw of the IMU's world in the point's frame

    def add_imu(self, t, acc, gyro, quat):
        self.imu_t.append(t); self.imu_f.append(acc); self.imu_w.append(gyro); self.imu_q.append(quat)

    def add_point(self, t, p):
        self.pt_t.append(t); self.pt_p.append(p)
        if self.next_block is None:
            self.next_block = t + self.pad - self.offsets[0]

    def _trim(self):
        now = self.pt_t[-1]
        for t, *lists in ((self.imu_t, self.imu_f, self.imu_w, self.imu_q), (self.pt_t, self.pt_p)):
            k = np.searchsorted(t, now - self.buffer)
            if k > 0:
                del t[:k]
                for values in lists:
                    del values[:k]

    def _grid(self, b0):
        tg = np.arange(b0 - self.pad, b0 + self.block + self.pad, self.dt)
        core = (tg >= b0) & (tg < b0 + self.block)
        tl = np.asarray(self.pt_t); pl = np.asarray(self.pt_p)
        ok = np.ones(len(tg), bool)
        for i in np.where(np.diff(tl) > self.gap)[0]:
            ok &= ~((tg > tl[i] - self.gap) & (tg < tl[i + 1] + self.gap))
        ti = np.asarray(self.imu_t)
        f = np.stack([np.interp(tg, ti, np.asarray(self.imu_f)[:, k]) for k in range(3)], 1)
        return tg, core, ok & core, tl, pl, ti, f

    def _point_acc(self, tg, tl, pl, offset, kernel, axes):
        return smooth(np.stack([np.interp(tg, tl + offset, pl[:, k]) for k in axes], 1), kernel)

    def _gyro_orientation(self, tg):
        """The orientation over tg integrated from the gyro, from the IMU's at tg's middle."""
        ti = np.asarray(self.imu_t)
        j0 = max(int(np.searchsorted(ti, tg[0])) - 1, 0)
        j1 = min(int(np.searchsorted(ti, tg[-1])) + 1, len(ti))
        jm = min(max(int(np.searchsorted(ti, tg[len(tg) // 2])), j0), j1 - 1)
        w = np.asarray(self.imu_w[j0:j1]); dts = np.diff(ti[j0:j1])
        q = np.zeros((j1 - j0, 4)); q[jm - j0] = self.imu_q[jm]
        for j in range(jm - j0, j1 - j0 - 1):
            q[j + 1] = qmul(q[j], qexp(w[j] * dts[j]))
        for j in range(jm - j0, 0, -1):
            q[j - 1] = qmul(q[j], qexp(-w[j - 1] * dts[j - 1]))
        return qinterp(tg, ti[j0:j1], q)

    def _vertical(self, b0):
        tg, core, sel, tl, pl, ti, f = self._grid(b0)
        if sel.sum() < self.min_samples:
            return
        z0 = self._point_acc(tg, tl, pl, 0.0, self.k_acc, [2])[sel, 0]
        if z0.std() < self.min_excitation:
            return
        y = smooth(np.linalg.norm(f, axis=1), self.k_smooth)[sel]
        for j, offset in enumerate(self.offsets):
            x = self._point_acc(tg, tl, pl, offset, self.k_acc, [2])[sel, 0]
            self.sums[j] += [len(x), x.sum(), y.sum(), (x * x).sum(), (y * y).sum(), (x * y).sum()]
        self.vertical_blocks += 1
        if self.vertical_blocks >= self.min_vertical_blocks:
            n, sx, sy, sxx, syy, sxy = self.sums.T
            corr = (sxy - sx * sy / n) / np.sqrt((sxx - sx * sx / n) * (syy - sy * sy / n))
            first = self.offset is None
            self.offset = float(self.offsets[int(np.argmax(corr))])
            return first
        return False

    def _horizontal(self, b0):
        tg, core, sel, tl, pl, ti, f = self._grid(b0)
        if sel.sum() < self.min_samples:
            return
        a = self._point_acc(tg, tl, pl, self.offset, self.k_acc, [0, 1])[sel]
        zl = a[:, 0] + 1j * a[:, 1]
        zl -= zl.mean()
        if np.sqrt(np.mean(np.abs(zl) ** 2)) < self.min_excitation:
            return
        fw = smooth(qrotate(self._gyro_orientation(tg), f), self.k_smooth)[sel]
        zi = fw[:, 0] + 1j * fw[:, 1]
        zi -= zi.mean()
        z = np.sum(zl * np.conj(zi))
        if abs(z) < self.min_coherence * np.sqrt(np.sum(np.abs(zl) ** 2) * np.sum(np.abs(zi) ** 2)):
            return
        self.yaw_sum = self.yaw_sum * math.exp(-self.block / self.yaw_forgetting) + z
        self.yaw = float(np.angle(self.yaw_sum))

    def update(self):
        """Process the blocks the buffered data now cover. Returns True if the estimates changed."""
        if self.next_block is None or not self.imu_t:
            return False
        changed = False
        while (self.pt_t[-1] > self.next_block + self.block + self.pad - self.offsets[0] + 0.5 and
               self.imu_t[-1] > self.next_block + self.block + self.pad):
            b0 = self.next_block
            self.next_block += self.block
            before = (self.offset, self.yaw)
            if self._vertical(b0):
                # the yaw from the buffered blocks, now that the time offset is estimated
                for past in np.arange(b0 - self.block * int(self.buffer / self.block / 2), b0, self.block):
                    if past - self.pad > max(self.pt_t[0], self.imu_t[0]):
                        self._horizontal(past)
            if self.offset is not None:
                self._horizontal(b0)
            changed |= before != (self.offset, self.yaw)
        self._trim()
        return changed

    def ready(self):
        return self.offset is not None and self.yaw is not None

    def orientation(self, t):
        """The IMU's orientation in the point's fixed frame at t (the IMU's time)."""
        q = qinterp(np.array([t]), np.asarray(self.imu_t), np.asarray(self.imu_q))[0]
        return qmul(np.array([0, 0, math.sin(self.yaw / 2), math.cos(self.yaw / 2)]), q)


class EurocLeicaGroundTruth(Node):

    def __init__(self):
        super().__init__('euroc_leica_ground_truth')

        # The prism's frame (with the IMU's orientation), and the Leica's frame
        self.declare_parameter('frame_id', 'leica')
        self.declare_parameter('fixed_frame_id', 'world')
        # s, time constant the yaw's older estimates fade with (the IMU's yaw drifts)
        self.declare_parameter('yaw_forgetting', 30.0)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.fixed_frame_id = self.get_parameter('fixed_frame_id').get_parameter_value().string_value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.aligner = PointImuAligner(
            yaw_forgetting=self.get_parameter('yaw_forgetting').get_parameter_value().double_value)
        self.get_logger().info('Waiting for vertical and horizontal motion to estimate the time '
                               'offset and yaw between the Leica and the IMU...')

        # Deep enough to keep all the messages published in a burst (e.g., replayed between two
        # camera frames)
        self.imu_subscription = self.create_subscription(Imu, 'imu', self.imu_callback, 1000)
        self.subscription = self.create_subscription(PointStamped, 'point', self.callback, 100)

    @staticmethod
    def seconds(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def imu_callback(self, imu):
        a, w, q = imu.linear_acceleration, imu.angular_velocity, imu.orientation
        self.aligner.add_imu(self.seconds(imu.header.stamp), [a.x, a.y, a.z], [w.x, w.y, w.z],
                             [q.x, q.y, q.z, q.w])

    def callback(self, point):
        stamp = self.seconds(point.header.stamp)
        was_ready, offset = self.aligner.ready(), self.aligner.offset
        self.aligner.add_point(stamp, [point.point.x, point.point.y, point.point.z])
        if self.aligner.update():
            message = 'time offset %.3f s, yaw %.1f deg' % (
                self.aligner.offset if self.aligner.offset is not None else float('nan'),
                math.degrees(self.aligner.yaw) if self.aligner.yaw is not None else float('nan'))
            if was_ready != self.aligner.ready() or offset != self.aligner.offset:
                self.get_logger().info(message)
            else:
                self.get_logger().debug(message)
        # No ground truth until both are estimated
        if not self.aligner.ready() or stamp + self.aligner.offset > self.aligner.imu_t[-1]:
            return
        stamp += self.aligner.offset

        t = TransformStamped()
        t.header.frame_id = self.fixed_frame_id
        t.header.stamp.sec = int(math.floor(stamp))
        t.header.stamp.nanosec = int((stamp - math.floor(stamp)) * 1e9)
        t.child_frame_id = self.frame_id
        t.transform.translation.x = point.point.x
        t.transform.translation.y = point.point.y
        t.transform.translation.z = point.point.z
        q = self.aligner.orientation(stamp)
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = EurocLeicaGroundTruth()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
