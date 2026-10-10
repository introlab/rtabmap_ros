#!/usr/bin/env python3
"""
Republish find_object_2d's detections as landmark detections for rtabmap.

find_object_2d publishes which objects it detected on `objectsStamped`, with their
homography in the image only; their 3D pose, when the depth allows it, goes on TF only,
as a frame <object_prefix>_<id> relative to the camera. For each detected object, this
node looks that frame up at the detection's stamp and publishes it on
`landmark_detections` (rtabmap_msgs/LandmarkDetections), with the object's id as the
landmark's id. An object detected twice in the same image is used once.

Covariance: rtabmap's Marker/* parameters apply only to the markers it detects itself,
a landmark detection is used with the covariance it carries. This node sets it from the
same parameters, as rtabmap does for its own markers, so that both can be given the same
values:
  Marker/VarianceLinear              linear variance (m^2), or with the orientation
                                     ignored and GTSAM, the range variance (9999: bearing
                                     only)
  Marker/VarianceAngular             angular variance (rad^2), or with the orientation
                                     ignored and GTSAM, the bearing variance
  Marker/VarianceOrientationIgnored  ignore the object's orientation, only its position
                                     (g2o) or its range and bearing (GTSAM) constrain the
                                     map
  Optimizer/Strategy                 rtabmap's optimizer: 2 is GTSAM. Needed with the
                                     orientation ignored, set it to rtabmap's.
  Reg/Force3DoF                      rtabmap's, for the 2D range and bearing layout
"""

import signal

import rclpy
from find_object_2d.msg import ObjectsStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rtabmap_msgs.msg import LandmarkDetection, LandmarkDetections
from tf2_ros import Buffer, TransformException, TransformListener

# objectsStamped's objects.data: for each object, its id, width, height and 3x3 homography.
VALUES_PER_OBJECT = 12
IGNORED = 9999.0  # a variance this large disables that part of the constraint


def _bool(value: str) -> bool:
    return str(value).strip().lower() == 'true'


class FindObjectToLandmarks(Node):

    def __init__(self):
        super().__init__('find_object_to_landmarks')
        self.object_prefix = self.declare_parameter('object_prefix', 'object').value
        self.wait_for_transform = self.declare_parameter('wait_for_transform', 0.2).value
        # As rtabmap's parameters: strings.
        linear = float(self.declare_parameter('Marker/VarianceLinear', '0.001').value)
        angular = float(self.declare_parameter('Marker/VarianceAngular', '0.01').value)
        orientation_ignored = _bool(
            self.declare_parameter('Marker/VarianceOrientationIgnored', 'false').value)
        strategy = self.declare_parameter('Optimizer/Strategy', '').value
        force_3dof = _bool(self.declare_parameter('Reg/Force3DoF', 'false').value)
        if orientation_ignored and not strategy:
            self.get_logger().warn(
                'Marker/VarianceOrientationIgnored is true but Optimizer/Strategy is not set: '
                'assuming it is not GTSAM. Set it to rtabmap\'s.')
        self.covariance = self._covariance(
            linear, angular, orientation_ignored, strategy == '2', force_3dof)
        self.get_logger().info(
            f'object_prefix={self.object_prefix}, covariance diagonal='
            f'{[self.covariance[i * 7] for i in range(6)]}')

        self.tf_buffer = Buffer()
        # Its own thread: on_objects() waits for the object's frame.
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.publisher = self.create_publisher(LandmarkDetections, 'landmark_detections', 10)
        self.create_subscription(ObjectsStamped, 'objectsStamped', self.on_objects, 10)

    @staticmethod
    def _covariance(linear, angular, orientation_ignored, gtsam, force_3dof):
        """The 6x6 covariance rtabmap gives its own markers (see Memory.cpp), row-major."""
        diagonal = [linear] * 3 + [angular] * 3
        if orientation_ignored:
            diagonal = [linear] * 3 + [IGNORED] * 3
            if gtsam and force_3dof:
                # 2D bearing and range: x is the bearing, y the range (see OptimizerGTSAM).
                diagonal[0:3] = [angular, linear, 1.0]
            elif gtsam:
                # 3D bearing and range: x and y are the bearing, z the range.
                diagonal[0:3] = [angular, angular, linear]
        covariance = [0.0] * 36
        for i, value in enumerate(diagonal):
            covariance[i * 7] = value
        return covariance

    def on_objects(self, msg: ObjectsStamped):
        detections = LandmarkDetections()
        detections.header = msg.header
        data = msg.objects.data
        used = set()
        for i in range(0, len(data) - VALUES_PER_OBJECT + 1, VALUES_PER_OBJECT):
            object_id = int(data[i])
            if object_id <= 0 or object_id in used:
                continue
            used.add(object_id)
            frame = f'{self.object_prefix}_{object_id}'
            try:
                transform = self.tf_buffer.lookup_transform(
                    msg.header.frame_id, frame, Time.from_msg(msg.header.stamp),
                    Duration(seconds=self.wait_for_transform))
            except TransformException as e:
                # find_object_2d publishes no frame for an object without valid depth.
                self.get_logger().debug(f'object {object_id} skipped: {e}')
                continue
            landmark = LandmarkDetection()
            landmark.header = msg.header
            landmark.landmark_frame_id = frame
            landmark.id = object_id
            t, r = transform.transform.translation, transform.transform.rotation
            pose = landmark.pose.pose
            pose.position.x, pose.position.y, pose.position.z = t.x, t.y, t.z
            pose.orientation = r
            landmark.pose.covariance = self.covariance
            detections.landmarks.append(landmark)
        if detections.landmarks:
            self.publisher.publish(detections)


def main():
    rclpy.init()
    node = FindObjectToLandmarks()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # A second Ctrl-C (one from the terminal, one from ros2 launch) would interrupt
        # the cleanup.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        # Its thread before rclpy: torn down after, it raises at exit.
        node.tf_listener.unregister()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
