#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_ros import TransformBroadcaster

class PointToTf(Node):

    def __init__(self):
        super().__init__('point_to_tf')

        # The point's frame (if its header doesn't give one), and the frame it is in
        self.declare_parameter('frame_id', 'point')
        self.declare_parameter('fixed_frame_id', 'world')
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.fixed_frame_id = self.get_parameter('fixed_frame_id').get_parameter_value().string_value

        self.tf_broadcaster = TransformBroadcaster(self)

        # Deep enough to keep all the points published in a burst (e.g., a laser tracker's
        # positions replayed between two camera frames)
        self.subscription = self.create_subscription(
            PointStamped,
            'point',
            self.callback,
            100)
        self.subscription  # prevent unused variable warning

    def callback(self, point):
        t = TransformStamped()
        t.header.stamp = point.header.stamp
        t.header.frame_id = self.fixed_frame_id
        t.child_frame_id = point.header.frame_id if point.header.frame_id else self.frame_id
        t.transform.translation.x = point.point.x
        t.transform.translation.y = point.point.y
        t.transform.translation.z = point.point.z
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    point_to_tf = PointToTf()
    rclpy.spin(point_to_tf)
    point_to_tf.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
