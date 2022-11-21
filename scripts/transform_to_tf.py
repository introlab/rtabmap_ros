#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class TransformToTf(Node):

    def __init__(self):
        super().__init__('transform_to_tf')
        
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('child_frame_id', 'transform')
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value

        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            TransformStamped,
            'transform',
            self.callback,
            1)
        self.subscription  # prevent unused variable warning

    def callback(self, t):
        if not t.header.frame_id:
            t.header.frame_id = self.frame_id
        if not t.child_frame_id:
            t.child_frame_id = self.child_frame_id

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    transform_to_tf = TransformToTf()
    rclpy.spin(transform_to_tf)
    transform_to_tf.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
   
