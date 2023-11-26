#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from tf2_msgs.msg import TFMessage

class StaticTransformRepublisher(Node):

    def __init__(self):
        super().__init__('static_transform_republisher')
        qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                )
        self.publisher_ = self.create_publisher(TFMessage, '/tf_static', qos)
        self.data = TFMessage()
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf_static_old',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        if len(self.data.transforms) == 0:
            self.data = msg
        else:
            self.data.transforms = self.data.transforms + msg.transforms
        self.get_logger().info('"Received /tf_static_old and republising latched /tf_static"')
        self.publisher_.publish(self.data)

def main(args=None):
    rclpy.init(args=args)
    static_transform_republisher = StaticTransformRepublisher()
    rclpy.spin(static_transform_republisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
