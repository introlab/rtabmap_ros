#!/usr/bin/env python3
import rclpy
import yaml
import sys
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

def yaml_to_CameraInfo(yaml_fname):
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
 
    msg = CameraInfo()
    msg.width = calib_data["image_width"]
    msg.height = calib_data["image_height"]
    msg.k = calib_data["camera_matrix"]["data"]
    msg.d = calib_data["distortion_coefficients"]["data"]
    msg.r = calib_data["rectification_matrix"]["data"]
    msg.p = calib_data["projection_matrix"]["data"]
    msg.distortion_model = calib_data["distortion_model"]
    return msg

class YamlToCameraInfo(Node):

    def __init__(self):
        super().__init__('yaml_to_camera_info')
        
        self.declare_parameter('yaml_path', '')
        yaml_path = self.get_parameter('yaml_path').get_parameter_value().string_value

        if not yaml_path:
            print('yaml_path parameter should be set to path of the calibration file!')
            sys.exit(1)

        self.declare_parameter('frame_id', '')
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        self.camera_info_msg = yaml_to_CameraInfo(yaml_path)

        self.publisher_ = self.create_publisher(CameraInfo, 'camera_info', 1)
        self.subscription = self.create_subscription(
            Image,
            'image',
            self.callback,
            1)
        self.subscription  # prevent unused variable warning

    def callback(self, image):
        self.camera_info_msg.header = image.header
        if self.frame_id:
            self.camera_info_msg.header.frame_id = self.frame_id
        self.publisher_.publish(self.camera_info_msg)


def main(args=None):
    rclpy.init(args=args)
    yaml_to_camera_info = YamlToCameraInfo()
    rclpy.spin(yaml_to_camera_info)
    yaml_to_camera_info.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
