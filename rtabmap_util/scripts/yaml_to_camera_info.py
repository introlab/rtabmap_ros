#!/usr/bin/env python
import rospy
import yaml
import sys
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

def yaml_to_CameraInfo(yaml_fname):
    with open(yaml_fname, "r") as file_handle:
        first_line = file_handle.readline()
        if "%YAML:" not in first_line:
            file_handle.seek(0)
        calib_data = yaml.load(file_handle, Loader=yaml.FullLoader)

    camera_info_msg = CameraInfo()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg

def callback(image):
    global publisher
    global camera_info_msg
    global frameId
    camera_info_msg.header = image.header
    if frameId:
        camera_info_msg.header.frame_id = frameId
    publisher.publish(camera_info_msg)

if __name__ == "__main__":

    rospy.init_node("yaml_to_camera_info", anonymous=True)

    yaml_path = rospy.get_param('~yaml_path', '')
    scale = rospy.get_param('~scale', 1.0)
    if not yaml_path:
        print('yaml_path parameter should be set to path of the calibration file!')
        sys.exit(1)

    frameId = rospy.get_param('~frame_id', '')
    camera_info_msg = yaml_to_CameraInfo(yaml_path)

    if scale!=1.0:
        camera_info_msg.K[0] = camera_info_msg.K[0]*scale
        camera_info_msg.K[2] = camera_info_msg.K[2]*scale
        camera_info_msg.K[4] = camera_info_msg.K[4]*scale
        camera_info_msg.K[5] = camera_info_msg.K[5]*scale
        camera_info_msg.P[0] = camera_info_msg.P[0]*scale
        camera_info_msg.P[2] = camera_info_msg.P[2]*scale
        camera_info_msg.P[3] = camera_info_msg.P[3]*scale
        camera_info_msg.P[5] = camera_info_msg.P[5]*scale
        camera_info_msg.P[6] = camera_info_msg.P[6]*scale
        camera_info_msg.width = int(camera_info_msg.width*scale)
        camera_info_msg.height = int(camera_info_msg.height*scale)

    publisher = rospy.Publisher("camera_info", CameraInfo, queue_size=1)
    rospy.Subscriber("image", Image, callback, queue_size=1)
    rospy.spin()
