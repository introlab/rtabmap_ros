#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CameraInfo

def callback(data):
    P = list(data.P);
    P[3] = P[3] * scale
    data.P = tuple(P);
    pub.publish(data)

if __name__ == '__main__':
    rospy.init_node('republish_camera_info', anonymous=True)
    pub = rospy.Publisher('camera_info_out', CameraInfo, queue_size=1)
    scale = rospy.get_param('~scale', 1.091664)
    rospy.Subscriber("camera_info_in", CameraInfo, callback)
    rospy.spin()
