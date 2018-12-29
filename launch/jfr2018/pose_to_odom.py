#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

def callback(data):
    odom = Odometry()
    odom.header = data.header
    odom.child_frame_id = child_frame_id
    odom.pose = data.pose
    pub.publish(odom)

if __name__ == '__main__':
    rospy.init_node('pose_to_odom', anonymous=True)
    pub = rospy.Publisher('odom_combined', Odometry, queue_size=1)
    child_frame_id = rospy.get_param('~child_frame_id', "base_footprint")
    rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, callback)
    rospy.spin()
