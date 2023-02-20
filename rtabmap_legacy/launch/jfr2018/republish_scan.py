#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def callback(data):
    t = rospy.Time(data.header.stamp.secs, data.header.stamp.nsecs)
    t+=rospy.Duration.from_sec(offset)
    data.header.stamp = t
    pub.publish(data)

if __name__ == '__main__':
    rospy.init_node('republish_scan', anonymous=True)
    pub = rospy.Publisher('base_scan_t', LaserScan, queue_size=1)
    offset = rospy.get_param('~offset', 0.0)
    rospy.Subscriber("base_scan", LaserScan, callback)
    rospy.spin()
