#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PointStamped

def callback(point):
    global br
    global frame_id
    local_frame_id = point.header.frame_id
    if not local_frame_id:
        local_frame_id = frame_id
    br.sendTransform(
        (point.point.x, point.point.y, point.point.z),
        tf.transformations.quaternion_from_euler(0,0,0),
        point.header.stamp,
        local_frame_id,
        fixed_frame_id)

if __name__ == "__main__":

    rospy.init_node("point_to_tf", anonymous=True)

    frame_id = rospy.get_param('~frame_id', 'point')
    fixed_frame_id = rospy.get_param('~fixed_frame_id', 'world')

    br = tf.TransformBroadcaster()
    rospy.Subscriber("point", PointStamped, callback, queue_size=1)
    rospy.spin()
