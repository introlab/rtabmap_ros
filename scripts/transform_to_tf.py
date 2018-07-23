#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import TransformStamped

def callback(transform):
    global br
    global frame_id
    global child_frame_id
    local_frame_id = transform.header.frame_id
    local_child_frame_id = transform.child_frame_id
    if not local_frame_id:
        local_frame_id = frame_id
    if not local_child_frame_id:
        local_child_frame_id = child_frame_id
    br.sendTransform(
        (transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z),
        (transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w),
        transform.header.stamp,
        child_frame_id,
        frame_id)

if __name__ == "__main__":

    rospy.init_node("transform_to_tf", anonymous=True)

    frame_id = rospy.get_param('~frame_id', 'world')
    child_frame_id = rospy.get_param('~child_frame_id', 'transform')

    br = tf.TransformBroadcaster()
    rospy.Subscriber("transform", TransformStamped, callback, queue_size=1)
    rospy.spin()
