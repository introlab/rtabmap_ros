#!/usr/bin/env python
import rospy
import tf

from tf2_msgs.msg import TFMessage
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import TransformStamped

target_frame_id = ""

def callBack(linkStates):
	global delta, first

	found = False
	for i in range(len(linkStates.name)):
		if linkStates.name[i] == gazebo_frame_id:
			p = linkStates.pose[i]
			found = True
			break
	
	if not found:
 	    roslog.warn("Gazebo link state \"" + gazebo_frame_id +"\" not found, cannot generate ground truth.")
	    return
   
	t = TransformStamped()
	t.header.frame_id = frame_id
        t.header.stamp = rospy.Time.now()

	t.child_frame_id = child_frame_id

	t.transform.translation.x = p.position.x
	t.transform.translation.y = p.position.y
	t.transform.translation.z = p.position.z 

	t.transform.rotation.x = p.orientation.x
	t.transform.rotation.y = p.orientation.y
	t.transform.rotation.z = p.orientation.z
	t.transform.rotation.w = p.orientation.w
	
	tf_pub.publish(TFMessage([t]))

if __name__ == '__main__':
    rospy.init_node('generate_gazebo_ground_truth', disable_signals=True)

    frame_id = rospy.get_param('~frame_id', 'world')
    child_frame_id = rospy.get_param('~child_frame_id', 'base_link_gt')
    gazebo_frame_id = rospy.get_param('~gazebo_frame_id', 'base_link')

    gazebo_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, callBack)
   
    tf_pub = rospy.Publisher('/tf', TFMessage, queue_size=10)
    tf.TransformBroadcaster()

    rospy.spin()
