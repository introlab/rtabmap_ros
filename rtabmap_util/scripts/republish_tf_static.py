#!/usr/bin/env python
import rospy
from tf2_msgs.msg import TFMessage
        
msg=TFMessage()
        
def callback(data):
    global msg
    if len(msg.transforms) == 0:
        msg = data
    else:
        msg.transforms = msg.transforms+ data.transforms
    rospy.loginfo("Received /tf_static_old and republising latched /tf_static")
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("tf_static_old", TFMessage, callback)
    pub = rospy.Publisher('tf_static', TFMessage, queue_size=10, latch=True)
    rospy.spin()
