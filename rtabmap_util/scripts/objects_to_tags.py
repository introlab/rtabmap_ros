#!/usr/bin/env python
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_ros.msg import AprilTagDetection
from find_object_2d.msg import ObjectsStamped
import tf
import geometry_msgs.msg

objFramePrefix_ = "object"
distanceMax_ = 0.0

def callback(data):
    global objFramePrefix_
    global distanceMax_
    if len(data.objects.data) > 0:
        output = AprilTagDetectionArray()
        output.header = data.header
        for i in range(0,len(data.objects.data),12):
            try:
                objId = data.objects.data[i]
                (trans,quat) = listener.lookupTransform(data.header.frame_id, objFramePrefix_+'_'+str(int(objId)), data.header.stamp)
                tag = AprilTagDetection()
                tag.id.append(objId)
                tag.pose.pose.pose.position.x = trans[0]
                tag.pose.pose.pose.position.y = trans[1]
                tag.pose.pose.pose.position.z = trans[2]
                tag.pose.pose.pose.orientation.x = quat[0]
                tag.pose.pose.pose.orientation.y = quat[1]
                tag.pose.pose.pose.orientation.z = quat[2]
                tag.pose.pose.pose.orientation.w = quat[3]
                tag.pose.header = output.header
                if distanceMax_ <= 0.0 or trans[2] < distanceMax_:
                    output.detections.append(tag)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        if len(output.detections) > 0:
            pub.publish(output)

if __name__ == '__main__':
    pub = rospy.Publisher('tag_detections', AprilTagDetectionArray, queue_size=10)
    rospy.init_node('objects_to_tags', anonymous=True)
    rospy.Subscriber("objectsStamped", ObjectsStamped, callback)
    objFramePrefix_ = rospy.get_param('~object_prefix', objFramePrefix_)
    distanceMax_ = rospy.get_param('~distance_max', distanceMax_)
    listener = tf.TransformListener()
    rospy.spin()
