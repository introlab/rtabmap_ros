#!/usr/bin/env python  
import rospy
import tf
import numpy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped

if __name__ == '__main__':
    rospy.init_node('tf_to_pose', anonymous=True)
    listener = tf.TransformListener()
    frame = rospy.get_param('~frame', 'world')
    childFrame = rospy.get_param('~child_frame', 'kinect_gt')
    outputFrame = rospy.get_param('~output_frame', 'kinect')
    cov = rospy.get_param('~cov', 1)
    rateParam = rospy.get_param('~rate', 30) # 10hz
    pub = rospy.Publisher('global_pose', PoseWithCovarianceStamped, queue_size=1)
   
    print 'start loop!'
    rate = rospy.Rate(rateParam) 
    while not rospy.is_shutdown():
        poseOut = PoseWithCovarianceStamped()
        try:
            now = rospy.get_rostime()
            listener.waitForTransform(frame, childFrame, now, rospy.Duration(0.033))
            (trans,rot) = listener.lookupTransform(frame, childFrame, now)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException), e:
            print str(e)
            rate.sleep()
            continue
        
        poseOut.header.stamp.nsecs = now.nsecs
        poseOut.header.stamp.secs = now.secs
        poseOut.header.frame_id = outputFrame
        poseOut.pose.pose.position.x = trans[0]
        poseOut.pose.pose.position.y = trans[1]
        poseOut.pose.pose.position.z = trans[2]
        poseOut.pose.pose.orientation.x = rot[0]
        poseOut.pose.pose.orientation.y = rot[1]
        poseOut.pose.pose.orientation.z = rot[2]
        poseOut.pose.pose.orientation.w = rot[3]
        poseOut.pose.covariance = (cov * numpy.eye(6, dtype=numpy.float64)).tolist()
        poseOut.pose.covariance = [item for sublist in poseOut.pose.covariance for item in sublist]

        print str(poseOut)
        pub.publish(poseOut)
        rate.sleep()
