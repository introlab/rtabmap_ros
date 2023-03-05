#!/usr/bin/env python
import roslib
import rospy
import os
import tf
import numpy
import evaluate_ate
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

def callback(data):
    global slamPoses
    global gtPoses
    global stamps
    global listener
    global rmse
    global lastTime

    if rospy.get_time() - lastTime < 1:
        return
    lastTime = rospy.get_time()

    t = rospy.Time(data.header.stamp.secs, data.header.stamp.nsecs)
    try:
        listener.waitForTransform(fixedFrame, baseFrame, t, rospy.Duration(0.2))
        (trans,rot) = listener.lookupTransform(fixedFrame, baseFrame, t)
        gtPoses.append(Point(trans[0], trans[1], 0))
        stamps.append(t.to_sec())
        slamPoses.append(data.pose.pose.position)
        first_xyz = numpy.empty([0,3])
        second_xyz = numpy.empty([0,3])
        for g in gtPoses:
            newrow = [g.x,g.y,g.z]
            first_xyz = numpy.vstack([first_xyz, newrow])
        for p in slamPoses:
            newrow = [p.x,p.y,p.z]
            second_xyz = numpy.vstack([second_xyz, newrow])
        first_xyz = numpy.matrix(first_xyz).transpose()
        second_xyz = numpy.matrix(second_xyz).transpose()
        rot,trans,trans_error = evaluate_ate.align(second_xyz, first_xyz)
        rmse_v = numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error))
        rmse.append(rmse_v)
        print " added=" + str(len(slamPoses)) + " rmse=" + str(rmse_v)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
        print str(e)

if __name__ == '__main__':
    rospy.init_node('sync_odom_gt', anonymous=True)
    listener = tf.TransformListener()
    fixedFrame = rospy.get_param('~fixed_frame_id', 'world')
    baseFrame = rospy.get_param('~frame_id', 'base_link_gt')
    rospy.Subscriber("icp_odom", Odometry, callback, queue_size=1)
    slamPoses = []
    gtPoses = []
    stamps = []
    rmse = []
    lastTime = rospy.get_time()
    rospy.spin()
    fileSlam = open('slam_poses.txt','w')
    fileGt = open('gt_poses.txt','w')
    fileRMSE = open('rmse.txt','w')
    print "slam= " + str(len(slamPoses))
    print "gt= " + str(len(gtPoses))
    print "stamps= " + str(len(stamps))
    for c, g, t, r in zip(slamPoses, gtPoses, stamps, rmse):
        fileSlam.write('%f %f %f 0 0 0 0 1\n' % (t, c.x, c.y))
        fileGt.write('%f %f %f 0 0 0 0 1\n' % (t, g.x, g.y))
        fileRMSE.write('%f %f\n' % (t, r))
    fileSlam.close()
    fileGt.close()
    fileRMSE.close()
