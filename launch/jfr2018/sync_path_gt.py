#!/usr/bin/env python
import roslib
import rospy
import os
import tf
import numpy
import evaluate_ate
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

def callback(data):
    global slamPoses
    global gtPoses
    global stamps
    global listener
    global lastSize
    global slamPosesInd
    global rmse
    if lastSize != len(data.poses):
        t = rospy.Time(data.poses[len(data.poses)-1].header.stamp.secs, data.poses[len(data.poses)-1].header.stamp.nsecs)
        try:
            listener.waitForTransform(fixedFrame, baseFrame, t, rospy.Duration(0.2))
            (trans,rot) = listener.lookupTransform(fixedFrame, baseFrame, t)
            gtPoses.append(Pose(trans, rot))
            stamps.append(t.to_sec())
            slamPoses.append(data.poses[len(data.poses)-1].pose)
            slamPosesInd.append(len(data.poses)-1)
            first_xyz = numpy.empty([0,3])
            second_xyz = numpy.empty([0,3])
            for g in gtPoses:
                newrow = [g.position[0],g.position[1],g.position[2]]
                first_xyz = numpy.vstack([first_xyz, newrow])
            for p in slamPoses:
                newrow = [p.position.x,p.position.y,p.position.z]
                second_xyz = numpy.vstack([second_xyz, newrow])
            first_xyz = numpy.matrix(first_xyz).transpose()
            second_xyz = numpy.matrix(second_xyz).transpose()
            rot,trans,trans_error = evaluate_ate.align(second_xyz, first_xyz)
            rmse_v = numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error))
            rmse.append(rmse_v)
            print "points= " + str(len(data.poses)) + " added=" + str(len(slamPoses)) + " rmse=" + str(rmse_v)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
            print str(e)
    if len(data.poses) > 0 and len(slamPosesInd) > 0:
        j=0
        for i in slamPosesInd:
           slamPoses[j] = data.poses[i].pose
           j+=1
    lastSize = len(data.poses)

if __name__ == '__main__':
    rospy.init_node('sync_path_gt', anonymous=True)
    listener = tf.TransformListener()
    fixedFrame = rospy.get_param('~fixed_frame_id', 'world')
    baseFrame = rospy.get_param('~frame_id', 'base_link_gt')
    rospy.Subscriber("mapPath", Path, callback, queue_size=1)
    slamPoses = []
    gtPoses = []
    stamps = []
    slamPosesInd = []
    rmse = []
    lastSize = 0
    rospy.spin()
    fileSlam = open('slam_poses.txt','w')
    fileGt = open('gt_poses.txt','w')
    fileRMSE = open('rmse.txt','w')
    print "slam= " + str(len(slamPoses))
    print "gt= " + str(len(gtPoses))
    print "stamps= " + str(len(stamps))
    for c, g, t, r in zip(slamPoses, gtPoses, stamps, rmse):
        fileSlam.write('%f %f %f 0 0 0 0 1\n' % (t, c.position.x, c.position.y))
        fileGt.write('%f %f %f 0 0 0 0 1\n' % (t, g.position[0], g.position[1]))
        fileRMSE.write('%f %f\n' % (t, r))
    fileSlam.close()
    fileGt.close()
    fileRMSE.close()
