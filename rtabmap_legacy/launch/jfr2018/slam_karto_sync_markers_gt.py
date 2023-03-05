#!/usr/bin/env python
import roslib
import rospy
import os
import tf
import numpy
import evaluate_ate
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point

def callback(data):
    global slamPoses
    global gtPoses
    global stamps
    global listener
    global lastSize
    global slamPosesInd
    global rmse

    point_markers = []
    for m in data.markers:
        if m.type==2:
            point_markers.append(m)

    if len(point_markers) > 0 and lastSize != len(point_markers):
        t = rospy.Time(point_markers[0].header.stamp.secs, point_markers[0].header.stamp.nsecs)
        try:
            listener.waitForTransform(fixedFrame, baseFrame, t, rospy.Duration(0.2))
            (trans,rot) = listener.lookupTransform(fixedFrame, baseFrame, t)
            gtPoses.append(Point(trans[0], trans[1], 0))
            stamps.append(t.to_sec())
            slamPoses.append(point_markers[len(point_markers)-1].pose.position)
            slamPosesInd.append(len(point_markers)-1)
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
            print "points= " + str(len(point_markers)) + " added=" + str(len(slamPoses)) + " rmse=" + str(rmse_v)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
            print str(e)
    if len(data.markers) > 0 and len(slamPosesInd) > 0:
        j=0
        for i in slamPosesInd:
           slamPoses[j] = point_markers[i].pose.position
           j+=1
    if len(data.markers) > 0:
        lastSize = len(point_markers)

if __name__ == '__main__':
    rospy.init_node('sync_markers_gt', anonymous=True)
    listener = tf.TransformListener()
    fixedFrame = rospy.get_param('~fixed_frame_id', 'world')
    baseFrame = rospy.get_param('~frame_id', 'base_link_gt')
    rospy.Subscriber("visualization_marker_array", MarkerArray, callback, queue_size=1)
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
        fileSlam.write('%f %f %f 0 0 0 0 1\n' % (t, c.x, c.y))
        fileGt.write('%f %f %f 0 0 0 0 1\n' % (t, g.x, g.y))
        fileRMSE.write('%f %f\n' % (t, r))
    fileSlam.close()
    fileGt.close()
    fileRMSE.close()
