#!/usr/bin/env python
import roslib
import rospy
import os
import tf
import numpy

if __name__ == '__main__':
    rospy.init_node('groundtruth_tf_broadcaster')
    fixedFrame = rospy.get_param('~fixed_frame_id', 'world')
    baseFrame = rospy.get_param('~frame_id', 'base_link_gt')
    offset_time = rospy.get_param('~offset_time', 0.0)
    offset_x = rospy.get_param('~offset_x', 0.0)
    offset_y = rospy.get_param('~offset_y', 0.0)
    offset_theta = rospy.get_param('~offset_theta', 0.0)
    gtFile = rospy.get_param('~file', 'groundtruth.txt')
    gtFile = os.path.expanduser(gtFile)
    br = tf.TransformBroadcaster()

    init_x = 0
    init_y = 0
    init = False

    # assuming format "timestamp,x,y,theta"
    for line in open(gtFile,'r'):
        mylist = line.split(',')
        if len(mylist) == 4 and not rospy.is_shutdown():
            stamp = float(int(mylist[0]))/1000000.0 + offset_time
            x = float(mylist[1])
            y = float(mylist[2])
            theta = float(mylist[3])

            if not init:
                init_x = x
                init_y = y
                init = True

            x -= init_x
            y -= init_y

            trans1_mat = tf.transformations.translation_matrix((x, y, 0))
            rot1_mat   = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0, 0, theta))
            mat1 = numpy.dot(trans1_mat, rot1_mat)

            trans2_mat = tf.transformations.translation_matrix((offset_x, offset_y, 0))
            rot2_mat   = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0, 0, offset_theta))
            mat2 = numpy.dot(trans2_mat, rot2_mat)

            mat3 = numpy.dot(mat1, mat2)
            trans3 = tf.transformations.translation_from_matrix(mat3)
            rot3 = tf.transformations.quaternion_from_matrix(mat3)

            #print(mat1)
            #print(mat2)
            #print(mat3)

            now = rospy.get_time()
            while not rospy.is_shutdown() and now < stamp:
                delay = stamp - now
                if delay > 0.05:
                    delay = 0.05
                rospy.sleep(delay)
                now = rospy.get_time()
            if not rospy.is_shutdown():
                br.sendTransform(trans3,
                    rot3,
                    rospy.Time.from_sec(stamp),
                    baseFrame,
                    fixedFrame)
        else:
            break
