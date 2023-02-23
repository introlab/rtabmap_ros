#!/usr/bin/env python

# Using netvlad tensorflow-v1 implementation from https://github.com/uzh-rpg/netvlad_tf_open/
# For ROS melodic, follow the following instructions to rebuild cv_bridge with Python3
# https://medium.com/@beta_b0t/how-to-setup-ros-with-python-3-44a69ca36674
# On Jetpack 4.4 (18.04 and OpenCV4), use vision_opencv's noetic branch. In cv_bridge/CMakeLists.txt,
# apply this patch:
#   -find_package(Boost REQUIRED python37)
#   +find_package(Boost REQUIRED python3)

from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
import tensorflow as tf
import time

import netvlad_tf.net_from_mat as nfm
import netvlad_tf.nets as nets

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rtabmap_python import compression as cp
from rtabmap_msgs.msg import GlobalDescriptor

class netvlad_ros:

  def __init__(self):

    self.dim = rospy.get_param('~dim', 4096)
    self.scale = rospy.get_param('~scale', 1.0)
    rospy.loginfo("Parameter dim=%d", self.dim)
    rospy.loginfo("Parameter scale=%d", self.scale)

    tf.reset_default_graph()

    self.image_batch = tf.placeholder(
        dtype=tf.float32, shape=[None, None, None, 3])

    self.net_out = nets.vgg16NetvladPca(self.image_batch)
    self.saver = tf.train.Saver()

    self.sess = tf.Session()
    self.saver.restore(self.sess, nets.defaultCheckpoint())

    self.pub = rospy.Publisher('netvlad_descriptor', GlobalDescriptor, queue_size=1)  

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image",Image,self.callback, queue_size=1)

  def callback(self,data):
    start = time.time()
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
    except CvBridgeError as e:
      print(e)

    if self.scale != 1.0:
      width = int(cv_image.shape[1] * self.scale)
      height = int(cv_image.shape[0] * self.scale)
      cv_image = cv2.resize(cv_image, (width, height), interpolation = cv2.INTER_AREA)

    batch = np.expand_dims(cv_image, axis=0)
    result = self.sess.run(self.net_out, feed_dict={self.image_batch: batch})
    result = result[:,:self.dim]

    descriptor = GlobalDescriptor()
    descriptor.type = 0
    descriptor.header = data.header
    descriptor.data = cp.compress(result)
    self.pub.publish(descriptor)
    end = time.time()
    rospy.loginfo("Extracting descriptor (img=%dx%d, dim=%d): %fs", cv_image.shape[1], cv_image.shape[0], self.dim, end-start)

def main(args):
  rospy.init_node('netvlad', anonymous=True)
  n = netvlad_ros()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)

