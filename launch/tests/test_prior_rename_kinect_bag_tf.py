#!/usr/bin/env python 
import rosbag
from tf.msg import tfMessage
with rosbag.Bag('rgbd_dataset_freiburg3_long_office_household_tf_renamed.bag', 'w') as outbag:
	for topic, msg, t in rosbag.Bag('rgbd_dataset_freiburg3_long_office_household.bag').read_messages():
		if topic == "/tf" and msg.transforms:
			newList = [];
			for m in msg.transforms:
				if m.child_frame_id != "/kinect":
					newList.append(m)
				else:
					m.child_frame_id = "/kinect_gt"
					newList.append(m)
					print 'kinect frame renamed!'
			if len(newList)>0:
				msg.transforms = newList
				outbag.write(topic, msg, t)
		else:
			outbag.write(topic, msg, t)
