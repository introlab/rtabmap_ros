import rosbag
from tf.msg import tfMessage
with rosbag.Bag('2012-01-25-12-33-29_scans-noodom.bag', 'w') as outbag:
	for topic, msg, t in rosbag.Bag('2012-01-25-12-33-29_scans.bag').read_messages():
		if topic == "/tf" and msg.transforms:
			newList = [];
			for m in msg.transforms:
				if m.header.frame_id != "/odom_combined":
					newList.append(m)
				else:
					print 'odom frame removed!'
			if len(newList)>0:
				msg.transforms = newList
				outbag.write(topic, msg, t)
		else:
			outbag.write(topic, msg, t)
