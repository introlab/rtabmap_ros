import rosbag
import tf
import sys
from tf.msg import tfMessage

if len(sys.argv) < 2:
	print 'Usage: $ python extract_scans.py "2012-01-25-12-14-25"'
	sys.exit(0)

bagName = sys.argv[1]
with rosbag.Bag(bagName + '_scans.bag', 'w') as outbag:
	print 'Processing ' + bagName + '.bag...'
	for topic, msg, t in rosbag.Bag(bagName + '.bag').read_messages():
		if topic == "/tf":
			outbag.write(topic, msg, t)
		elif topic == "/base_scan":
			outbag.write(topic, msg, t)
                elif topic == "/robot_pose_ekf/odom_combined":
			outbag.write(topic, msg, t)

print 'Output: ' + bagName + '_out.bag'
