import rosbag
import tf
import sys
from tf.msg import tfMessage

if len(sys.argv) < 2:
	print 'Usage: $ python extract_rgbd.py "2012-01-25-12-14-25"'
	sys.exit(0)

bagName = sys.argv[1]
with rosbag.Bag(bagName + '_rgbd.bag', 'w') as outbag:
	print 'Processing ' + bagName + '.bag...'
	for topic, msg, t in rosbag.Bag(bagName + '.bag').read_messages():
		if topic == "/tf" or topic == "/camera/depth/image_raw" or topic == "/camera/rgb/camera_info" or topic == "/camera/rgb/image_raw":
			outbag.write(topic, msg, t)
print 'Output: ' + bagName + '_out.bag'
