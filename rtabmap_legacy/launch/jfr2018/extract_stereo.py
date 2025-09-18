import rosbag
import tf
import sys
from tf.msg import tfMessage

if len(sys.argv) < 2:
	print('Usage: $ python extract_stereo.py "2012-01-25-12-14-25"')
	sys.exit(0)

bagName = sys.argv[1]
with rosbag.Bag(bagName + '_stereo.bag', 'w') as outbag:
	print(f'Processing {bagName}.bag...')
	for topic, msg, t in rosbag.Bag(bagName + '.bag').read_messages():
		if topic == "/tf":
			outbag.write(topic, msg, t)
		elif topic == "/wide_stereo/left/camera_info":
			outbag.write(topic, msg, t)
		elif topic == "/wide_stereo/right/camera_info":
			outbag.write(topic, msg, t)
		elif topic == "/wide_stereo/left/image_raw":
			outbag.write(topic, msg, t)
		elif topic == "/wide_stereo/right/image_raw":
			outbag.write(topic, msg, t)

print(f'Output: {bagName}_stereo.bag')
