import rosbag
import tf
import sys
from tf.msg import tfMessage

if len(sys.argv) < 2:
	print 'Usage: $ python extract_stereo.py "2012-01-25-12-14-25"'
	sys.exit(0)

bagName = sys.argv[1]
with rosbag.Bag(bagName + '_stereo.bag', 'w') as outbag:
	print 'Processing ' + bagName + '.bag...'
	leftCamInfoStatus = True
	leftImageStatus = True
	rightCamInfoStatus = True
	rightImageStatus = True
	for topic, msg, t in rosbag.Bag(bagName + '.bag').read_messages():
		if topic == "/tf":
			outbag.write(topic, msg, t)
		elif topic == "/wide_stereo/left/camera_info":
			if leftCamInfoStatus:
				outbag.write(topic, msg, t)
			leftCamInfoStatus = not leftCamInfoStatus
		elif topic == "/wide_stereo/right/camera_info":
			if rightCamInfoStatus:
				outbag.write(topic, msg, t)
			rightCamInfoStatus = not rightCamInfoStatus
		elif topic == "/wide_stereo/left/image_raw":
			if leftImageStatus:
				outbag.write(topic, msg, t)
			leftImageStatus = not leftImageStatus
		elif topic == "/wide_stereo/right/image_raw":
			if rightImageStatus:
				outbag.write(topic, msg, t)
			rightImageStatus = not rightImageStatus

print 'Output: ' + bagName + '_out.bag'
