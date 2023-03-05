
Header header

# For stereo, rgb corresponds to left camera, and depth the right camera.

# camera info
sensor_msgs/CameraInfo rgb_camera_info
sensor_msgs/CameraInfo depth_camera_info

# Raw
sensor_msgs/Image rgb
sensor_msgs/Image depth

# Compressed
sensor_msgs/CompressedImage rgb_compressed
sensor_msgs/CompressedImage depth_compressed

# Local features
KeyPoint[] key_points
Point3f[] points
# compressed descriptors
# use rtabmap::util3d::uncompressData() from "rtabmap/core/util3d.h"
uint8[] descriptors

GlobalDescriptor global_descriptor
