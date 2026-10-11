
std_msgs/Header header

# For stereo, rgb corresponds to left camera, and depth the right camera.

# camera info
sensor_msgs/CameraInfo rgb_camera_info
sensor_msgs/CameraInfo depth_camera_info

# Raw
sensor_msgs/Image rgb
sensor_msgs/Image depth

# Compressed
# rgb_compressed: JPEG or PNG in compressed_image_transport's format (e.g.,
#   "bgr8; jpeg compressed bgr8"), readable by the "compressed" image_transport plugin.
# depth_compressed: right image (stereo) in compressed_image_transport's format, or depth
#   image in compressed_depth_image_transport's format (e.g., "16UC1; compressedDepth png",
#   "32FC1; compressedDepth rvl"), readable by the "compressedDepth" image_transport
#   plugin (RVL since Jazzy).
#   Exception: depth compressed in rtabmap's own format (rtabmap_ros < 0.24, or
#   depth_compression_format "legacy[:<format>]", e.g., to keep 32FC1 depth lossless), with format
#   "png", "rvl" or "png:<max>:<quantization>" / "rvl:<max>:<quantization>", which
#   image_transport plugins cannot read. rtabmap_conversions::uncompressDepthImage()
#   decodes every format, and rgbd_split republishes them on a compressedDepth
#   image_transport topic.
sensor_msgs/CompressedImage rgb_compressed
sensor_msgs/CompressedImage depth_compressed

# Local features
KeyPoint[] key_points
Point3f[] points
# compressed descriptors
# use rtabmap::uncompressData() from "rtabmap/core/Compression.h"
uint8[] descriptors

GlobalDescriptor global_descriptor
