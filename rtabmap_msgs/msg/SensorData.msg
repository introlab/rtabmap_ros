#class rtabmap::SensorData

Header header

# For RGB-D, left corresponds to rgb camera, and right corresponds to depth camera.

# Raw images
sensor_msgs/Image left
sensor_msgs/Image right

# Compressed images
# use rtabmap::util3d::uncompressImage() from "rtabmap/core/util3d.h"
uint8[] left_compressed
uint8[] right_compressed

# Camera info
sensor_msgs/CameraInfo[] left_camera_info
sensor_msgs/CameraInfo[] right_camera_info

# Transform from base frame to camera frame
geometry_msgs/Transform[] local_transform

# raw 2d or 3D laser scan
sensor_msgs/PointCloud2 laser_scan
# compressed 2D or 3D laser scan
# use rtabmap::util3d::uncompressData() from "rtabmap/core/util3d.h"
uint8[] laser_scan_compressed
int32 laser_scan_max_pts
float32 laser_scan_max_range
int32 laser_scan_format
# local transform (base frame -> laser frame)
geometry_msgs/Transform laser_scan_local_transform

# compressed user data
# use rtabmap::util3d::uncompressData() from "rtabmap/core/util3d.h"
uint8[] user_data

# compressed occupancy grid
# use rtabmap::util3d::uncompressData() from "rtabmap/core/util3d.h"
uint8[] grid_ground
uint8[] grid_obstacles
uint8[] grid_empty_cells
float32 grid_cell_size
Point3f grid_view_point

# Local features
KeyPoint[] key_points
Point3f[] points
# compressed descriptors
# use rtabmap::util3d::uncompressData() from "rtabmap/core/util3d.h"
uint8[] descriptors

GlobalDescriptor[] global_descriptors

EnvSensor[] env_sensors

sensor_msgs/Imu imu
geometry_msgs/Transform imu_local_transform

LandmarkDetection[] landmarks

# Ground truth
geometry_msgs/Pose ground_truth_pose

# GPS
GPS gps
