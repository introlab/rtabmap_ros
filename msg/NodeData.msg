
int32 id
int32 map_id
int32 weight
float64 stamp
string label

# Pose from odometry not corrected
geometry_msgs/Pose pose

# Ground truth (optional)
geometry_msgs/Pose ground_truth_pose

# GPS (optional)
GPS gps

# compressed image in /camera_link frame
# use rtabmap::util3d::uncompressImage() from "rtabmap/core/util3d.h"
uint8[] image

# compressed depth image in /camera_link frame
# use rtabmap::util3d::uncompressImage() from "rtabmap/core/util3d.h"
uint8[] depth

# Camera models
float32[] fx
float32[] fy
float32[] cx
float32[] cy
float32[] width
float32[] height
float32[] baseline
# local transform (/base_link -> /camera_link)
geometry_msgs/Transform[] local_transform

# compressed 2D or 3D laser scan
# use rtabmap::util3d::uncompressData() from "rtabmap/core/util3d.h"
uint8[] laser_scan
int32 laser_scan_max_pts
float32 laser_scan_max_range
int32 laser_scan_format
# local transform (/base_link -> /base_laser)
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

# std::multimap<wordId, index>
# std::vector<cv::Keypoint>
# std::vector<cv::Point3f>
int32[] word_id_keys
int32[] word_id_values
KeyPoint[] word_kpts
Point3f[] word_pts
# compressed descriptors
# use rtabmap::util3d::uncompressData() from "rtabmap/core/util3d.h"
uint8[] word_descriptors

GlobalDescriptor[] global_descriptors

EnvSensor[] env_sensors
