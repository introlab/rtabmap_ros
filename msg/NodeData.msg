
int32 id
int32 mapId
int32 weight
float64 stamp
string label

# Pose from odometry not corrected
geometry_msgs/Pose pose

# Ground truth (optional)
geometry_msgs/Pose groundTruthPose

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
float32 baseline
# local transform (/base_link -> /camera_link)
geometry_msgs/Transform[] localTransform

# compressed 2D laser scan in /base_link frame
# use rtabmap::util3d::uncompressData() from "rtabmap/core/util3d.h"
uint8[] laserScan
int32 laserScanMaxPts
float32 laserScanMaxRange
int32 laserScanFormat
geometry_msgs/Transform laserScanLocalTransform

# compressed user data
# use rtabmap::util3d::uncompressData() from "rtabmap/core/util3d.h"
uint8[] userData

# compressed occupancy grid
# use rtabmap::util3d::uncompressData() from "rtabmap/core/util3d.h"
uint8[] grid_ground
uint8[] grid_obstacles
uint8[] grid_empty_cells
float32 grid_cell_size
Point3f grid_view_point

# std::multimap<wordId, cv::Keypoint>
# std::multimap<wordId, pcl::PointXYZ>
int32[] wordIds
KeyPoint[] wordKpts
sensor_msgs/PointCloud2 wordPts

# compressed descriptors
# use rtabmap::util3d::uncompressData() from "rtabmap/core/util3d.h"
uint8[] descriptors
