
int32 id
int32 mapId
int32 weight
float64 stamp
string label
UserData userData

# Pose from odometry not corrected
geometry_msgs/Pose pose

# compressed image in /camera_link frame
# use rtabmap::util3d::uncompressImage() from "rtabmap/core/util3d.h"
uint8[] image

# compressed depth image in /camera_link frame
# use rtabmap::util3d::uncompressImage() from "rtabmap/core/util3d.h"
uint8[] depth

float32 fx
float32 fy
float32 cx
float32 cy

# compressed 2D laser scan in /base_link frame
# use rtabmap::util3d::uncompressData() from "rtabmap/core/util3d.h"
uint8[] laserScan

# local transform (/base_link -> /camera_link)
geometry_msgs/Transform localTransform


# std::multimap<wordId, cv::Keypoint>
# std::multimap<wordId, pcl::PointXYZ>
int32[] wordIds
KeyPoint[] wordKpts
sensor_msgs/PointCloud2 wordPts