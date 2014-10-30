
int32 id
int32 mapId

# Pose from odometry not corrected
geometry_msgs/Pose pose

# compressed image in /camera_link frame
# use rtabmap::util3d::uncompressImage() from <rtabmap/core/util3d.h>
rtabmap/Bytes image

# compressed depth image in /camera_link frame
# use rtabmap::util3d::uncompressImage() from <rtabmap/core/util3d.h>
rtabmap/Bytes depth

float32 fx
float32 fy
float32 cx
float32 cy

# compressed 2D point cloud (laser scan) in /base_link frame
# use rtabmap::util3d::uncompressData() from <rtabmap/core/util3d.h>
rtabmap/Bytes depth2D

# local transform (/base_link -> /camera_link)
geometry_msgs/Transform localTransform


# std::multimap<int, cv::Keypoint> words
# std::multimap<int, pcl::PointXYZ> words3D
int32[] wordsKeys
rtabmap/KeyPoint[] wordsValues
sensor_msgs/PointCloud2 words3DValues