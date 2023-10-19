#class rtabmap::Signature

int32 id
int32 map_id
int32 weight
float64 stamp
string label

# Pose from odometry not corrected
geometry_msgs/Pose pose

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

SensorData data
