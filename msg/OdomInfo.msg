
std_msgs/Header header

bool lost
int32 matches
int32 inliers
float32 icp_inliers_ratio
float32 icp_rotation
float32 icp_translation
float32 icp_structural_complexity
float64[36] covariance
int32 features
int32 local_map_size
int32 local_scan_map_size
int32 local_key_frames
int32 local_bundle_outliers
int32 local_bundle_constraints
float32 local_bundle_time
bool key_frame_added
float32 time_estimation
float32 time_particle_filtering
float32 stamp
float32 interval
float32 distance_travelled
int32 memory_usage # MB

geometry_msgs/Transform transform
geometry_msgs/Transform transform_filtered
geometry_msgs/Transform transform_ground_truth
geometry_msgs/Transform guess_velocity

# 0=F2M, 1=F2F
int32 type

# F2M odometry
#   std::multimap<int, cv::KeyPoint> words;
#   std::vector<int> wordMatches;
#   std::vector<int> wordInliers;
int32[] words_keys
KeyPoint[] words_values
int32[] word_matches
int32[] word_inliers
int32[] local_map_keys
Point3f[] local_map_values

# compressed local scan map data
# use rtabmap::util3d::uncompressData() from "rtabmap/core/util3d.h"
uint8[] local_scan_map

# F2F odometry
#   std::vector<cv::Point2f> ref_corners;
#   std::vector<cv::Point2f> new_corners;
#   std::vector<int> corner_inliers;
Point2f[] ref_corners
Point2f[] new_corners
int32[] corner_inliers

