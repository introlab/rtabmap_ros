
# header.stamp: the timestamp of the detection (e.g. image timestamp)
# header.frame_id: the base frame of pose (e.g., camera optical frame)
std_msgs/Header header

# Landmark's frame id
string landmark_frame_id

# Landmark's unique ID: should be >0
int32 id

# Size in meters of the landmark/tag (optional, set 0 to not use it).
float32 size

# Pose of the landmark in header.frame_id frame.
# If covariance is unknown, keep it as null matrix.
# rtabmap_slam/rtabmap's landmark_angular_variance and
# landmark_linear_variance parameters can be used
# for convenience if covariance is null.
geometry_msgs/PoseWithCovariance pose


