
Header header

# Map ids
int32[] mapIDs
int32[] maps

# compressed images
# use rtabmap::util3d::uncompressImage() from <rtabmap/core/util3d.h>
# std::map<int, uint8[]> images;
int32[] imageIDs
rtabmap/Bytes[] images

# compressed depth images
# use rtabmap::util3d::uncompressImage() from <rtabmap/core/util3d.h>
# std::map<int, uint8[]> depths;
int32[] depthIDs
rtabmap/Bytes[] depths

# compressed 2D point clouds (laser scans)
# use rtabmap::util3d::uncompressData() from <rtabmap/core/util3d.h>
# std::map<int, uint8[]> depths2d;
int32[] depth2DIDs
rtabmap/Bytes[] depth2Ds

# depth constants
int32[] depthConstantIDs
float32[] depthConstants

# local transforms
int32[] localTransformIDs
geometry_msgs/Transform[] localTransforms

# std::map<int, Pose> poses;
int32[] poseIDs
geometry_msgs/Pose[] poses

# std::multimap<int, Link> constraints;
int32[] constraintFromIDs
int32[] constraintToIDs
int32[] constraintTypes
geometry_msgs/Transform[] constraints

