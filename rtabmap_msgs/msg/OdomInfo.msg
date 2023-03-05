
Header header

bool lost
int32 matches
int32 inliers
float32 icpInliersRatio
float32 icpRotation
float32 icpTranslation
float32 icpStructuralComplexity
float32 icpStructuralDistribution
int32 icpCorrespondences
float64[36] covariance
int32 features
int32 localMapSize
int32 localScanMapSize
int32 localKeyFrames
int32 localBundleOutliers
int32 localBundleConstraints
float32 localBundleTime
bool keyFrameAdded
float32 timeEstimation
float32 timeParticleFiltering
float32 stamp
float32 interval
float32 distanceTravelled
int32 memoryUsage # MB
float32 gravityRollError
float32 gravityPitchError

# Local bundle camera ids
int32[] localBundleIds

# Local bundle camera models
CameraModels[] localBundleModels

# Local bundle camera poses
geometry_msgs/Pose[] localBundlePoses

geometry_msgs/Transform transform
geometry_msgs/Transform transformFiltered
geometry_msgs/Transform transformGroundTruth
geometry_msgs/Transform guess

# 0=F2M, 1=F2F
int32 type

# F2M odometry
#   std::multimap<int, cv::KeyPoint> words;
#   std::vector<int> wordMatches;
#   std::vector<int> wordInliers;
int32[] wordsKeys
KeyPoint[] wordsValues
int32[] wordMatches
int32[] wordInliers
int32[] localMapKeys
Point3f[] localMapValues

# local scan map data
sensor_msgs/PointCloud2 localScanMap

# F2F odometry
#   std::vector<cv::Point2f> refCorners;
#   std::vector<cv::Point2f> newCorners;
#   std::vector<int> cornerInliers;
Point2f[] refCorners
Point2f[] newCorners
int32[] cornerInliers

