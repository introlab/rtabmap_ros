
Header header

#class rtabmap::OdometryInfo
#{
#    bool lost;
#    int matches;
#    int inliers;
#    float variance;
#    int features;
#    int localMapSize;
#    float time;
#
#    int type; // 0=BOW, 1=Optical Flow, 2=ICP
#
#    // BOW odometry
#    std::multimap<int, cv::KeyPoint> words;
#    std::vector<int> wordMatches;
#    std::vector<int> wordInliers;
#
#    // Optical Flow odometry
#    std::vector<cv::Point2f> refCorners;
#    std::vector<cv::Point2f> newCorners;
#    std::vector<int> cornerInliers;
#}

bool lost
int32 matches
int32 inliers
float32 variance
int32 features
int32 localMapSize
int32 localScanMapSize
float32 timeEstimation
float32 timeParticleFiltering
float32 stamp
float32 interval
float32 distanceTravelled

int32 type

int32[] wordsKeys
KeyPoint[] wordsValues
int32[] wordMatches
int32[] wordInliers
int32[] localMapKeys
Point3f[] localMapValues

Point2f[] refCorners
Point2f[] newCorners
int32[] cornerInliers

geometry_msgs/Transform transform
geometry_msgs/Transform transformFiltered

# compressed local scan map data
# use rtabmap::util3d::uncompressData() from "rtabmap/core/util3d.h"
uint8[] localScanMap

