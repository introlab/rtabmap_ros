
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
#    std::vector<cv::KeyPoint> refCorners;
#    std::vector<cv::KeyPoint> newCorners;
#    std::vector<int> cornerInliers;
#}

bool lost
int32 matches
int32 inliers
float32 variance
int32 features
int32 localMapSize
float32 time

int32 type

int32[] wordsKeys
KeyPoint[] wordsValues
int32[] wordMatches
int32[] wordInliers

KeyPoint[] refCorners
KeyPoint[] newCorners
int32[] cornerInliers

