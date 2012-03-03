
########################################
# Statistics stuff: 
# These fields are empty if RTAb-Map's 
# parameter publishStats=false
########################################

Header header

sensor_msgs/CompressedImage refImage
int32 refChild

sensor_msgs/CompressedImage loopClosureImage

#std::map<int, float> posterior;
int32[] posteriorKeys
float32[] posteriorValues

#std::map<int, float> likelihood;
int32[] likelihoodKeys
float32[] likelihoodValues

#std::map<int, int> weights;
int32[] weightsKeys
int32[] weightsValues

#std::map<std::string, float> stats
string[] statsKeys
float32[] statsValues

##Keypoints##
#std::multimap<int, cv::KeyPoint> refWords
int32[] refWordsKeys
rtabmap/KeyPoint[] refWordsValues

#std::multimap<int, cv::KeyPoint> loopWords
int32[] loopWordsKeys
rtabmap/KeyPoint[] loopWordsValues

##SM masks##
uint8[] refMotionMask
uint8[] loopMotionMask

