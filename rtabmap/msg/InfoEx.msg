
########################################
# Extended info msg with statistics 
########################################

Header header

int32 refId
int32 loopClosureId

# std::map<int, float> posterior;
int32[] posteriorKeys
float32[] posteriorValues

# std::map<int, float> likelihood;
int32[] likelihoodKeys
float32[] likelihoodValues

# std::map<int, float> rawLikelihood;
int32[] rawLikelihoodKeys
float32[] rawLikelihoodValues

# std::map<int, int> weights;
int32[] weightsKeys
int32[] weightsValues

# std::map<std::string, float> stats
string[] statsKeys
float32[] statsValues

sensor_msgs/Image refImage
sensor_msgs/Image loopImage

#
# For features2d : std::multimap<int, cv::Keypoint> words
#
int32[] refWordsKeys
rtabmap/KeyPoint[] refWordsValues

int32[] loopWordsKeys
rtabmap/KeyPoint[] loopWordsValues