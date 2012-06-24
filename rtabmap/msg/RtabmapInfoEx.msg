
########################################
# Statistics stuff: 
# These fields are empty if RTAb-Map's 
# parameter publishStats=false
########################################

Header header

int32 refChild

# std::map<int, float> posterior;
int32[] posteriorKeys
float32[] posteriorValues

# std::map<int, float> likelihood;
int32[] likelihoodKeys
float32[] likelihoodValues

# std::map<int, int> weights;
int32[] weightsKeys
int32[] weightsValues

# std::map<std::string, float> stats
string[] statsKeys
float32[] statsValues

rtabmap/SensorMsg[] refRawData
rtabmap/SensorMsg[] loopRawData

#
# For features2d : std::multimap<int, cv::Keypoint> words
#
int32[] refWordsKeys
rtabmap/KeyPoint[] refWordsValues

int32[] loopWordsKeys
rtabmap/KeyPoint[] loopWordsValues