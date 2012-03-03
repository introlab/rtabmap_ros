
########################################
# If a loop is found with the current image ("refId"), 
# "loopClosureId" is not null. Field "actuators" contains 
# (when actions are used) actions executed after the 
# "loopClosureId".
########################################

Header header

int32 refId
int32 loopClosureId

uint32 actuatorStep
float32[] actuators

rtabmap/RtabmapInfoEx infoEx