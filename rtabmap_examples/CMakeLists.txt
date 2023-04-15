cmake_minimum_required(VERSION 3.5)
project(rtabmap_examples)

find_package(catkin REQUIRED COMPONENTS
             roscpp rtabmap_conversions message_filters
)

catkin_package(
  CATKIN_DEPENDS roscpp rtabmap_conversions message_filters
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(rtabmap_external_loop_detection_example src/ExternalLoopDetectionExample.cpp)
target_link_libraries(rtabmap_external_loop_detection_example ${catkin_LIBRARIES})
set_target_properties(rtabmap_external_loop_detection_example PROPERTIES OUTPUT_NAME "external_loop_detection_example")

#############
## Install ##
#############

install(TARGETS 
   rtabmap_external_loop_detection_example
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Install ##
#############

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

