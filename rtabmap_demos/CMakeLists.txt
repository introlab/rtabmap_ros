cmake_minimum_required(VERSION 3.5)
project(rtabmap_demos)

find_package(catkin REQUIRED COMPONENTS
              rtabmap_conversions rtabmap_msgs visualization_msgs
)

# Optional components
find_package(find_object_2d)

SET(optional_dependencies "")
IF(find_object_2d_FOUND)
   SET(optional_dependencies ${optional_dependencies} find_object_2d)
ENDIF(find_object_2d_FOUND)

catkin_package(
  CATKIN_DEPENDS rtabmap_conversions rtabmap_msgs visualization_msgs ${optional_dependencies}
)

include_directories(${catkin_INCLUDE_DIRS})

IF(NOT WIN32)
  add_executable(rtabmap_wifi_signal_pub src/WifiSignalPubNode.cpp)
  target_link_libraries(rtabmap_wifi_signal_pub ${catkin_LIBRARIES})
  set_target_properties(rtabmap_wifi_signal_pub PROPERTIES OUTPUT_NAME "wifi_signal_pub")
ENDIF(NOT WIN32)

add_executable(rtabmap_wifi_signal_sub src/WifiSignalSubNode.cpp)
target_link_libraries(rtabmap_wifi_signal_sub ${catkin_LIBRARIES})
set_target_properties(rtabmap_wifi_signal_sub PROPERTIES OUTPUT_NAME "wifi_signal_sub")

# If find_object_2d is found, add save objects example
IF(find_object_2d_FOUND)
    MESSAGE(STATUS "WITH find_object_2d")
    include_directories(${find_object_2d_INCLUDE_DIRS})
    add_executable(rtabmap_save_objects_example src/SaveObjectsExample.cpp)
    target_link_libraries(rtabmap_save_objects_example ${catkin_LIBRARIES} ${find_object_2d_LIBRARIES})
    set_target_properties(rtabmap_save_objects_example PROPERTIES OUTPUT_NAME "save_objects_example")
ENDIF(find_object_2d_FOUND)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/wifi_signal_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

IF(NOT WIN32)
install(TARGETS 
   rtabmap_wifi_signal_pub
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
ENDIF(NOT WIN32)

install(TARGETS 
   rtabmap_wifi_signal_sub
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

IF(find_object_2d_FOUND)
	install(TARGETS 
	   rtabmap_save_objects_example
	   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)
ENDIF(find_object_2d_FOUND)

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

