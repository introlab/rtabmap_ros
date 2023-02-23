cmake_minimum_required(VERSION 3.5)
project(rtabmap_viz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(rtabmap_sync REQUIRED)
find_package(tf2 REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# libraries
SET(Libraries
   cv_bridge
   geometry_msgs
   rclcpp
   std_msgs
   std_srvs
   nav_msgs
   rtabmap_msgs
   rtabmap_sync
   tf2
)

###########
## Build ##
###########

add_executable(rtabmap_viz src/GuiNode.cpp src/GuiWrapper.cpp src/PreferencesDialogROS.cpp)
ament_target_dependencies(rtabmap_viz ${Libraries})

#############
## Install ##
#############

install(TARGETS 
   rtabmap_viz
   DESTINATION lib/${PROJECT_NAME}
)

ament_package()
