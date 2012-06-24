cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

find_package(OpenCV REQUIRED)
rosbuild_add_executable(camera src/CameraNode.cpp)
target_link_libraries(camera ${OpenCV_LIBS})

rosbuild_add_executable(rgb2ind src/RGB2IndexedNode.cpp)
target_link_libraries(rgb2ind ${OpenCV_LIBS})

rosbuild_add_executable(xy2polar src/Cartesian2PolarNode.cpp)
target_link_libraries(xy2polar ${OpenCV_LIBS})

rosbuild_add_executable(motion_filter src/MotionFilterNode.cpp)
target_link_libraries(motion_filter ${OpenCV_LIBS})

FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui REQUIRED)
INCLUDE(${QT_USE_FILE})
#This will generate moc_* for Qt
QT4_WRAP_CPP(moc_srcs src/ImageViewQt.hpp)
rosbuild_add_executable(image_view_qt src/ImageViewQtNode.cpp ${moc_srcs})
target_link_libraries(image_view_qt ${QT_LIBRARIES} ${OpenCV_LIBS})
