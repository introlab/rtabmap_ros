cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

project(rtabmap-pkg)

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

# For CMake 2.6
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

find_package(OpenCV REQUIRED)

rosbuild_add_executable(rtabmap src/CoreNode.cpp src/CoreWrapper.cpp)
target_link_libraries(rtabmap ${OpenCV_LIBS})

rosbuild_add_executable(rtabmap_out src/OutputNode.cpp)
rosbuild_add_executable(abtr_velocity src/AbtrVelocityNode.cpp)
rosbuild_add_executable(twist_to_turtle_vel src/CmdVelToTurtleVelNode.cpp)
rosbuild_add_executable(twist_to_pose src/Twist2Pose.cpp)
rosbuild_add_executable(twist_to_twist_stamped src/Twist2TwistStamped.cpp)

# Input nodes
rosbuild_add_boost_directories()
rosbuild_add_executable(input_image_audio_node src/ImageAudioInputNode.cpp)
target_link_libraries(input_image_audio_node ${OpenCV_LIBS})
rosbuild_link_boost(input_image_audio_node signals)

rosbuild_add_executable(input_image_audio_twist_node src/ImageAudioTwistInputNode.cpp)
target_link_libraries(input_image_audio_twist_node ${OpenCV_LIBS})
rosbuild_link_boost(input_image_audio_twist_node signals)

rosbuild_add_executable(input_image_twist_node src/ImageTwistInputNode.cpp)
target_link_libraries(input_image_twist_node ${OpenCV_LIBS})
rosbuild_link_boost(input_image_twist_node signals)

FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui)
IF(QT4_FOUND AND QT_QTCORE_FOUND AND QT_QTGUI_FOUND)
    INCLUDE(${QT_USE_FILE})
    rosbuild_add_executable(rtabmap_gui src/GuiNode.cpp src/GuiWrapper.cpp src/PreferencesDialogROS.cpp)
    target_link_libraries(rtabmap_gui ${QT_LIBRARIES} ${OpenCV_LIBS} "-lrtabmap_gui")
    
    rosbuild_add_executable(visual_attention src/VisualAttentionNode.cpp)
    target_link_libraries(visual_attention ${QT_LIBRARIES} ${OpenCV_LIBS})
ELSE()
    MESSAGE(STATUS "[WARNING] Qt4 not found, the GUI node will not be compiled...")
ENDIF()
