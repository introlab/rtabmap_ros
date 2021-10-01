# Requirements:
#   A realsense D400 series
#   Install realsense2 ros2 package (refactor branch)
# Example:
#   $ ros2 launch realsense2_camera rs_launch.py align_depth:=true
#
#   $ ros2 launch rtabmap_ros realsense_d400.launch.py
#   OR
#   $ ros2 launch rtabmap_ros rtabmap.launch.py frame_id:=camera_link args:="-d" rgb_topic:=/camera/color/image_raw depth_topic:=/camera/aligned_depth_to_color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    parameters=[{
          'frame_id':'camera_link',
          'subscribe_depth':True,
          'approx_sync':False}]

    remappings=[
          ('rgb/image', '/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/aligned_depth_to_color/image_raw')]

    return LaunchDescription([

        # Nodes to launch
        Node(
            package='rtabmap_ros', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),

        Node(
            package='rtabmap_ros', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

        Node(
            package='rtabmap_ros', executable='rtabmapviz', output='screen',
            parameters=parameters,
            remappings=remappings),
    ])
