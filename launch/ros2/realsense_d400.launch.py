# Requirements:
#   A realsense D400 series
#   Install realsense2 ros2 package (refactor branch)
# Example:
#   $ ros2 run realsense_node realsense_node
#   $ ros2 launch rtabmap_ros realsense_d400.launch.py

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
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),

        # Nodes to launch
        Node(
            package='rtabmap_ros', node_executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),
            
        Node(
            package='rtabmap_ros', node_executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

        Node(
            package='rtabmap_ros', node_executable='rtabmapviz', output='screen',
            parameters=parameters,
            remappings=remappings),
    ])
