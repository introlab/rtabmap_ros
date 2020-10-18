# Requirements:
#   Install Turtlebot3 packages
#   Install https://github.com/mlherd/ros2_turtlebot3_waffle_intel_realsense
# Example:
#   $ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
#   $ ros2 launch rtabmap_ros turtlebot3_rgbd.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    parameters=[{
          'frame_id':'base_footprint',
          'use_sim_time':use_sim_time,
          'subscribe_depth':True}]

    remappings=[
          ('rgb/image', '/intel_realsense_r200_depth/image_raw'),
          ('rgb/camera_info', '/intel_realsense_r200_depth/camera_info'),
          ('depth/image', '/intel_realsense_r200_depth/depth/image_raw')]

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        # Nodes to launch
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
