# Requirements:
#   Install Turtlebot3 packages
# Example:
#   $ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
#   $ ros2 launch rtabmap_ros turtlebot3_scan.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    parameters=[{
          'frame_id':'base_footprint',
          'use_sim_time':use_sim_time,
          'subscribe_depth':False,
          'subscribe_rgb':False,
          'subscribe_scan':True,
          'approx_sync':True,
          'Reg/Strategy':'1',
          'RGBD/NeighborLinkRefining':'True'}]

    remappings=[
          ('scan', '/scan')]

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
