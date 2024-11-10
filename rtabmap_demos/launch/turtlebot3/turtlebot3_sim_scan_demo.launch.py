# Requirements:
#   Install Turtlebot3 packages
#   Modify turtlebot3_waffle SDF:
#     1) Edit /opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf
#     2) We can increase min scan range from 0.12 to 0.2 to avoid having scans 
#        hitting the robot itself
# Example:
#   $ ros2 launch rtabmap_demos turtlebot3_sim_scan_demo.launch.py
#
#   Teleop:
#     $ ros2 run turtlebot3_teleop teleop_keyboard

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os

def generate_launch_description():

    if not 'TURTLEBOT3_MODEL' in os.environ:
        os.environ['TURTLEBOT3_MODEL'] = 'waffle'

    # Directories
    pkg_turtlebot3_gazebo = get_package_share_directory(
        'turtlebot3_gazebo')
    pkg_nav2_bringup = get_package_share_directory(
        'nav2_bringup')
    pkg_rtabmap_demos = get_package_share_directory(
        'rtabmap_demos')

    # Paths
    gazebo_launch = PathJoinSubstitution(
        [pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py'])
    nav2_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])
    rviz_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'rviz_launch.py'])
    rtabmap_launch = PathJoinSubstitution(
        [pkg_rtabmap_demos, 'launch', 'turtlebot3', 'turtlebot3_scan.launch.py'])

    # Includes
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch])
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch]),
        launch_arguments=[
            ('use_sim_time', 'true')
        ]
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch])
    )
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rtabmap_launch]),
        launch_arguments=[
            ('localization', LaunchConfiguration('localization')),
            ('use_sim_time', 'true')
        ]
    )
    
    return LaunchDescription([
        
        # Launch arguments
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),

        # Nodes to launch
        nav2,
        rviz,
        rtabmap,
        gazebo
    ])
