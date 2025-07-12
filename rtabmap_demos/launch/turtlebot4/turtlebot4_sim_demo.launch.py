#
# Note: Make sure you have this fix for turtlebot4_description https://github.com/turtlebot/turtlebot4/pull/434,
#       otherwise, the lidar and camera point cloud won't be aligned correctly.
#
# Example:
#   1) Launch simulator (turtlebot4, nav2 and rtabmap):
#     $ ros2 launch rtabmap_demos turtlebot4_sim_demo.launch.py
#
#   2) Click on "Play" button on bottom-left of gazebo.
#
#   3) Click on double points ".." button on top-right next to power button to undock.
#
#   4) Move the robot:
#     b) By sending goals with RVIZ's "Nav2 Goal" button in action bar.
#     a) By teleoperating:
#        $ ros2 run teleop_twist_keyboard teleop_twist_keyboard
#     c) By using autonomous exploration node (tested with https://github.com/robo-friends/m-explore-ros2):
#        $ ros2 launch explore_lite explore.launch.py
#

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='true',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('rtabmap_viz', default_value='true',
                          choices=['true', 'false'], description='Start rtabmap_viz.'),
    DeclareLaunchArgument('localization', default_value='false',
                          choices=['true', 'false'], description='Start rtabmap in localization mode (a map should have been already created).'),
    DeclareLaunchArgument('nav2', default_value='true',
                          choices=['true', 'false'], description='Start nav2.'),
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Ignition World'),
]

def generate_launch_description():
    # Directories
    pkg_turtlebot4_ignition_bringup = get_package_share_directory(
        'turtlebot4_ignition_bringup')
    pkg_rtabmap_demos = get_package_share_directory(
        'rtabmap_demos')

    # Paths
    ignition_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'turtlebot4_ignition.launch.py'])
    rtabmap_launch = PathJoinSubstitution(
        [pkg_rtabmap_demos, 'launch', 'turtlebot4', 'turtlebot4_slam.launch.py'])

    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ignition_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world')),
            ('slam', 'false'),
            ('localization', 'false'),
            ('nav2', LaunchConfiguration('nav2')),
            ('rviz', LaunchConfiguration('rviz'))
        ]
    )

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rtabmap_launch]),
        launch_arguments=[
            ('rtabmap_viz', LaunchConfiguration('rtabmap_viz')),
            ('localization', LaunchConfiguration('localization')),
            ('use_sim_time', 'true')
        ]
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(rtabmap) # put it first so that localization arg is not overwritten by the same used by ignition
    ld.add_action(ignition)
    return ld
