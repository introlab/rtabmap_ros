# Requirements:
#   Install Turtlebot3 packages
#   Modify turtlebot3_waffle SDF:
#     1) Edit /opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf
#     2) Add
#          <joint name="camera_rgb_optical_joint" type="fixed">
#            <parent>camera_rgb_frame</parent>
#            <child>camera_rgb_optical_frame</child>
#            <pose>0 0 0 -1.57079632679 0 -1.57079632679</pose>
#            <axis>
#              <xyz>0 0 1</xyz>
#            </axis>
#          </joint> 
#     3) Rename <link name="camera_rgb_frame"> to <link name="camera_rgb_optical_frame">
#     4) Add <link name="camera_rgb_frame"/>
#     5) Change <sensor name="camera" type="camera"> to <sensor name="camera" type="depth">
#     6) Change image width/height from 1920x1080 to 640x480
#     7) Note that we can increase min scan range from 0.12 to 0.2 to avoid having scans 
#        hitting the robot itself
# Example:
#   $ ros2 launch rtabmap_demos turtlebot3_sim_rgbd_scan_demo.launch.py
#
#   Teleop:
#     $ ros2 run turtlebot3_teleop teleop_keyboard

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

import os

def launch_setup(context, *args, **kwargs):
    if not 'TURTLEBOT3_MODEL' in os.environ:
        os.environ['TURTLEBOT3_MODEL'] = 'waffle'

    # Directories
    pkg_turtlebot3_gazebo = get_package_share_directory(
        'turtlebot3_gazebo')
    pkg_nav2_bringup = get_package_share_directory(
        'nav2_bringup')
    pkg_rtabmap_demos = get_package_share_directory(
        'rtabmap_demos')

    world = LaunchConfiguration('world').perform(context)
    
    nav2_params_file = PathJoinSubstitution(
        [FindPackageShare('rtabmap_demos'), 'params', 'turtlebot3_rgbd_scan_nav2_params.yaml']
    )

    # Paths
    gazebo_launch = PathJoinSubstitution(
        [pkg_turtlebot3_gazebo, 'launch', f'turtlebot3_{world}.launch.py'])
    nav2_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])
    rviz_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'rviz_launch.py'])
    rtabmap_launch = PathJoinSubstitution(
        [pkg_rtabmap_demos, 'launch', 'turtlebot3', 'turtlebot3_rgbd_scan.launch.py'])

    # Includes
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments=[
            ('x_pose', LaunchConfiguration('x_pose')),
            ('y_pose', LaunchConfiguration('y_pose'))
        ]
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch]),
        launch_arguments=[
            ('use_sim_time', 'true'),
            ('params_file', nav2_params_file)
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
    return [
        # Nodes to launch
        nav2,
        rviz,
        rtabmap,
        gazebo
    ]

def generate_launch_description():
    return LaunchDescription([
        
        # Launch arguments
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),
        
        DeclareLaunchArgument(
            'world', default_value='house',
            choices=['world', 'house', 'dqn_stage1', 'dqn_stage2', 'dqn_stage3', 'dqn_stage4'],
            description='Turtlebot3 gazebo world.'),
        
        DeclareLaunchArgument(
            'x_pose', default_value='-2.0',
            description='Initial position of the robot in the simulator.'),
        
        DeclareLaunchArgument(
            'y_pose', default_value='0.5',
            description='Initial position of the robot in the simulator.'),

        OpaqueFunction(function=launch_setup)
    ])
