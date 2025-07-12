
# Requires installed https://github.com/chvmp/champ/tree/ros2
#
# Example:
#   1) Launch simulator (gazebo, nav2 and rtabmap):
#     $ ros2 launch rtabmap_demos champ_sim_vslam.launch.py
#
#   Note that the first time we launch gazebo, it may take a 
#   while to download all assets. You may need to restart the 
#   launch to make sure all nodes are started after the sim is ready.
#
#   2) Move the robot:
#     b) By sending goals with RVIZ's "Nav2 Goal" button in action bar.
#     a) By teleoperating:
#        $ ros2 launch champ_teleop teleop.launch.py 
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

import os

def launch_setup(context, *args, **kwargs):
    
    sim_launch_path = PathJoinSubstitution(
        [FindPackageShare('champ_config'), 'launch', 'gazebo.launch.py']
    )
    
    gz_pkg_share = FindPackageShare(package="champ_gazebo").find("champ_gazebo")

    champ_vslam = PathJoinSubstitution(
        [FindPackageShare('rtabmap_demos'), 'launch', 'champ', 'champ_vslam.launch.py']
    )
    
    rviz = LaunchConfiguration('rviz').perform(context)
    world = LaunchConfiguration('world').perform(context)

    return [
        TimerAction(
            actions = [
               IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(champ_vslam),
                    launch_arguments={
                        'use_sim_time': 'true',
                        'rviz': rviz,
                        'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),
                        'localization': LaunchConfiguration('localization'),
                    }.items()
                )], period = 5.0), # Wait 5 sec to make sure simulator is ready

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch_path),
            launch_arguments={'rviz': 'false',
                              'world': os.path.join(gz_pkg_share, f"worlds/{world}.world")}.items()
        ),
    ]

def generate_launch_description():

    return LaunchDescription([
        
        DeclareLaunchArgument(
            name='rviz', 
            default_value='true',
            description='Run rviz'
        ),
        
        DeclareLaunchArgument(
            name='rtabmap_viz', 
            default_value='true',
            description='Run rtabmap_viz'
        ),
        
        DeclareLaunchArgument(
            'localization', default_value='false', choices=['true', 'false'],
            description='Launch rtabmap in localization mode (a map should have been already created).'),
        
        DeclareLaunchArgument(
            'world', default_value='playground',
            choices=['outdoor', 'playground'],
            description='Champ gazebo world.'),
        
        OpaqueFunction(function=launch_setup)
    ])
