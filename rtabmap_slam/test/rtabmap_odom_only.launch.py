# rtabmap alone, mapping from odometry only, for test_shutdown.py.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('database_path'),
        DeclareLaunchArgument('working_directory'),
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[{
                'database_path': LaunchConfiguration('database_path'),
                'Rtabmap/WorkingDirectory': LaunchConfiguration('working_directory'),
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'Rtabmap/DetectionRate': '0'}],
            arguments=['-d']),
    ])
