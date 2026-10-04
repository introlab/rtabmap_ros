# Description:
#   3D lidar SLAM with an Ouster OS1-32 turning on a motorized mast (a dynamixel
#   joint), carried by a drone frame. The mast rotation sweeps the lidar's 32 beams
#   over the whole sphere, but it also moves the lidar by a few degrees during each
#   sweep: scans are deskewed point by point with TF, using the IMU orientation for
#   the base and the joint states for the mast. This is rtabmap_examples'
#   lidar3d_assemble.launch.py, set up for the bag.
#
# Requirements:
#   Download the rosbag:
#    * netherdrone_ouster_vertige_bag.zip: https://github.com/introlab/rtabmap_ros/releases/download/0.23.13/netherdrone_ouster_vertige_bag.zip
#
# Example:
#
#   SLAM:
#     $ ros2 launch rtabmap_demos netherdrone_lidar3d_demo.launch.py
#
#   Rosbag:
#     $ ros2 bag play netherdrone_ouster_vertige_bag --clock
#

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    lidar3d_assemble_launch = os.path.join(
        get_package_share_directory('rtabmap_examples'), 'launch', 'lidar3d_assemble.launch.py')

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument('rtabmap_viz', default_value='true', description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('localization', default_value='false', description='Launch in localization mode.'),
        DeclareLaunchArgument('voxel_size', default_value='0.1',
                              description='Voxel size (m) of the downsampled lidar point cloud. In this bag, the median range is 5-7 m and the farthest points are ~23 m.'),
        DeclareLaunchArgument('database_path', default_value=os.path.join(os.environ.get('ROS_HOME', '~/.ros'), 'rtabmap.db'),
                              description='Database where the map is saved (deleted on start in SLAM mode).'),
        DeclareLaunchArgument('ground_truth_frame_id', default_value='',
                              description='Fixed frame of a ground truth trajectory in TF. If set, RTAB-Map reports its error against it in its statistics (Gt/*).'),
        DeclareLaunchArgument('ground_truth_base_frame_id', default_value='base_link_gt',
                              description='Robot frame of the ground truth trajectory in TF.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar3d_assemble_launch),
            launch_arguments={
                'use_sim_time': 'true',
                'frame_id': 'base_link',
                'lidar_topic': '/os_cloud_node/points',
                # Despite its name, it has the orientation set. The fixed frame for
                # deskewing (base_link_stabilized) is made from it.
                'imu_topic': '/imu/data_raw',
                'voxel_size': LaunchConfiguration('voxel_size'),
                'expected_update_rate': '15.0',  # the Ouster runs at 10 Hz
                # Look TF up at every point's own time ("t" field) rather than
                # interpolating between the first and last points of the sweep: exact,
                # for more TF lookups.
                'deskewing_slerp': 'false',
                'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),
                'localization': LaunchConfiguration('localization'),
                'database_path': LaunchConfiguration('database_path'),
                'ground_truth_frame_id': LaunchConfiguration('ground_truth_frame_id'),
                'ground_truth_base_frame_id': LaunchConfiguration('ground_truth_base_frame_id'),
            }.items()),
    ])
