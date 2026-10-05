# Description:
#   3D lidar SLAM with an Ouster OS1-32 turning on a motorized mast (a dynamixel
#   joint), carried by a drone frame. The mast rotation sweeps the lidar's 32 beams
#   over the whole sphere, but it also moves the lidar by a few degrees during each
#   sweep: scans are deskewed point by point with TF, using the IMU orientation for
#   the base and the joint states for the mast. This is rtabmap_examples'
#   lidar3d_assemble.launch.py, set up for the bag: scans are assembled over half a
#   turn of the mast, so each node of the map holds a full sphere.
#
#   Note: the camera's rotation relative to the lidar (box_link -> camera_link in the
#   bags' TF) was calibrated with rtabmap's rtabmap-lidarCameraCalibration tool; its
#   translation is as measured.
#
# Requirements:
#   Download one or more rosbags, the consecutive minutes of the same run:
#    * netherdrone_ouster_vertige_bag_0.zip: https://github.com/introlab/rtabmap_ros/releases/download/0.23.13/netherdrone_ouster_vertige_bag_0.zip
#    * netherdrone_ouster_vertige_bag_1.zip: https://github.com/introlab/rtabmap_ros/releases/download/0.23.13/netherdrone_ouster_vertige_bag_1.zip
#    * netherdrone_ouster_vertige_bag_2.zip: https://github.com/introlab/rtabmap_ros/releases/download/0.23.13/netherdrone_ouster_vertige_bag_2.zip
#    * netherdrone_ouster_vertige_bag_3.zip: https://github.com/introlab/rtabmap_ros/releases/download/0.23.13/netherdrone_ouster_vertige_bag_3.zip
#    * netherdrone_ouster_vertige_bag_4.zip: https://github.com/introlab/rtabmap_ros/releases/download/0.23.13/netherdrone_ouster_vertige_bag_4.zip
#    * netherdrone_ouster_vertige_bag_5.zip: https://github.com/introlab/rtabmap_ros/releases/download/0.23.13/netherdrone_ouster_vertige_bag_5.zip
#
# Example:
#
#   SLAM:
#     $ ros2 launch rtabmap_demos netherdrone_lidar3d_demo.launch.py
#
#   Rosbag:
#     $ ros2 bag play netherdrone_ouster_vertige_bag_0 --clock
#     when done, you can play the next bag(s):
#     $ ros2 bag play netherdrone_ouster_vertige_bag_1 --clock
#     $ ros2 bag play netherdrone_ouster_vertige_bag_2 --clock
#     $ ros2 bag play netherdrone_ouster_vertige_bag_3 --clock
#     $ ros2 bag play netherdrone_ouster_vertige_bag_4 --clock
#     $ ros2 bag play netherdrone_ouster_vertige_bag_5 --clock
#

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    lidar3d_assemble_launch = os.path.join(
        get_package_share_directory('rtabmap_examples'), 'launch', 'lidar3d_assemble.launch.py')
    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_demos'), 'config', 'demo_lidar3d.rviz')
    config_rtabmap_viz = os.path.join(
        get_package_share_directory('rtabmap_demos'), 'config', 'demo_lidar3d_gui.ini')

    camera = LaunchConfiguration('camera')
    rgbd_image_topic = PythonExpression(
        ["'/camera/rgbd_image' if '", camera, "'.lower() == 'true' else ''"])


    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument('rtabmap_viz', default_value='true', description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('rtabmap_viz_cfg', default_value=config_rtabmap_viz, description='Configuration path of rtabmap_viz.'),
        DeclareLaunchArgument('rviz', default_value='false', description='Launch RVIZ (optional).'),
        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz, description='Configuration path of rviz2.'),
        DeclareLaunchArgument('localization', default_value='false', description='Launch in localization mode.'),
        DeclareLaunchArgument('voxel_size', default_value='0.3',
                              description='Voxel size (m) of the downsampled lidar point cloud. In this bag, the median range is 5-7 m and the farthest points are ~23 m.'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='Add the camera images to the map, with a depth made from the lidar, for visual loop closure detection (bag-of-words). Registration stays lidar only.'),
        DeclareLaunchArgument('lidar_range_min', default_value='0.84',
                              description='Lidar points closer than this (m) are ignored. About 6% of each scan hits the drone itself, all within 0.6 m; the surroundings start at ~0.9 m.'),
        DeclareLaunchArgument('assembler_voxel_size', default_value='0.05',
                              description='Voxel size (m) of the assembled clouds added to the map; 0 keeps every point (larger database, e.g., for camera-lidar calibration).'),
        DeclareLaunchArgument('assembling_time', default_value='4.3',
                              description='How long (s) scans are assembled before being added to the map. The mast turns at 42 deg/s: 4.3 s is half a turn, in which the lidar\'s scanning plane sweeps the whole sphere.'),
        DeclareLaunchArgument('intermediate_nodes', default_value='false',
                              description='Also save every odometry pose (10 Hz) between the map\'s nodes in the database.'),
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
                # The fixed frame for deskewing (base_link_stabilized) is made from its
                # orientation.
                'imu_topic': '/imu/data_filtered',
                'voxel_size': LaunchConfiguration('voxel_size'),
                'max_correspondence_distance': '1.0',
                'icp_outlier_ratio': '0.3',
                'odom_key_frame_threshold': '0.9',
                'odom_local_map_size': '20000',
                'assembler_voxel_size': LaunchConfiguration('assembler_voxel_size'),
                'rgbd_image_topic': rgbd_image_topic,
                # The images are raw (k1=-0.34): rectify them, so that the lidar projected
                # into them as their depth (gen_depth) lines up.
                'rectify_images': 'true',
                'gen_depth': camera,
                'lidar_range_min': LaunchConfiguration('lidar_range_min'),
                'expected_update_rate': '15.0',  # the Ouster runs at 10 Hz
                'assembling_time': LaunchConfiguration('assembling_time'),
                # Look TF up at every point's own time ("t" field) rather than
                # interpolating between the first and last points of the sweep: exact,
                # for more TF lookups.
                'deskewing_slerp': 'false',
                'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),
                'rtabmap_viz_cfg': LaunchConfiguration('rtabmap_viz_cfg'),
                'localization': LaunchConfiguration('localization'),
                'database_path': LaunchConfiguration('database_path'),
                'intermediate_nodes': LaunchConfiguration('intermediate_nodes'),
                'ground_truth_frame_id': LaunchConfiguration('ground_truth_frame_id'),
                'ground_truth_base_frame_id': LaunchConfiguration('ground_truth_base_frame_id'),
            }.items()),

        # Camera image and calibration, together in one topic for rtabmap. Both have
        # the same stamps. The camera has no depth: rtabmap makes it from the lidar.
        Node(
            condition=IfCondition(camera),
            package='rtabmap_sync', executable='rgb_sync', output='screen',
            parameters=[{
                'use_sim_time': True,
                'approx_sync': False,
                'image_transport': 'compressed'}],
            remappings=[('rgb/image', '/camera/image_raw'),
                        ('rgb/camera_info', '/camera/camera_info'),
                        ('rgbd_image', '/camera/rgbd_image')]),

        # Orientation from the gyro and accelerometer of /imu/data_raw. Its own
        # orientation (despite the name, it has one) is off from the accelerometer by a
        # constant ~2 deg in roll and ~4 deg in pitch, so it is not used.
        Node(
            package='imu_complementary_filter', executable='complementary_filter_node', output='screen',
            parameters=[{
                'use_sim_time': True,
                'use_mag': False,
                'publish_tf': False,
                'do_bias_estimation': True,
                'do_adaptive_gain': True}],
            remappings=[('imu/data_raw', '/imu/data_raw'),
                        ('imu/data', '/imu/data_filtered')]),

        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            parameters=[{'use_sim_time': True}],
            arguments=['-d', LaunchConfiguration('rviz_cfg')]),
    ])
