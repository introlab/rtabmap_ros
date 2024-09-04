# Requirements:
#   A OAK-D camera
#   Install depthai-ros package (https://github.com/luxonis/depthai-ros)
# Example:
#   $ ros2 launch rtabmap_examples depthai.launch.py camera_model:=OAK-D

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    parameters=[{'frame_id':'oak-d-base-frame',
                 'subscribe_rgbd':True,
                 'subscribe_odom_info':True,
                 'approx_sync':False,
                 'wait_imu_to_init':True}]

    remappings=[('imu', '/imu/data')]

    return LaunchDescription([

        # Launch camera driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('depthai_examples'), 'launch'),
                '/stereo_inertial_node.launch.py']),
                launch_arguments={'depth_aligned': 'false',
                                  'enableRviz': 'false',
                                  'monoResolution': '400p'}.items(),
        ),

        # Sync right/depth/camera_info together
        Node(   
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=parameters,
            remappings=[('rgb/image', '/right/image_rect'),
                        ('rgb/camera_info', '/right/camera_info'),
                        ('depth/image', '/stereo/depth')]),

        # Compute quaternion of the IMU
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame':'enu', 
                         'publish_tf':False}],
            remappings=[('imu/data_raw', '/imu')]),

        # Visual odometry
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),

        # VSLAM
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

        # Visualization
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings)
    ])
