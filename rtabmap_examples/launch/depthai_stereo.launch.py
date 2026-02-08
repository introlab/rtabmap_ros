# Requirements:
#   A OAK-D camera
#   Install depthai-ros package (https://github.com/luxonis/depthai-ros). Tested on Humble branch!
#
#   Issue: To have compatible stereo camera info with rtabmap (P[0,3] should be positive on left camera info or negative in right camera info), 
#          we should add the following line here (https://github.com/luxonis/depthai-ros/blob/887248d72cc6b9515793f828645346408b9cad47/depthai_examples/src/stereo_inertial_publisher.cpp#L593)
#          so that left camera info has a positive P[0,3] instead of negative to correctly compute the baseline:
#
#   <line 593> leftCameraInfo.p[3] *=-1;
# 
# Example:
#   $ ros2 launch rtabmap_examples depthai_stereo.launch.py camera_model:=OAK-D
#
# Description: In this example, we feed stereo IR images to rtabmap
#
# Note: The first frames may be too bright or too dark till camera exposure adjusts 
#       to an appriopriate level. Do "Detection->Reset odometry", then 
#       "Edit->Delete memory" if tracking is lost on start.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    parameters={'frame_id':'oak-d-base-frame',
                 'subscribe_rgbd':True,
                 'subscribe_odom_info':True,
                 'approx_sync':False,
                 'wait_imu_to_init':True}
    
    sync_parameters=[{'approx_sync':True,
                      'approx_sync_max_interval':0.005}]

    remappings=[('imu', '/imu/data')]

    return LaunchDescription([

        # Launch camera driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('depthai_examples'), 'launch'),
                '/stereo_inertial_node.launch.py']),
                launch_arguments={'depth_aligned': 'false', # not color mode
                                  'enableRviz': 'false',
                                  'monoResolution': '400p'}.items(),
        ),

        # Sync right/depth/camera_info together
        Node(   
            package='rtabmap_sync', executable='stereo_sync', output='screen',
            parameters=sync_parameters,
            remappings=[('left/image', '/left/image_rect'),
                        ('left/camera_info', '/left/camera_info'),
                        ('right/image', '/right/image_rect'),
                        ('right/camera_info', '/right/camera_info')]),

        # Compute quaternion of the IMU
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame':'enu', 
                         'publish_tf':False}],
            remappings=[('imu/data_raw', '/imu')]),

        # Visual odometry
        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            parameters=[parameters],
            remappings=remappings),

        # VSLAM
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']),

        # Visualization
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[parameters,
                        {'odometry_node_name': "stereo_odometry"}],
            remappings=remappings)
    ])
