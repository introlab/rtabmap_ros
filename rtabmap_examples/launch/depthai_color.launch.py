# Requirements:
#   A OAK-D camera
#   Install depthai-ros package (https://github.com/luxonis/depthai-ros). Tested on Humble branch!
# Example:
#   $ ros2 launch rtabmap_examples depthai_color.launch.py camera_model:=OAK-D
#
# Description: In this example, we feed RGB-D images to rtabmap
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
    parameters=[{'frame_id':'oak-d-base-frame',
                 'subscribe_rgbd':True,
                 'subscribe_odom_info':True,
                 'approx_sync':False}]
    
    sync_parameters=[{'approx_sync':True,
                      'approx_sync_max_interval':0.005}]
    
    remappings=[('imu', '/imu/data')]

    return LaunchDescription([

        # Launch camera driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('depthai_examples'), 'launch'),
                '/stereo_inertial_node.launch.py']),
                launch_arguments={'enableRviz': 'false',
                                  'rgbResolution': '1080p',
                                  'rgbScaleNumerator': '2', # Convert to 720p (same size than depth)
                                  'rgbScaleDinominator': '3'}.items(),
        ),

        # Sync right/depth/camera_info together
        Node(   
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=sync_parameters,
            remappings=[('rgb/image', '/color/image'),
                        ('rgb/camera_info', '/color/camera_info'),
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
