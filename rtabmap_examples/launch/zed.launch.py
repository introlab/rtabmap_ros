# Requirements:
#   A ZED camera
#   Install zed ros2 wrapper package (https://github.com/stereolabs/zed-ros2-wrapper)
# Example:
#   $ ros2 launch rtabmap_examples zed.launch.py camera_model:=zed2i

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition

import tempfile

def generate_launch_description():

    # Hack to override grab_resolution parameter without changing any files
    with tempfile.NamedTemporaryFile(mode='w+t', delete=False) as zed_override_file:
        zed_override_file.write("---\n"+
                  "/**:\n"+
                  "    ros__parameters:\n"+
                  "        general:\n"+
                  "            grab_resolution: 'VGA'")

    camera_model = LaunchConfiguration('camera_model')
    use_zed_odometry = LaunchConfiguration('use_zed_odometry')

    parameters=[{'frame_id':'zed_camera_link',
                 'subscribe_rgbd':True,
                 'subscribe_odom_info': not use_zed_odometry,
                 'approx_sync':False,
                 'wait_imu_to_init':True}]

    remappings=[('imu', '/zed/zed_node/imu/data')]

    if use_zed_odometry:
        remappings.append(('odom', '/zed/zed_node/odom'))
        
    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_zed_odometry', default_value='false',
            description='Use zed\'s computed odometry instead of using rtabmap\'s odometry.'),

        # Launch camera driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('zed_wrapper'), 'launch'),
                '/zed_camera.launch.py']),
                launch_arguments={'camera_model': camera_model,
                                  'ros_params_override_path': zed_override_file.name,
                                  'publish_tf': use_zed_odometry,
                                  'publish_map_tf': 'false'}.items(),
        ),

        # Sync right/depth/camera_info together
        Node(   
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=parameters,
            remappings=[('rgb/image', '/zed/zed_node/rgb/image_rect_color'),
                        ('rgb/camera_info', '/zed/zed_node/rgb/camera_info'),
                        ('depth/image', '/zed/zed_node/depth/depth_registered')]),

        # Visual odometry
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            condition=UnlessCondition(use_zed_odometry),
            parameters=parameters,
            remappings=remappings,),

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
