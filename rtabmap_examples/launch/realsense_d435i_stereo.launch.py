# Requirements:
#   A realsense D435i
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
# Example:
#   $ ros2 launch rtabmap_examples realsense_d435i_stereo.launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    parameters=[{
          'frame_id':'camera_link',
          'subscribe_stereo':True,
          'subscribe_odom_info':True,
          'wait_imu_to_init':True}]

    remappings=[
          ('imu', '/imu/data'),
          ('left/image_rect', '/camera/infra1/image_rect_raw'),
          ('left/camera_info', '/camera/infra1/camera_info'),
          ('right/image_rect', '/camera/infra2/image_rect_raw'),
          ('right/camera_info', '/camera/infra2/camera_info')]

    return LaunchDescription([

        #Hack to disable IR emitter
        SetParameter(name='depth_module.emitter_enabled', value=0),

        # Launch camera driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('realsense2_camera'), 'launch'),
                '/rs_launch.py']),
                launch_arguments={'enable_gyro': 'true',
                                  'enable_accel': 'true',
                                  'unite_imu_method': '1',
                                  'enable_infra1': 'true',
                                  'enable_infra2': 'true',
                                  'enable_sync': 'true'}.items(),
        ),

        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings),
                
        # Compute quaternion of the IMU
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame':'enu', 
                         'publish_tf':False}],
            remappings=[('imu/data_raw', '/camera/imu')]),
        
        # The IMU frame is missing in TF tree, add it:
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'camera_gyro_optical_frame', 'camera_imu_optical_frame']),
    ])
