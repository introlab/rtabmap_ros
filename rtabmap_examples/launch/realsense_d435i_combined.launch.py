# Requirements:
#   A realsense D435i
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
# Example:
#   $ ros2 launch rtabmap_examples realsense_d435i_combined.launch.py
#
# Description: In this example, we feed visual odometry with IR stereo images 
#              for better pose estimation while seding RGB-D data to slam for 
#              a colored map.
#
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    vo_parameters={
          'frame_id':'camera_link',
          'wait_imu_to_init':True}

    vo_remappings=[
          ('imu', '/imu/data'),
          ('left/image_rect', '/camera/infra1/image_rect_raw'),
          ('left/camera_info', '/camera/infra1/camera_info'),
          ('right/image_rect', '/camera/infra2/image_rect_raw'),
          ('right/camera_info', '/camera/infra2/camera_info')]
    
    slam_parameters={
          'frame_id':'camera_link',
          'subscribe_depth':True,
          'subscribe_odom_info':True,
          'approx_sync':False}

    slam_remappings=[
          ('imu', '/imu/data'),
          ('rgb/image', '/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/aligned_depth_to_color/image_raw')]

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'unite_imu_method', default_value='2',
            description='0-None, 1-copy, 2-linear_interpolation. Use unite_imu_method:="1" if imu topics stop being published.'),

        #Hack to disable IR emitter
        SetParameter(name='depth_module.emitter_enabled', value=0),
        
        DeclareLaunchArgument(
            'args', default_value='',
            description='Extra arguments set to rtabmap and odometry nodes.'),
        
        DeclareLaunchArgument(
            'odom_args', default_value='',
            description='Extra arguments just for odometry node. If the same argument is already set in \"args\", it will be overwritten by the one in \"odom_args\".'),


        # Launch camera driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('realsense2_camera'), 'launch'),
                '/rs_launch.py']),
                launch_arguments={'camera_namespace': '',
                                  'enable_gyro': 'true',
                                  'enable_accel': 'true',
                                  'unite_imu_method': LaunchConfiguration('unite_imu_method'),
                                  'enable_infra1': 'true',
                                  'enable_infra2': 'true',
                                  'align_depth.enable': 'true',
                                  'enable_sync': 'true',
                                  'rgb_camera.profile': '640x360x30'}.items(),
        ),

        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            parameters=[vo_parameters],
            arguments=[LaunchConfiguration("args"), LaunchConfiguration("odom_args")],
            remappings=vo_remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[slam_parameters],
            remappings=slam_remappings,
            arguments=['-d', LaunchConfiguration("args")]),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[slam_parameters,
                        {'odometry_node_name': "stereo_odometry"}],
            remappings=slam_remappings),
                
        # Compute quaternion of the IMU
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame':'enu', 
                         'publish_tf':False}],
            remappings=[('imu/data_raw', '/camera/imu')]),
    ])
