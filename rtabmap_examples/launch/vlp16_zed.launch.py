# Example using zed odometry for lidar deskewing:
#   $ ros2 launch rtabmap_examples vlp16_zed.launch.py camera_model:=zed2i
#
# To use only zed's imu for deskewing:
#   $ ros2 launch rtabmap_examples vlp16_zed.launch.py camera_model:=zed2i use_zed_odometry:=false
#


import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import tempfile

def launch_setup(context: LaunchContext, *args, **kwargs):
  
  assemble = LaunchConfiguration('assemble').perform(context)
  assemble = assemble == 'true' or assemble == 'True'
  
  lidar3d_launch_file = 'lidar3d.launch.py'
  if assemble:
    lidar3d_launch_file = 'lidar3d_assemble.launch.py'
    
  use_zed_odometry = LaunchConfiguration('use_zed_odometry').perform(context)
  use_zed_odometry = use_zed_odometry == 'true' or use_zed_odometry == 'True'
  
  fixed_frame_id = ''
  if use_zed_odometry:
    fixed_frame_id = 'odom'
  
  # Hack to override grab_resolution parameter without changing any files
  with tempfile.NamedTemporaryFile(mode='w+t', delete=False) as zed_override_file:
      zed_override_file.write("---\n"+
                "/**:\n"+
                "    ros__parameters:\n"+
                "        general:\n"+
                "            grab_resolution: 'VGA'")

  return [
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('velodyne_driver'), 'launch'),
            '/velodyne_driver_node-VLP16-launch.py']),
    ),
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('velodyne_pointcloud'), 'launch'),
            '/velodyne_transform_node-VLP16-launch.py']),
    ),
    
    # Launch camera driver
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('zed_wrapper'), 'launch'),
            '/zed_camera.launch.py']),
            launch_arguments={'camera_model': LaunchConfiguration('camera_model'),
                                'ros_params_override_path': zed_override_file.name,
                                'publish_tf': LaunchConfiguration('use_zed_odometry'), # publish VIO frame
                                'publish_map_tf': 'false'}.items(),
    ),
    
    # Static transform between zed and velodyne frame (zed will be our base frame because VIO is already linked to it)
    Node(package='tf2_ros', executable='static_transform_publisher', arguments=["0", "0", "-0.05", "0", "0", "0", "zed_camera_link", "velodyne"]),
    
    # Sync rgb/depth/camera_info together
    Node(
        package='rtabmap_sync', executable='rgbd_sync', output='screen',
        parameters=[{'approx_sync': False}],
        remappings=[('rgb/image', '/zed/zed_node/rgb/image_rect_color'),
                    ('rgb/camera_info', '/zed/zed_node/rgb/camera_info'),
                    ('depth/image', '/zed/zed_node/depth/depth_registered')]),

    IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rtabmap_examples'), 'launch'),
            '/', lidar3d_launch_file]),
        launch_arguments={'voxel_size': LaunchConfiguration('voxel_size'),
                          'localization': LaunchConfiguration('localization'),
                          'frame_id': 'zed_camera_link',
                          'lidar_topic': 'velodyne_points',
                          'imu_topic': '/zed/zed_node/imu/data',
                          'rgbd_image_topic': 'rgbd_image',
                          'fixed_frame_id': fixed_frame_id}.items()),
  ]
  
def generate_launch_description():
  return LaunchDescription([
    # Launch arguments         
    DeclareLaunchArgument(
      'camera_model', default_value='',
      description="[REQUIRED] The model of the camera. Using a wrong camera model can disable camera features. Valid choices are: ['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'virtual']"),

    DeclareLaunchArgument(
      'use_zed_odometry', default_value='true',
      description='Use ZED\'s odometry for deskewing.'),
    
    DeclareLaunchArgument(
      'qos', default_value='1',
      description='Quality of Service: 0=system default, 1=reliable, 2=best effort'),

    DeclareLaunchArgument(
      'localization', default_value='false',
      description='Localization mode.'),

    DeclareLaunchArgument(
      'voxel_size', default_value='0.1',
      description='Voxel size (m) of the downsampled lidar point cloud. For indoor, set it between 0.1 and 0.3. For outdoor, set it to 0.5 or over.'),

    DeclareLaunchArgument(
      'assemble', default_value='false',
      description='Assemble ALL lidar scans.'),

    OpaqueFunction(function=launch_setup),
  ])