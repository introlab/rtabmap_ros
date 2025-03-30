# Description:
#   In this example, we keep only minimal data to do LiDAR SLAM.
#
# Example:
#   Launch your lidar sensor:
#   $ ros2 launch velodyne_driver velodyne_driver_node-VLP16-launch.py
#   $ ros2 launch velodyne_pointcloud velodyne_transform_node-VLP16-launch.py
#
#   If an IMU is used, make sure TF between lidar/base frame and imu is
#     already calibrated. In this example, we assume the imu topic has 
#     already the orientation estimated, if not, you can use 
#     imu_filter_madgwick_node (with use_mag:=false publish_tf:=false)
#     and set imu_topic to output topic of the filter.
#
#   If a camera is used, make sure TF between lidar/base frame and camera is
#     already calibrated. To provide image data to this example, you should use
#     rtabmap_sync's rgbd_sync or stereo_sync node.
#
#   Launch the example by adjusting the lidar topic and base frame:
#   $ ros2 launch rtabmap_examples lidar3d.launch.py lidar_topic:=/velodyne_points frame_id:=velodyne

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context: LaunchContext, *args, **kwargs):
  
  frame_id = LaunchConfiguration('frame_id')
  
  imu_topic = LaunchConfiguration('imu_topic')
  imu_used =  imu_topic.perform(context) != ''
  
  rgbd_image_topic = LaunchConfiguration('rgbd_image_topic')
  rgbd_image_used =  rgbd_image_topic.perform(context) != ''
  
  voxel_size = LaunchConfiguration('voxel_size')
  voxel_size_value = float(voxel_size.perform(context))
  
  use_sim_time = LaunchConfiguration('use_sim_time')
  
  lidar_topic = LaunchConfiguration('lidar_topic')
  lidar_topic_value = lidar_topic.perform(context)
  lidar_topic_deskewed = lidar_topic_value + "/deskewed"
  
  localization = LaunchConfiguration('localization').perform(context)
  localization = localization == 'true' or localization == 'True'
  
  deskewing_slerp = LaunchConfiguration('deskewing_slerp').perform(context)
  deskewing_slerp = deskewing_slerp == 'true' or deskewing_slerp == 'True'
  
  fixed_frame_from_imu = False
  fixed_frame_id =  LaunchConfiguration('fixed_frame_id').perform(context)
  if not fixed_frame_id and imu_used:
    fixed_frame_from_imu = True
    fixed_frame_id = frame_id.perform(context) + "_stabilized"
  
  if not fixed_frame_id:
    lidar_topic_deskewed = lidar_topic
  
  # Rule of thumb:
  max_correspondence_distance = voxel_size_value * 10.0

  shared_parameters = {
    'use_sim_time': use_sim_time,
    'frame_id': frame_id,
    'qos': LaunchConfiguration('qos'),
    'approx_sync': rgbd_image_used,
    'wait_for_transform': 0.2,
    # RTAB-Map's internal parameters are strings:
    'Icp/PointToPlane': 'true',
    'Icp/Iterations': '10',
    'Icp/VoxelSize': str(voxel_size_value),
    'Icp/Epsilon': '0.001',
    'Icp/PointToPlaneK': '20',
    'Icp/PointToPlaneRadius': '0',
    'Icp/MaxTranslation': '3',
    'Icp/MaxCorrespondenceDistance': str(max_correspondence_distance),
    'Icp/Strategy': '1',
    'Icp/OutlierRatio': '0.7',
  }
  
  icp_odometry_parameters = {
    'expected_update_rate': LaunchConfiguration('expected_update_rate'),
    'deskewing': not fixed_frame_id, # If fixed_frame_id is set, we do deskewing externally below
    'odom_frame_id': 'icp_odom',
    'guess_frame_id': fixed_frame_id,
    'deskewing_slerp': deskewing_slerp,
    # RTAB-Map's internal parameters are strings:
    'Odom/ScanKeyFrameThr': '0.4',
    'OdomF2M/ScanSubtractRadius': str(voxel_size_value),
    'OdomF2M/ScanMaxSize': '15000',
    'OdomF2M/BundleAdjustment': 'false',
    'Icp/CorrespondenceRatio': '0.01'
  }
  if imu_used:
    icp_odometry_parameters['wait_imu_to_init'] = True

  rtabmap_parameters = {
    'subscribe_depth': False,
    'subscribe_rgb': False,
    'subscribe_odom_info': True,
    'subscribe_scan_cloud': True,
    # RTAB-Map's internal parameters are strings:
    'RGBD/ProximityMaxGraphDepth': '0',
    'RGBD/ProximityPathMaxNeighbors': '1',
    'RGBD/AngularUpdate': '0.05',
    'RGBD/LinearUpdate': '0.05',
    'RGBD/CreateOccupancyGrid': 'false',
    'Mem/NotLinkedNodesKept': 'false',
    'Mem/STMSize': '30',
    'Reg/Strategy': '1',
    'Icp/CorrespondenceRatio': LaunchConfiguration('min_loop_closure_overlap')
  }
  
  arguments = []
  if localization:
    rtabmap_parameters['Mem/IncrementalMemory'] = 'False'
    rtabmap_parameters['Mem/InitWMWithAllNodes'] = 'True'
  else:
    arguments.append('-d') # This will delete the previous database (~/.ros/rtabmap.db)
  
  remappings = [('odom', 'icp_odom')]
  if imu_used:
    remappings.append(('imu', LaunchConfiguration('imu_topic')))
  else:
    remappings.append(('imu', 'imu_not_used'))
  if rgbd_image_used:
    remappings.append(('rgbd_image', LaunchConfiguration('rgbd_image_topic')))
  
  nodes = [
    Node(
      package='rtabmap_odom', executable='icp_odometry', output='screen',
      parameters=[shared_parameters, icp_odometry_parameters],
      remappings=remappings + [('scan_cloud', lidar_topic_deskewed)]),
    
    Node(
      package='rtabmap_slam', executable='rtabmap', output='screen',
      parameters=[shared_parameters, rtabmap_parameters, {'subscribe_rgbd': rgbd_image_used}],
      remappings=remappings + [('scan_cloud', lidar_topic_deskewed)],
      arguments=arguments), 
  
    Node(
      package='rtabmap_viz', executable='rtabmap_viz', output='screen',
      parameters=[shared_parameters, rtabmap_parameters],
      remappings=remappings + [('scan_cloud', 'odom_filtered_input_scan')])
  ]
  
  if fixed_frame_from_imu:
    # Create a stabilized base frame based on imu for lidar deskewing
    nodes.append(
      Node(
        package='rtabmap_util', executable='imu_to_tf', output='screen',
        parameters=[{
          'use_sim_time': use_sim_time,
          'fixed_frame_id': fixed_frame_id,
          'base_frame_id': frame_id,
          'wait_for_transform_duration': 0.001}],
        remappings=[('imu/data', imu_topic)]))

  if fixed_frame_id:
    # Lidar deskewing
    nodes.append(
      Node(
        package='rtabmap_util', executable='lidar_deskewing', output='screen',
        parameters=[{
          'use_sim_time': use_sim_time,
          'fixed_frame_id': fixed_frame_id,
          'wait_for_transform': 0.2,
          'slerp': deskewing_slerp}],
        remappings=[
            ('input_cloud', lidar_topic)
        ])
    )
      
  return nodes
  
def generate_launch_description():
  return LaunchDescription([

    # Launch arguments
    DeclareLaunchArgument(
      'use_sim_time', default_value='false',
      description='Use simulated clock.'),
    
    DeclareLaunchArgument(
      'deskewing', default_value='true',
      description='Enable lidar deskewing.'),
    
    DeclareLaunchArgument(
      'frame_id', default_value='velodyne',
      description='Base frame of the robot.'),
    
    DeclareLaunchArgument(
      'fixed_frame_id', default_value='',
      description='Fixed frame used for lidar deskewing. If not set, we will generate one from IMU.'),
    
    DeclareLaunchArgument(
      'localization', default_value='false',
      description='Localization mode.'),

    DeclareLaunchArgument(
      'lidar_topic', default_value='/velodyne_points',
      description='Name of the lidar PointCloud2 topic.'),

    DeclareLaunchArgument(
      'imu_topic', default_value='',
      description='IMU topic (ignored if empty).'),
    
    DeclareLaunchArgument(
      'rgbd_image_topic', default_value='',
      description='RGBD image topic (ignored if empty). Would be the output of a rtabmap_sync\'s rgbd_sync, stereo_sync or rgb_sync node.'),
    
    DeclareLaunchArgument(
      'expected_update_rate', default_value='15.0',
      description='Expected lidar frame rate. Ideally, set it slightly higher than actual frame rate, like 15 Hz for 10 Hz lidar scans.'),
    
    DeclareLaunchArgument(
      'voxel_size', default_value='0.1',
      description='Voxel size (m) of the downsampled lidar point cloud. For indoor, set it between 0.1 and 0.3. For outdoor, set it to 0.5 or over.'),
    
    DeclareLaunchArgument(
      'min_loop_closure_overlap', default_value='0.2',
      description='Minimum scan overlap pourcentage to accept a loop closure.'),
    
    DeclareLaunchArgument(
      'deskewing_slerp', default_value='true',
      description='Use fast slerp interpolation between first and last stamps of the scan for deskewing. It would less accruate than requesting TF for every points, but a lot faster. Enable this if the delay of the deskewed scan is significant larger than the original scan.'),

    DeclareLaunchArgument(
      'qos', default_value='1',
      description='Quality of Service: 0=system default, 1=reliable, 2=best effort.'),

    OpaqueFunction(function=launch_setup),
  ])

    
