# Description:
#   In this example, we will record ALL lidar scans. An IMU or low latency odometry is required for this example.
# 
# Example:
#   Launch your lidar sensor:
#   $ ros2 launch velodyne_driver velodyne_driver_node-VLP16-launch.py
#   $ ros2 launch velodyne_pointcloud velodyne_transform_node-VLP16-launch.py
#   
#   Launch your IMU sensor, make sure TF between lidar/base frame and imu is already calibrated.
#     In this example, we assume the imu topic has 
#     already the orientation estimated, if not, you can launch 
#     imu_filter_madgwick_node (with use_mag:=false publish_tf:=false)
#     and set imu_topic to output topic of the filter.
#
#   If a camera is used, make sure TF between lidar/base frame and camera is
#     already calibrated. To provide image data to this example, you should use
#     rtabmap_sync's rgbd_sync or stereo_sync node.
#
#   Launch the example by adjusting the lidar topic, imu topic and base frame:
#   $ ros2 launch rtabmap_examples lidar3d.launch.py lidar_topic:=/velodyne_points imu_topic:=/imu/data frame_id:=velodyne

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context: LaunchContext, *args, **kwargs):
  
  frame_id = LaunchConfiguration('frame_id')

  external_odom_frame_id =  LaunchConfiguration('external_odom_frame_id').perform(context)

  fixed_frame_from_imu = False
  fixed_frame_id =  LaunchConfiguration('fixed_frame_id').perform(context)
  if not fixed_frame_id:
    if external_odom_frame_id:
      fixed_frame_id = external_odom_frame_id
    else:
      fixed_frame_from_imu = True
      fixed_frame_id = frame_id.perform(context) + "_stabilized"
  
  imu_topic = LaunchConfiguration('imu_topic')
  
  rgbd_image_topic = LaunchConfiguration('rgbd_image_topic')
  rgbd_image_used =  rgbd_image_topic.perform(context) != ''
  
  lidar_topic = LaunchConfiguration('lidar_topic')
  lidar_topic_value = lidar_topic.perform(context)
  lidar_topic_deskewed = lidar_topic_value + "/deskewed"
  
  voxel_size = LaunchConfiguration('voxel_size')
  voxel_size_value = float(voxel_size.perform(context))
  
  use_sim_time = LaunchConfiguration('use_sim_time')
  
  localization = LaunchConfiguration('localization').perform(context)
  localization = localization == 'true' or localization == 'True'
  
  deskewing_slerp = LaunchConfiguration('deskewing_slerp').perform(context)
  deskewing_slerp = deskewing_slerp == 'true' or deskewing_slerp == 'True'
  
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
    'wait_imu_to_init': True,
    'odom_frame_id': 'icp_odom',
    'guess_frame_id': fixed_frame_id,
    # RTAB-Map's internal parameters are strings:
    'Odom/ScanKeyFrameThr': '0.4',
    'OdomF2M/ScanSubtractRadius': str(voxel_size_value),
    'OdomF2M/ScanMaxSize': '15000',
    'OdomF2M/BundleAdjustment': 'false',
    'Icp/CorrespondenceRatio': '0.01'
  }

  rtabmap_parameters = {
    'subscribe_depth': False,
    'subscribe_rgb': False,
    'subscribe_odom_info': not external_odom_frame_id,
    'subscribe_scan_cloud': True,
    'odom_frame_id': (external_odom_frame_id if external_odom_frame_id else ""),
    'odom_sensor_sync': True, # This will adjust camera position based on difference between lidar and camera stamps.
    # RTAB-Map's internal parameters are strings:
    'Rtabmap/DetectionRate': '0', # indirectly set to 1 Hz by the assembling time below (1s)
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
  
  remappings = [('imu', imu_topic),
                ('odom', 'icp_odom')]
  if rgbd_image_used:
    remappings.append(('rgbd_image', LaunchConfiguration('rgbd_image_topic')))
    
  arguments = []
  if localization:
    rtabmap_parameters['Mem/IncrementalMemory'] = 'False'
    rtabmap_parameters['Mem/InitWMWithAllNodes'] = 'True'
  else:
    arguments.append('-d') # This will delete the previous database (~/.ros/rtabmap.db)
    
  if external_odom_frame_id:
    viz_topic = lidar_topic_deskewed
  else:
    viz_topic = 'odom_filtered_input_scan'
  
  nodes = [
    # Lidar deskewing
    Node(
      package='rtabmap_util', executable='lidar_deskewing', output='screen',
      parameters=[{
        'use_sim_time': use_sim_time,
        'fixed_frame_id': fixed_frame_id,
        'wait_for_transform': 0.2,
        'slerp': deskewing_slerp}],
      remappings=[
          ('input_cloud', lidar_topic)
      ]),
    
    # Assemble deskewed scans based on icp odometry
    Node(
      package='rtabmap_util', executable='point_cloud_assembler', output='screen',
      parameters=[{
        'use_sim_time': use_sim_time,
        'assembling_time': LaunchConfiguration('assembling_time'), 
        'fixed_frame_id': (external_odom_frame_id if external_odom_frame_id else "")}], # This will make the node subscribing to icp odometry topic "icp_odom"
      remappings=[('cloud', lidar_topic_deskewed),
                  ('odom', 'icp_odom')]),
    
    # Update the map
    Node(
      package='rtabmap_slam', executable='rtabmap', output='screen',
      parameters=[shared_parameters, rtabmap_parameters,
                  {'subscribe_rgbd': rgbd_image_used,
                   'topic_queue_size': 40,
                   'sync_queue_size': 40,}],
      remappings=remappings + [('scan_cloud', 'assembled_cloud'), ('gps/fix', LaunchConfiguration('gps_topic'))],
      arguments=arguments), 

    # Just for visualization
    Node(
      package='rtabmap_viz', executable='rtabmap_viz', output='screen',
      parameters=[shared_parameters, rtabmap_parameters],
      remappings=remappings + [('scan_cloud', viz_topic)])
  ]
  
  if not external_odom_frame_id:
    # Lidar odometry
    nodes.append(
      Node(
        package='rtabmap_odom', executable='icp_odometry', output='screen',
        parameters=[shared_parameters, icp_odometry_parameters],
        remappings=remappings + [('scan_cloud', lidar_topic_deskewed)]))
  
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
  
  return nodes
  
def generate_launch_description():
  return LaunchDescription([

    # Launch arguments
    DeclareLaunchArgument(
      'use_sim_time', default_value='false',
      description='Use simulated clock.'),
    
    DeclareLaunchArgument(
      'frame_id', default_value='velodyne',
      description='Base frame of the robot.'),
    
    DeclareLaunchArgument(
      'fixed_frame_id', default_value='',
      description='Fixed frame used for lidar deskewing. If not set, we will generate one from IMU or external_odom_frame_id if not null.'),
    
    DeclareLaunchArgument(
      'external_odom_frame_id', default_value='',
      description='Provide external odometry with TF, disabling icp_odometry.'),
    
    DeclareLaunchArgument(
      'localization', default_value='false',
      description='Localization mode.'),

    DeclareLaunchArgument(
      'lidar_topic', default_value='/velodyne_points',
      description='Name of the lidar PointCloud2 topic.'),

    DeclareLaunchArgument(
      'imu_topic', default_value='/imu/data',
      description='Name of an IMU topic.'),
    
    DeclareLaunchArgument(
      'gps_topic', default_value='/gps/fix',
      description='Name of a GPS topic.'),
    
    DeclareLaunchArgument(
      'rgbd_image_topic', default_value='',
      description='RGBD image topic (ignored if empty). Would be the output of a rtabmap_sync\'s rgbd_sync, stereo_sync or rgb_sync node.'),
    
    DeclareLaunchArgument(
      'voxel_size', default_value='0.1',
      description='Voxel size (m) of the downsampled lidar point cloud. For indoor, set it between 0.1 and 0.3. For outdoor, set it to 0.5 or over.'),
    
    DeclareLaunchArgument(
      'min_loop_closure_overlap', default_value='0.2',
      description='Minimum scan overlap pourcentage to accept a loop closure.'),
    
    DeclareLaunchArgument(
      'expected_update_rate', default_value='15.0',
      description='Expected lidar frame rate. Ideally, set it slightly higher than actual frame rate, like 15 Hz for 10 Hz lidar scans.'),
    
    DeclareLaunchArgument(
      'assembling_time', default_value='1.0',
      description='How much time (sec) we assemble lidar scans before sending them to mapping node.'),

    DeclareLaunchArgument(
      'deskewing_slerp', default_value='true',
      description='Use fast slerp interpolation between first and last stamps of the scan for deskewing. It would less accruate than requesting TF for every points, but a lot faster. Enable this if the delay of the deskewed scan is significant larger than the original scan.'),

    DeclareLaunchArgument(
      'qos', default_value='1',
      description='Quality of Service: 0=system default, 1=reliable, 2=best effort.'),

    OpaqueFunction(function=launch_setup),
  ])

    
