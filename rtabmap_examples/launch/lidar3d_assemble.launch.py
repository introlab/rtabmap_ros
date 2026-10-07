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
#     rtabmap_sync's rgbd_sync or stereo_sync node. For a camera without depth, use
#     rgb_sync and set gen_depth:=true to make its depth from the lidar.
#
#   Launch the example by adjusting the lidar topic, imu topic and base frame:
#   $ ros2 launch rtabmap_examples lidar3d.launch.py lidar_topic:=/velodyne_points imu_topic:=/imu/data frame_id:=velodyne
#
#   For a complete example with a recorded bag (Ouster on a rotating mast, IMU and
#   a camera without depth), see rtabmap_demos' netherdrone_lidar3d_demo.launch.py.

import os

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
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
  rgbd_images_topic = LaunchConfiguration('rgbd_images_topic')
  rgbd_image_used =  rgbd_image_topic.perform(context) != '' or rgbd_images_topic.perform(context) != ''
  rgbd_cameras = 0 if rgbd_images_topic.perform(context) != '' else 1
  
  lidar_topic = LaunchConfiguration('lidar_topic')
  lidar_topic_value = lidar_topic.perform(context)
  lidar_topic_deskewed = lidar_topic_value + "/deskewed"
  
  voxel_size = LaunchConfiguration('voxel_size')
  voxel_size_value = float(voxel_size.perform(context))

  lidar_range_min = float(LaunchConfiguration('lidar_range_min').perform(context))
  
  use_sim_time = LaunchConfiguration('use_sim_time')
  
  localization = LaunchConfiguration('localization').perform(context)
  localization = localization == 'true' or localization == 'True'
  
  deskewing_slerp = LaunchConfiguration('deskewing_slerp').perform(context)
  deskewing_slerp = deskewing_slerp == 'true' or deskewing_slerp == 'True'
  
  max_correspondence_distance = LaunchConfiguration('max_correspondence_distance').perform(context)
  if max_correspondence_distance:
    max_correspondence_distance = float(max_correspondence_distance)
  else:
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
    'Icp/OutlierRatio': LaunchConfiguration('icp_outlier_ratio').perform(context),
  }

  icp_odometry_parameters = {
    'expected_update_rate': LaunchConfiguration('expected_update_rate'),
    'wait_imu_to_init': True,
    'odom_frame_id': 'icp_odom',
    'guess_frame_id': fixed_frame_id,
    'scan_range_min': lidar_range_min,
    # RTAB-Map's internal parameters are strings:
    'Odom/ScanKeyFrameThr': LaunchConfiguration('odom_key_frame_threshold').perform(context),
    'OdomF2M/ScanSubtractRadius': str(voxel_size_value),
    'OdomF2M/ScanMaxSize': LaunchConfiguration('odom_local_map_size').perform(context),
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
    'RGBD/ProximityAngle': '0', # assuming 360 lidar
    'RGBD/AngularUpdate': '0.05',
    'RGBD/LinearUpdate': '0.05',
    'RGBD/CreateOccupancyGrid': 'false',
    'Mem/NotLinkedNodesKept': 'false',
    'Mem/STMSize': '30',
    'Reg/Strategy': '1',
    'Icp/CorrespondenceRatio': str(LaunchConfiguration('min_loop_closure_overlap').perform(context))
  }

  # Only for the camera, if any
  camera_parameters = {
    'gen_depth': LaunchConfiguration('gen_depth').perform(context).lower() == 'true',
    'gen_depth_decimation': int(LaunchConfiguration('gen_depth_decimation').perform(context)),
    'gen_depth_fill_holes_size': int(LaunchConfiguration('gen_depth_fill_holes_size').perform(context)),
    'Rtabmap/ImagesAlreadyRectified': str(LaunchConfiguration('rectify_images').perform(context).lower() != 'true').lower(),
  }
  if camera_parameters['gen_depth']:
    # The depth made from the lidar is 32 bits float: save it in 16 bits (mm), the
    # format compressed depth images use.
    camera_parameters['Mem/SaveDepth16Format'] = 'true'

  database_parameters = {
  }
  
  remappings = [('imu', imu_topic),
                ('odom', 'icp_odom')]
  if rgbd_image_used:
    if rgbd_cameras == 1:
      remappings.append(('rgbd_image', LaunchConfiguration('rgbd_image_topic')))
    else:
      remappings.append(('rgbd_images', LaunchConfiguration('rgbd_images_topic')))
    
  intermediate_nodes = LaunchConfiguration('intermediate_nodes').perform(context).lower() == 'true'
  if intermediate_nodes and not external_odom_frame_id:
    # Every odometry pose between the nodes is saved as a node without data.
    rtabmap_parameters['Rtabmap/CreateIntermediateNodes'] = 'true'

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
        'slerp': deskewing_slerp,
        'qos': LaunchConfiguration('qos')}],
      remappings=[
          ('input_cloud', lidar_topic)
      ]),
    
    # Assemble deskewed scans based on icp odometry
    Node(
      package='rtabmap_util', executable='point_cloud_assembler', output='screen',
      parameters=[{
        'use_sim_time': use_sim_time,
        'assembling_time': LaunchConfiguration('assembling_time'), 
        'range_min': lidar_range_min,
        'voxel_size': float(LaunchConfiguration('assembler_voxel_size').perform(context)),
        'qos': LaunchConfiguration('qos'),
        'qos_odom': LaunchConfiguration('qos'),
        'fixed_frame_id': (external_odom_frame_id if external_odom_frame_id else "")}], # This will make the node subscribing to icp odometry topic "icp_odom"
      remappings=[('cloud', lidar_topic_deskewed),
                  ('odom', 'icp_odom')]),
    
    # Update the map
    Node(
      package='rtabmap_slam', executable='rtabmap', output='screen',
      parameters=[shared_parameters, rtabmap_parameters, database_parameters, camera_parameters,
                  {'subscribe_rgbd': rgbd_image_used,
                   'rgbd_cameras': rgbd_cameras,
                   'topic_queue_size': 40,
                   'sync_queue_size': 40,}],
      remappings=remappings + [('scan_cloud', 'assembled_cloud'), ('gps/fix', LaunchConfiguration('gps_topic')),
                               ('inter_odom', 'icp_odom')],
      arguments=arguments), 

    # Just for visualization
    Node(
      condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
      package='rtabmap_viz', executable='rtabmap_viz', output='screen',
      parameters=[shared_parameters, rtabmap_parameters,
                  {'odometry_node_name': "icp_odometry"}],
      remappings=remappings + [('scan_cloud', viz_topic)],
      arguments=['-d', LaunchConfiguration('rtabmap_viz_cfg')])
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
          'wait_for_transform_duration': 0.001,
          'qos': LaunchConfiguration('qos')}],
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
      'rgbd_images_topic', default_value='',
      description='RGBD images topic (ignored if empty, override "rgbd_image_topic" if set). Would be the output of a rtabmap_sync\'s rgbdx_sync node.'),
    
    DeclareLaunchArgument(
      'gen_depth', default_value='false',
      description='With a camera without depth (rgbd_image_topic from rgb_sync), make its depth image by projecting the assembled lidar cloud into it, so that its visual features get 3D positions.'),

    DeclareLaunchArgument(
      'gen_depth_decimation', default_value='4',
      description='Resolution divider of the depth made by gen_depth; it must divide the image size. Lidar points are sparse in a full resolution image anyway.'),

    DeclareLaunchArgument(
      'gen_depth_fill_holes_size', default_value='2',
      description='Fill holes up to this many pixels (after decimation) in the depth made by gen_depth, between lidar points; 0 disables.'),

    DeclareLaunchArgument(
      'rectify_images', default_value='false',
      description='The camera images are not rectified: rectify them with their camera_info before use.'),

    DeclareLaunchArgument(
      'voxel_size', default_value='0.1',
      description='Voxel size (m) of the downsampled lidar point cloud. For indoor, set it between 0.1 and 0.3. For outdoor, set it to 0.5 or over.'),
    
    DeclareLaunchArgument(
      'max_correspondence_distance', default_value='',
      description='Maximum distance (m) between ICP correspondences. Empty: 10 times voxel_size.'),

    DeclareLaunchArgument(
      'icp_outlier_ratio', default_value='0.7',
      description='Icp/OutlierRatio: expected ratio of outliers between scans.'),

    DeclareLaunchArgument(
      'odom_key_frame_threshold', default_value='0.4',
      description='Odom/ScanKeyFrameThr: a new scan is added to the odometry local map when its overlap with it is below this ratio.'),

    DeclareLaunchArgument(
      'odom_local_map_size', default_value='15000',
      description='OdomF2M/ScanMaxSize: maximum number of points of the odometry local map.'),

    DeclareLaunchArgument(
      'assembler_voxel_size', default_value='0.0',
      description='Voxel size (m) of the clouds assembled for the map; 0 keeps every point.'),

    DeclareLaunchArgument(
      'lidar_range_min', default_value='0.0',
      description='Lidar points closer than this (m) are ignored, by odometry and in the map; 0 keeps them all. Set it to remove hits on the robot itself.'),

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
      'intermediate_nodes', default_value='false',
      description='Also save every odometry pose between the map\'s nodes in the database, as nodes without data (Rtabmap/CreateIntermediateNodes). Only with icp_odometry (external_odom_frame_id empty).'),

    DeclareLaunchArgument(
      'qos', default_value='1',
      description='Quality of Service: 0=system default, 1=reliable, 2=best effort.'),

    DeclareLaunchArgument(
      'rtabmap_viz', default_value='true',
      description='Launch RTAB-Map UI.'),

    DeclareLaunchArgument(
      'rtabmap_viz_cfg', default_value='~/.ros/rtabmapGUI.ini',
      description='Configuration file of rtabmap_viz, where it also saves its settings.'),




    OpaqueFunction(function=launch_setup),
  ])

    
