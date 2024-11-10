# Example:
#   $ ros2 launch rtabmap_examples vlp16_imu.launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context, *args, **kwargs):      
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    deskewing = LaunchConfiguration('deskewing')
    qos = LaunchConfiguration('qos')
    scan_topic = LaunchConfiguration('scan_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    imu_filtered_topic = ''.join([imu_topic.perform(context), "/filtered"])
    frame_id = LaunchConfiguration('frame_id')
    frame_id_stabilized = ''.join([frame_id.perform(context), "_stabilized"])
    
    return [
        # Hardware specific nodes - BEGIN
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
        
        # The IMU driver
        Node(package='imu_brick', executable='imu_brick_node'),
        
        # Static transform between velodyne and imu frame
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=["0", "0", "0.05", "-1.57", "0", "0", "velodyne", "imu_link"]),
        # Hardware specific nodes - END
        
        # Nodes below should work "as is" with different hardware
        
        # You can skip the imu filter node if your imu data has already the orientation computed
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{
              'use_sim_time':use_sim_time,
              'use_mag':False,
              'world_frame':'enu',
              'publish_tf':False}],
            remappings=[
                ('imu/data_raw', imu_topic),
                ('imu/data', imu_filtered_topic)
            ]),

        Node(
            package='rtabmap_util', executable='imu_to_tf', output='screen',
            parameters=[{
              'use_sim_time':use_sim_time,
              'fixed_frame_id':frame_id_stabilized,
              'base_frame_id':frame_id,
              'wait_for_transform_duration': 0.001}],
            remappings=[
                ('imu/data', imu_filtered_topic)
            ]),

        Node(
            package='rtabmap_odom', executable='icp_odometry', output='screen',
            parameters=[{
              'frame_id':frame_id,
              'odom_frame_id':'odom',
              'guess_frame_id':frame_id_stabilized,
              'wait_for_transform':0.2,
              'expected_update_rate':15.0,
              'deskewing':deskewing,
              'use_sim_time':use_sim_time,
              'wait_imu_to_init': True,
              'qos':qos,
              # RTAB-Map's internal parameters are strings:
              'Icp/PointToPlane': 'true',
              'Icp/Iterations': '10',
              'Icp/VoxelSize': '0.1',
              'Icp/Epsilon': '0.001',
              'Icp/PointToPlaneK': '20',
              'Icp/PointToPlaneRadius': '0',
              'Icp/MaxTranslation': '2',
              'Icp/MaxCorrespondenceDistance': '1',
              'Icp/Strategy': '1',
              'Icp/OutlierRatio': '0.7',
              'Icp/CorrespondenceRatio': '0.01',
              'Odom/ScanKeyFrameThr': '0.4',
              'OdomF2M/ScanSubtractRadius': '0.1',
              'OdomF2M/ScanMaxSize': '15000',
              'OdomF2M/BundleAdjustment': 'false'
            }],
            remappings=[
              ('scan_cloud', scan_topic),
              ('imu', imu_filtered_topic)
            ]),
        
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[{
              'frame_id':frame_id,
              'subscribe_depth':False,
              'subscribe_rgb':False,
              'subscribe_scan_cloud':True,
              'approx_sync':False,
              'wait_for_transform':0.2,
              'use_sim_time':use_sim_time,
              'qos':qos,
              # RTAB-Map's internal parameters are strings:
              'RGBD/ProximityMaxGraphDepth': '0',
              'RGBD/ProximityPathMaxNeighbors': '1',
              'RGBD/AngularUpdate': '0.05',
              'RGBD/LinearUpdate': '0.05',
              'RGBD/CreateOccupancyGrid': 'false',
              'Mem/NotLinkedNodesKept': 'false',
              'Mem/STMSize': '30',
              'Mem/LaserScanNormalK': '20',
              'Reg/Strategy': '1',
              'Icp/VoxelSize': '0.1',
              'Icp/PointToPlaneK': '20',
              'Icp/PointToPlaneRadius': '0',
              'Icp/PointToPlane': 'true',
              'Icp/Iterations': '10',
              'Icp/Epsilon': '0.001',
              'Icp/MaxTranslation': '3',
              'Icp/MaxCorrespondenceDistance': '1',
              'Icp/Strategy': '1',
              'Icp/OutlierRatio': '0.7',
              'Icp/CorrespondenceRatio': '0.2'
            }],
            remappings=[
              ('scan_cloud', 'odom_filtered_input_scan'),
              ('imu', imu_filtered_topic)
            ],
            arguments=[
              '-d' # This will delete the previous database (~/.ros/rtabmap.db)
            ]), 
      
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[{
              'frame_id':frame_id,
              'odom_frame_id':'odom',
              'subscribe_odom_info':True,
              'subscribe_scan_cloud':True,
              'approx_sync':False,
              'use_sim_time':use_sim_time,
              'qos':qos,
            }],
            remappings=[
                ('scan_cloud', 'odom_filtered_input_scan')
            ]),
      ]

def generate_launch_description():
    
    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'deskewing', default_value='true',
            description='Enable lidar deskewing'),
        
        DeclareLaunchArgument(
            'qos', default_value='1',
            description='Quality of Service: 0=system default, 1=reliable, 2=best effort'),
        
        DeclareLaunchArgument(
            'frame_id', default_value='velodyne',
            description='Base frame of the robot'),
        
        DeclareLaunchArgument(
            'imu_topic', default_value='/imu/data',
            description='Imu topic name'),
        
        DeclareLaunchArgument(
            'scan_topic', default_value='/velodyne_points',
            description='Scan point cloud topic name'),
        
        OpaqueFunction(function=launch_setup)
    ])
    
    
