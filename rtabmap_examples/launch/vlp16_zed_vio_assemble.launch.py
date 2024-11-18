# Example:
#   $ ros2 launch rtabmap_examples vlp16_zed_vio_assemble.launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import tempfile

def generate_launch_description():
    
    qos = LaunchConfiguration('qos')
    
    # Hack to override grab_resolution parameter without changing any files
    with tempfile.NamedTemporaryFile(mode='w+t', delete=False) as zed_override_file:
        zed_override_file.write("---\n"+
                  "/**:\n"+
                  "    ros__parameters:\n"+
                  "        general:\n"+
                  "            grab_resolution: 'VGA'")
        
    shared_icp_parameters ={
        'Icp/PointToPlane': 'true',
        'Icp/Iterations': '10',
        'Icp/VoxelSize': '0.1',
        'Icp/Epsilon': '0.001',
        'Icp/PointToPlaneK': '20',
        'Icp/PointToPlaneRadius': '0',
        'Icp/MaxTranslation': '3',
        'Icp/MaxCorrespondenceDistance': '1',
        'Icp/Strategy': '1',
        'Icp/OutlierRatio': '0.7',
    }
    
    return LaunchDescription([
        
        # Launch arguments       
        DeclareLaunchArgument(
            'qos', default_value='1',
            description='Quality of Service: 0=system default, 1=reliable, 2=best effort'),
        
        DeclareLaunchArgument(
            'camera_model', default_value='',
            description="[REQUIRED] The model of the camera. Using a wrong camera model can disable camera features. Valid choices are: ['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'virtual']"),
        
        #################################
        # Hardware specific nodes - BEGIN
        #################################
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
                                    'publish_tf': 'true', # publish VIO frame
                                    'publish_map_tf': 'false'}.items(),
        ),
        
        # Static transform between zed and velodyne frame (zed will be our base frame because VIO is already linked to it)
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=["0", "0", "-0.05", "0", "0", "0", "zed_camera_link", "velodyne"]),
        #################################
        # Hardware specific nodes - END
        #################################
        
        # Nodes below should work "as is" with different hardware
        
        # Sync rgb/depth/camera_info together
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{'approx_sync': False}],
            remappings=[('rgb/image', '/zed/zed_node/rgb/image_rect_color'),
                        ('rgb/camera_info', '/zed/zed_node/rgb/camera_info'),
                        ('depth/image', '/zed/zed_node/depth/depth_registered')]),
        
        # External Deskewing using VIO
        Node(
            package='rtabmap_util', executable='lidar_deskewing', output='screen',
            parameters=[{
              'fixed_frame_id':'odom',
              'wait_for_transform':0.1}],
            remappings=[
                ('input_cloud', 'velodyne_points')
            ]),

        Node(
            package='rtabmap_odom', executable='icp_odometry', output='screen',
            parameters=[{
              'frame_id':'zed_camera_link',
              'odom_frame_id':'icp_odom',
              'guess_frame_id':'odom',
              'wait_for_transform':0.2,
              'expected_update_rate':15.0,
              'wait_imu_to_init': True,
              'qos':qos,
              # RTAB-Map's internal parameters are strings:
              'Odom/ScanKeyFrameThr': '0.4',
              'OdomF2M/ScanSubtractRadius': '0.1',
              'OdomF2M/ScanMaxSize': '15000',
              'OdomF2M/BundleAdjustment': 'false',
              'Icp/CorrespondenceRatio': '0.01'
            }, shared_icp_parameters],
            remappings=[
              ('scan_cloud', 'velodyne_points/deskewed'), # Subscribing to deskewed scan topic
              ('imu', '/zed/zed_node/imu/data'),
              ('odom', 'icp_odom')
            ]),
        
        #Assemble deskewed scans based on icp odometry
        Node(
            package='rtabmap_util', executable='point_cloud_assembler', output='screen',
            parameters=[{'assembling_time': 1.0, 
                         'fixed_frame_id': ""}],
            remappings=[('cloud', 'velodyne_points/deskewed'),
                        ('odom', 'icp_odom')]),
        
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[{
              'frame_id':'zed_camera_link',
              'subscribe_rgbd': True,
              'subscribe_depth':False,
              'subscribe_rgb':False,
              'subscribe_scan_cloud':True,
              'approx_sync':True,
              'wait_for_transform':0.2,
              'qos':qos,
              'topic_queue_size': 30,
              'sync_queue_size': 20,
              # RTAB-Map's internal parameters are strings:
              'Rtabmap/DetectionRate': '0', # Rate fixed by assembling time above (1 Hz)
              'RGBD/ProximityMaxGraphDepth': '0',
              'RGBD/ProximityPathMaxNeighbors': '1',
              'RGBD/AngularUpdate': '0.05',
              'RGBD/LinearUpdate': '0.05',
              'RGBD/CreateOccupancyGrid': 'false',
              'Mem/NotLinkedNodesKept': 'false',
              'Mem/STMSize': '30',
              'Mem/LaserScanNormalK': '20',
              'Reg/Strategy': '1',
              'Icp/CorrespondenceRatio': '0.2'
            }, shared_icp_parameters],
            remappings=[
              ('scan_cloud', 'assembled_cloud'), # We subscribe to assembled scans
              ('imu', '/zed/zed_node/imu/data'),
              ('odom', '/icp_odom')
            ],
            arguments=[
              '-d' # This will delete the previous database (~/.ros/rtabmap.db)
            ]), 
      
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[{
              'frame_id':'zed_camera_link',
              'odom_frame_id':'icp_odom',
              'subscribe_odom_info':True,
              'subscribe_scan_cloud':True,
              'approx_sync':False,
              'qos':qos,
            }],
            remappings=[
                ('scan_cloud', 'velodyne_points/deskewed'),
            ]),
      ]
    
    )