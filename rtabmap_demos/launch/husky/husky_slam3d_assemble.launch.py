#
#
# Example with gazebo:
#   1) Launch simulator (husky):
#     $ ros2 launch clearpath_gz simulation.launch.py
#     Click on "Play" button on bottom-left of gazebo as soon as you can see it to avoid controllers crashing after 5 sec.
#
#   2) Launch rviz:
#     $ ros2 launch clearpath_viz view_navigation.launch.py namespace:=a200_0000
#
#   3) Launch SLAM:
#     $ ros2 launch rtabmap_demos husky_slam3d_assemble.launch.py use_sim_time:=true
#
#   4) Launch nav2"
#     $ ros2 launch clearpath_nav2_demos nav2.launch.py setup_path:=$HOME/clearpath/ use_sim_time:=true
#
#   4) Click on "Play" button on bottom-left of gazebo.
#
#   5) Move the robot:
#     b) By sending goals with RVIZ's "Nav2 Goal" button in action bar.
#     a) By teleoperating:
#        $ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/a200_0000/cmd_vel
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_ns = LaunchConfiguration('robot_ns')

    icp_odom_parameters={
          'odom_frame_id':'icp_odom',
          'guess_frame_id':'odom',
          'OdomF2M/ScanSubtractRadius': '0.3', # match voxel size
          'OdomF2M/ScanMaxSize': '10000'
    }

    rtabmap_parameters={
          'subscribe_rgbd':True,
          'subscribe_depth':False,
          'subscribe_rgb':False,
          'subscribe_scan_cloud':True,
          'use_action_for_goal':True,
          'odom_sensor_sync': True,
          'topic_queue_size': 30,
          'sync_queue_size': 30,
          'approx_sync': True,
          'qos': 1,
          # RTAB-Map's parameters should be strings:
          'Mem/NotLinkedNodesKept':'false',
          'Grid/RangeMin':'0.5', # ignore laser scan points on the robot itself
          'Grid/NormalsSegmentation':'false', # Use passthrough filter to detect obstacles
          'Grid/MaxGroundHeight':'0.05', # All points above 5 cm are obstacles
          'Grid/MaxObstacleHeight':'1',  # All points over 1 meter are ignored
          'Grid/RayTracing':'true', # Fill empty space
          'Grid/3D':'false', # Use 2D occupancy
          'RGBD/OptimizeMaxError':'0.3', # There are a lot of repetitive patterns, be more strict in accepting loop closures
          'Rtabmap/DetectionRate': '0' # Rate is limited by the assembling time below (1 Hz)
    }

    # Shared parameters between different nodes
    shared_parameters={
          'frame_id':'base_link',
          'use_sim_time':use_sim_time,
          # RTAB-Map's parameters should be strings:
          'Reg/Strategy':'1',
          'Reg/Force3DoF':'true', # we are moving on a 2D flat floor
          'Mem/NotLinkedNodesKept':'false',
          'Icp/VoxelSize': '0.3',
          'Icp/MaxCorrespondenceDistance': '3', # roughly 10x voxel size
          'Icp/PointToPlaneGroundNormalsUp': '0.9',
          'Icp/RangeMin': '0.5',
          'Icp/MaxTranslation': '2'
    }

    remappings=[
          ('/tf', 'tf'),
          ('/tf_static', 'tf_static'),
          ('odom', 'icp_odom'),
          ('rgb/image', 'sensors/camera_0/color/image'),
          ('rgb/camera_info', 'sensors/camera_0/color/camera_info'),
          ('depth/image', 'sensors/camera_0/depth/image')]

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', choices=['true', 'false'],
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'robot_ns', default_value='a200_0000',
            description='Robot namespace.'),

        # Nodes to launch
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            namespace=robot_ns,
            parameters=[{'approx_sync':False, 'use_sim_time':use_sim_time}],
            remappings=remappings),

        Node(
            package='rtabmap_odom', executable='icp_odometry', output='screen',
            namespace=robot_ns,
            parameters=[icp_odom_parameters, shared_parameters],
            remappings=remappings + [('scan_cloud', 'sensors/lidar3d_0/points')],
            arguments=["--ros-args", "--log-level", 'warn']),
        
        #Assemble scans
        Node(
            package='rtabmap_util', executable='point_cloud_assembler', output='screen',
            namespace=robot_ns,
            parameters=[{'assembling_time': 1.0, 'range_min': 0.5, 'fixed_frame_id': "", 'use_sim_time':use_sim_time, 'sync_queue_size': 30, 'topic_queue_size':30}],
            remappings=remappings + [('cloud', 'sensors/lidar3d_0/points')]),

        # SLAM Mode:
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            namespace=robot_ns,
            parameters=[rtabmap_parameters, shared_parameters],
            remappings=remappings + [('scan_cloud', 'assembled_cloud')],
            arguments=['-d']),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            namespace=robot_ns,
            parameters=[rtabmap_parameters, shared_parameters],
            remappings=remappings + [('scan_cloud', 'sensors/lidar3d_0/points')]),
    ])
