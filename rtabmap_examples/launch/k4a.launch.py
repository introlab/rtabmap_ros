# Requirements:
#   A Kinect for Azure
#   Install Azure_Kinect_ROS_Driver ros2 package (https://github.com/microsoft/Azure_Kinect_ROS_Driver/tree/humble)
#   Install imu_filter_madgwick ros2 package
# Example:
#   $ ros2 launch rtabmap_examples k4a.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    parameters=[{
          'frame_id':'camera_base',
          'subscribe_rgbd':True,
          'subscribe_odom_info':True,
          'qos':1}]

    remappings=[
          ('imu', '/imu/data'),
          ('rgb/image', '/rgb/image_raw'),
          ('rgb/camera_info', '/rgb/camera_info'),
          ('depth/image', '/depth_to_rgb/image_raw'),
          ('rgbd_image', 'odom_rgbd_image')]

    return LaunchDescription([

        # Nodes to launch 
        
        # Visual odometry node      
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=[{ 'frame_id':'camera_base',
                          'subscribe_odom_info':True,
                          'approx_sync':True,
                          'approx_sync_max_interval':0.01,
                          'wait_imu_to_init':True,
                          'qos':1,
                          'queue_size':30,
                          'keep_color':True,
                          # Color image needs to be rectified, 
                          # this will tell vo to rectify them for convenience:
                          'Rtabmap/ImagesAlreadyRectified':'False'}],
            remappings=remappings),

        # SLAM node
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

        # Visualization node
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings),
            
        # Kinect for azure
        Node(
            package='azure_kinect_ros_driver', executable='node', output='screen',
            parameters=[{'color_enabled': True, 
                         'fps':15, 
                         'depth_mode':'WFOV_2X2BINNED'}]),
        
        # Compute quaternion of the IMU
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame':'enu', 
                         'publish_tf':False}],
            remappings=[('imu/data_raw', '/imu')]),
    ])
