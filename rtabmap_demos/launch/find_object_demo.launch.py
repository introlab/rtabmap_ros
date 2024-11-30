# Requirements:
#   find_object_2d package installed
#   Download rosbag:
#    * demo_find_object.db3: https://drive.google.com/file/d/1web54yQkxeGFr2UwOjKeoajGGDm0fZXT/view?usp=drive_link
#
# Example:
#
#   SLAM:
#     $ ros2 launch rtabmap_demos find_object_demo.launch.py
#
#   Rosbag:
#     $ ros2 bag play demo_find_object.db3 --clock
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    localization = LaunchConfiguration('localization')

    parameters={
          'frame_id':'base_footprint',
          'odom_frame_id':'odom',
          'odom_tf_linear_variance':0.001,
          'odom_tf_angular_variance':0.001,
          'subscribe_rgbd':True,
          'subscribe_scan':True,
          'approx_sync':True,
          'sync_queue_size': 10,
          # RTAB-Map's internal parameters should be strings
          'RGBD/NeighborLinkRefining': 'true',    # Do odometry correction with consecutive laser scans
          'Reg/Strategy':              '1',       # 0=Visual, 1=ICP, 2=Visual+ICP
          'Reg/Force3DoF':             'true',    # 2D SLAM          
    }
    
    remappings=[
         ('rgb/image',       '/camera/data_throttled_image'),
         ('depth/image',     '/camera/data_throttled_image_depth'),
         ('rgb/camera_info', '/camera/data_throttled_camera_info'),
         ('scan',            '/base_scan')]
    
    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_demos'), 'config', 'demo_robot_mapping.rviz'
    )
    
    config_find_object = os.path.join(
        get_package_share_directory('rtabmap_demos'), 'config', 'find_object.ini'
    )
    
    data_find_object = os.path.join(
        get_package_share_directory('rtabmap_demos'), 'data', 'books'
    )

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument('rtabmap_viz',  default_value='false',  description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('rviz',         default_value='true',   description='Launch RVIZ (optional).'),
        DeclareLaunchArgument('localization', default_value='false',  description='Launch in localization mode.'),
        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz,  description='Configuration path of rviz2.'),

        SetParameter(name='use_sim_time', value=True),

        # Nodes to launch
        
        # Uncompress images for find_object
        Node(
            package='image_transport', executable='republish', name='republish_rgb', output='screen',
            arguments=['compressed', 'raw'],
            remappings=[('in/compressed', '/camera/data_throttled_image/compressed'),
                        ('out',           '/camera/data_throttled_image')]),
        Node(
            package='image_transport', executable='republish', name='republish_depth', output='screen',
            arguments=['compressedDepth', 'raw'],
            remappings=[('in/compressedDepth', '/camera/data_throttled_image_depth/compressedDepth'),
                        ('out',                '/camera/data_throttled_image_depth')]),
        
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[parameters,
              {'approx_sync_max_interval': 0.02}],
            remappings=remappings),
        
        # SLAM mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']), # This will delete the previous database (~/.ros/rtabmap.db)
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings),

        # Visualization:
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
            parameters=[parameters],
            remappings=remappings),
        Node(
            package='rviz2', executable='rviz2', name="rviz2", output='screen',
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=[["-d"], [LaunchConfiguration("rviz_cfg")]]),
        
        # Find-Object
        Node(
            package='find_object_2d', executable='find_object_2d', output='screen',
            parameters=[{'gui': True,
                         'subscribe_depth': True,
                         'settings_path': config_find_object,
                         'objects_path': data_find_object}],
            remappings=[('rgb/image_rect_color', '/camera/data_throttled_image'),
                        ('depth_registered/image_raw', '/camera/data_throttled_image_depth'),
                        ('depth_registered/camera_info', '/camera/data_throttled_camera_info')]),
    ])