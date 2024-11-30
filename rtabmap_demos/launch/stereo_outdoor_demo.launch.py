# Requirements:
#   Download one or both rosbags:
#    * stereo_outdoorA.db3: https://drive.google.com/file/d/1O7mCXg_sw4tZY1S88a-n96O6OulmqvqI/view?usp=drive_link
#    * stereo_outdoorB.db3: https://drive.google.com/file/d/1mSu7418Fkbe-hIz2-3Mi936PrWuD2un_/view?usp=drive_link
#
# Example:
#
#   SLAM:
#     $ ros2 launch rtabmap_demos stereo_outdoor_demo.launch.py  rviz:=true rtabmap_viz:=true
#
#   Rosbag:
#     $ ros2 bag play stereo_outdoorA.db3 --clock
#     when done, you can play the secon bag:
#     $ ros2 bag play stereo_outdoorB.db3 --clock
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetParameter, SetRemap
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_stereo_image_proc = get_package_share_directory(
        'stereo_image_proc')

    # Paths
    stereo_image_proc_launch = PathJoinSubstitution(
        [pkg_stereo_image_proc, 'launch', 'stereo_image_proc.launch.py'])

    localization = LaunchConfiguration('localization')

    parameters={
          'frame_id':'base_footprint',
          'subscribe_rgbd':True,
          'approx_sync':False, # odom is generated from images, so we can exactly sync all inputs
          'map_negative_poses_ignored':True,
          'subscribe_odom_info': True,
          # RTAB-Map's internal parameters should be strings
          'OdomF2M/MaxSize': '1000',
          'GFTT/MinDistance': '10',
          'GFTT/QualityLevel': '0.00001',
          #'Kp/DetectorStrategy': '6', # Uncommment to match ros1 noetic results, but opencv should be built with xfeatures2d
          #'Vis/FeatureType': '6'      # Uncommment to match ros1 noetic results, but opencv should be built with xfeatures2d
    }

    remappings=[
         ('rgbd_image', '/stereo_camera/rgbd_image'),
         ('odom',       '/vo')]
    
    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_demos'), 'config', 'demo_robot_mapping.rviz'
    )

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument('rtabmap_viz',  default_value='false',  description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('rviz',         default_value='true',   description='Launch RVIZ (optional).'),
        DeclareLaunchArgument('localization', default_value='false',  description='Launch in localization mode.'),
        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz,  description='Configuration path of rviz2.'),

        SetParameter(name='use_sim_time', value=True),

        # Nodes to launch

        # Uncompress images for stereo_image_rect and remap to expected names from stereo_image_proc
        Node(
            package='image_transport', executable='republish', name='republish_left', output='screen',
            namespace='stereo_camera',
            arguments=['compressed', 'raw'],
            remappings=[('in/compressed', 'left/image_raw_throttle/compressed'),
                        ('out',           'left/image_raw')]),
        Node(
            package='image_transport', executable='republish', name='republish_right', output='screen',
            namespace='stereo_camera',
            arguments=['compressed', 'raw'],
            remappings=[('in/compressed', 'right/image_raw_throttle/compressed'),
                        ('out',           'right/image_raw')]),

        # Run the ROS package stereo_image_proc for image rectification   
        GroupAction(
            actions=[

                SetRemap(src='camera_info',dst='camera_info_throttle'),
                SetRemap(src='camera_info',dst='camera_info_throttle'),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([stereo_image_proc_launch]),
                    launch_arguments=[
                        ('left_namespace', 'stereo_camera/left'),
                        ('right_namespace', 'stereo_camera/right'),
                        ('disparity_range', '128'),
                    ]
                ),
            ]
        ),
        
        # Synchronize stereo data together in a single topic
        # Issue: stereo_img_proc doesn't produce color and 
        #        grayscale images exactly the same (there is a small 
        #        vertical shift with color), we should use grayscale for 
        #        left and right images to get similar results than on ros1 noetic.
        Node(
            package='rtabmap_sync', executable='stereo_sync', output='screen',
            namespace='stereo_camera',
            remappings=[
                ('left/image_rect',   'left/image_rect'),
                ('right/image_rect',  'right/image_rect'),
                ('left/camera_info',  'left/camera_info_throttle'),
                ('right/camera_info', 'right/camera_info_throttle')]),

        # Visual odometry
        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            parameters=[parameters],
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
    ])


   
   

