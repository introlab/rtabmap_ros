# Refer to this paper for more info: https://arxiv.org/abs/2407.15305
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

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
          'RGBD/NeighborLinkRefining': 'false',
          'RGBD/ProximityBySpace':     'false',   # Referred paper did only global loop closure detection
          'RGBD/OptimizeFromGraphEnd': 'true',
          'Reg/Strategy':              '1',
          'Icp/Iterations':            '30',
          'Icp/VoxelSize':             '0',
          'Vis/MinInliers':            '12',
          'Vis/MaxDepth':              '0',
          'RGBD/AngularUpdate':        '0.01',
          'RGBD/LinearUpdate':         '0.01',
          'Rtabmap/TimeThr':           '700',
          'Mem/RehearsalSimilarity':   '0.30',    # Referred paper used 0.45 with SURF, here with SIFT, we will use 0.3
          'Kp/TfIdfLikelihoodUsed':    'false',
          'Bayes/FullPredictionUpdate': 'true',
          'Kp/DetectorStrategy':       '1',       # Referred paper used SURF (0), here use SIFT as it is available with opencv binaries
          'Vis/FeatureType':           '1',       # Referred paper used SURF (0), here use SIFT as it is available with opencv binaries
          'Kp/MaxFeatures':            '400',
          'Reg/Force3DoF':             'true',
          'RGBD/OptimizeMaxError':     '10',
          'Optimizer/Strategy':        '2',       # Referred paper used TORO (0), latest version recommends GTSAM (2)
          'Optimizer/Iterations':      '100',
          'Kp/IncrementalFlann':       'false',   # Referred paper didn't use incremental FLANN
          'Grid/Sensor':               '1',
          'Icp/MaxTranslation':        '0.5',
    }
    
    remappings=[
         ('rgb/image',       '/data_throttled_image'),
         ('depth/image',     '/data_throttled_image_depth'),
         ('rgb/camera_info', '/data_throttled_camera_info'),
         ('scan',            '/base_scan')]
    
    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_demos'), 'launch', 'config', 'demo_robot_mapping.rviz'
    )

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument('rtabmap_viz',  default_value='true',  description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('rviz',         default_value='false', description='Launch RVIZ (optional).'),
        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz,  description='Configuration path of rviz2.'),

        SetParameter(name='use_sim_time', value=True),

        # Nodes to launch
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[parameters,
              {'rgb_image_transport':'compressed',
               'depth_image_transport':'compressedDepth',
               'approx_sync_max_interval': 0.02}],
            remappings=remappings),
        
        # SLAM node:
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
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
