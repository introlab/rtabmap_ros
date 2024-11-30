
# Similar to gazebo example on https://github.com/chvmp/champ/tree/ros2, we can do:
#
#   Run the Gazebo environment:
#     $ ros2 launch champ_config gazebo.launch.py 
#
#   Run Nav2's navigation and rtabmap:
#     $ ros2 launch rtabmap_demos champ_vslam.launch.py use_sim_time:=true rviz:=true rtabmap_viz:=true
#
#   When a map is already created using command above, we can re-launch in localization-only mode with:
#     $ ros2 launch rtabmap_demos champ_vslam.launch.py use_sim_time:=true rviz:=true rtabmap_viz:=true localization:=true
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    
    localization = LaunchConfiguration('localization')
    
    navigation_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']
    )
    
    nav2_params_file = PathJoinSubstitution(
        [FindPackageShare('rtabmap_demos'), 'params', 'champ_nav2_params.yaml']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('champ_navigation'), 'rviz', 'navigation.rviz']
    )
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # With the simulator, the imu is not published fast enough 
    # and have a huge delay, disabling imu usage from VO
    use_imu = use_sim_time.perform(context) in ["false", "False"]

    vslam_params ={
        'frame_id':'base_link',
        'guess_frame_id':'odom',
        'approx_sync': False,
        'use_sim_time':use_sim_time,
        'subscribe_rgbd':True,
        'subscribe_odom_info':True,
        'use_action_for_goal':True,
        'wait_imu_to_init': use_imu,
        'wait_for_transform': 0.5,
        # RTAB-Map's parameters should be strings
        'Grid/DepthDecimation': '1',
        'Grid/RangeMax': '2',
        'GridGlobal/MinSize': '20',
        'Grid/MinClusterSize': '20',
        'Grid/MaxObstacleHeight': '2',
        'Odom/ResetCountdown': '2', # sim is very flaky
        'Kp/RoiRatios': '0.0 0.0 0.0 0.4' # ignore ground for loop closure detection (sim uses a very repetitive texture)
    }
    
    vslam_remappings=[('imu', 'imu/data/filtered'),
                      ('odom', 'vo')]
    
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch_path),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_params_file
            }.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # compute imu orientation
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{
              'use_mag':False,
              'world_frame':'enu',
              'publish_tf':False}],
            remappings=[
                ('imu/data_raw', 'imu/data'),
                ('imu/data', 'imu/data/filtered')
            ]),
        
        # VSLAM nodes:
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[vslam_params],
            remappings=[('rgb/image', '/camera/image_raw'),
                        ('rgb/camera_info', '/camera/camera_info'),
                        ('depth/image', '/camera/depth/image_raw')]),

        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=[vslam_params, {'odom_frame_id': 'vo'}],
            remappings=vslam_remappings,
            arguments=["--ros-args", "--log-level", 'info']),

        # SLAM Mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[vslam_params],
            remappings=vslam_remappings,
            arguments=['-d']),
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[vslam_params, 
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=vslam_remappings),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
            parameters=[vslam_params],
            remappings=vslam_remappings),
        
        # Compute ground/obstacle clouds for nav2 voxel layers
        Node(
            package='rtabmap_util', executable='point_cloud_xyz', output='screen',
            parameters=[{'decimation': 2,
                         'max_depth': 3.0,
                         'voxel_size': 0.02}],
            remappings=[('depth/image', '/camera/depth/image_raw'),
                        ('depth/camera_info', '/camera/depth/camera_info'),
                        ('cloud', '/camera/cloud')]),
        
        Node(
            package='rtabmap_util', executable='obstacles_detection', output='screen',
            parameters=[vslam_params],
            remappings=[('cloud', '/camera/cloud'),
                        ('obstacles', '/camera/obstacles'),
                        ('ground', '/camera/ground')]),
    ]

def generate_launch_description():
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='false',
            description='Enable use_sime_time to true'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz'
        ),
        
        DeclareLaunchArgument(
            name='rtabmap_viz', 
            default_value='false',
            description='Run rtabmap_viz'
        ),

        DeclareLaunchArgument(
            'localization', default_value='false', choices=['true', 'false'],
            description='Launch rtabmap in localization mode (a map should have been already created).'),
        
        OpaqueFunction(function=launch_setup)
    ])