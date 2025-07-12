
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    localization_value = localization.perform(context)
    localization_value = localization_value == 'True' or localization_value == 'true'
    enable_vo = LaunchConfiguration('enable_vo')
    enable_vo_value = enable_vo.perform(context)
    enable_vo_value = enable_vo_value == 'True' or enable_vo_value == 'true'
    stereo = LaunchConfiguration('stereo')
    stereo_value = stereo.perform(context)
    stereo_value = stereo_value == 'True' or stereo_value == 'true'
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    stereo_ns = LaunchConfiguration('stereo_camera_namespace').perform(context)

    parameters={
          'frame_id':'base_link',
          'use_sim_time': use_sim_time,
          'subscribe_rgbd': True,
          'subscribe_odom': enable_vo,
          'subscribe_odom_info': enable_vo,
          'approx_sync': False,
          'use_action_for_goal':True,
          'Reg/Force3DoF':'true',
          'Vis/MinDepth': '0.2',
          'GFTT/MinDistance': '5',
          'GFTT/QualityLevel': '0.00001',
          'Grid/RayTracing':'true', # Fill empty space
          'Grid/3D':'false', # Use 2D occupancy
          'Grid/NormalsSegmentation':'false', # Use passthrough filter to detect obstacles
          'Grid/MaxGroundHeight':'0.15', # All points above 5 cm are obstacles
          'Grid/MaxObstacleHeight':'0.5',  # All points over 0.5 meter are ignored
          'Grid/RangeMin':'0.2',  # Ignore invalid points close to camera
          'Grid/NoiseFilteringMinNeighbors':'8',  # Default stereo is quite noisy, enable noise filter
          'Grid/NoiseFilteringRadius':'0.1',  # Default stereo is quite noisy, enable noise filter
          'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
    }
    if enable_vo_value:
        parameters['guess_frame_id'] = 'odom'
    else:
        parameters['odom_frame_id'] = 'odom'
    
    arguments = []
    if localization_value:
        parameters['Mem/IncrementalMemory'] = 'True'
        parameters['Mem/InitWMWithAllNodes'] = 'True'
    else:
        arguments.append('-d') # This will delete the previous database (~/.ros/rtabmap.db)

    remappings=[('rgbd_image', '/'+stereo_ns+'/rgbd_image'),
                ('map', '/map')]
    vo_node_prefix = 'rgbd'
    if stereo_value:
        vo_node_prefix = 'stereo'
    
    return [
        # Sync image data together
        Node(
            condition=UnlessCondition(stereo),
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            namespace=stereo_ns,
            parameters=[{'approx_sync':False, 'use_sim_time':use_sim_time}],
            remappings=[
                ('rgb/image', 'left/image_rect'),
                ('rgb/camera_info', 'left/camera_info_rect'),
                ('depth/image', 'depth')]),

        Node(
            condition=IfCondition(stereo),
            package='rtabmap_sync', executable='stereo_sync', output='screen',
            namespace=stereo_ns,
            parameters=[{'approx_sync':False, 'use_sim_time':use_sim_time}],
            remappings=[
                ('left/image_rect', 'left/image_rect'),
                ('left/camera_info', 'left/camera_info_rect'),
                ('right/image_rect', 'right/image_rect'),
                ('right/camera_info', 'right/camera_info_rect')]),

        Node(
            condition=IfCondition(enable_vo),
            package='rtabmap_odom', executable=vo_node_prefix+'_odometry', output='screen',
            namespace='rtabmap',
            parameters=[parameters, {'odom_frame_id': 'vo'}],
            remappings=remappings),
        
        # VSLAM:
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            namespace='rtabmap',
            parameters=[parameters],
            remappings=remappings,
            arguments=arguments), 

        # Visualization:
        Node(
            condition=IfCondition(rtabmap_viz),
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            namespace='rtabmap',
            parameters=[parameters],
            remappings=remappings),
    ]

def generate_launch_description():
    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),
        
        DeclareLaunchArgument(
            'enable_vo', default_value='false',
            description='Enable RTAB-Map\'s visual odometry.'),
        
        DeclareLaunchArgument(
            'rtabmap_viz', default_value='true',
            description='Launch rtabmap_viz for visualization.'),
        
        DeclareLaunchArgument(
            'stereo', default_value='false',
            description='Use stereo images as input instead of left+depth images.'),
        
        DeclareLaunchArgument(
            'stereo_camera_namespace', default_value='front_stereo_camera',
            description='Namespace of the stereo camera.'),
        
        OpaqueFunction(function=launch_setup)
    ])
