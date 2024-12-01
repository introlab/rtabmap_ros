
# Example to run euroc datasets:
#   $ sudo pip install rosbags     # See https://docs.openvins.com/dev-ros1-to-ros2.html
#   $ rosbags-convert V1_01_easy.bag
#   $ rosbags-convert MH_01_easy.bag
#
#   $ ros2 launch rtabmap_examples euroc_datasets.launch.py gt:=true
#   $ cd V1_01_easy
#   $ ros2 bag play V1_01_easy.db3 --clock
#
#   $ ros2 launch rtabmap_examples euroc_datasets.launch.py gt:=false
#   $ cd MH_01_easy
#   $ ros2 bag play MH_01_easy.db3 --clock


from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import SetParameter

def generate_launch_description():

    ground_truth = LaunchConfiguration('gt')

    parameters={
          'frame_id':'base_link',
          'subscribe_stereo':True,
          'subscribe_odom_info':True,
          'wait_imu_to_init':True,
          'approx_sync':False,
          # RTAB-Map's parameters should all be string type:
          'RGBD/CreateOccupancyGrid':'false',
          'Rtabmap/CreateIntermediateNodes':'true',
          'RGBD/LinearUpdate':'0',
          'RGBD/AngularUpdate':'0'}
          
    remappings=[
          ('left/image_rect', '/stereo_camera/left/image_rect'),
          ('left/camera_info', '/stereo_camera/left/camera_info'),
          ('right/image_rect', '/stereo_camera/right/image_rect'),
          ('right/camera_info', '/stereo_camera/right/camera_info'),
          ('imu', '/imu/data')]
          

    return LaunchDescription([

        DeclareLaunchArgument(
            'gt', default_value='false',
            description='If the VH rosbag sequence is used, you can enable ground truth.'),

        SetParameter(name='use_sim_time', value=True),
        # 'use_sim_time' will be set on all nodes following the line above

        # Nodes to launch
        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            parameters=[parameters],
            remappings=remappings),

        Node(
            condition=IfCondition(ground_truth),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
                { 'ground_truth_frame_id':'world',
                  'ground_truth_base_frame_id':'base_link_gt'}],
            remappings=remappings,
            arguments=['-d']),
        Node(
            condition=UnlessCondition(ground_truth),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[parameters],
            remappings=remappings),
        
        # Image rectification and publishing synchronized camera_info
        Node(
            package='rtabmap_util', executable='yaml_to_camera_info.py', output='screen',
            parameters=[{'yaml_path': [FindPackageShare('rtabmap_examples'), '/config/euroc_left.yaml']}],
            remappings=[
              ('image', '/cam0/image_raw'),
              ('camera_info', 'left/camera_info')],
            namespace='stereo_camera'),
              
        Node(
            package='rtabmap_util', executable='yaml_to_camera_info.py', output='screen',
            parameters=[{'yaml_path': [FindPackageShare('rtabmap_examples'), '/config/euroc_right.yaml']}],
            remappings=[
              ('image', '/cam1/image_raw'),
              ('camera_info', 'right/camera_info')],
            namespace='stereo_camera'),
            
        Node(
            package='image_proc', executable='image_proc', output='screen',
            remappings=[
              ('image_raw', '/cam0/image_raw'),
              ('image', '/cam0/image_raw')],
            namespace='stereo_camera/left'),
        Node(
            package='image_proc', executable='image_proc', output='screen',
            remappings=[
              ('image_raw', '/cam1/image_raw'),
              ('image', '/cam1/image_raw')],
            namespace='stereo_camera/right'),

        Node(
            package='imu_complementary_filter', executable='complementary_filter_node', output='screen',
            parameters=[{'use_mag': False, 'world_frame':'enu', 'publish_tf':False}],
            remappings=[('imu/data_raw', '/imu0')]),
       
        # create a fake tf tree
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0', '0', '0', '3.1415926', '-1.570796', '0', 'base_link', 'imu4']),
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['-0.021640', '-0.064677', '0.009811', '1.555925', '0.025777', '0.003757', 'imu4', 'cam0']),
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['-0.019844', '0.045369', '0.007862', '1.558237', '0.025393', '0.017907', 'imu4', 'cam1']),
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0.12395', '-0.02781', '-0.06901', '0', '0', '0', 'vicon/firefly_sbx/firefly_sbx', 'base_link_gt']),
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']),
        
        Node(
            package='rtabmap_util', executable='transform_to_tf.py', output='screen',
            parameters=[{'frame_id': 'world', 'child_frame_id': 'vicon/firefly_sbx/firefly_sbx'}],
            remappings=[('transform', '/vicon/firefly_sbx/firefly_sbx')]),    
    ])


