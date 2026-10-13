
# Example to run euroc datasets:
#   $ sudo pip install rosbags     # See https://docs.openvins.com/dev-ros1-to-ros2.html
#   $ rosbags-convert V1_01_easy.bag
#   $ rosbags-convert MH_01_easy.bag
#
#   $ ros2 launch rtabmap_examples euroc_datasets.launch.py gt:=vicon
#   $ cd V1_01_easy
#   $ ros2 bag play V1_01_easy --clock
#
#   $ ros2 launch rtabmap_examples euroc_datasets.launch.py gt:=leica
#   $ cd MH_01_easy
#   $ ros2 bag play MH_01_easy --clock
#
# Notes when comparing with published results:
#   * The stereo calibration (config/euroc_*.yaml, same as the datasets' sensor.yaml) was
#     recorded with the V1 sequences. On the V2 sequences, recorded months later, the
#     rectified stereo pairs are ~1.8 px apart vertically (the right camera rotated by
#     ~0.2 deg), which degrades stereo matching and odometry there.
#   * The ground truth used here (gt:=vicon) is the raw Vicon pose from the bags
#     (vicon/firefly_sbx/firefly_sbx), moved to the base frame with the datasets' Vicon
#     calibration, not the ground truth published with them
#     (state_groundtruth_estimate0/data.csv): they differ by ~2 cm and ~1 deg, and up to
#     ~0.2 s in time on the V2 sequences. The RMSE computed here may then differ from the
#     one in papers using the published ground truth.
#   * On the MH sequences (gt:=leica), the ground truth is the Leica laser tracker's
#     position of the MAV's prism, which has no orientation: euroc_leica_ground_truth.py
#     gives it the IMU's (from the complementary filter), aligned in yaw, and corrects the
#     Leica's latency, both estimated from the accelerations seen by the two (see the
#     script). There is no ground truth until they are estimated, after the MAV has moved
#     up and down and horizontally (~10 to 60 s into the sequences). Against the
#     published ground truth: ~1 to 4 cm.


from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import SetParameter

def generate_launch_description():

    ground_truth = LaunchConfiguration('gt')
    def ground_truth_is(value):
        return IfCondition(PythonExpression(["'", ground_truth, "' == '", value, "'"]))

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
            description='Ground truth: vicon (V1/V2 sequences), leica (MH sequences) or false.'),

        DeclareLaunchArgument(
            'rtabmap_viz', default_value='true',
            description='Launch RTAB-Map UI (optional).'),

        SetParameter(name='use_sim_time', value=True),
        # 'use_sim_time' will be set on all nodes following the line above

        # Nodes to launch
        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            parameters=[parameters,
                { 'stereo_exposure_compensation': True,
                  # Motion guess predicted with the IMU acceleration, from the velocity
                  # over the last 0.5 s
                  'Odom/GuessSmoothingDelay':'0.5',
                  # Gravity estimated while the MAV is still at the start of the sequence
                  'Odom/ImuGravity':'0',
                  # As in the JFR2018 EuRoC evaluation: denser corners and a stricter
                  # matching ratio, more inliers through the blur of fast motions, and
                  # features matched within 20 pixels of their projection with the guess
                  'GFTT/MinDistance':'3',
                  'Vis/CorNNDR':'0.6',
                  'Vis/CorGuessWinSize':'20'}],
            remappings=remappings),

        Node(
            condition=IfCondition(PythonExpression(["'", ground_truth, "' != 'false'"])),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
                { 'ground_truth_frame_id':'world',
                  'ground_truth_base_frame_id':'base_link_gt'}],
            remappings=remappings,
            arguments=['-d']),
        Node(
            condition=ground_truth_is('false'),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']),

        Node(
            condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[parameters,
                        {'odometry_node_name': "stereo_odometry"}],
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
            
        # Its camera_info is the one beside the image topic since Jazzy (image_transport's
        # camera subscriber), relative to the node's namespace before: both are remapped to
        # the one yaml_to_camera_info.py publishes.
        Node(
            package='image_proc', executable='rectify_node', output='screen',
            remappings=[
              ('image', '/cam0/image_raw'),
              ('/cam0/camera_info', '/stereo_camera/left/camera_info')],
            namespace='stereo_camera/left'),
        Node(
            package='image_proc', executable='rectify_node', output='screen',
            remappings=[
              ('image', '/cam1/image_raw'),
              ('/cam1/camera_info', '/stereo_camera/right/camera_info')],
            namespace='stereo_camera/right'),

        Node(
            package='imu_complementary_filter', executable='complementary_filter_node', output='screen',
            parameters=[{'use_mag': False, 'world_frame':'enu', 'publish_tf':False}],
            remappings=[('imu/data_raw', '/imu0')]),
       
        # create a fake tf tree (static_transform_publisher's named arguments: Rolling no longer
        # takes the old positional ones)
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--yaw', '3.1415926', '--pitch', '-1.570796', '--roll', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'imu4']),
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['--x', '-0.021640', '--y', '-0.064677', '--z', '0.009811',
                       '--yaw', '1.555925', '--pitch', '0.025777', '--roll', '0.003757',
                       '--frame-id', 'imu4', '--child-frame-id', 'cam0']),
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['--x', '-0.019844', '--y', '0.045369', '--z', '0.007862',
                       '--yaw', '1.558237', '--pitch', '0.025393', '--roll', '0.017907',
                       '--frame-id', 'imu4', '--child-frame-id', 'cam1']),
        # Vicon ground truth (V1/V2 sequences): its base frame, in two steps. The IMU in the Vicon marker's frame: the
        # inverse of the datasets' mav0/vicon0/sensor.yaml T_BS (the Vicon marker in the IMU
        # frame), which is:
        #   T_BS: [ 0.33638, -0.01749,  0.94156,  0.06901,
        #          -0.02078, -0.99972, -0.01114, -0.02781,
        #           0.94150, -0.01582, -0.33665, -0.12395,
        #           0.0,      0.0,      0.0,      1.0]
        Node(
            condition=ground_truth_is('vicon'),
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['--x', '0.09291', '--y', '-0.02856', '--z', '-0.10701',
                       '--yaw', '-0.051939', '--pitch', '-1.227243', '--roll', '-3.108506',
                       '--frame-id', 'vicon/firefly_sbx/firefly_sbx', '--child-frame-id', 'imu4_gt']),
        # Then base_link from the IMU (Vicon and Leica ground truths): the inverse of
        # base_link -> imu4 above (as a quaternion, its pitch being -90 deg)
        Node(
            condition=IfCondition(PythonExpression(["'", ground_truth, "' != 'false'"])),
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['--qx', '-0.707107', '--qy', '0', '--qz', '-0.707107', '--qw', '0',
                       '--frame-id', 'imu4_gt', '--child-frame-id', 'base_link_gt']),
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--yaw', '0', '--pitch', '0', '--roll', '0',
                       '--frame-id', 'world', '--child-frame-id', 'map']),
        
        Node(
            condition=ground_truth_is('vicon'),
            package='rtabmap_util', executable='transform_to_tf.py', output='screen',
            parameters=[{'frame_id': 'world', 'child_frame_id': 'vicon/firefly_sbx/firefly_sbx'}],
            remappings=[('transform', '/vicon/firefly_sbx/firefly_sbx')]),

        # Leica ground truth (MH sequences): the prism's position with the IMU's orientation
        # (see the notes above)
        Node(
            condition=ground_truth_is('leica'),
            package='rtabmap_examples', executable='euroc_leica_ground_truth.py', output='screen',
            parameters=[{'frame_id': 'leica', 'fixed_frame_id': 'world'}],
            remappings=[('point', '/leica/position'), ('imu', '/imu/data')]),
        # Then the IMU from the prism: the inverse of the datasets' mav0/leica0/sensor.yaml
        # T_BS (the prism in the IMU frame, without rotation), which is:
        #   T_BS: [1.0, 0.0, 0.0,  7.48903e-02,
        #          0.0, 1.0, 0.0, -1.84772e-02,
        #          0.0, 0.0, 1.0, -1.20209e-01,
        #          0.0, 0.0, 0.0,  1.0]
        Node(
            condition=ground_truth_is('leica'),
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['--x', '-0.0748903', '--y', '0.0184772', '--z', '0.120209',
                       '--frame-id', 'leica', '--child-frame-id', 'imu4_gt']),
    ])


