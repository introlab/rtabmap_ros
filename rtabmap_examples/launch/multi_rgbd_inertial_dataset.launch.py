#
#  Example launch file to run VSLAM on this dataset: https://github.com/seungsang07/multi-rgbd-inertial-dataset
#
#  Requirement(s):
#    * RTAB-Map should be built with OpenGV support.
#
#  To convert ROS1 bags to ROS2 (https://docs.openvins.com/dev-ros1-to-ros2.html):
#     sudo pip install rosbags
#     rosbags-convert --src Indoor.bag --dst Indoor
#
#  Usage:
#    ros2 launch rtabmap_examples multi_rgbd_inertial_dataset.launch.py
#    ros2 bag play Indoor/Indoor.db3 --clock
#
#  Note(s):
#    * Communication performance could be improved using the composable nodes of these nodes instead,
#      but we are using nodes here to better understand what is going on with rqt_graph.
#
#  To get RMSE after the run (using https://github.com/MichaelGrupp/evo):
#    rtabmap-export --poses --pose_format 10 ~/.ros/rtabmap.db
#    evo_ape tum -a -p --plot_mode xy ~/Downloads/GroundTruth/gt_indoor.txt ~/.ros/rtabmap_poses.txt
#

from launch import LaunchDescription, LaunchContext
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def make_yaml_to_camera_info_node(camera):
  # the dataset doesn't provide camera_info topics of the camera in the rosbags, so we need to generate them
  return Node(
        package='rtabmap_util', executable='yaml_to_camera_info.py', name=f'yaml_to_camera_info_{camera}', output='screen',
        parameters=[{'yaml_path': [FindPackageShare('rtabmap_examples'), f'/config/multi_rgbd_inertial_dataset_{camera}.yaml']}],
        remappings=[
          ('image', 'color/image_raw'),
          ('camera_info', 'color/camera_info')],
        namespace=f'camera_{camera}')

def make_rgbd_sync_node(camera):
  # synchronize topics of each camera together
  return Node(
        package='rtabmap_sync', executable='rgbd_sync', name=f'rgbd_sync_{camera}', output="screen",
        parameters=[{"approx_sync": False}],
        remappings=[
            ("rgb/image", 'color/image_raw'),
            ("depth/image", 'aligned_depth_to_color/image_raw'),
            ("rgb/camera_info", 'color/camera_info'),
            ("rgbd_image", 'rgbd_image')],
        namespace=f'camera_{camera}')

def make_static_transform_publisher_node(x, y, z, yaw, pitch, roll, parent_frame, child_frame):
  return Node(
          package='tf2_ros', executable='static_transform_publisher', name=f"static_transform_publisher_{parent_frame}_{child_frame}", output='screen',
          arguments=['--frame-id', parent_frame, 
                     '--child-frame-id', child_frame,
                     '--x', str(x),
                     '--y', str(y),
                     '--z', str(z),
                     '--yaw', str(yaw),
                     '--pitch', str(pitch),
                     '--roll', str(roll)])

def launch_setup(context: LaunchContext, *args, **kwargs):
  
  use_imu = LaunchConfiguration('use_imu').perform(context)
  use_imu = use_imu == 'true' or use_imu == 'True'
  
  # Synchronize all cameras together
  rgbdx_sync_node = Node(
      package='rtabmap_sync', executable='rgbdx_sync', output="screen",
      parameters=[{
          "rgbd_cameras": 4,
          "approx_sync": True,
          "approx_sync_max_interval": 0.015}],
      remappings=[
          ("rgbd_image0", '/camera_left/rgbd_image'),
          ("rgbd_image1", '/camera_front/rgbd_image'),
          ("rgbd_image2", '/camera_right/rgbd_image'),
          ("rgbd_image3", '/camera_rear/rgbd_image')],
      namespace='rtabmap'
  )
  
  # RGB-D odometry
  remappings = []
  if use_imu:
    remappings = [("imu", '/imu')]
  rgbd_odometry_node = Node(
      package='rtabmap_odom', executable='rgbd_odometry', output="screen",
      parameters=[{
          "frame_id": 'base_link',
          "rgbd_cameras": 0, # make it subscribe to rgbd_images topic from rgbdx_sync
          "subscribe_rgbd": True,
          "wait_imu_to_init": use_imu}],
      remappings=remappings,
      namespace='rtabmap'
  )

  # SLAM 
  remappings=[("sensor_data", 'odom_sensor_data/raw')]
  if(use_imu):
    remappings.append(('imu', '/imu'))
  slam_node = Node(
      package='rtabmap_slam', executable='rtabmap', output="screen",
      parameters=[{
          "subscribe_sensor_data": True,
          "frame_id": 'base_link',
          "approx_sync": False,
          "Grid/3D": 'false',
          "Grid/RayTracing": 'true',
          "Grid/NormalsSegmentation": 'false',
          "Grid/MaxGroundHeight": '0.05',
          "Rtabmap/CreateIntermediateNodes": 'true' # Only to record all odometry poses for trajectory evaluation purpose
      }],
      remappings=remappings,
      arguments=["--delete_db_on_start"],
      namespace='rtabmap'
  )
      
  # Visualization
  viz_node = Node(
    package='rtabmap_viz', executable='rtabmap_viz', output="screen",
    parameters=[{
        "subscribe_sensor_data": True,
        "frame_id": 'base_link',
        "approx_sync": False,
        "subscribe_odom_info": True
    }],
    remappings=[("sensor_data", 'odom_sensor_data/raw')],
    arguments=["-d", [FindPackageShare('rtabmap_examples'), '/config/multi_rgbd_inertial_dataset.ini']],
    namespace='rtabmap'
  )
  
  return [
        make_yaml_to_camera_info_node('left'),
        make_rgbd_sync_node('left'),
        make_yaml_to_camera_info_node('front'),
        make_rgbd_sync_node('front'),
        make_yaml_to_camera_info_node('right'),
        make_rgbd_sync_node('right'),
        make_yaml_to_camera_info_node('rear'),
        make_rgbd_sync_node('rear'),
        # The dataset doesn't provide /tf or /tf_static for the extrinsics between imu and the cameras, so we add them here
        make_static_transform_publisher_node(0., 0., 0.22, 3.1415926, 0., 0., 'base_link', 'imu_link'),
        make_static_transform_publisher_node(-0.099307, -0.208806, 0.024309, 3.108592, -0.051480, -1.592415, 'imu_link', 'camera_left_color_optical_frame'),
        make_static_transform_publisher_node(-0.435392, 0.022256, 0.053441, 1.532258, -0.007768, -1.580204, 'imu_link', 'camera_front_color_optical_frame'),
        make_static_transform_publisher_node(-0.063987, 0.212966, 0.032071, -0.010030, 0.021977, -1.553814, 'imu_link', 'camera_right_color_optical_frame'),
        make_static_transform_publisher_node(0.178893, -0.006307, 0.017677, -1.606156, 0.027545, -1.587179, 'imu_link', 'camera_rear_color_optical_frame'),
        make_static_transform_publisher_node(0.045872, -0.026775, 0.284806, 3.141063, -0.014869, -0.014969, 'imu_link', 'os_sensor'),
        rgbdx_sync_node,
        rgbd_odometry_node,
        slam_node,
        viz_node
      ]

def generate_launch_description():
  return LaunchDescription([
    DeclareLaunchArgument('use_imu', default_value='false', description='Use IMU'),
    SetParameter(name='use_sim_time', value=True),
    OpaqueFunction(function=launch_setup),
  ])