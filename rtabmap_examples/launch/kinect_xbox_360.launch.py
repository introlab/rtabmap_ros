# Requirements:
#   A kinect for xbox 360
#   Install kinect_ros2 package (use this fork: https://github.com/matlabbe/kinect_ros2)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    parameters=[{
          'frame_id':'camera_link',
          'subscribe_depth':True,
          'subscribe_odom_info':True,
          'approx_sync':True}]

    remappings=[
          ('rgb/image', '/kinect/rgb/image_raw'),
          ('rgb/camera_info', '/kinect/rgb/camera_info'),
          ('depth/image', '/kinect/depth_registered/image_raw')]

    return LaunchDescription([

        # Nodes to launch
        Node(
            package='kinect_ros2', executable='kinect_ros2_node', output='screen',
            parameters=[{'depth_registration':True}],
            namespace="kinect"),

        # Optical rotation
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=["0", "0", "0", "-1.57", "0", "-1.57", "camera_link", "kinect_rgb"]),

        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings,
            namespace="rtabmap"),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d'],
            namespace="rtabmap"),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings,
            namespace="rtabmap"),
    ])
