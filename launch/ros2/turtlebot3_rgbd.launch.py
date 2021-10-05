# Requirements:
#   Install Turtlebot3 packages
#   Install https://github.com/mlherd/ros2_turtlebot3_waffle_intel_realsense
# Example:
#   $ export TURTLEBOT3_MODEL=waffle
#   $ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
#   $ ros2 run turtlebot3_teleop teleop_keyboard
#
#   $ ros2 launch rtabmap_ros turtlebot3_rgbd.launch.py
#   OR
#   $ ros2 launch rtabmap_ros rtabmap.launch.py visual_odometry:=false frame_id:=base_footprint odom_topic:=/odom args:="-d" use_sim_time:=true rgb_topic:=/intel_realsense_r200_depth/image_raw depth_topic:=/intel_realsense_r200_depth/depth/image_raw camera_info_topic:=/intel_realsense_r200_depth/camera_info approx_sync:=true

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    parameters=[{
          'frame_id':'base_footprint',
          'use_sim_time':use_sim_time,
          'subscribe_depth':True,
          'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
    }]

    remappings=[
          ('rgb/image', '/intel_realsense_r200_depth/image_raw'),
          ('rgb/camera_info', '/intel_realsense_r200_depth/camera_info'),
          ('depth/image', '/intel_realsense_r200_depth/depth/image_raw')]

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        # Nodes to launch
        Node(
            package='rtabmap_ros', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

        Node(
            package='rtabmap_ros', executable='rtabmapviz', output='screen',
            parameters=parameters,
            remappings=remappings),
    ])
