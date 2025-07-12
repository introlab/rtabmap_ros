#
#
# Example with gazebo:
#   1) Launch simulator (husky):
#     $ ros2 launch clearpath_gz simulation.launch.py
#     Click on "Play" button on bottom-left of gazebo as soon as you can see it to avoid controllers crashing after 5 sec.
#
#   2) Launch rviz:
#     $ ros2 launch clearpath_viz view_navigation.launch.py namespace:=a200_0000
#
#   3) Launch SLAM:
#     $ ros2 launch rtabmap_demos husky_slam2d.launch.py use_sim_time:=true
#
#   4) Launch nav2"
#     $ ros2 launch clearpath_nav2_demos nav2.launch.py setup_path:=$HOME/clearpath/ use_sim_time:=true
#
#   4) Click on "Play" button on bottom-left of gazebo.
#
#   5) Move the robot:
#     b) By sending goals with RVIZ's "Nav2 Goal" button in action bar.
#     a) By teleoperating:
#        $ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/a200_0000/cmd_vel
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    robot_ns = LaunchConfiguration('robot_ns')

    icp_odom_parameters={
          'odom_frame_id':'icp_odom',
          'guess_frame_id':'odom'
    }

    rtabmap_parameters={
          'subscribe_rgbd':True,
          'subscribe_scan':True,
          'use_action_for_goal':True,
          'odom_sensor_sync': True,
          # RTAB-Map's parameters should be strings:
          'Mem/NotLinkedNodesKept':'false',
          'Grid/RangeMin':'0.7', # ignore laser scan points on the robot itself
          'RGBD/OptimizeMaxError':'2',
    }

    # Shared parameters between different nodes
    shared_parameters={
          'frame_id':'base_link',
          'use_sim_time':use_sim_time,
          # RTAB-Map's parameters should be strings:
          'Reg/Strategy':'1',
          'Reg/Force3DoF':'true', # we are moving on a 2D flat floor
          'Mem/NotLinkedNodesKept':'false',
          'Icp/PointToPlaneMinComplexity':'0.04', # to be more robust to long corridors with low geometry
          'Icp/MaxTranslation': '1'
    }

    remappings=[
          ('/tf', 'tf'),
          ('/tf_static', 'tf_static'),
          ('odom', 'icp_odom'),
          ('scan', 'sensors/lidar2d_0/scan'),
          ('rgb/image', 'sensors/camera_0/color/image'),
          ('rgb/camera_info', 'sensors/camera_0/color/camera_info'),
          ('depth/image', 'sensors/camera_0/depth/image')]

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', choices=['true', 'false'],
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'localization', default_value='false', choices=['true', 'false'],
            description='Launch rtabmap in localization mode (a map should have been already created).'),

        DeclareLaunchArgument(
            'robot_ns', default_value='a200_0000',
            description='Robot namespace.'),

        # Nodes to launch
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            namespace=robot_ns,
            parameters=[{'approx_sync':False, 'use_sim_time':use_sim_time}],
            remappings=remappings),

        Node(
            package='rtabmap_odom', executable='icp_odometry', output='screen',
            namespace=robot_ns,
            parameters=[icp_odom_parameters, shared_parameters],
            remappings=remappings,
            arguments=["--ros-args", "--log-level", 'warn']),

        # SLAM Mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            namespace=robot_ns,
            parameters=[rtabmap_parameters, shared_parameters],
            remappings=remappings,
            arguments=['-d']),
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            namespace=robot_ns,
            parameters=[rtabmap_parameters, shared_parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            namespace=robot_ns,
            parameters=[rtabmap_parameters, shared_parameters],
            remappings=remappings),
    ])
