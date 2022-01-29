# Requirements:
#   Install Turtlebot3 packages
#   Modify turtlebot3_waffle SDF:
#     1) Edit turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf
#     2) Add
#          <joint name="camera_rgb_optical_joint" type="fixed">
#            <parent>camera_rgb_frame</parent>
#            <child>camera_rgb_optical_frame</child>
#            <pose>0 0 0 -1.57079632679 0 -1.57079632679</pose>
#            <axis>
#              <xyz>0 0 1</xyz>
#            </axis>
#          </joint> 
#     3) Rename <link name="camera_rgb_frame"> to <link name="camera_rgb_optical_frame">
#     4) Add <link name="camera_rgb_frame"/>
#     5) Change <sensor name="camera" type="camera"> to <sensor name="camera" type="depth">
#     6) Change image width/height from 1920x1080 to 640x480
#     7) Note that we can increase min scan range from 0.12 to 0.2 to avoid having scans 
#        hitting the robot itself
# Example:
#   $ export TURTLEBOT3_MODEL=waffle
#   $ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
#
#   SLAM:
#   $ ros2 launch rtabmap_ros turtlebot3_rgbd.launch.py
#   OR
#   $ ros2 launch rtabmap_ros rtabmap.launch.py visual_odometry:=false frame_id:=base_footprint odom_topic:=/odom args:="-d" use_sim_time:=true rgb_topic:=/camera/image_raw depth_topic:=/camera/depth/image_raw camera_info_topic:=/camera/camera_info approx_sync:=true
#
#   Navigation (install nav2_bringup package):
#     $ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
#     $ ros2 launch nav2_bringup rviz_launch.py
#
#   Teleop:
#     $ ros2 run turtlebot3_teleop teleop_keyboard

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')

    parameters={
          'frame_id':'base_footprint',
          'use_sim_time':use_sim_time,
          'subscribe_depth':True,
          'use_action_for_goal':True,
          'qos_image':qos,
          'qos_imu':qos,
          'Reg/Force3DoF':'true',
          'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
    }

    remappings=[
          ('rgb/image', '/camera/image_raw'),
          ('rgb/camera_info', '/camera/camera_info'),
          ('depth/image', '/camera/depth/image_raw')]

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'qos', default_value='2',
            description='QoS used for input sensor topics'),
            
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),

        # Nodes to launch
        
        # SLAM mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_ros', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']), # This will delete the previous database (~/.ros/rtabmap.db)
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_ros', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings),

        Node(
            package='rtabmap_ros', executable='rtabmapviz', output='screen',
            parameters=[parameters],
            remappings=remappings),
    ])
