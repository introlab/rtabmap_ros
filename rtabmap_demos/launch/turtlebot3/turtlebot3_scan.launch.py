# Example:
#
#   Bringup turtlebot3:
#     $ export TURTLEBOT3_MODEL=waffle
#     $ export LDS_MODEL=LDS-01
#     $ ros2 launch turtlebot3_bringup robot.launch.py
#
#   SLAM:
#     $ ros2 launch rtabmap_demos turtlebot3_scan.launch.py
#
#   Navigation (install nav2_bringup package):
#     $ ros2 launch nav2_bringup navigation_launch.py
#     $ ros2 launch nav2_bringup rviz_launch.py
#
#   Teleop:
#     $ ros2 run turtlebot3_teleop teleop_keyboard

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization').perform(context)
    localization = localization == 'True' or localization == 'true'
    icp_odometry = LaunchConfiguration('icp_odometry').perform(context)
    icp_odometry = icp_odometry == 'True' or icp_odometry == 'true'
    
    parameters={
          'frame_id':'base_footprint',
          'use_sim_time':use_sim_time,
          'subscribe_depth':False,
          'subscribe_rgb':False,
          'subscribe_scan':True,
          'approx_sync':True,
          'use_action_for_goal':True,
          'Reg/Strategy':'1',
          'Reg/Force3DoF':'true',
          'RGBD/NeighborLinkRefining':'True',
          'Grid/RangeMin':'0.2', # ignore laser scan points on the robot itself
          'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
    }
    arguments = []
    if localization:
        parameters['Mem/IncrementalMemory'] = 'False'
        parameters['Mem/InitWMWithAllNodes'] = 'True'
    else:
        arguments.append('-d') # This will delete the previous database (~/.ros/rtabmap.db)
               
    remappings=[
          ('scan', '/scan')]
    if icp_odometry:
        remappings.append(('odom', 'icp_odom'))
    
    return [
        # Nodes to launch
        
        # ICP odometry (optional)
        Node(
            condition=IfCondition(LaunchConfiguration('icp_odometry')),
            package='rtabmap_odom', executable='icp_odometry', output='screen',
            parameters=[parameters, 
                        {'odom_frame_id':'icp_odom',
                         'guess_frame_id':'odom'}],
            remappings=remappings),
        
        # SLAM:
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=arguments),

        # Visualization
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
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
            'icp_odometry', default_value='false',
            description='Launch ICP odometry on top of wheel odometry.'),

        OpaqueFunction(function=launch_setup)
    ])
