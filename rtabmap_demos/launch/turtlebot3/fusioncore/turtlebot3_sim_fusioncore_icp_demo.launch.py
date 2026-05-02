"""
Complete TurtleBot3 demo: Gazebo (new) + FusionCore + icp_odometry + Nav2.

Launches in order:
  1. Gazebo Harmonic (via ros_gz_sim) with turtlebot3_world
  2. Robot state publisher + spawn TurtleBot3
  3. Custom ros_gz_bridge WITHOUT the odom TF  (FusionCore owns odom->base_footprint)
  4. FusionCore lifecycle node  (configure -> activate automatically)
  5. icp_odometry using FusionCore's odom frame as scan-match initial guess
  6. rtabmap SLAM subscribing to icp_odometry output
  7. Nav2 using /fusion/odom

Usage:
  export TURTLEBOT3_MODEL=waffle
  ros2 launch rtabmap_demos turtlebot3_sim_fusioncore_icp_demo.launch.py

  # Localization mode (requires existing map):
  ros2 launch rtabmap_demos turtlebot3_sim_fusioncore_icp_demo.launch.py localization:=true

  # Different world:
  ros2 launch rtabmap_demos turtlebot3_sim_fusioncore_icp_demo.launch.py world:=house

Note on TF ownership:
  The standard turtlebot3_gazebo bridge forwards the DiffDrive TF to ROS, which
  conflicts with FusionCore's odom->base_footprint. This demo uses a custom bridge
  config (fusioncore_tb3_bridge.yaml) that suppresses the Gazebo TF entry.
  FusionCore is the sole publisher of odom->base_footprint.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                             IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    if 'TURTLEBOT3_MODEL' not in os.environ:
        os.environ['TURTLEBOT3_MODEL'] = 'waffle'

    tb3_model   = os.environ['TURTLEBOT3_MODEL']
    pkg_tb3_gz  = get_package_share_directory('turtlebot3_gazebo')
    pkg_ros_gz  = get_package_share_directory('ros_gz_sim')
    pkg_nav2    = get_package_share_directory('nav2_bringup')
    pkg_demos   = get_package_share_directory('rtabmap_demos')

    world_name = LaunchConfiguration('world').perform(context)
    world_file = os.path.join(pkg_tb3_gz, 'worlds', f'turtlebot3_{world_name}.world')

    # ── Gazebo server + client ────────────────────────────────────────────────
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r -s -v2 {world_file}',
            'on_exit_shutdown': 'true',
        }.items(),
    )
    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-g -v2', 'on_exit_shutdown': 'true'}.items(),
    )

    # ── Robot state publisher ─────────────────────────────────────────────────
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gz, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # ── Spawn TurtleBot3 (entity only, no bridge) ─────────────────────────────
    urdf_path = os.path.join(pkg_tb3_gz, 'models',
                              f'turtlebot3_{tb3_model}', 'model.sdf')
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', tb3_model,
            '-file', urdf_path,
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', '0.01',
        ],
        output='screen',
    )

    # ── Custom bridge: all topics EXCEPT odom TF ──────────────────────────────
    # FusionCore publishes odom->base_footprint; suppress the Gazebo DiffDrive TF.
    bridge_config = os.path.join(pkg_demos, 'params', 'fusioncore_tb3_bridge.yaml')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        output='screen',
    )

    # Camera image bridge (waffle only)
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen',
    )

    # ── FusionCore + icp_odometry + rtabmap ───────────────────────────────────
    fusioncore_icp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_demos, 'launch', 'turtlebot3', 'fusioncore',
                         'turtlebot3_fusioncore_icp.launch.py')),
        launch_arguments=[
            ('localization', LaunchConfiguration('localization')),
            ('use_sim_time', 'true'),
        ],
    )

    # ── Nav2 ──────────────────────────────────────────────────────────────────
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')),
        launch_arguments=[
            ('use_sim_time', 'true'),
            ('params_file', PathJoinSubstitution(
                [FindPackageShare('rtabmap_demos'), 'params',
                 'turtlebot3_fusioncore_icp_nav2_params.yaml'])),
        ],
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'rviz_launch.py')),
    )

    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_tb3_gz, 'models'),
    )

    nodes = [
        set_gz_resource_path,
        gz_server,
        gz_client,
        robot_state_publisher,
        spawn_robot,
        bridge,
        fusioncore_icp,
        nav2,
        rviz,
    ]
    if tb3_model == 'waffle':
        nodes.insert(6, image_bridge)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),

        DeclareLaunchArgument(
            'world', default_value='world',
            choices=['world', 'house', 'dqn_stage1', 'dqn_stage2',
                     'dqn_stage3', 'dqn_stage4'],
            description='Turtlebot3 Gazebo world.'),

        DeclareLaunchArgument(
            'x_pose', default_value='-2.0',
            description='Initial X position in Gazebo.'),

        DeclareLaunchArgument(
            'y_pose', default_value='-0.5',
            description='Initial Y position in Gazebo.'),

        OpaqueFunction(function=launch_setup),
    ])
