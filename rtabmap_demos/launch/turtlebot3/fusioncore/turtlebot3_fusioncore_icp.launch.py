"""
FusionCore + icp_odometry feedback loop for TurtleBot3.

Architecture (Option A from rtabmap_ros issue #1418):

  FusionCore (wheels + IMU)
    |-- publishes: odom -> base_footprint TF,  /fusion/odom
    |-- provides initial pose guess to icp_odometry via guess_frame_id

  icp_odometry (/scan)
    |-- guess_frame_id: odom   (uses FusionCore's stable odom as scan match seed)
    |-- publish_tf: false      (FusionCore owns the odom TF)
    |-- publishes: /rtabmap/icp_odometry

  FusionCore encoder2
    |-- topic: /rtabmap/icp_odometry
    |-- ICP corrections fed back as a second velocity source

  rtabmap SLAM
    |-- subscribes to /rtabmap/icp_odometry for mapping
    |-- Odom/ResetCountdown: 1 for auto-recovery if ICP loses tracking
    |-- publishes: map -> odom TF
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                             OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization').perform(context)
    localization = localization in ('True', 'true')

    pkg_demos = get_package_share_directory('rtabmap_demos')
    fusioncore_config = os.path.join(pkg_demos, 'params', 'fusioncore_tb3.yaml')

    # ── FusionCore lifecycle node ─────────────────────────────────────────────
    fc = LifecycleNode(
        package='fusioncore_ros',
        executable='fusioncore_node',
        name='fusioncore',
        namespace='',
        output='screen',
        parameters=[fusioncore_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/imu/data',    '/imu'),    # TB3 Gazebo IMU topic
            ('/odom/wheels', '/odom'),   # TB3 Gazebo wheel odometry topic
        ],
    )

    # Wait 2 s for node to spin up, then configure
    configure = TimerAction(
        period=2.0,
        actions=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=lambda a: a is fc,
            transition_id=Transition.TRANSITION_CONFIGURE,
        ))],
    )

    # As soon as configuring -> inactive, activate
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=fc,
        start_state='configuring',
        goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=lambda a: a is fc,
            transition_id=Transition.TRANSITION_ACTIVATE,
        ))],
    ))

    # ── icp_odometry ──────────────────────────────────────────────────────────
    icp_parameters = {
        'frame_id':               'base_footprint',
        'odom_frame_id':          'odom',
        'guess_frame_id':         'odom',
        'publish_tf':             False,
        'publish_null_when_lost': False,
        'use_sim_time':           use_sim_time,
        'Reg/Strategy':           '1',
        'Reg/Force3DoF':          'true',
        'Odom/ResetCountdown':    '1',
        'RGBD/NeighborLinkRefining': 'True',
        'Grid/RangeMin':          '0.2',
    }

    icp_odometry_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        output='screen',
        parameters=[icp_parameters],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/rtabmap/icp_odometry'),
        ],
    )

    # ── rtabmap SLAM ──────────────────────────────────────────────────────────
    slam_parameters = {
        'frame_id':              'base_footprint',
        'odom_frame_id':         'odom',
        'use_sim_time':          use_sim_time,
        'subscribe_depth':       False,
        'subscribe_rgb':         False,
        'subscribe_scan':        True,
        'approx_sync':           True,
        'use_action_for_goal':   True,
        'Reg/Strategy':          '1',
        'Reg/Force3DoF':         'true',
        'RGBD/NeighborLinkRefining': 'True',
        'Grid/RangeMin':         '0.2',
        'Optimizer/GravitySigma': '0',
        'Odom/ResetCountdown':   '1',
    }

    if localization:
        slam_parameters['Mem/IncrementalMemory'] = 'False'
        slam_parameters['Mem/InitWMWithAllNodes'] = 'True'

    rtabmap_args = [] if localization else ['-d']

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[slam_parameters],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/rtabmap/icp_odometry'),
        ],
        arguments=rtabmap_args,
    )

    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        output='screen',
        parameters=[slam_parameters, {'odometry_node_name': 'icp_odometry'}],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/rtabmap/icp_odometry'),
        ],
    )

    return [fc, configure, activate, icp_odometry_node, rtabmap_node, rtabmap_viz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode (requires existing map)'),

        OpaqueFunction(function=launch_setup),
    ])
