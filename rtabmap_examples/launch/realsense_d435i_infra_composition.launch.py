# Requirements:
#   A realsense D435i
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
# Example:
#   $ ros2 launch rtabmap_examples realsense_d435i_infra_composition.launch.py
#
# This is the "composition" variant of realsense_d435i_infra.launch.py: the
# camera driver, IMU filter, RGB-D odometry and SLAM all run as composable
# nodes in a single component container (rtabmap_container) with
# use_intra_process_comms enabled, so messages can be passed by pointer instead
# of being serialized/copied between processes.
#
# As in the non-composed example, the left infrared image (infra1) is used as
# the grayscale "RGB" input and paired with the depth stream. This works because
# on the D435i the depth is computed in the left-infrared frame, so infra1 and
# depth share the same intrinsics/frame (already registered, no align needed).
#
# Notes:
#  * Unlike the non-composed example, we do NOT include realsense2's rs_launch.py:
#    that launch file always starts the camera as a standalone node and exposes
#    no way to load it into an existing container. Instead we instantiate the
#    camera component (realsense2_camera::RealSenseNodeFactory) ourselves, the
#    same way realsense's own rs_intra_process_demo_launch.py does.
#  * ComposableNode has no "arguments" field, so the args/odom_args/-d
#    command-line mechanism of the non-composed example is not available here.
#    To override rtabmap parameters, add them directly to the 'parameters' dict
#    below. '-d' (delete database on start) becomes the 'delete_db_on_start'
#    parameter.
#
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    parameters={
          'frame_id':'camera_link',
          'subscribe_depth':True,
          'subscribe_odom_info':True,
          'approx_sync':False,
          'wait_imu_to_init':True}

    remappings=[
          ('imu', '/imu/data'),
          ('rgb/image', '/camera/infra1/image_rect_raw'),
          ('rgb/camera_info', '/camera/infra1/camera_info'),
          ('depth/image', '/camera/depth/image_rect_raw')]

    # Enable zero-copy intra-process communication on every composable node.
    intra_process = [{'use_intra_process_comms': True}]

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'unite_imu_method', default_value='2',
            description='0-None, 1-copy, 2-linear_interpolation. Use unite_imu_method:="1" if imu topics stop being published.'),
        DeclareLaunchArgument(
            'rtabmap_viz', default_value='true', description='Launch RTAB-Map UI (optional).'),

        # Single component container holding the whole pipeline.
        ComposableNodeContainer(
            name='rtabmap_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            composable_node_descriptions=[

                # Camera driver (replaces the rs_launch.py include).
                ComposableNode(
                    package='realsense2_camera', plugin='realsense2_camera::RealSenseNodeFactory',
                    name='camera', namespace='',
                    parameters=[{
                        'enable_gyro': True,
                        'enable_accel': True,
                        'unite_imu_method': LaunchConfiguration('unite_imu_method'),
                        'enable_infra1': True,
                        'enable_infra2': True,
                        'enable_sync': True,
                        'depth_module.emitter_enabled': 0}], # Hack to disable IR emitter
                    extra_arguments=intra_process),

                # Compute quaternion of the IMU
                ComposableNode(
                    package='imu_filter_madgwick', plugin='ImuFilterMadgwickRos',
                    name='imu_filter', namespace='',
                    parameters=[{'use_mag': False,
                                 'world_frame':'enu',
                                 'publish_tf':False}],
                    remappings=[('imu/data_raw', '/camera/imu')],
                    extra_arguments=intra_process),

                # RGB-D odometry (infra1 as grayscale RGB + depth)
                ComposableNode(
                    package='rtabmap_odom', plugin='rtabmap_odom::RGBDOdometry',
                    parameters=[parameters],
                    remappings=remappings,
                    extra_arguments=intra_process),

                # SLAM
                # Note: latch=False. rtabmap latches its map/cloud/octomap/mapGraph
                # topics by default (transient_local QoS), which is incompatible
                # with intra-process comms ("intraprocess communication allowed
                # only with volatile durability"). latch=False makes them volatile
                # so the node can join the zero-copy container.
                ComposableNode(
                    package='rtabmap_slam', plugin='rtabmap_slam::CoreWrapper',
                    parameters=[parameters,
                      {'delete_db_on_start': True, # Equivalent of '-d'
                       'latch': False}],
                    remappings=remappings,
                    extra_arguments=intra_process),
            ]),

        # Visualization:
        # Note: rtabmap_viz is launched as a standalone node, not as a component.
        # It is a Qt application and its UI must run in the process main thread,
        # while components run in container worker threads, so it cannot be
        # composed (the same applies to rviz2).
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
            parameters=[parameters],
            remappings=remappings),
    ])
