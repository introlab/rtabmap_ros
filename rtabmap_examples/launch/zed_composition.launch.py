# Requirements:
#   A ZED camera
#   Install zed ros2 wrapper package (https://github.com/stereolabs/zed-ros2-wrapper)
# Example:
#   $ ros2 launch rtabmap_examples zed_composition.launch.py camera_model:=zed2i
#
# This is the "composition" variant of zed.launch.py: the ZED driver, RGB-D
# synchronization, visual odometry and SLAM all run as composable nodes in a
# single component container with use_intra_process_comms enabled, so messages
# can be passed by pointer instead of being serialized/copied between processes.
#
# Notes:
#  * The ZED wrapper's zed_camera.launch.py already creates its own component
#    container ("zed_container") and loads the ZedCamera component into it with
#    intra-process comms enabled by default (enable_ipc:=true). So instead of
#    creating our own container, we let the ZED wrapper create it and load the
#    rtabmap nodes into the SAME container (/zed/zed_container) with
#    LoadComposableNodes. This assumes the default ZED namespace ("zed"), which
#    is independent of camera_model.
#  * ComposableNode has no "arguments" or "condition" field. So the '-d'
#    argument becomes the 'delete_db_on_start' parameter, and the conditional
#    odometry node is included in Python depending on use_zed_odometry.
#
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

import tempfile

def launch_setup(context: LaunchContext, *args, **kwargs):

    use_zed_odometry = LaunchConfiguration('use_zed_odometry').perform(context) in ["True", "true"]

    # Override some ZED parameters without changing any files:
    #  * grab_resolution: VGA
    #  * pos_tracking_enabled: disabled when rtabmap computes the odometry, so
    #    the ZED node does not publish the odom->camera_link TF (which would
    #    conflict with rtabmap's odometry). We still set publish_tf:=true below
    #    so the ZED node keeps broadcasting the IMU TF, which rtabmap needs
    #    (wait_imu_to_init). sensors.publish_imu_tf is ignored when publish_tf
    #    is false, and the IMU frame is not in the ZED URDF, so this is the only
    #    way to get the IMU TF while rtabmap owns the odometry.
    pos_tracking_enabled = 'true' if use_zed_odometry else 'false'
    with tempfile.NamedTemporaryFile(mode='w+t', delete=False) as zed_override_file:
        zed_override_file.write("---\n"+
                  "/**:\n"+
                  "    ros__parameters:\n"+
                  "        general:\n"+
                  "            grab_resolution: 'VGA'\n"+
                  "        pos_tracking:\n"+
                  "            pos_tracking_enabled: "+pos_tracking_enabled)

    # ZED topics are /zed/zed_node/* and the container created by the wrapper is
    # /zed/zed_container (default ZED namespace "zed").
    zed_ns = '/zed/zed_node'
    zed_container = '/zed/zed_container'

    parameters=[{'frame_id':'zed_camera_link',
                 'subscribe_rgbd':True,
                 'approx_sync':False,
                 'wait_imu_to_init':True}]

    remappings=[('imu', zed_ns + '/imu/data')]

    if use_zed_odometry:
        remappings.append(('odom', zed_ns + '/odom'))
    else:
        parameters.append({'subscribe_odom_info': True})

    # Enable zero-copy intra-process communication on every composable node.
    intra_process = [{'use_intra_process_comms': True}]

    # rtabmap nodes loaded into the ZED container.
    composable_nodes = [
        # Sync rgb/depth/camera_info together
        ComposableNode(
            package='rtabmap_sync', plugin='rtabmap_sync::RGBDSync',
            parameters=parameters,
            remappings=[('rgb/image', zed_ns + '/rgb/color/rect/image'),
                        ('rgb/camera_info', zed_ns + '/rgb/color/rect/camera_info'),
                        ('depth/image', zed_ns + '/depth/depth_registered')],
            extra_arguments=intra_process),
    ]

    # Visual odometry (only when not using ZED's own odometry). ComposableNode
    # has no 'condition', so we add it here based on use_zed_odometry.
    if not use_zed_odometry:
        composable_nodes.append(
            ComposableNode(
                package='rtabmap_odom', plugin='rtabmap_odom::RGBDOdometry',
                parameters=parameters,
                remappings=remappings,
                extra_arguments=intra_process))

    # VSLAM
    # Note: latch=False. rtabmap latches its map/cloud/octomap/mapGraph topics
    # by default (transient_local QoS), which is incompatible with intra-process
    # comms ("intraprocess communication allowed only with volatile durability").
    # latch=False makes them volatile so the node can join the container.
    composable_nodes.append(
        ComposableNode(
            package='rtabmap_slam', plugin='rtabmap_slam::CoreWrapper',
            parameters=parameters + [{'delete_db_on_start': True, # Equivalent of '-d'
                                      'latch': False}],
            remappings=remappings,
            extra_arguments=intra_process))

    return [
        # Launch camera driver. It creates the "zed_container" component
        # container (enable_ipc:=true by default) and loads the ZedCamera
        # component into it.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('zed_wrapper'), 'launch'),
                '/zed_camera.launch.py']),
                launch_arguments={'camera_model': LaunchConfiguration('camera_model'),
                                    'ros_params_override_path': zed_override_file.name,
                                    # publish_tf must be true so the ZED node broadcasts the
                                    # IMU TF (gated by publish_tf). The odom->camera_link TF is
                                    # disabled via pos_tracking_enabled=false (override file)
                                    # when rtabmap computes the odometry.
                                    'publish_tf': 'true',
                                    'publish_imu_tf': 'true',
                                    'publish_map_tf': 'false'}.items(),
        ),

        # Load the rtabmap pipeline into the ZED container.
        LoadComposableNodes(
            target_container=zed_container,
            composable_node_descriptions=composable_nodes),

        # Visualization
        # Note: rtabmap_viz is a Qt application; its UI must run in the process
        # main thread, while components run in container worker threads. So it
        # cannot be composed and stays a standalone node.
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings)
    ]


def generate_launch_description():
    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_zed_odometry', default_value='false',
            description='Use zed\'s computed odometry instead of using rtabmap\'s odometry.'),

        DeclareLaunchArgument(
            'camera_model', default_value='',
            description="[REQUIRED] The model of the camera. Using a wrong camera model can disable camera features. Valid choices are: ['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'virtual']"),

        OpaqueFunction(function=launch_setup)
    ])
