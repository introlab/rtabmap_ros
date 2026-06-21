# Requirements:
#   Download one or both rosbags:
#    * stereo_outdoorA.db3: https://drive.google.com/file/d/1O7mCXg_sw4tZY1S88a-n96O6OulmqvqI/view?usp=drive_link
#    * stereo_outdoorB.db3: https://drive.google.com/file/d/1mSu7418Fkbe-hIz2-3Mi936PrWuD2un_/view?usp=drive_link
#
# This is the "composition" variant of stereo_outdoor_demo.launch.py: the whole
# pipeline (image_proc rectification, stereo synchronization, visual odometry
# and SLAM) runs as composable nodes in a single component container
# (rtabmap_container). We can set 'use_intra_process_comms' on all of them.
# That way images are passed between rectify -> disparity/sync -> odometry ->
# SLAM by pointer, without inter-process serialization/copies.
#
# Example:
#
#   SLAM:
#     $ ros2 launch rtabmap_demos stereo_outdoor_demo_composition.launch.py  rviz:=true rtabmap_viz:=true
#
#   Rosbag:
#     $ ros2 bag play stereo_outdoorA.db3 --clock
#     when done, you can play the secon bag:
#     $ ros2 bag play stereo_outdoorB.db3 --clock
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetParameter, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    localization = LaunchConfiguration('localization')

    parameters={
          'frame_id':'base_footprint',
          'subscribe_rgbd':True,
          'approx_sync':False, # odom is generated from images, so we can exactly sync all inputs
          'map_negative_poses_ignored':True,
          'subscribe_odom_info': True,
          # RTAB-Map's internal parameters should be strings
          'OdomF2M/MaxSize': '1000',
          'GFTT/MinDistance': '10',
          'GFTT/QualityLevel': '0.00001',
          #'Kp/DetectorStrategy': '6', # Uncommment to match ros1 noetic results, but opencv should be built with xfeatures2d
          #'Vis/FeatureType': '6'      # Uncommment to match ros1 noetic results, but opencv should be built with xfeatures2d
    }

    remappings=[
         ('rgbd_image', '/stereo_camera/rgbd_image'),
         ('odom',       '/vo')]

    # Enable zero-copy intra-process communication between all composable nodes
    # loaded in the container.
    intra_process = [{'use_intra_process_comms': True}]

    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_demos'), 'config', 'demo_robot_mapping.rviz'
    )

    # ---- image_proc rectification per camera ----
    def image_proc_nodes(side, color=False):
        ns = 'stereo_camera/' + side
        rectify = ComposableNode(
            package='image_proc', plugin='image_proc::RectifyNode',
            name='rectify_color_node' if color else 'rectify_mono_node', namespace=ns,
            remappings=[
                ('image',       'image_color' if color else 'image_mono'),
                ('camera_info', 'camera_info_throttle'),
                ('image_rect',  'image_rect_color' if color else 'image_rect')],
            extra_arguments=intra_process)
        return [
            ComposableNode(
                package='image_proc', plugin='image_proc::DebayerNode',
                name='debayer_node', namespace=ns,
                extra_arguments=intra_process),
            rectify,
        ]

    # ---- rtabmap pipeline (always-on nodes) ----
    rtabmap_nodes = [
        # Synchronize stereo data together in a single topic
        # Issue: stereo_img_proc doesn't produce color and
        #        grayscale images exactly the same (there is a small
        #        vertical shift with color), we should use grayscale for
        #        left and right images to get similar results than on ros1 noetic.
        ComposableNode(
            package='rtabmap_sync', plugin='rtabmap_sync::StereoSync',
            namespace='stereo_camera',
            remappings=[
                ('left/image_rect',   'left/image_rect'),
                ('right/image_rect',  'right/image_rect'),
                ('left/camera_info',  'left/camera_info_throttle'),
                ('right/camera_info', 'right/camera_info_throttle')],
            extra_arguments=intra_process),

        # Visual odometry
        ComposableNode(
            package='rtabmap_odom', plugin='rtabmap_odom::StereoOdometry',
            parameters=[parameters],
            remappings=remappings,
            extra_arguments=intra_process),
    ]
    
    # Name of the shared component container.
    container_name = '/rtabmap_container'

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument('rtabmap_viz',  default_value='false',  description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('rviz',         default_value='true',   description='Launch RVIZ (optional).'),
        DeclareLaunchArgument('localization', default_value='false',  description='Launch in localization mode.'),
        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz,  description='Configuration path of rviz2.'),

        SetParameter(name='use_sim_time', value=True),

        # Nodes to launch

        # Uncompress images for stereo_image_rect and remap to expected names from stereo_image_proc.
        Node(
            package='image_transport', executable='republish', name='republish_left', output='screen',
            namespace='stereo_camera',
            arguments=['compressed', 'raw'],
            remappings=[('in/compressed', 'left/image_raw_throttle/compressed'),
                        ('out',           'left/image_raw')]),
        Node(
            package='image_transport', executable='republish', name='republish_right', output='screen',
            namespace='stereo_camera',
            arguments=['compressed', 'raw'],
            remappings=[('in/compressed', 'right/image_raw_throttle/compressed'),
                        ('out',           'right/image_raw')]),

        # Single component container holding the whole pipeline. All nodes set
        # use_intra_process_comms=True, so images are passed by pointer.
        ComposableNodeContainer(
            name='rtabmap_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            composable_node_descriptions=
                image_proc_nodes('left') +
                image_proc_nodes('right') +
                rtabmap_nodes),

        # SLAM mode (loaded into the shared container):
        # Note: latch=False. rtabmap latches its map/cloud/octomap/mapGraph
        # topics by default (transient_local QoS), which is incompatible with
        # intra-process comms ("intraprocess communication allowed only with
        # volatile durability"). Setting latch=False makes those topics volatile
        # so the node can join the zero-copy container. Trade-off: viewers that
        # start after a map is published won't get the retained last message,
        # but rtabmap republishes the map as it updates.
        LoadComposableNodes(
            condition=UnlessCondition(localization),
            target_container=container_name,
            composable_node_descriptions=[
                ComposableNode(
                    package='rtabmap_slam', plugin='rtabmap_slam::CoreWrapper',
                    parameters=[parameters,
                      {'delete_db_on_start': True, # Equivalent of '-d': delete the previous database (~/.ros/rtabmap.db)
                       'latch': False}],
                    remappings=remappings,
                    extra_arguments=intra_process),
            ]),

        # Localization mode (loaded into the shared container):
        LoadComposableNodes(
            condition=IfCondition(localization),
            target_container=container_name,
            composable_node_descriptions=[
                ComposableNode(
                    package='rtabmap_slam', plugin='rtabmap_slam::CoreWrapper',
                    parameters=[parameters,
                      {'Mem/IncrementalMemory':'False',
                       'Mem/InitWMWithAllNodes':'True',
                       'latch': False}],  # volatile QoS, see SLAM-mode note above
                    remappings=remappings,
                    extra_arguments=intra_process),
            ]),

        # Visualization:
        # Note: rtabmap_viz is launched as a standalone node, not as a component
        # in the container above. It is a Qt application and its UI must run in
        # the process main thread, while components are loaded in container
        # worker threads. So it cannot be composed and does not benefit from
        # intra-process comms here (the same applies to rviz2).
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
            parameters=[parameters,
                        {"odometry_node_name": 'stereo_odometry'}],
            remappings=remappings),
        Node(
            package='rviz2', executable='rviz2', name="rviz2", output='screen',
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=[["-d"], [LaunchConfiguration("rviz_cfg")]]),
    ])
