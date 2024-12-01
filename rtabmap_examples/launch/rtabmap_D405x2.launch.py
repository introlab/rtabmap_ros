# Program dedicated to launch 2 realsense cameras. D405 were tested!
# (Please note that this program can manage maximum 4 depth cameras)

# Program modified by: Adrian Ricardez (https://github.com/adricort)
# Date: 07.07.2023
# Deutsches Zentrum für Luft- und Raumfahrt

# Requirements:

# Be sure that you did the build on your rtabmap workspace with the -DRTABMAP_SYNC_MULTI_RGBD=ON parameter.

# Launching the 2 realsense cameras (change your serial numbers):
#   $ ros2 launch realsense2_camera rs_multi_camera_launch.py pointcloud.enable1:=true pointcloud.enable2:=true filters:=colorizer align_depth:=true serial_no1:=_128422271521 serial_no2:=_128422272518

# Running the static publishers depending on the position of your cameras:
#   $ ros2 run tf2_ros static_transform_publisher --x 0.039 --y 0 --z 0 --yaw 0 --pitch 0 --roll 1.5708 --frame-id base_link --child-frame-id camera1_link
#   $ ros2 run tf2_ros static_transform_publisher --x -0 --y 0 --z 0.02 --yaw 1.5708 --pitch -1.5708 --roll 0 --frame-id base_link --child-frame-id camera2_link

#   $ ros2 launch rtabmap_examples rtabmap_D405x2.launch.py

# You should be able to visualize now, with the right rviz config, the camera's SLAM
# Have fun!

import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
          
def generate_launch_description():     

    config_rviz = os.path.join(
            get_package_share_directory('rtabmap_examples'), 'config', 'slam_D405x2_config.rviz')         

    rviz_node = launch_ros.actions.Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=[["-d"], [config_rviz]]
    )

    rgbd_sync1_node = launch_ros.actions.Node(
        package='rtabmap_sync', executable='rgbd_sync', name='rgbd_sync1', output="screen",
        parameters=[{
            "approx_sync": False
            }],
        remappings=[
            ("rgb/image", '/camera1/color/image_rect_raw'),
            ("depth/image", '/camera1/depth/image_rect_raw'),
            ("rgb/camera_info", '/camera1/color/camera_info'),
            ("rgbd_image", 'rgbd_image')],
        namespace='realsense_camera1'
    )
    rgbd_sync2_node = launch_ros.actions.Node(
        package='rtabmap_sync', executable='rgbd_sync', name='rgbd_sync2', output="screen",
        parameters=[{
            "approx_sync": False
            }],
        remappings=[
            ("rgb/image", '/camera2/color/image_rect_raw'),
            ("depth/image", '/camera2/depth/image_rect_raw'),
            ("rgb/camera_info", '/camera2/color/camera_info'),
            ("rgbd_image", 'rgbd_image')],
        namespace='realsense_camera2'
    )
            
    # RGB-D odometry
    rgbd_odometry_node = launch_ros.actions.Node(
        package='rtabmap_odom', executable='rgbd_odometry', output="screen",
        parameters=[{
            "frame_id": 'base_link',
            "odom_frame_id": 'odom',
            "publish_tf": True,
            "approx_sync": True,
            "subscribe_rgbd": True,
            }],
        remappings=[
            ("rgbd_image", '/realsense_camera1/rgbd_image'),
            ("odom", 'odom')],
        arguments=["--delete_db_on_start", ''],
        prefix='',
        namespace='rtabmap'
    )

    # SLAM 
    slam_node = launch_ros.actions.Node(
        package='rtabmap_slam', executable='rtabmap', output="screen",
        parameters=[{
            "rgbd_cameras":2,
            "subscribe_depth": True,
            "subscribe_rgbd": True,
            "subscribe_rgb": True,
            "subscribe_odom_info": True,
            "frame_id": 'base_link',
            "map_frame_id": 'map',
            "publish_tf": True,
            "database_path": '~/.ros/rtabmap.db',
            "approx_sync": True,
            "Mem/IncrementalMemory": "true",
            "Mem/InitWMWithAllNodes": "true"
        }],
        remappings=[
            ("rgbd_image0", '/realsense_camera1/rgbd_image'),
            ("rgbd_image1", '/realsense_camera2/rgbd_image'),
            ("odom", 'odom')],
        arguments=["--delete_db_on_start"],
        prefix='',
        namespace='rtabmap'
    )

    voxelcloud1_node = launch_ros.actions.Node(
        package='rtabmap_util', executable='point_cloud_xyzrgb', name='point_cloud_xyzrgb1', output='screen',
        parameters=[{
            "approx_sync": True,
        }],
        remappings=[
            ('rgb/image', '/camera1/color/image_rect_raw'),
            ('depth/image', '/camera1/depth/image_rect_raw'),
            ('rgb/camera_info', '/camera1/color/camera_info'),
            ('rgbd_image', 'rgbd_image'),
            ('cloud', 'voxel_cloud1')]
    )

    voxelcloud2_node = launch_ros.actions.Node(
        package='rtabmap_util', executable='point_cloud_xyzrgb', name='point_cloud_xyzrgb2', output='screen',
        parameters=[{
            "approx_sync": True,
        }],
        remappings=[
            ('rgb/image', '/camera2/color/image_rect_raw'),
            ('depth/image', '/camera2/depth/image_rect_raw'),
            ('rgb/camera_info', '/camera2/color/camera_info'),
            ('rgbd_image', 'rgbd_image'),
            ('cloud', 'voxel_cloud2')]
    )    

    return launch.LaunchDescription(
        [
        rviz_node,
        rgbd_sync1_node,
        rgbd_sync2_node,
        rgbd_odometry_node,
        slam_node,
        voxelcloud1_node,
        voxelcloud2_node
        ]
    )