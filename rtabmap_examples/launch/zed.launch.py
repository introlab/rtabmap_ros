# Requirements:
#   A ZED camera
#   Install zed ros2 wrapper package (https://github.com/stereolabs/zed-ros2-wrapper)
# Example:
#   $ ros2 launch rtabmap_examples zed.launch.py camera_model:=zed2i

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition

import tempfile

parameters = []
remappings = []

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

    parameters=[{'frame_id':'zed_camera_link',
                 'subscribe_rgbd':True,
                 'approx_sync':False,
                 'wait_imu_to_init':True}]

    remappings=[('imu', '/zed/zed_node/imu/data')]

    if use_zed_odometry:
        remappings.append(('odom', '/zed/zed_node/odom'))
    else:
        parameters.append({'subscribe_odom_info': True})
    
    return [
        # Launch camera driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('zed_wrapper'), 'launch'),
                '/zed_camera.launch.py']),
                launch_arguments={'camera_model': LaunchConfiguration('camera_model'),
                                    'ros_params_override_path': zed_override_file.name,
                                    'publish_tf': 'true',
                                    'publish_imu_tf': 'true',
                                    'publish_map_tf': 'false'}.items(),
        ),

        # Sync rgb/depth/camera_info together
        Node(   
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=parameters,
            remappings=[('rgb/image', '/zed/zed_node/rgb/color/rect/image'),
                        ('rgb/camera_info', '/zed/zed_node/rgb/color/rect/camera_info'),
                        ('depth/image', '/zed/zed_node/depth/depth_registered')]),

        # Visual odometry
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            condition=UnlessCondition(LaunchConfiguration('use_zed_odometry')),
            parameters=parameters,
            remappings=remappings,),

        # VSLAM
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

        # Visualization
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
