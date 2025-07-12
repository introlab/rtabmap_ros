
# Example to run rgbd datasets:
#
# [ROS1] Prepare ROS1 rosbag for conversion to ROS2
#   $ wget http://vision.in.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_long_office_household.bag
#   $ rosbag decompress rgbd_dataset_freiburg3_long_office_household.bag
#   $ wget https://gist.githubusercontent.com/matlabbe/897b775c38836ed8069a1397485ab024/raw/45e6ac01541973a17505000fc8c7ca399ae275eb/tum_rename_world_kinect_frame.py
#   $ python3 tum_rename_world_kinect_frame.py rgbd_dataset_freiburg3_long_office_household.bag
#   $ wget https://raw.githubusercontent.com/srv/srv_tools/kinetic/bag_tools/scripts/change_frame_id.py
#
#   Edit change_frame_id.py, remove/comment lines beginning with "PKG" and "import roslib", change line "Exception, e" to "Exception"
#   $ roscore
#   $ python3 change_frame_id.py -o rgbd_dataset_freiburg3_long_office_household_frameid_fixed.bag -i rgbd_dataset_freiburg3_long_office_household.bag -f openni_rgb_optical_frame -t /camera/rgb/image_color
#
# [ROS2]
#   $ sudo pip install rosbags     # See https://docs.openvins.com/dev-ros1-to-ros2.html
#   $ rosbags-convert --src rgbd_dataset_freiburg3_long_office_household_frameid_fixed.bag --dst rgbd_dataset_freiburg3_long_office_household_frameid_fixed
#
#   $ ros2 launch rtabmap_examples rgbdslam_datasets.launch.py
#   $ cd rgbd_dataset_freiburg3_long_office_household_frameid_fixed
#   $ ros2 bag play rgbd_dataset_freiburg3_long_office_household_frameid_fixed.db3 --clock
#
# To get RMSE after the run:
#   $ rtabmap-report ~/.ros/rtabmap.db

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import SetParameter

def generate_launch_description():

    odom_parameters=[{
        'frame_id':'kinect',
        # ground truth here is just used to align odometry with ground truth's first pose
        'ground_truth_frame_id':'world',
        'ground_truth_base_frame_id':'kinect_gt',
        'keep_color': True,
        'wait_for_transform': 0.5,
        # RTAB-Map's parameters should all be string type:
        'Odom/Strategy':'0',
        'Odom/ResetCountdown':'15',
        'Odom/GuessSmoothingDelay':'0',
    }]
    slam_parameters=[{
          'frame_id':'kinect',
          # Record ground truth to compute RMSE
          'ground_truth_frame_id':'world',
          'ground_truth_base_frame_id':'kinect_gt',
          'subscribe_rgb':False,
          'subscribe_depth':False,
          'subscribe_rgbd':True,
          'subscribe_odom_info':True,
          # RTAB-Map's parameters should all be string type:
          'Mem/UseOdomFeatures': 'true',
          'Rtabmap/StartNewMapOnLoopClosure':'true',
          'RGBD/CreateOccupancyGrid':'false',
          'Rtabmap/CreateIntermediateNodes':'true',
          'RGBD/LinearUpdate':'0',
          'RGBD/AngularUpdate':'0'}]
          
    odom_remappings=[
          ('rgb/image', '/camera/rgb/image_color'),
          ('rgb/camera_info', '/camera/rgb/camera_info'),
          ('depth/image', '/camera/depth/image')]
    
    # We will use the output of odometry to avoid re-extracting 
    # the same features on slam side.
    slam_remappings=[
        ("rgbd_image", "odom_rgbd_image")]
          

    return LaunchDescription([

        SetParameter(name='use_sim_time', value=True),
        # 'use_sim_time' will be set on all nodes following the line above

        # Nodes to launch
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=odom_parameters,
            remappings=odom_remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=slam_parameters,
            remappings=slam_remappings,
            arguments=['-d']),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=slam_parameters,
            remappings=slam_remappings),
       
        # /tf topic is missing in the converted ROS2 bag, create a fake tf
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0.0', '0.0', '0.0', '-1.57079632679', '0.0', '-1.57079632679', 'kinect', 'openni_rgb_optical_frame']),
    ])
