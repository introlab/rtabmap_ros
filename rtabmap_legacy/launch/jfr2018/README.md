

This is the raw files used to generate results for MIT Stata Center dataset of this paper (for KITTI, EuRoC and TUM datasets, see this [page](https://github.com/introlab/rtabmap/tree/master/docker/jfr2018)):
 * M. Labbé and F. Michaud, “RTAB-Map as an Open-Source Lidar and Visual SLAM Library for Large-Scale and Long-Term Online Operation,” in Journal of Field Robotics, accepted, 2018. ([pdf](https://introlab.3it.usherbrooke.ca/mediawiki-introlab/images/7/7a/Labbe18JFR_preprint.pdf)) ([Wiley](https://doi.org/10.1002/rob.21831))

Manual instructions are in [launch_lidar](https://github.com/introlab/rtabmap_ros/blob/master/rtabmap_legacy/launch/jfr2018/launch_lidar), [launch_stereo](https://github.com/introlab/rtabmap_ros/blob/master/rtabmap_legacy/launch/jfr2018/launch_stereo) and [launch_rgbd](https://github.com/introlab/rtabmap_ros/blob/master/rtabmap_legacy/launch/jfr2018/launch_rgbd) files depending on the sensor configuration. Refer also to explanations in the paper. Below are some examples of usage per sensor type.

# Long-Range LiDAR with WheelIMU→S2M odometry

1. Launch rtabmap following config "Long-Range LiDAR with WheelIMU→S2M odometry"
   ```
   roslaunch rtabmap_launch rtabmap.launch \
      args:="\
         -d \
         --Rtabmap/PublishRAMUsage true \
         --Rtabmap/StartNewMapOnLoopClosure true \
         --Reg/Force3DoF false \
         --RGBD/ProximityPathMaxNeighbors 0 \
         --Mem/STMSize 15 \
         --Mem/BinDataKept true \
         --Kp/FlannRebalancingFactor 1.0 \
         --RGBD/LinearUpdate 0 \
         --RGBD/ProximityBySpace true \
         --RGBD/OptimizeMaxError 3 \
         --FAST/Threshold 7" \
      odom_args:="\
         --Odom/Strategy 0 \
         --Vis/CorType 0 \
         --Odom/KeyFrameThr 0.3 \
         --OdomF2M/MaxSize 2000 \
         --uwarn" \
      rgbd_sync:=true \
      depth_scale:=1.043 \
      frame_id:=base_footprint \
      ground_truth_frame_id:=world \
      ground_truth_base_frame_id:=scan_gt \
      use_sim_time:=true  \
      odom_topic:=odom \
      rgb_topic:=/camera/rgb/image_raw \
      depth_topic:=/camera/depth/image_raw \
      camera_info_topic:=/camera/rgb/camera_info \
      approx_sync:=false \
      odom_guess_frame_id:=odom_combined \
      odom_always_process_most_recent_frame:=false
   ```

2. Publish the ground truth in TF
   ```
   python3 gt_tf_broadcaster.py \
      _file:=2012-01-25-12-33-29_part1_floor2.gt.laser.poses \
      _frame_id:=scan_gt \
      _fixed_frame_id:=world \
      _offset_time:=82.2 \
      _offset_x:=-0.275
   ```

3. Launch the rosbag. Make sure you are running the rosbag on a SSD directly connected inside the computer (not USB SSD) to limit the lags. Note that in contrast to visual odometry approaches below, the lags seem affecting less lidar odometry, so we are still using the original rosbag here. 
   ```
   rosbag play --clock --pause 2012-01-25-12-33-29.bag
   ```

# RGB-D Camera with F2M odometry
1. To avoid lags when replaying the original rosbag, just extract the RGB-D data into another rosbag. With the script in this folder, do:
   ```
   python3 extract_rgbd.py 2012-01-25-12-33-29
   ```
   This will create a new rosbag called `2012-01-25-12-33-29_rgbd.bag`.

2. Launch rtabmap following config "RGB-D Camera with F2M odometry"
   ```
   roslaunch rtabmap_launch rtabmap.launch \
      args:="-d \
         --Rtabmap/PublishRAMUsage true \
         --Rtabmap/StartNewMapOnLoopClosure true \
         --Reg/Force3DoF false \
         --RGBD/ProximityPathMaxNeighbors 0 \
         --Mem/STMSize 15 \
         --Mem/BinDataKept true \
         --Kp/FlannRebalancingFactor 1.0 \
         --RGBD/LinearUpdate 0 \
         --RGBD/ProximityBySpace true \
         --RGBD/OptimizeMaxError 3 \
         --GFTT/QualityLevel 0.01 \
         --Vis/MinInliers 10" \
      odom_args:="\
         --Odom/Strategy 0 \
         --Vis/CorType 0 \
         --Odom/KeyFrameThr 0.3 \
         --OdomF2M/MaxSize 2000 \
         --uwarn" \
      rgbd_sync:=true \
      depth_scale:=1.043 \
      frame_id:=base_footprint \
      ground_truth_frame_id:=world \
      ground_truth_base_frame_id:=scan_gt \
      use_sim_time:=true  \
      odom_topic:=odom \
      rgb_topic:=/camera/rgb/image_raw \
      depth_topic:=/camera/depth/image_raw  \
      camera_info_topic:=/camera/rgb/camera_info  \
      approx_sync:=false \
      approx_sync_max_interval:=0.015 \
      odom_guess_frame_id:=odom_combined \
      odom_always_process_most_recent_frame:=false
   ```
3. Publish the ground truth in TF
   ```
   python3 gt_tf_broadcaster.py \
      _file:=2012-01-25-12-33-29_part1_floor2.gt.laser.poses \
      _frame_id:=scan_gt \
      _fixed_frame_id:=world \
      _offset_time:=82.2 \
      _offset_x:=-0.275
   ```

4. Launch the rosbag. Make sure you are running the rosbag on a SSD directly connected inside the computer (not USB SSD) to limit the lags.
   ```
   rosbag play --clock --pause 2012-01-25-12-33-29_rgbd.bag
   ```

# Stereo Camera with F2M odometry
1. To avoid lags when replaying the original rosbag, just extract the stereo data into another rosbag. With the script in this folder, do:
   ```
   python3 extract_stereo.py 2012-01-25-12-33-29
   ```
   This will create a new rosbag called `2012-01-25-12-33-29_stereo.bag`.

2. Launch rtabmap following config "Stereo Camera with F2M odometry"
   ```
   roslaunch rtabmap_launch rtabmap.launch \
      args:="-d \
         --Rtabmap/PublishRAMUsage true \
         --Rtabmap/StartNewMapOnLoopClosure true \
         --Reg/Force3DoF false \
         --RGBD/ProximityPathMaxNeighbors 0 \
         --Mem/STMSize 15 \
         --Mem/BinDataKept true \
         --Kp/FlannRebalancingFactor 1.0 \
         --RGBD/LinearUpdate 0 \
         --RGBD/ProximityBySpace true \
         --Odom/KeyFrameThr 0.3 \
         --Odom/Strategy 0 \
         --OdomF2M/MaxSize 2000 \
         --RGBD/OptimizeMaxError 3 \
         --GFTT/QualityLevel 0.01 \
         --Vis/MinInliers 10" \
      odom_args:="--Vis/CorType 0 --uwarn" \
      frame_id:=base_footprint \
      ground_truth_frame_id:=world \
      ground_truth_base_frame_id:=scan_gt \
      use_sim_time:=true \
      stereo:=true \
      stereo_namespace:=/wide_stereo \
      left_camera_info_topic:=/wide_stereo/left/camera_info \
      right_camera_info_topic:=/wide_stereo/right/camera_info_scaled \
      odom_topic:=odom  \
      approx_sync:=false \
      odom_guess_frame_id:=odom_combined \
      odom_always_process_most_recent_frame:=false
   ```
3. Publish the ground truth in TF
   ```
   python3 gt_tf_broadcaster.py \
      _file:=2012-01-25-12-33-29_part1_floor2.gt.laser.poses \
      _frame_id:=scan_gt \
      _fixed_frame_id:=world \
      _offset_time:=82.2 \
      _offset_x:=-0.275
   ```

4. Republish the stereo camera calibration with baseline correctly scaled
   ```
   python3 republish_camera_info.py \
      camera_info_in:=/wide_stereo/right/camera_info \
      camera_info_out:=/wide_stereo/right/camera_info_scaled
   ```

5. Rectify the raw stereo images
   ```
   export ROS_NAMESPACE=wide_stereo
   rosrun stereo_image_proc stereo_image_proc \
      left/image_raw:=left/image_raw \
      right/image_raw:=right/image_raw \
      left/camera_info:=left/camera_info \
      right/camera_info:=right/camera_info_scaled
   ```

6. Launch the rosbag. Make sure you are running the rosbag on a SSD directly connected inside the computer (not USB SSD) to limit the lags.
   ```
   rosbag play --clock --pause 2012-01-25-12-33-29_stereo.bag
   ```
