# rtabmap_demos
+ [Outdoor Stereo VSLAM](#outdoor-stereo-vslam)
+ [Indoor 2D LiDAR and RGB-D SLAM](#indoor-2d-lidar-and-rgb-d-slam)
+ [Multi-Session Indoor 2D LiDAR and RGB-D SLAM](#multi-session-indoor-2d-lidar-and-rgb-d-slam)
+ [Find-Object with SLAM](#find-object-with-slam)
+ [Turtlebot4 Nav2, 2D LiDAR and RGB-D SLAM](#turtlebot4-nav2-2d-lidar-and-rgb-d-slam)
+ [Turtlebot3 Nav2 and 2D LiDAR SLAM](#turtlebot3-nav2-and-2d-lidar-slam)
+ [Turtlebot3 Nav2 and RGB-D SLAM](#turtlebot3-nav2-and-rgb-d-slam)
+ [Turtlebot3 Nav2, 2D LiDAR and RGB-D SLAM](#turtlebot3-nav2-2d-lidar-and-rgb-d-slam)
+ [Turtlebot3 Nav2, Fake 2D LiDAR and RGB-D SLAM](#turtlebot3-nav2-fake-2d-lidar-and-rgb-d-slam)
+ [Champ Quadruped Nav2, Elevation Map and VSLAM](#champ-quadruped-nav2-elevation-map-and-vslam)
+ [Clearpath Husky Nav2, 2D LiDAR and RGB-D SLAM](#clearpath-husky-nav2-2d-lidar-and-rgb-d-slam)
+ [Clearpath Husky Nav2, 3D LiDAR and RGB-D SLAM](#clearpath-husky-nav2-3d-lidar-and-rgb-d-slam)
+ [Clearpath Husky Nav2, 3D LiDAR Assembling and RGB-D SLAM](#clearpath-husky-nav2-3d-lidar-assembling-and-rgb-d-slam)
+ [Isaac Sim Nav2 and Stereo SLAM](#isaac-sim-nav2-and-stereo-slam)
+ [Isaac Sim Nav2 and RGB-D VSLAM](#isaac-sim-nav2-and-rgb-d-vslam)

### Outdoor Stereo VSLAM
[stereo_outdoor_demo.launch.py](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_demos/launch/stereo_outdoor_demo.launch.py) ([Video](https://youtu.be/qpTS7kg9J3A))

![Peek 2024-11-29 10-52](https://github.com/user-attachments/assets/b6dd4a1c-5bd5-4cfa-936d-e8e707bbcb23)

### Indoor 2D LiDAR and RGB-D SLAM
[robot_mapping_demo.launch.py](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_demos/launch/robot_mapping_demo.launch.py) (Videos: [rtabmap_viz](https://youtu.be/c0qrEd5rR7M), [rviz](https://youtu.be/MQoSDpAsqps))

![Peek 2024-11-29 11-07](https://github.com/user-attachments/assets/b02beeea-28ed-4fde-932d-c89bef1a046d)

### Multi-Session Indoor 2D LiDAR and RGB-D SLAM
[multisession_mapping_demo.launch.py](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_demos/launch/multisession_mapping_demo.launch.py) ([Video](https://youtu.be/XrnyhaxPCro))

![Peek 2024-11-29 11-48](https://github.com/user-attachments/assets/b130e5ab-618f-4c8b-840f-f926b65ab53b)

### Find-Object with SLAM
[find_object_demo.launch.py](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_demos/launch/find_object_demo.launch.py) ([Video](https://youtu.be/o1GSQanY-Do))

![Peek 2024-11-29 12-01](https://github.com/user-attachments/assets/b3cc0c67-517a-4f69-b4cc-35d288e96165)

### Turtlebot4 Nav2, 2D LiDAR and RGB-D SLAM
[turtlebot4_sim_demo.launch.py](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_demos/launch/turtlebot4/turtlebot4_sim_demo.launch.py)

![Peek 2024-11-29 12-19](https://github.com/user-attachments/assets/5914e34c-19f1-4b7c-b4df-2e7084946888)
### Turtlebot3 Nav2 and 2D LiDAR SLAM
[turtlebot3_sim_scan_demo.launch.py](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_demos/launch/turtlebot3/turtlebot3_sim_scan_demo.launch.py)

![Peek 2024-11-29 12-23](https://github.com/user-attachments/assets/e3c31c5a-5c46-4370-ad17-38c795db7917)
### Turtlebot3 Nav2 and RGB-D SLAM
[turtlebot3_sim_rgbd_demo.launch.py](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_demos/launch/turtlebot3/turtlebot3_sim_rgbd_demo.launch.py)

![Peek 2024-11-29 14-22](https://github.com/user-attachments/assets/5088be17-0875-42cc-b863-d14468c67f26)
### Turtlebot3 Nav2, 2D LiDAR and RGB-D SLAM
[turtlebot3_sim_rgbd_scan_demo.launch.py](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_demos/launch/turtlebot3/turtlebot3_sim_rgbd_scan_demo.launch.py)

![Peek 2024-11-29 13-41](https://github.com/user-attachments/assets/2e878158-b1b6-48a4-801c-72cdb41b4783)
### Turtlebot3 Nav2, Fake 2D LiDAR and RGB-D SLAM
[turtlebot3_sim_rgbd_fake_scan_demo.launch.py](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_demos/launch/turtlebot3/turtlebot3_sim_rgbd_fake_scan_demo.launch.py)

 * Red: Scan generated from camera's depth.
 * Orange: Locally assembled scans used for proximity detection.
 * Yellow: The map.

![Peek 2025-07-04 16-57](https://github.com/user-attachments/assets/8869cf57-35a1-4236-bdab-151b88ae2ea1)
### Champ Quadruped Nav2, Elevation Map and VSLAM
[champ_sim_vslam.launch.py](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_demos/launch/champ/champ_sim_vslam.launch.py)

![Peek 2024-11-29 15-00](https://github.com/user-attachments/assets/d1a27c78-27bc-4901-82a7-59b5d24e6454)
### Clearpath Husky Nav2, 2D LiDAR and RGB-D SLAM
[husky_sim_scan2d_demo.launch.py](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_demos/launch/husky/husky_sim_scan2d_demo.launch.py)

![Peek 2024-11-29 15-30](https://github.com/user-attachments/assets/c8f79b86-253e-4c8e-ac7a-c26584f43fa4)
### Clearpath Husky Nav2, 3D LiDAR and RGB-D SLAM
[husky_sim_scan3d_demo.launch.py](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_demos/launch/husky/husky_sim_scan3d_demo.launch.py)

![Peek 2024-11-29 15-36](https://github.com/user-attachments/assets/a4b6e6ae-38ed-44da-bbfb-d3c30a301f9c)
### Clearpath Husky Nav2, 3D LiDAR Assembling and RGB-D SLAM
[husky_sim_scan3d_assemble_demo.launch.py](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_demos/launch/husky/husky_sim_scan3d_assemble_demo.launch.py)

![Peek 2024-11-29 16-16](https://github.com/user-attachments/assets/b2235bd2-33d2-4c44-b6e9-9923a524632b)
### Isaac Sim Nav2 and Stereo SLAM
[isaac_sim_vslam_demo.launch.py](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_demos/launch/isaac/isaac_sim_vslam_demo.launch.py)

![Peek 2024-11-29 17-49](https://github.com/user-attachments/assets/54cd0c82-aaed-47e5-911a-f286b6d2cc17)
### Isaac Sim Nav2 and RGB-D VSLAM
[isaac_sim_vslam_demo.launch.py](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_demos/launch/isaac/isaac_sim_vslam_demo.launch.py) stereo:=false vo:=rtabmap

![Peek 2024-11-30 13-22](https://github.com/user-attachments/assets/240820c6-4dea-4cbf-9431-b4b3af695d51)
