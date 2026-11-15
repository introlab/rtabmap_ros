# rtabmap_demos

Ready-to-run demos of RTAB-Map for ROS 1 (Noetic). Most launch files set up a complete pipeline for one of the demo bags, so you can watch RTAB-Map map, close loops and localize without any hardware. For ROS 2, see [rtabmap_demos](https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_demos) on the `ros2` branch.

+ [Robot mapping](#robot-mapping)
+ [Robot mapping with RVIZ](#robot-mapping-with-rviz)
+ [Multi-session mapping](#multi-session-mapping)
+ [Robot mapping with Find-Object](#robot-mapping-with-find-object)
+ [IROS 2014 Kinect Challenge](#iros-2014-kinect-challenge)
+ [Stereo mapping](#stereo-mapping)
+ [Stereo navigation](#stereo-navigation)
+ [Appearance-based loop closure detection-only](#appearance-based-loop-closure-detection-only)

### Robot mapping
[![Robot mapping](https://img.youtube.com/vi/c0qrEd5rR7M/0.jpg)](https://www.youtube.com/watch?v=c0qrEd5rR7M)

For this demo, you will need the ROS bag [demo_mapping.bag](https://drive.google.com/file/d/1RJxe6nU-of5_AbyoYGobNi6y-Z00Hyh7/view?usp=drive_link) (295 MB, **fixed camera TF 2016/06/28**, **fixed not normalized quaternions 2017/02/24**, **fixed compressedDepth encoding format 2020/05/27**, **fixed odom child_frame_id not set 2021/01/22**).

Launch: [demo_robot_mapping.launch](launch/demo_robot_mapping.launch)
```bash
$ roslaunch rtabmap_demos demo_robot_mapping.launch
$ rosbag play --clock demo_mapping.bag
```

After mapping, you could try the localization mode:
```bash
$ roslaunch rtabmap_demos demo_robot_mapping.launch localization:=true
$ rosbag play --clock demo_mapping.bag
```
* Note that the GUI node doesn't download automatically the map when started. You will need to click "Edit->Download Map" on the GUI to download the map from the core node.

### Robot mapping with RVIZ
[![Robot mapping with RVIZ](https://img.youtube.com/vi/MQoSDpAsqps/0.jpg)](https://www.youtube.com/watch?v=MQoSDpAsqps)

For this demo, you will need the ROS bag [demo_mapping.bag](https://drive.google.com/file/d/1RJxe6nU-of5_AbyoYGobNi6y-Z00Hyh7/view?usp=drive_link) (295 MB, **fixed camera TF 2016/06/28**, **fixed not normalized quaternions 2017/02/24**, **fixed compressedDepth encoding format 2020/05/27**, **fixed odom child_frame_id not set 2021/01/22**).

Launch: [demo_robot_mapping.launch](launch/demo_robot_mapping.launch)
```bash
$ roslaunch rtabmap_demos demo_robot_mapping.launch rviz:=true rtabmap_viz:=false
$ rosbag play --clock demo_mapping.bag
```

### Multi-session mapping
[![Multi-session mapping](https://img.youtube.com/vi/XrnyhaxPCro/0.jpg)](https://www.youtube.com/watch?v=XrnyhaxPCro)

Detailed results are shown on the [Multi-session](https://github.com/introlab/rtabmap/wiki/Multi-session) page on RTAB-Map's wiki.

For this demo, you will need the ROS bags of five mapping sessions:
* **Updated 2015/09/21**: fixed TF rotation of the camera
* **Updated 2017/09/08**: fixed TF quaternion not normalized error
* [map1.bag.zip](https://drive.google.com/file/d/1KgrPTgq4IFIGg6C_gOvPVXGHj0k8m1Pu/view?usp=drive_link) (542 MB)
* [map2.bag.zip](https://drive.google.com/file/d/1uFkSCHIyoN2t6LNW16syLin4pMgp3yOs/view?usp=drive_link) (490 MB)
* [map3.bag.zip](https://drive.google.com/file/d/1jwXkSZFL2aoDLIyqfKZfdHfZDIxrAd69/view?usp=drive_link) (318 MB)
* [map4.bag.zip](https://drive.google.com/file/d/1mFYi82fjeZQEm1aNRwtzNRAt-Y3a3mrU/view?usp=drive_link) (1.1 GB)
* [map5.bag.zip](https://drive.google.com/file/d/1cPjxrinY8cSLHGAvKO8LM0BioEsYP6TE/view?usp=drive_link) (843 MB)

For the first launch, you can do "Edit->Delete memory" to make sure that you start from a clean memory. You may need to do this after starting the first bag with "--pause" so that rtabmap node is initialized to avoid a "service /reset cannot be called" error.

Launch: [demo_multi-session_mapping.launch](launch/demo_multi-session_mapping.launch)
```bash
$ roslaunch rtabmap_demos demo_multi-session_mapping.launch
$ rosbag play --clock --pause map1.bag
$ (...)
$ rosbag play --clock map2.bag
$ (...)
$ rosbag play --clock map3.bag
$ (...)
$ rosbag play --clock map4.bag
$ (...)
$ rosbag play --clock map5.bag
```

### Robot mapping with Find-Object
[![Robot mapping with Find-Object](https://img.youtube.com/vi/o1GSQanY-Do/0.jpg)](https://www.youtube.com/watch?v=o1GSQanY-Do)

Find-Object's ros-pkg [find_object_2d](https://github.com/introlab/find-object/tree/noetic-devel) should be installed.

ROS Bag: [demo_find_object.bag](https://drive.google.com/file/d/1Q2rxVk8tJMCj8WBZfw-R-ZKXCtnOcgXb/view?usp=drive_link) (416 MB, **fixed compressedDepth encoding format 2024/04/27**)

Launch: [demo_find_object.launch](launch/demo_find_object.launch)
```bash
$ roslaunch rtabmap_demos demo_find_object.launch
$ rosbag play --clock demo_find_object.bag
```

### IROS 2014 Kinect Challenge
[![IROS 2014 Kinect Challenge](https://img.youtube.com/vi/_qiLAWp7AqQ/0.jpg)](https://www.youtube.com/watch?v=_qiLAWp7AqQ)

There is no bag recorded for this demo but how to reproduce this setup is described on the page [IROS 2014 Kinect Challenge](https://github.com/introlab/rtabmap/wiki/IROS-2014-Kinect-Challenge) of the RTAB-Map's wiki.

### Stereo mapping
[![Stereo mapping](https://img.youtube.com/vi/qpTS7kg9J3A/0.jpg)](https://www.youtube.com/watch?v=qpTS7kg9J3A)

Visit the tutorial [Stereo Outdoor Mapping](https://github.com/introlab/rtabmap_ros/wiki/Stereo-Outdoor-Mapping-(ROS1)) for detailed information. It is also shown how to create 2D occupancy grid map for navigation.

ROS bags:
* [stereo_outdoorA.bag](https://drive.google.com/file/d/1yxSpRa-W_Ayp1oMLDP66L0QAE-SkW78V/view?usp=drive_link) (668 MB)
* [stereo_outdoorB.bag](https://drive.google.com/file/d/1dSXDbHBLco8dOIeWKMddWvGq4s9vfGq8/view?usp=drive_link) (987 MB)

Launch: [demo_stereo_outdoor.launch](launch/demo_stereo_outdoor.launch)
```bash
$ roslaunch rtabmap_demos demo_stereo_outdoor.launch
$ rosbag play --clock stereo_outdoorA.bag
[...]
$ rosbag play --clock stereo_outdoorB.bag
```

### Stereo navigation
[![Stereo navigation](https://img.youtube.com/vi/X885QsH0szo/0.jpg)](https://www.youtube.com/watch?v=X885QsH0szo)

There is no bag recorded for this demo but how to reproduce this setup is described in the tutorial [Stereo Outdoor Navigation](https://github.com/introlab/rtabmap_ros/wiki/Stereo-Outdoor-Navigation-(ROS1)).

### Appearance-based loop closure detection-only
[![Appearance-based loop closure detection-only](https://img.youtube.com/vi/LlcqK5HJeQU/0.jpg)](https://www.youtube.com/watch?v=LlcqK5HJeQU)

Data:
* [samples.zip](https://github.com/introlab/rtabmap/raw/master/bin/data/samples.zip)
  * Set `video_or_images_path` parameter of [rtabmap_legacy/camera](../rtabmap_legacy) node in the launch file below accordingly.

Launch: [demo_appearance_mapping.launch](launch/demo_appearance_mapping.launch)
```bash
$ roslaunch rtabmap_demos demo_appearance_mapping.launch
```

The GUI shows a plenty of information about the loop closures detected. If you only need the ID of the matched past image of a loop closure, you can do that:
```bash
$ rostopic echo /rtabmap/info/loopClosureId
6
---
0
---
7
---
```

A "0" means no loop closure detected. This can be also used in localization mode:
```bash
$ roslaunch rtabmap_demos demo_appearance_mapping.launch localization:=true
```

For **more videos** and information about the loop closure detection approach used in RTAB-Map, visit [RTAB-Map on IntRoLab](https://introlab.3it.usherbrooke.ca/mediawiki-introlab/index.php?title=RTAB-Map&setlang=en).
