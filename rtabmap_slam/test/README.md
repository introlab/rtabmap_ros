# Scan transform recovery regression

`scan_tf_recovery.py` is a standalone ROS 2 integration reproducer for the
scan-conversion error paths in `CoreWrapper::commonLaserScanCallback`.
It is not registered with CTest yet.

Source your ROS installation and the workspace containing the RTAB-Map build
under test. Use an unused ROS domain and run the two cases sequentially:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=171
python3 src/rtabmap_ros/rtabmap_slam/test/scan_tf_recovery.py
python3 src/rtabmap_ros/rtabmap_slam/test/scan_tf_recovery.py --late-tf
```

Adjust the script path for your checkout layout. Python dependencies are
`rclpy`, `ament_index_python`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`,
`sensor_msgs_py`, `std_msgs`, `tf2_ros` and PyYAML.

The script starts the installed `rtabmap_slam/rtabmap` executable, publishes
odometry and `odom -> base_link`, and begins publishing PointCloud2 data after
four seconds. The control provides `base_link -> lidar` immediately; `--late-tf`
withholds it until eight seconds. No simulator, hardware or recorded data is
required. Parameters and captured logs are written under `/tmp`.

Both cases require a nonempty occupancy grid within 18 seconds and a clean
process exit after SIGINT. The late-TF case also requires a scan-conversion error
in the log, ensuring the failure path was exercised. A JSON summary reports the
map dimensions, conversion failure and process exit code.

On stock Jazzy 0.23.7, the control passes but the late-TF case produces no map and
requires forced shutdown. With the two missing unlocks added, both pass. The
patched late-TF case passed five runs, including two using default FastDDS and
two using UDP-only transport. This reproducer exercises the 3D scan path; the
analogous 2D error path is fixed by inspection but is not independently tested.
