# rtabmap_examples

Starting points for running RTAB-Map with your own sensor. Each launch file connects one camera or lidar driver (or a public dataset) to RTAB-Map's odometry and SLAM nodes with working parameters. Copy the one closest to your setup and adapt it.

## Large images between processes

Most examples run the camera driver, odometry, `rtabmap` and `rtabmap_viz` as separate processes, so every image is sent through DDS to each of them. A 640x480 depth image is already 600 kB, which DDS splits into many fragments, and two settings decide how many frames make it:

* **Reliable subscriptions** (`qos: 1`, set in the RealSense examples). With best effort (the default `qos: 0` is best effort for images), a single lost fragment drops the whole image, and odometry, which needs the image and the depth of a frame, loses the frame.
* **Fast DDS's `LARGE_DATA` transport** (Fast DDS is ROS 2's default middleware). Its default shared memory segments are smaller than a depth image, which makes the large images arrive in bursts or late. Set it in the environment of every process, e.g. in your `~/.bashrc`:
  ```bash
  export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
  ```
  then restart the ROS daemon (`ros2 daemon stop`). Set it on all the computers of the system or none: `LARGE_DATA` sends the data over TCP between computers where the default uses UDP.

For example, with a D435i at 30 Hz (`realsense_d435i_infra.launch.py`, 848x480 infrared and depth), odometry received 17.5 frames per second with best effort, 26 with reliable subscriptions, and every frame the camera sent (27) with reliable subscriptions and `LARGE_DATA`.

The `*_composition.launch.py` examples avoid DDS altogether: the nodes run in one process and pass the images by pointer.
