

# Usage

`rtabmap.launch` from ros1 has been ported to ROS2 as `rtabmap.launch.py` with same arguments. If you see [ROS1 examples](http://wiki.ros.org/rtabmap_ros/Tutorials/HandHeldMapping) like this:

```bash
roslaunch zed_wrapper zed_no_tf.launch

roslaunch rtabmap_ros rtabmap.launch \
    rtabmap_args:="--delete_db_on_start" \
    rgb_topic:=/zed/zed_node/rgb/image_rect_color \
    depth_topic:=/zed/zed_node/depth/depth_registered \
    camera_info_topic:=/zed/zed_node/rgb/camera_info \
    frame_id:=base_link \
    approx_sync:=false \
    wait_imu_to_init:=true \
    imu_topic:=/zed_node/imu/data

```

The ROS2 equivalent is (using latest zed_wrapper launch file):

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i \
    publish_tf:=false \
    publish_map_tf:=false

ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start" \
    rgb_topic:=/zed/zed_node/rgb/image_rect_color \
    depth_topic:=/zed/zed_node/depth/depth_registered \
    camera_info_topic:=/zed/zed_node/rgb/camera_info \
    frame_id:=zed_camera_link \
    approx_sync:=false \
    wait_imu_to_init:=true \
    imu_topic:=/zed/zed_node/imu/data \
    rviz:=true
```

