Docker image example to include pytorch/CUDA support (SuperPoint, SuperGlue, OpenCV+nonfree+xfeatures2d)

# Create image:
```bash
cd rtabmap_ros
docker build -t rtabmap_ros:superpoint -f docker/jazzy/superpoint/Dockerfile .
```
# Example of usage:

We launch the [realsense_d435i_infra.launch.py](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_examples/launch/realsense_d435i_infra.launch.py) example with arguments to use superpoint + superglue for loop closure detection. Note that visual odometry is done with default parameters in this case.

```bash
# X11 Setup for rtabmap_viz, not required if you don't launch any UI
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

# Docker Run Command
docker run -it --rm \
  --user $(id -u) \
  --privileged \
  --gpus all \
  -e LD_PRELOAD="/opt/hpcx/ucc/lib/libucc.so.1" \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY=$XAUTH \
  -v $XAUTH:$XAUTH \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e ROS_HOME=/tmp/.ros \
  --network host \
  -v ~/.ros:/tmp/.ros \
  rtabmap_ros:superpoint \
  ros2 launch rtabmap_examples realsense_d435i_infra.launch.py \
    args:=" \
        --SuperPoint/ModelPath /workspace/superpoint_v1.pt \
        --PyMatcher/Path /workspace/SuperGluePretrainedNetwork/rtabmap_superglue.py \
        --Kp/DetectorStrategy 11 \
        --Kp/NndrRatio 0.6 \
        --Vis/CorNNType 6 \
        --Vis/CorNNDR 0.6 \
        --Reg/RepeatOnce false \
        --Vis/CorGuessWinSize 0" \
     odom_args:=" \
        --Vis/CorNNType 1 \
        --Reg/RepeatOnce true \
        --Vis/CorGuessWinSize 40 \
        --Vis/CorNNDR 0.8"
```

The resulting database will be saved to `~/.ros/rtabmap.db` on the host computer. You can also use the `launch.sh` file in this folder for convenience.

To use superpoint for odometry, remove `odom_args` and add this to `args`:
```bash
--Vis/FeatureType 11 \
```

Performance tip: to avoid extracting again in `rtabmap` superpoint features already extracted in `rgbd_odometry`, we would need to edit `realsense_d435i_infra.launch.py` and add the parameter `subscribe_sensor_data:=true` to `rtabmap` and `rtabmap_viz`, then remap `sensor_data:=odom_sensor_data/raw`.