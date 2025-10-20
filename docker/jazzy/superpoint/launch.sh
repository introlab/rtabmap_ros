#!/bin/bash

# X11 Setup
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
