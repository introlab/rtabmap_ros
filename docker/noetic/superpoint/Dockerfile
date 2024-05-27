# Image built with pytorch/CUDA support (SuperPoint, SuperGlue, OpenCV+nonfree+xfeatures2d)
#
#
# Create image:
#
# cd rtabmap_ros
# docker build -t rtabmap_ros:superpoint -f docker/noetic/superpoint/Dockerfile .
#
#
#
# Example of usage (using superpoint + superglue for loop closure detection) with D435i:
#
# [Camera]
# docker run -it --rm --network host --privileged rtabmap_ros:superpoint roslaunch realsense2_camera rs_camera.launch enable_infra1:=true enable_infra2:=true unite_imu_method:=linear_interpolation enable_gyro:=true enable_accel:=true enable_sync:=true
#
# [IMU filter]
# docker run -it --rm --network host rtabmap_ros:superpoint rosrun imu_filter_madgwick imu_filter_node _use_mag:=false _publish_tf:=false _world_frame:="enu" /imu/data_raw:=/camera/imu /imu/data:=/rtabmap/imu
#
# [VSLAM]
# docker run -it --rm --user $UID -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all --runtime=nvidia -e ROS_HOME=/tmp/.ros --network host -v ~/.ros:/tmp/.ros rtabmap_ros:superpoint roslaunch rtabmap_launch rtabmap.launch rtabmap_viz:=false database_path:=/tmp/.ros/rtabmap.db rtabmap_args:="--delete_db_on_start --SuperPoint/ModelPath /workspace/superpoint_v1.pt --PyMatcher/Path /workspace/SuperGluePretrainedNetwork/rtabmap_superglue.py --Kp/DetectorStrategy 11 --Vis/CorNNType 6 --Reg/RepeatOnce false --Vis/CorGuessWinSize 0" odom_args:="--Vis/CorNNType 1 --Reg/RepeatOnce true --Vis/CorGuessWinSize 40" left_image_topic:=/camera/infra1/image_rect_raw right_image_topic:=/camera/infra2/image_rect_raw left_camera_info_topic:=/camera/infra1/camera_info right_camera_info_topic:=/camera/infra2/camera_info stereo:=true wait_imu_to_init:=true imu_topic:=/rtabmap/imu
#
# [Visualization - optional]
# XAUTH=/tmp/.docker.xauth
# touch $XAUTH
# xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
# docker run -it --rm --privileged -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -e XAUTHORITY=$XAUTH --runtime=nvidia --network host -v $XAUTH:$XAUTH -v /tmp/.X11-unix:/tmp/.X11-unix rtabmap_ros:superpoint /bin/bash -c "export ROS_NAMESPACE=rtabmap && rosrun rtabmap_viz rtabmap_viz _subscribe_odom_info:=true left/image_rect:=/camera/infra1/image_rect_raw right/image_rect:=/camera/infra2/image_rect_raw left/camera_info:=/camera/infra1/camera_info right/camera_info:=/camera/infra2/camera_info _subscribe_stereo:=true"
#
FROM nvcr.io/nvidia/pytorch:22.08-py3

ENV DEBIAN_FRONTEND=noninteractive

# Install build dependencies
RUN apt-get update && apt-get install -y \
      libsqlite3-dev \
      git \
      cmake \
      libyaml-cpp-dev \
      software-properties-common \
      pkg-config \ 
      wget \
      curl \
      libpdal-dev && \
      apt-get clean && rm -rf /var/lib/apt/lists/

# Install ros keys
RUN apt update && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Install ros dependencies
RUN apt-get update && \
    apt upgrade -y && \
    apt-get install -y git ros-noetic-ros-base python3-catkin-tools python3-rosdep build-essential ros-noetic-rtabmap-ros ros-noetic-pybind11-catkin ros-noetic-image-transport-plugins ros-noetic-stereo-image-proc && \
    apt-get remove -y ros-noetic-rtabmap && \
    rosdep init && \
    apt-get clean && rm -rf /var/lib/apt/lists/

# GTSAM
RUN add-apt-repository ppa:borglab/gtsam-release-4.0 && \
    apt install -y libgtsam-dev libgtsam-unstable-dev

# MRPT
RUN add-apt-repository ppa:joseluisblancoc/mrpt-stable -y && \
    apt-get update && apt install libmrpt-poses-dev -y && \
    apt-get clean && rm -rf /var/lib/apt/lists/

# OpenCV with xfeatures2d, cuda and nonfree modules (use same version used by noetic to avoid cv_bridge conflicts)
RUN git clone https://github.com/opencv/opencv_contrib.git && \
    git clone https://github.com/opencv/opencv.git && \
    cd opencv_contrib && \
    git checkout tags/4.2.0 && \
    cd /workspace && \
    cd opencv && \
    git checkout tags/4.2.0 && \
    mkdir build && \
    cd build && \
    cmake -DOPENCV_EXTRA_MODULES_PATH=/workspace/opencv_contrib/modules -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DOPENCV_ENABLE_NONFREE=ON .. && \
    make -j$(nproc) && \
    make install && \
    cd /workspace && \
    rm -rf opencv opencv_contrib

# OpenGV
RUN git clone https://github.com/laurentkneip/opengv.git && \
    cd opengv && \
    git checkout 91f4b19c73450833a40e463ad3648aae80b3a7f3 && \
    wget https://gist.githubusercontent.com/matlabbe/a412cf7c4627253874f81a00745a7fbb/raw/accc3acf465d1ffd0304a46b17741f62d4d354ef/opengv_disable_march_native.patch && \
    git apply opengv_disable_march_native.patch && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    cd /workspace && \
    rm -r opengv

# Setup ROS entrypoint
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

COPY ./docker/noetic/superpoint/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT [ "/ros_entrypoint.sh" ]

# Get rtabmap library
# Create Superpoint model with current pytorch version
# Setup Superglue
#Build/install rtabmap library
RUN git clone https://github.com/introlab/rtabmap rtabmap && \
    cd rtabmap/archive/2022-IlluminationInvariant/scripts && \
    wget https://github.com/magicleap/SuperPointPretrainedNetwork/raw/master/superpoint_v1.pth && \
    wget https://raw.githubusercontent.com/magicleap/SuperPointPretrainedNetwork/master/demo_superpoint.py && \
    python3 trace.py && \
    mv superpoint_v1.pt /workspace/. && \
    cd /workspace && \
    git clone https://github.com/magicleap/SuperGluePretrainedNetwork && \
    cp rtabmap/corelib/src/python/rtabmap_superglue.py SuperGluePretrainedNetwork/. && \
    source /ros_entrypoint.sh && \
    cd rtabmap/build && \
    cmake -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic -DTorch_DIR=/opt/conda/lib/python3.8/site-packages/torch/share/cmake/Torch -DWITH_TORCH=ON -DWITH_PYTHON=ON .. && \
    make -j$(nproc) && \
    make install && \
    cd ../.. && \
    rm -rf rtabmap && \
    ldconfig

# Setup catkin workspace
RUN source /ros_entrypoint.sh && \
    mkdir -p catkin_ws/src && \
    cd catkin_ws && \
    catkin init && \
    catkin config --install --install-space /opt/ros/noetic

COPY . catkin_ws/src/rtabmap_ros

# build packages
RUN source /ros_entrypoint.sh && \
    cd catkin_ws && \
    apt update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    apt remove ros-$ROS_DISTRO-rtabmap* -y && \
    apt-get clean && rm -rf /var/lib/apt/lists/ && \
    catkin build --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DRTABMAP_SYNC_MULTI_RGBD=ON && \
    cd /workspace && \
    rm -rf catkin_ws

