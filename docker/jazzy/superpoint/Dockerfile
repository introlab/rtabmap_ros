# Latest version with CUDA 12, to be compatible with Opencv 4.12.0
FROM nvcr.io/nvidia/pytorch:25.06-py3

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
      build-essential && \
      apt-get clean && rm -rf /var/lib/apt/lists/

# Install ros keys
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb

# Install ros dependencies
RUN apt-get update && \
    apt upgrade -y && \
    apt-get install -y \
        ros-jazzy-ros-base \
        ros-jazzy-rtabmap-ros \
        ros-jazzy-ros-environment \
        ros-jazzy-ament-cmake-auto \
        ros-jazzy-camera-info-manager \
        ros-jazzy-librealsense2 \
        python3-rosdep \
        python3-flake8-docstrings \
        python3-pip \
        python3-pytest-cov \
        ros-dev-tools && \
    apt-get remove -y ros-jazzy-rtabmap* libopencv* && \
    rosdep init && \
    rosdep update && \
    apt-get clean && rm -rf /var/lib/apt/lists/

# Optional: MRPT
RUN add-apt-repository ppa:joseluisblancoc/mrpt-stable -y && \
    apt-get update && apt install libmrpt-poses-dev -y && \
    apt-get clean && rm -rf /var/lib/apt/lists/

# Optional: OpenCV with xfeatures2d, cuda and nonfree modules (use same version used by jazzy to avoid cv_bridge conflicts)
RUN git clone -b 4.12.0 https://github.com/opencv/opencv_contrib.git && \
    git clone -b 4.12.0 https://github.com/opencv/opencv.git && \
    cd opencv && \
    mkdir build && \
    cd build && \
    cmake -DOPENCV_EXTRA_MODULES_PATH=/workspace/opencv_contrib/modules \
          -DCMAKE_CXX_STANDARD=17 \
          -DCMAKE_CUDA_STANDARD=17 \
          -DCMAKE_BUILD_TYPE=Release \
          -DBUILD_SHARED_LIBS=ON \
          -DBUILD_TESTS=OFF \
          -DBUILD_PERF_TESTS=OFF \
          -DOPENCV_ENABLE_NONFREE=ON \
          -DWITH_VTK=OFF \
          -DWITH_TBB=ON \
          -DWITH_CUDA=ON .. && \
    make -j6 && \
    make install && \
    cd /workspace && \
    rm -rf opencv opencv_contrib

# Optional: OpenGV (multi-camera support)
RUN git clone https://github.com/laurentkneip/opengv.git && \
    cd opengv && \
    git checkout 91f4b19c73450833a40e463ad3648aae80b3a7f3 && \
    wget https://gist.githubusercontent.com/matlabbe/a412cf7c4627253874f81a00745a7fbb/raw/accc3acf465d1ffd0304a46b17741f62d4d354ef/opengv_disable_march_native.patch && \
    git apply opengv_disable_march_native.patch && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j6 && \
    make install && \
    cd /workspace && \
    rm -r opengv

# Setup catkin workspace
RUN mkdir -p ros2_ws/src
COPY . ros2_ws/src/rtabmap_ros

# Get rtabmap library
# Create Superpoint model with current pytorch version
# Setup Superglue
# build ros packages (rebuild all packages depending on opencv)
RUN source /opt/ros/jazzy/setup.bash && \
    git clone -b jazzy https://github.com/ros-perception/image_pipeline.git ros2_ws/src/image_pipeline && \
    git clone -b 4.1.0 https://github.com/ros-perception/vision_opencv.git ros2_ws/src/vision_opencv && \
    git clone -b jazzy https://github.com/ros-perception/image_transport_plugins.git ros2_ws/src/image_transport_plugins && \
    git clone -b r/4.56.4 https://github.com/IntelRealSense/realsense-ros.git ros2_ws/src/realsense-ros && \
    git clone https://github.com/introlab/rtabmap ros2_ws/src/rtabmap && \
    cd ros2_ws/src/rtabmap/archive/2022-IlluminationInvariant/scripts && \
    wget https://github.com/magicleap/SuperPointPretrainedNetwork/raw/master/superpoint_v1.pth && \
    wget https://raw.githubusercontent.com/magicleap/SuperPointPretrainedNetwork/master/demo_superpoint.py && \
    python3 trace.py && \
    mv superpoint_v1.pt /workspace/. && \
    cd /workspace && \
    git clone https://github.com/magicleap/SuperGluePretrainedNetwork && \
    cp ros2_ws/src/rtabmap/corelib/src/python/rtabmap_superglue.py SuperGluePretrainedNetwork/. && \
    cd ros2_ws && \
    export MAKEFLAGS="-j6" && \
    colcon build --install-base /usr/local/ros --event-handlers console_direct+ --cmake-args \
        --no-warn-unused-cli \
        -DTorch_DIR=/usr/local/lib/python3.12/dist-packages/torch/share/cmake/Torch \
        -DWITH_TORCH=ON \
        -DWITH_PYTHON=ON \
        -DRTABMAP_SYNC_MULTI_RGBD=ON \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF && \
    cd /workspace && \
    rm -rf ros2_ws

# Setup ROS entrypoint
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

RUN echo -e '#!/bin/bash\nset -e\n\n# setup ros2 environment\nsource "/opt/ros/jazzy/setup.bash"\nsource "/usr/local/ros/setup.bash"\nexec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh
ENTRYPOINT [ "/ros_entrypoint.sh" ]

RUN source /ros_entrypoint.sh && ldconfig