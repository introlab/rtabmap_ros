FROM introlab3it/rtabmap:jammy-iron

RUN apt-get update && apt-get install -y \
    ros-iron-pcl-ros \
    && rm -rf /var/lib/apt/lists/

RUN source /ros_entrypoint.sh && \
    mkdir -p ros2_ws/src && \
    cd ros2_ws/src

COPY . ros2_ws/src/rtabmap_ros

RUN source /ros_entrypoint.sh && \
    cd ros2_ws && \
    export MAKEFLAGS="-j1" && \
    apt update && apt install -y ros-iron-pcl-ros && \
    colcon build --event-handlers console_direct+ --install-base /opt/ros/iron --merge-install --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON -DCMAKE_BUILD_TYPE=Release && \
    cd && \
    rm -rf ros2_ws
