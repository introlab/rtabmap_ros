FROM introlab3it/rtabmap:20.04

RUN source /ros_entrypoint.sh && \
    mkdir -p catkin_ws/src && \
    cd catkin_ws/src && \
    catkin_init_workspace

COPY . catkin_ws/src/rtabmap_ros

RUN if [ "$TARGETPLATFORM" = "linux/arm/v7" ]; then \
    source /ros_entrypoint.sh && \
    cd catkin_ws && \
    catkin_make -j1 -DRTABMAP_SYNC_MULTI_RGBD=ON -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic install && \
    cd && \
    rm -rf catkin_ws ;fi

RUN if [ "$TARGETPLATFORM" != "linux/arm/v7" ]; then \
    source /ros_entrypoint.sh && \
    cd catkin_ws && \
    apt update && \
    rosdep install --from-paths src --ignore-src -y && \
    apt remove ros-$ROS_DISTRO-rtabmap* -y && \
    apt-get clean && rm -rf /var/lib/apt/lists/ && \
    catkin_make -j1 -DRTABMAP_SYNC_MULTI_RGBD=ON -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic install && \
    cd && \
    rm -rf catkin_ws ;fi
