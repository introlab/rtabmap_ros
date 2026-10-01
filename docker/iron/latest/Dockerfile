FROM introlab3it/rtabmap:jammy-iron

RUN source /ros_entrypoint.sh && \
    mkdir -p ros2_ws/src && \
    cd ros2_ws/src

COPY . ros2_ws/src/rtabmap_ros

# No rosdep "-r": a bad mirror must fail here.
RUN source /ros_entrypoint.sh && \
    cd ros2_ws && \
    echo 'Acquire::Retries "5";' > /etc/apt/apt.conf.d/80-retries && \
    rosdep init && \
    rosdep update && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -y -t build_export -t test -t build -t buildtool_export -t buildtool && \
    apt remove ros-$ROS_DISTRO-rtabmap* -y && \
    bash src/rtabmap_ros/docker/verify_deps.sh && \
    apt-get clean && rm -rf /var/lib/apt/lists/

RUN source /ros_entrypoint.sh && \
    cd ros2_ws && \
    export MAKEFLAGS="-j1" && \
    colcon build --event-handlers console_direct+ --install-base /opt/ros/iron --merge-install --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON -DCMAKE_BUILD_TYPE=Release && \
    cd && \
    rm -rf ros2_ws
