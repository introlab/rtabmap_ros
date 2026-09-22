FROM introlab3it/rtabmap:resolute

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
    rosdep install --from-paths src --ignore-src -y -t build_export -t test -t build -t buildtool_export -t buildtool --skip-keys="rtabmap nav2_bringup realsense2_camera nav2_msgs grid_map_ros" && \
    bash src/rtabmap_ros/docker/verify_deps.sh && \
    apt-get clean && rm -rf /var/lib/apt/lists/

ARG RUN_TESTS=0

RUN source /ros_entrypoint.sh && \
    cd ros2_ws && \
    export MAKEFLAGS="-j2" && \
    colcon build --event-handlers console_direct+ --install-base /opt/ros/$ROS_DISTRO --merge-install --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON -DCMAKE_BUILD_TYPE=Release && \
    if [ "$RUN_TESTS" = "1" ]; then \
        colcon test --executor sequential --event-handlers console_direct+ --install-base /opt/ros/$ROS_DISTRO --merge-install && \
        colcon test-result --verbose; \
    fi && \
    cd && \
    rm -rf ros2_ws
