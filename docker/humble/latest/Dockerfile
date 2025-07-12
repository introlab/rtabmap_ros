FROM introlab3it/rtabmap:jammy

RUN mkdir -p ros2_ws/src

COPY . ros2_ws/src/rtabmap_ros

RUN source /ros_entrypoint.sh && \
    cd ros2_ws && \
    export MAKEFLAGS="-j2" && \
    rosdep init && \
    rosdep update && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y -t build_export -t test -t build -t buildtool_export -t buildtool --skip-keys="rtabmap" && \
    apt-get clean && rm -rf /var/lib/apt/lists/ && \
    colcon build --executor sequential --event-handlers console_direct+ --install-base /opt/ros/humble --merge-install --cmake-args --debug-find -DRTABMAP_SYNC_MULTI_RGBD=ON -DCMAKE_BUILD_TYPE=Release && \
    cd && \
    rm -rf ros2_ws
