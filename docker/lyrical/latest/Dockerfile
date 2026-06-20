FROM introlab3it/rtabmap:resolute

RUN source /ros_entrypoint.sh && \
    mkdir -p ros2_ws/src && \
    cd ros2_ws/src

COPY . ros2_ws/src/rtabmap_ros

RUN source /ros_entrypoint.sh && \
    cd ros2_ws && \
    export MAKEFLAGS="-j2" && \
    rosdep init && \
    rosdep update && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y -t build_export -t test -t build -t buildtool_export -t buildtool --skip-keys="rtabmap nav2_bringup realsense2_camera nav2_msgs grid_map_ros nav2_costmap_2d" && \
    apt-get clean && rm -rf /var/lib/apt/lists/ && \
    colcon build --packages-skip rtabmap_costmap_plugins rtabmap_ros --event-handlers console_direct+ --install-base /opt/ros/$ROS_DISTRO --merge-install --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON -DCMAKE_BUILD_TYPE=Release && \
    cd && \
    rm -rf ros2_ws
