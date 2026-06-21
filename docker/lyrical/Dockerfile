FROM osrf/ros:lyrical-desktop
# install rtabmap packages
RUN apt-get update && apt-get install -y \
    ros-lyrical-rtabmap \
    ros-lyrical-rtabmap-ros \
    && rm -rf /var/lib/apt/lists/
