FROM osrf/ros:kilted-desktop
# install rtabmap packages
RUN apt-get update && apt-get install -y \
    ros-kilted-rtabmap \
    ros-kilted-rtabmap-ros \
    && rm -rf /var/lib/apt/lists/
