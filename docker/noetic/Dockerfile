FROM ros:noetic-perception
# install rtabmap packages
RUN apt-get update && apt-get install -y \
    ros-noetic-rtabmap \
    ros-noetic-rtabmap-ros \
    && rm -rf /var/lib/apt/lists/
