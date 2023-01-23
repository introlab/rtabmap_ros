FROM osrf/ros:foxy-desktop
# install rtabmap packages
ARG CACHE_DATE=2016-01-01
RUN apt-get update && apt-get install -y \
    ros-foxy-rtabmap \
    ros-foxy-rtabmap-ros \
    && rm -rf /var/lib/apt/lists/
