FROM ros:kinetic-perception
# install rtabmap packages
ARG CACHE_DATE=2016-01-01
RUN apt-get update && apt-get install -y \
    ros-kinetic-rtabmap \
    ros-kinetic-rtabmap-ros \
    && rm -rf /var/lib/apt/lists/
