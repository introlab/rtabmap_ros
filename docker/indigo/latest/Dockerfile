FROM ros:indigo-perception
# install rtabmap packages
RUN apt-get update && apt-get install -y \
    ros-indigo-rtabmap-ros \
    && apt-get remove -y \
    ros-indigo-rtabmap \
    && rm -rf /var/lib/apt/lists/

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

WORKDIR /root/

ARG CACHE_DATE=2016-01-01

RUN git clone https://github.com/introlab/rtabmap.git

# Build RTAB-Map project
RUN source /ros_entrypoint.sh && \
    cd rtabmap/build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    cd ../.. && \
    rm -rf rtabmap && \
    ldconfig

RUN source /ros_entrypoint.sh && \
    mkdir -p catkin_ws/src && \
    cd catkin_ws/src && \
    catkin_init_workspace && \
    git clone https://github.com/introlab/rtabmap_ros.git && \
    cd .. && \
    catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/indigo install && \
    cd && \
    rm -rf catkin_ws
