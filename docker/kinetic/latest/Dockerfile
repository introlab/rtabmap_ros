FROM introlab3it/rtabmap:16.04

ARG CACHE_DATE=2016-01-01

RUN source /ros_entrypoint.sh && \
    mkdir -p catkin_ws/src && \
    cd catkin_ws/src && \
    catkin_init_workspace && \
    git clone https://github.com/introlab/rtabmap_ros.git && \
    cd .. && \
    catkin_make -j1 -DCMAKE_INSTALL_PREFIX=/opt/ros/kinetic install && \
    cd && \
    rm -rf catkin_ws
