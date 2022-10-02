FROM introlab3it/rtabmap:18.04

RUN source /ros_entrypoint.sh && \
    mkdir -p catkin_ws/src && \
    cd catkin_ws/src && \
    catkin_init_workspace
    
COPY . catkin_ws/src/rtabmap_ros

RUN source /ros_entrypoint.sh && \
    cd catkin_ws && \
    catkin_make -j1 -DRTABMAP_SYNC_MULTI_RGBD=ON -DCMAKE_INSTALL_PREFIX=/opt/ros/melodic install && \
    cd && \
    rm -rf catkin_ws
