### Docker

* Available images on [introlab3it/rtabmap_ros](https://hub.docker.com/r/introlab3it/rtabmap_ros/):
    ```
    humble, humble-latest
    jazzy, jazzy-latest
    ```
    * The `-latest` images are automatically built from latest version of `rtabmap` and `rtabmap_ros` from source (including dependencies that are not available with ROS binaries). The other images have the same version than the binaries released on ROS. 


* Launch `rtabmap` from inside the container (**no gui**), saving the database on host `~/.ros/rtabmap.db`:
    ```bash
    docker run -it --rm \
     --user $UID \
     -e ROS_HOME=/tmp/.ros \
     -e OMP_WAIT_POLICY=passive \
     --network=host \
     --ipc=host \
     -v ~/.ros:/tmp/.ros \
     introlab3it/rtabmap_ros:humble-latest \
     ros2 launch rtabmap_launch rtabmap.launch.py rtabmap_viz:=false database_path:=/tmp/.ros/rtabmap.db rtabmap_args:="--delete_db_on_start"
   ```
   
* Launch `rtabmap_viz` from inside the container, using [nvidia-docker2 approach](http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration#nvidia-docker2):

   ```bash
    # those following 3 lines would need to be done only one time
    XAUTH=/tmp/.docker.xauth
    touch $XAUTH
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

    docker run -it --rm \
      --privileged \
      -e DISPLAY=$DISPLAY \
      -e QT_X11_NO_MITSHM=1 \
      -e NVIDIA_VISIBLE_DEVICES=all \
      -e NVIDIA_DRIVER_CAPABILITIES=all \
      -e XAUTHORITY=$XAUTH \
      -e OMP_WAIT_POLICY=passive \
      --user $UID \
      -e ROS_HOME=/tmp/.ros \
      -v ~/.ros:/tmp/.ros \
      --runtime=nvidia \
      --network=host \
      --ipc=host \
      -v $XAUTH:$XAUTH \
      -v /tmp/.X11-unix:/tmp/.X11-unix \
      introlab3it/rtabmap_ros:humble-latest \
      /bin/bash -c "ros2 run rtabmap_viz rtabmap_viz --ros-args -r __ns:=/rtabmap"
    ```

