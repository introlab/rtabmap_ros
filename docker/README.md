### Docker

* Available images on [introlab3it/rtabmap_ros](https://hub.docker.com/r/introlab3it/rtabmap_ros/):
    ```
    foxy, foxy-latest
    humble, humble-latest
    ```
    * The `-latest` images are automatically built from latest version of `rtabmap` and `rtabmap_ros` from source (including GTSAM and libpointmatcher dependencies that are not available with ROS binaries). The other images have the same version than the binaries released on ROS. 


* Launch `rtabmap` from inside the container (**no gui**), saving the database on host `~/.ros/rtabmap.db`:
    ```bash
    $ docker run -it --rm \
     --network=host \
     -v ~/.ros:/root  \
     introlab3it/rtabmap_ros:humble-latest \
     ros2 launch rtabmap_ros rtabmap.launch.py rtabmapviz:=false database_path:=/root/rtabmap.db rtabmap_args:="--delete_db_on_start"
   ```
   
 * Launch `rtabmap` from inside the container (**with gui**, if you have a nvidia GPU, follow those [instructions](http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration#nvidia-docker2) with nvidia-docker2 using one of the images above instead of the ros image example, or for other GPUs try the intel/AMD [instructions](http://wiki.ros.org/action/fullsearch/docker/Tutorials/Hardware%20Acceleration) to build a `rtabmap_ros3d` image with your GPU driver), saving the database on host `~/.ros/rtabmap.db`:

   * With [nvidia-docker2 approach](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html), we would have this Dockerfile:
       ```docker
       FROM introlab3it/rtabmap_ros:humble-latest

       # nvidia-container-runtime
       ENV NVIDIA_VISIBLE_DEVICES \
           ${NVIDIA_VISIBLE_DEVICES:-all}
       ENV NVIDIA_DRIVER_CAPABILITIES \
           ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
       ```
       Building the image and run it:
       ```bash
       $ docker build -t rtabmap_ros3d .
       
       # those following 3 lines would need to be done only one time
       $ XAUTH=/tmp/.docker.xauth
       $ touch $XAUTH
       $ xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
    
       $ docker run -it --rm \
         --privileged \
         --env="DISPLAY=$DISPLAY" \
         --env="QT_X11_NO_MITSHM=1" \
         --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
         --env="XAUTHORITY=$XAUTH" \
         --volume="$XAUTH:$XAUTH" \
         --runtime=nvidia \
         --network=host \
         -v ~/.ros:/root  \
         rtabmap_ros3d \
         ros2 launch rtabmap_ros rtabmap.launch.py database_path:=/root/rtabmap.db rtabmap_args:="--delete_db_on_start"
       ```
