### Docker

* Available images on [introlab3it/rtabmap_ros](https://hub.docker.com/r/introlab3it/rtabmap_ros/):
    ```
    indigo, indigo-latest
    kinetic, kinetic-latest
    melodic, melodic-latest
    noetic, noetic-latest
    ```
    * The `-latest` images are automatically built from latest version of `rtabmap` and `rtabmap_ros` from source (including GTSAM and libpointmatcher dependencies that are not available with ROS binaries). The other images have the same version than the binaries released on ROS. 


* The following example show how to launch a camera on host computer and run our pre-built rtabmap container. All examples from [RGB-D tutorial](http://wiki.ros.org/rtabmap_ros/Tutorials/HandHeldMapping) and [stereo tutorial](http://wiki.ros.org/rtabmap_ros/Tutorials/StereoHandHeldMapping) should work using rtabmap from the container instead. Launch camera on host computer (set ROS_IP as the IP used for docker):
    ```bash
    $ export ROS_IP=172.17.0.1 && roslaunch openni2_launch openni2.launch depth_registration:=true
    # In another terminal, launch rtabmapviz or RVIZ. We do visualization 
    # on host computer to avoid GPU problems with the container (see "with gui" below to launch rtabmapviz from the container):
    $ export ROS_NAMESPACE=rtabmap && rosrun rtabmap_ros rtabmapviz _frame_id:=camera_link
    ```

* Launch `rtabmap` from inside the container (**no gui**), saving the database on host `~/.ros/rtabmap.db`:
    ```bash
    $ docker run -it --rm \
     --env ROS_MASTER_URI=http://172.17.0.1:11311 --env ROS_IP=172.17.0.2 \
     -v ~/.ros:/root  \
     introlab3it/rtabmap_ros:kinetic-latest \
     roslaunch rtabmap_ros rtabmap.launch rtabmapviz:=false database_path:=/root/rtabmap.db rtabmap_args:="--delete_db_on_start"
   ```
   
 * Launch `rtabmap` from inside the container (**with gui**, if you have a nvidia GPU, follow those [instructions](http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration#nvidia-docker2) with nvidia-docker2 using one of the images above instead of the ros image example, or for other GPUs try the intel/AMD [instructions](http://wiki.ros.org/action/fullsearch/docker/Tutorials/Hardware%20Acceleration) to build a `rtabmap_ros3d` image with your GPU driver), saving the database on host `~/.ros/rtabmap.db`:

   * With [nvidia-docker2 approach](http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration#nvidia-docker2), we would have this Dockerfile:
       ```docker
       FROM introlab3it/rtabmap_ros:noetic

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
         --env ROS_MASTER_URI=http://172.17.0.1:11311 --env ROS_IP=172.17.0.2 \
         -v ~/.ros:/root  \
         rtabmap_ros3d \
         roslaunch rtabmap_ros rtabmap.launch database_path:=/root/rtabmap.db rtabmap_args:="--delete_db_on_start"
       ```
