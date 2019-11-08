### Docker

* Available images (https://hub.docker.com/repository/docker/introlab3it/rtabmap_ros):
    ```
    introlab3it/rtabmap_ros:indigo-latest
    introlab3it/rtabmap_ros:kinetic-latest
    introlab3it/rtabmap_ros:melodic-latest
    ```
    * Those images are automatically built from latest version of `rtabmap` and `rtabmap_ros` from source.


* The following example show how to launch a camera on host computer and run our pre-built rtabmap container. All examples from [RGB-D tutorial](http://wiki.ros.org/rtabmap_ros/Tutorials/HandHeldMapping) and [stereo tutorial](http://wiki.ros.org/rtabmap_ros/Tutorials/StereoHandHeldMapping) should work using rtabmap from the container instead. Launch camera on host computer (set ROS_IP as the IP used for docker):
    ```bash
    $ export ROS_IP=172.17.0.1 && roslaunch openni2_launch openni2.launch depth_registration:=true
    # In another terminal, launch rtabmapviz or RVIZ. We do visualization 
    # on host computer to avoid GPU problems with the container:
    $ export ROS_NAMESPACE=rtabmap && rosrun rtabmap_ros rtabmapviz _frame_id:=camera_link
    ```

* Launch `rtabmap` from inside the container (no gui), saving the database on host `~/.ros/rtabmap.db`:
    ```bash
    $ docker run -it --rm \
     --env ROS_MASTER_URI=http://172.17.0.1:11311 --env ROS_IP=172.17.0.2 \
     -v ~/.ros:/root  \
     introlab3it/rtabmap_ros:kinetic-latest \
     roslaunch rtabmap_ros rtabmap.launch rtabmapviz:=false database_path:=/root/rtabmap.db rtabmap_args:="--delete_db_on_start"
   ```
