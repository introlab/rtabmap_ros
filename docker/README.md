### Docker

* Create rtabmap image (example with melodic) from this Dockerfile:
    ```dockerfile
    FROM ros:melodic-perception
    # install rtabmap packages
    RUN apt-get update && apt-get install -y \
        ros-melodic-rtabmap \
        ros-melodic-rtabmap-ros \
        && rm -rf /var/lib/apt/lists/
    ```
    * To build an image with latest rtabmap version from source, use Dockerfile inside one of the `latest` subdirectories.

* Build it:
    ```bash
    $ docker build --tag ros:rtabmap .
    ```

* The following example show how to launch a camera on host computer and run our pre-built rtabmap container. All examples from [RGB-D tutorial](http://wiki.ros.org/rtabmap_ros/Tutorials/HandHeldMapping) and [stereo tutorial](http://wiki.ros.org/rtabmap_ros/Tutorials/StereoHandHeldMapping) should work using rtabmap from the container instead. Launch camera on host computer (set ROS_IP as the IP used for docker):
    ```bash
    $ export ROS_IP=172.17.0.1 && roslaunch openni2_launch openni2.launch depth_registration:=true
    # In another terminal, launch rtabmapviz or RVIZ. We do visualization 
    # on host computer to avoid GPU problems with the container:
    $ export ROS_NAMESPACE=rtabmap && rosrun rtabmap_ros rtabmapviz _frame_id:=camera_link
    ```

* Launch rtabmap from inside the container, saving the database on host `~/.ros/rtabmap.db`:
    ```bash
    $ docker run -it --rm \
     --env ROS_MASTER_URI=http://172.17.0.1:11311 --env ROS_IP=172.17.0.2 \
     -v ~/.ros:/root  \
     ros:rtabmap \
     roslaunch rtabmap_ros rtabmap.launch rtabmapviz:=false database_path:=/root/rtabmap.db rtabmap_args:="--delete_db_on_start"
   ```
