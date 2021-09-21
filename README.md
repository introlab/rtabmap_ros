rtabmap_ros [![Build Status](https://github.com/introlab/rtabmap_ros/actions/workflows/ros1.yml/badge.svg)](https://github.com/introlab/rtabmap_ros/actions/workflows/ros1.yml) [![docker](https://github.com/introlab/rtabmap_ros/actions/workflows/docker.yml/badge.svg)](https://github.com/introlab/rtabmap_ros/actions/workflows/docker.yml)
=======

RTAB-Map's ROS package.

For more information, demos and tutorials about this package, visit [rtabmap_ros](http://wiki.ros.org/rtabmap_ros) page on ROS wiki.

For the RTAB-Map libraries and standalone application, visit [RTAB-Map's home page](http://introlab.github.io/rtabmap) or [RTAB-Map's wiki](https://github.com/introlab/rtabmap/wiki).

# Installation 

## ROS2 distribution
**Under construction**: see [ros2 branch](https://github.com/introlab/rtabmap_ros/tree/ros2#rtabmap_ros).

## ROS distribution 
RTAB-Map is released as binaries in the ROS distribution.
* Noetic
    ```
    $ sudo apt install ros-noetic-rtabmap-ros
    ```
* Melodic
    ```
    $ sudo apt install ros-melodic-rtabmap-ros
    ```
* Kinetic
    ```
    $ sudo apt-get install ros-kinetic-rtabmap-ros
    ```
* Indigo
    ```
    $ sudo apt-get install ros-indigo-rtabmap-ros
    ```
    * For armhf architecture, `ros-indigo-rtabmap-ros` is not available. Install `ros-indigo-rtabmap` and build from source `rtabmap_ros` using the `indigo-devel` branch.
        ```
        $ cd catkin_ws
        $ git clone -b indigo-devel https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros
        $ catkin_make -j1
        ```

When launching `rtabmap_ros`'s nodes, if you have the error `error while loading shared libraries...`, try `ldconfig` or add the next line at the end of your `~/.bashrc` to fix it:
    
```bash
$ export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/noetic/lib/x86_64-linux-gnu
```

### Docker

* Go to [docker](https://github.com/introlab/rtabmap_ros/tree/master/docker) directory for an example.


## Build from source
This section shows how to install RTAB-Map ros-pkg on **ROS Hydro/Indigo/Jade/Kinetic/Lunar/Melodic/Noetic** (Catkin build). RTAB-Map works only with the PCL >=1.7, which is the default version installed with ROS Hydro/Indigo/Jade/Kinetic/Lunar/Melodic/Noetic (**Fuerte and Groovy are not supported**).

* The next instructions assume that you have set up your ROS workspace using this [tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace). I will use `noetic` prefix for convenience, but it should work with Hydro, Indigo, Jade, Kinetic, Lunar and Melodic. The workspace path is `~/catkin_ws` and your `~/.bashrc` contains:
 
    ```bash
    $ source /opt/ros/noetic/setup.bash
    $ source ~/catkin_ws/devel/setup.bash
    ```

 0. Required dependencies
     * The easiest way to get all them (Qt, PCL, VTK, OpenCV, ...) is to install/uninstall rtabmap binaries:
          ```bash
          $ sudo apt install ros-noetic-rtabmap ros-noetic-rtabmap-ros
          $ sudo apt remove ros-noetic-rtabmap ros-noetic-rtabmap-ros
          ```
 
 1. Optional dependencies
     * If you want SURF/SIFT on Indigo/Jade/Melodic/Noetic (Hydro/Kinetic has already SIFT/SURF), you have to build [OpenCV]([OpenCV](http://opencv.org/)) from source to have access to *xfeatures2d* and *nonfree* modules (note that SIFT is not in *nonfree* anymore since OpenCV 4.4.0). Install it in `/usr/local` (default) and rtabmap library should link with it instead of the one installed in ROS. 
         * On Indigo, I recommend to use latest 2.4 version ([2.4.11](https://github.com/Itseez/opencv/archive/2.4.11.zip)) and build it from source following these [instructions](http://docs.opencv.org/doc/tutorials/introduction/linux_install/linux_install.html#building-opencv-from-source-using-cmake-using-the-command-line). RTAB-Map can build with OpenCV3+[xfeatures2d](https://github.com/Itseez/opencv_contrib/tree/master/modules/xfeatures2d) module, but `rtabmap_ros` package will have libraries conflict as `cv_bridge` is depending on OpenCV2. If you want OpenCV3+, you should build [vision-opencv](https://github.com/ros-perception/vision_opencv) package yourself (and all ros packages depending on it) so it can link on OpenCV3+.
         * On Kinetic/Melodic/Noetic, build from source with *xfeatures2d* module (and *nonfree* module if needed) the same OpenCV version already installed on the system. You will then avoid breaking `cv_bridge` with `rtabmap_ros`. If you want to install a more recent OpenCV version, I recommend to uninstall `libopencv*` libraries (with all ros packages depending on it) and rebuild all those ros packages in your catkin workspace (to make sure `cv_bridge` is linked on the OpenCV version you just compiled).
  
    * [g2o](https://github.com/RainerKuemmerle/g2o): Should be already installed by `ros-noetic-libg2o`.

    * [GTSAM](https://gtsam.org/get_started/): Install via PPA to avoid building from source. If you install from source, make sure to build with `cmake  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON`.
    
    * [libpointmatcher](https://github.com/ethz-asl/libpointmatcher): **Recommended** if you are going to use lidars. Follow their [instructions](https://github.com/ethz-asl/libpointmatcher#quick-start) to install.

2. Install RTAB-Map standalone libraries. Add `-DCMAKE_INSTALL_PREFIX=~/catkin_ws/devel` to `cmake` command below if you want to install in your Catkin's devel folder without `sudo`. **Do not clone in your Catkin workspace**.
    ```bash
    $ cd ~
    $ git clone https://github.com/introlab/rtabmap.git rtabmap
    $ cd rtabmap/build
    $ cmake ..  [<---double dots included]
    $ make
    $ sudo make install
    ```

3. Install RTAB-Map ros-pkg in your src folder of your Catkin workspace.
 
    ```bash
    $ cd ~/catkin_ws
    $ git clone https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros
    $ catkin_make -j1
    ```
    * Use `catkin_make -j1` if compilation requires more RAM than you have (e.g., some files require up to ~2 GB to build depending on gcc version).
    * Options:
        * Add `-DRTABMAP_SYNC_MULTI_RGBD=ON` to `catkin_make` if you plan to use multiple cameras.
        * Add `-DRTABMAP_SYNC_USER_DATA=ON` to `catkin_make` if you plan to use user data synchronized topics.

## Build from source for Nvidia Jetson
These instructions are for Jetpack 3 (Ubuntu 16.04 with ROS Kinetic). For **Jetpack 4** (Ubuntu 18.04 with ROS Melodic), see this [post](https://github.com/introlab/rtabmap/issues/427#issuecomment-608052821).

To use `rtabmap_ros` on Jetson, you can follow the instructions above if you don't care if OpenCV is built for Tegra. However, if you want `rtabmap` to use OpenCV 4 Tegra, we must re-build [vision_opencv](https://github.com/ros-perception/vision_opencv) stack from source too to avoid conflicts with [vision_opencv](https://github.com/ros-perception/vision_opencv) stack binaries from ros (which are linked on a not optimized version of OpenCV). Here are the steps:
1. Install [JetPack](https://developer.nvidia.com/embedded/jetpack) with OpenCV on the Jetson.
2. Do steps 1.2 and 1.3 from http://wiki.ros.org/kinetic/Installation/Ubuntu
3. Install non-opencv dependent ros packages:
    * Jetpack 3: `sudo apt-get install ros-kinetic-ros-base ros-kinetic-image-transport ros-kinetic-tf ros-kinetic-tf-conversions ros-kinetic-eigen-conversions ros-kinetic-laser-geometry ros-kinetic-pcl-conversions ros-kinetic-pcl-ros ros-kinetic-move-base-msgs ros-kinetic-rviz ros-kinetic-octomap-ros ros-kinetic-move-base libhdf5-openmpi-dev libsuitesparse-dev`
4. Do step 1.6 from http://wiki.ros.org/kinetic/Installation/Ubuntu
5. [Create your catkin workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace)
6. *Optional:* Install g2o and/or GTSAM dependencies as above (increase visual odometry and graph optimization accuracy).
7. To avoid [libGL undefined errors](https://devtalk.nvidia.com/default/topic/1007290/jetson-tx2/building-opencv-with-opengl-support-/post/5141945/#5141945):
     ```
     $ cd /usr/lib/aarch64-linux-gnu/
     # Jetpack 3:
     $ sudo ln -sf tegra/libGL.so libGL.so
     # Jetpack 4:
     sudo ln -sf libGL.so.1.0.0 libGL.so
     ```
     
8. To avoid [libvtkproj4 errors](https://github.com/PointCloudLibrary/pcl/issues/1594#issuecomment-283873617):
    ```
    $ sudo ln -s /usr/lib/aarch64-linux-gnu/libvtkCommonCore-6.2.so /usr/lib/libvtkproj4.so
    $ sudo ln -s /usr/lib/aarch64-linux-gnu/libvtkCommonCore-6.2.so /usr/lib/aarch64-linux-gnu/libvtkproj4-6.2.so.6.2.0
    ```
9. Install RTAB-Map standalone libraries. Add `-DCMAKE_INSTALL_PREFIX=~/catkin_ws/devel` to `cmake` command below if you want to install in your Catkin's devel folder without `sudo`. **Do not clone in your Catkin workspace**.
 
    ```bash
    $ cd ~
    $ git clone https://github.com/introlab/rtabmap.git rtabmap
    $ cd rtabmap/build
    $ cmake ..  [<---double dots included]
    $ make
    $ sudo make install
    ```
10. Clone [vision_opencv](https://github.com/ros-perception/vision_opencv), [image_transport_plugins](https://github.com/ros-perception/image_transport_plugins) and `rtabmap_ros` packages in your catkin_ws:

   ```bash
    $ cd ~/catkin_ws
    $ git clone https://github.com/ros-perception/vision_opencv src/vision_opencv
    $ git clone https://github.com/ros-perception/image_transport_plugins.git src/image_transport_plugins
    $ git clone https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros
    $ catkin_make -j2
   ```

### Update to new version 

```bash
###########
# rtabmap
###########
$ cd rtabmap
$ git pull origin master
$ cd build
$ make
$ make install
# Do "sudo make install" if you installed rtabmap in "/usr/local"

###########
# rtabmap_ros
###########
$ roscd rtabmap_ros
$ git pull origin master
$ roscd
$ cd ..
$ catkin_make -j1 --pkg rtabmap_ros
```


