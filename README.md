rtabmap_ros [![Build Status](https://travis-ci.org/introlab/rtabmap_ros.svg?branch=master)](https://travis-ci.org/introlab/rtabmap_ros)
===========

RTAB-Map's ROS package.

For more information, demos and tutorials about this package, visit [rtabmap_ros](http://wiki.ros.org/rtabmap_ros) page on ROS wiki.

For the RTAB-Map libraries and standalone application, visit [RTAB-Map's home page](http://introlab.github.io/rtabmap) or [RTAB-Map's wiki](https://github.com/introlab/rtabmap/wiki).

# Installation 

## ROS distribution 
RTAB-Map is released as binaries in the ROS distribution.
* Lunar
    ```
    $ sudo apt-get install ros-lunar-rtabmap-ros
    ```
* Kinetic
    ```
    $ sudo apt-get install ros-kinetic-rtabmap-ros
    ```
* Jade
    ```
    $ sudo apt-get install ros-jade-rtabmap-ros
    ```
* Indigo
    ```
    $ sudo apt-get install ros-indigo-rtabmap-ros
    ```
* Hydro:
    ```
    $ sudo apt-get install ros-hydro-rtabmap-ros
    ```
    * Note that rtabmap_ros Hydro binaries are stuck at version 0.8.12. To use the latest version, see [Build from source](https://github.com/introlab/rtabmap_ros#build-from-source) below.

When launching `rtabmap_ros`'s nodes, if you have the error `error while loading shared libraries...`, add the next line at the end of your `~/.bashrc` to fix it:
    
```bash
$ export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/kinetic/lib/x86_64-linux-gnu
```

## Build from source
This section shows how to install RTAB-Map ros-pkg on **ROS Hydro/Indigo/Jade/Kinetic/Lunar** (Catkin build). RTAB-Map works only with the PCL >=1.7, which is the default version installed with ROS Hydro/Indigo/Jade/Kinetic/Lunar (**Fuerte and Groovy are not supported**).

* The next instructions assume that you have set up your ROS workspace using this [tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace). I will use kinetic prefix for convenience, but it should work with Hydro, Indigo, Jade and Lunar. The workspace path is `~/catkin_ws` and your `~/.bashrc` contains:
 
    ```bash
    $ source /opt/ros/kinetic/setup.bash
    $ source ~/catkin_ws/devel/setup.bash
    ```

 0. Required dependencies
     * The easiest way to get all them (Qt, PCL, VTK, OpenCV, ...) is to install/uninstall rtabmap binaries:
          ```bash
          $ sudo apt-get install ros-kinetic-rtabmap ros-kinetic-rtabmap-ros
          $ sudo apt-get remove ros-kinetic-rtabmap ros-kinetic-rtabmap-ros
          ```
 
 1. Optional dependencies
     * If you want SURF/SIFT on Indigo/Jade (Hydro has already SIFT/SURF), you have to build [OpenCV]([OpenCV](http://opencv.org/)) from source to have access to *nonfree* module. Install it in `/usr/local` (default) and the rtabmap library should link with it instead of the one installed in ROS. 
         * On Indigo/Jade, I recommend to use latest 2.4 version ([2.4.11](https://github.com/Itseez/opencv/archive/2.4.11.zip)) and build it from source following these [instructions](http://docs.opencv.org/doc/tutorials/introduction/linux_install/linux_install.html#building-opencv-from-source-using-cmake-using-the-command-line). RTAB-Map can build with OpenCV3+[xfeatures2d](https://github.com/Itseez/opencv_contrib/tree/master/modules/xfeatures2d) module, but rtabmap_ros package will have libraries conflict as cv-bridge is depending on OpenCV2. If you want OpenCV3, you should build ros [vision-opencv](https://github.com/ros-perception/vision_opencv) package yourself (and all ros packages depending on it) so it can link on OpenCV3.
         * On Kinetic/Lunar, I recommend to use OpenCV3+[xfeatures2d](https://github.com/Itseez/opencv_contrib/tree/master/modules/xfeatures2d) module already installed by ROS. You can also install OpenCV2, but rtabmap_ros package will have libraries conflict as cv-bridge is depending on OpenCV3. Thus if you want OpenCV2 on Kinetic/Lunar, you should build ros [vision-opencv](https://github.com/ros-perception/vision_opencv) package yourself (and all ros packages depending on it) so it can link on OpenCV2.
  
    * g2o: Use directly the binaries `ros-kinetic-libg2o`. However, [this g2o version](https://github.com/felixendres/g2o/tree/c++03) (c++03 branch) built from source may be faster than the binaries (install `libsuitesparse-dev` before building `g2o`) and would be [required to avoid some crashes](http://official-rtab-map-forum.67519.x6.nabble.com/ROS-2D-occupancy-grid-tp1204p1215.html). To build RTAB-Map against [latest official g2o version](https://github.com/RainerKuemmerle/g2o) built from source, g2o should be built with `-DBUILD_WITH_MARCH_NATIVE=OFF` to avoid some segmentation faults caused by Eigen.

    * [GTSAM](https://collab.cc.gatech.edu/borg/gtsam): Follow installation instructions from [here](https://collab.cc.gatech.edu/borg/gtsam/#quickstart). RTAB-Map needs latest version from source, it will **not build** with 3.2.1. Set cmake variable `GTSAM_USE_SYSTEM_EIGEN=ON` to make sure the same Eigen version is used across all dependencies to avoid crashes.
        ```bash
        git clone https://bitbucket.org/gtborg/gtsam.git
        ```

    * [Freenect2](https://github.com/OpenKinect/libfreenect2): Follow installation instructions from [here](https://github.com/OpenKinect/libfreenect2#debianubuntu-1404-perhaps-earlier).

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
    * Use `catkin_make -j1` if compilation requires more RAM than you have (e.g., some files require up to ~1.8 GB to build).

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


