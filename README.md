rtabmap_ros
=======

RTAB-Map's ROS package.

For more information, demos and tutorials about this package, visit [rtabmap_ros](http://wiki.ros.org/rtabmap_ros) page on ROS wiki.

For the RTAB-Map libraries and standalone application, visit [RTAB-Map's home page](http://introlab.github.io/rtabmap) or [RTAB-Map's wiki](https://github.com/introlab/rtabmap/wiki).

#### CI Latest

  <table>
    <tbody>
        <tr>
           <td>ROS 1</td>
           <td><a href="https://github.com/introlab/rtabmap_ros/actions/workflows/ros1.yml"><img src="https://github.com/introlab/rtabmap_ros/actions/workflows/ros1.yml/badge.svg" alt="Build Status"/> <br> <a href="https://github.com/introlab/rtabmap_ros/actions/workflows/docker.yml"><img src="https://github.com/introlab/rtabmap_ros/actions/workflows/docker.yml/badge.svg" alt="Build Status"/>
           </td>
        </tr>
        <tr>
           <td>ROS 2</td>
           <td><a href="https://github.com/introlab/rtabmap_ros/actions/workflows/ros2.yml"><img src="https://github.com/introlab/rtabmap_ros/actions/workflows/ros2.yml/badge.svg" alt="Build Status"/>
           </td>
        </tr>
     </tbody>
  </table>
 
 #### ROS Binaries
 
 <table>
    <tbody>
        <tr>
           <td rowspan="2">ROS 1</td>
           <td>Melodic</td>
            <td><a href="http://build.ros.org/job/Mbin_ubv8_uBv8__rtabmap_ros__ubuntu_bionic_arm64__binary/"><img src="http://build.ros.org/buildStatus/icon?job=Mbin_ubv8_uBv8__rtabmap_ros__ubuntu_bionic_arm64__binary" alt="Build Status"/></td>
        </tr>
        <tr>
            <td>Noetic</td>
            <td><a href="http://build.ros.org/job/Nbin_ufv8_uFv8__rtabmap_ros__ubuntu_focal_arm64__binary/"><img src="http://build.ros.org/buildStatus/icon?job=Nbin_ufv8_uFv8__rtabmap_ros__ubuntu_focal_arm64__binary" alt="Build Status"/></td>
        </tr>
        <tr>
            <td rowspan="4">ROS 2</td>
            <td>Foxy</td>
            <td><a href="http://build.ros2.org/job/Fbin_uF64__rtabmap_ros__ubuntu_focal_amd64__binary/"><img src="http://build.ros2.org/buildStatus/icon?job=Fbin_uF64__rtabmap_ros__ubuntu_focal_amd64__binary" alt="Build Status"/></td>
        </tr>
        <tr>
            <td>Galactic</td>
            <td><a href="http://build.ros2.org/job/Gbin_uF64__rtabmap_ros__ubuntu_focal_amd64__binary/"><img src="http://build.ros2.org/buildStatus/icon?job=Gbin_uF64__rtabmap_ros__ubuntu_focal_amd64__binary" alt="Build Status"/></td>
        </tr>
        <tr>
            <td>Humble</td>
            <td><a href="http://build.ros2.org/job/Hbin_uJ64__rtabmap_ros__ubuntu_jammy_amd64__binary/"><img src="http://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__rtabmap_ros__ubuntu_jammy_amd64__binary" alt="Build Status"/></td>
        </tr>
        <tr>
            <td>Rolling</td>
            <td><a href="http://build.ros2.org/job/Rbin_uJ64__rtabmap_ros__ubuntu_jammy_amd64__binary/"><img src="http://build.ros2.org/buildStatus/icon?job=Rbin_uJ64__rtabmap_ros__ubuntu_jammy_amd64__binary" alt="Build Status"/></td>
        </tr>
    </tbody>
</table>

# Installation 

## ROS2 distribution
**Under construction**: see [ros2 branch](https://github.com/introlab/rtabmap_ros/tree/ros2#rtabmap_ros).

## ROS distribution 
RTAB-Map is released as binaries in the ROS distribution.

```bash
sudo apt install ros-$ROS_DISTRO-rtabmap-ros
```

When launching `rtabmap_ros`'s nodes, if you have the error `error while loading shared libraries...`, try `ldconfig` or add the next line at the end of your `~/.bashrc` to fix it:
    
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/noetic/lib/x86_64-linux-gnu
```

### Docker

* Go to [docker](https://github.com/introlab/rtabmap_ros/tree/master/docker) directory for an example.


## Build from source
This section shows how to install RTAB-Map ros-pkg on **ROS Melodic/Noetic** (Catkin build).

* The next instructions assume that you have set up your ROS workspace using this [tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace). The workspace path is `~/catkin_ws` and your `~/.bashrc` contains:
 
    ```bash
    $ source /opt/ros/$ROS_DISTRO/setup.bash
    $ source ~/catkin_ws/devel/setup.bash
    ```

 0. Required dependencies
     * The easiest way to get all them (Qt, PCL, VTK, OpenCV, ...) is to install/uninstall rtabmap binaries:
          ```bash
          sudo apt install ros-$ROS_DISTRO-rtabmap ros-$ROS_DISTRO-rtabmap-ros
          sudo apt remove ros-$ROS_DISTRO-rtabmap ros-$ROS_DISTRO-rtabmap-ros
          ```
 
 1. Optional dependencies
     * If you want SURF/SIFT on Melodic/Noetic, you have to build [OpenCV]([OpenCV](http://opencv.org/)) from source to have access to *xfeatures2d* and *nonfree* modules (note that SIFT is not in *nonfree* anymore since OpenCV 4.4.0). Install it in `/usr/local` (default) and rtabmap library should link with it instead of the one installed in ROS. 
         * On Melodic/Noetic, build from source with *xfeatures2d* module (and *nonfree* module if needed) the same OpenCV version already installed on the system. You will then avoid breaking `cv_bridge` with `rtabmap_ros`. If you want to install a more recent OpenCV version, I recommend to uninstall `libopencv*` libraries (with all ros packages depending on it) and rebuild all those ros packages in your catkin workspace (to make sure `cv_bridge` is linked on the OpenCV version you just compiled).
  
    * [g2o](https://github.com/RainerKuemmerle/g2o): Should be already installed by `ros-$ROS_DISTRO-libg2o`.

    * [GTSAM](https://gtsam.org/get_started/): Install via PPA to avoid building from source. If you install from source, make sure to build with `cmake  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON`.
    
    * [libpointmatcher](https://github.com/ethz-asl/libpointmatcher): **Recommended** if you are going to use lidars. Follow their [instructions](https://github.com/ethz-asl/libpointmatcher#quick-start) to install. Should be alread installed by `ros-$ROS_DISTRO-libpointmatcher`.

2. Install RTAB-Map standalone libraries. **Do not clone in your Catkin workspace**.
    ```bash
    cd ~
    git clone https://github.com/introlab/rtabmap.git rtabmap
    cd rtabmap/build
    cmake ..  [<---double dots included]
    make -j6
    sudo make install
    ```

3. Install RTAB-Map ros-pkg in your src folder of your Catkin workspace.
 
    ```bash
    cd ~/catkin_ws
    git clone https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros
    catkin_make -j4
    ```
    * Use `catkin_make -j1` if compilation requires more RAM than you have (e.g., some files require up to ~2 GB to build depending on gcc version).
    * Options:
        * Add `-DRTABMAP_SYNC_MULTI_RGBD=ON` to `catkin_make` if you plan to use multiple cameras.
        * Add `-DRTABMAP_SYNC_USER_DATA=ON` to `catkin_make` if you plan to use user data synchronized topics.

## Build from source for Nvidia Jetson
 * For **Jetpack 4** (Ubuntu 18.04 with ROS Melodic), see this [post](https://github.com/introlab/rtabmap/issues/427#issuecomment-608052821).
 * For **Jetpack 3** (Ubuntu 16.04 with ROS Kinetic), see this [post](https://github.com/introlab/rtabmap_ros/issues/655).


### Update to new version 

```bash
###########
# rtabmap
###########
cd rtabmap
git pull origin master
cd build
make
make install
# Do "sudo make install" if you installed rtabmap in "/usr/local"

###########
# rtabmap_ros
###########
roscd rtabmap_ros
git pull origin master
roscd
cd ..
catkin_make -j1 --pkg rtabmap_ros
```


