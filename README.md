# ROS Wrapper for GPD

* [Author's website](http://www.ccs.neu.edu/home/atp/)
* [License](https://github.com/atenpas/gpd_ros/blob/master/LICENSE.md)
* [GPD library](https://github.com/atenpas/gpd)

## Overview

A ROS wrapper around the [GPD](https://github.com/atenpas/gpd) package for detecting 6-DOF grasp poses for a
2-finger robot hand (e.g., a parallel jaw gripper) in 3D point clouds.

## 1) Installation

The following instructions have been tested on **Ubuntu 16.04**. Similar
instructions should work for other Linux distributions.

1. Install GPD. You can follow [these instructions](https://github.com/atenpas/gpd#install). Make sure to run `make install` to install GPD as a library.
   1.1 Requirements

   1. [PCL 1.9 or newer](http://pointclouds.org/)
   2. [Eigen 3.0 or newer](https://eigen.tuxfamily.org)
   3. [OpenCV 3.4 or newer](https://opencv.org)

   1.2 Installation

   The following instructions have been tested on **Ubuntu 16.04**. Similar
   instructions should work for other Linux distributions.

   1.2.1. Install [PCL](http://pointclouds.org/) and
   [Eigen](https://eigen.tuxfamily.org). If you have ROS Indigo or Kinetic
   installed, you should be good to go.

   1.2.2. Install OpenCV 3.4 ([tutorial](https://www.python36.com/how-to-install-opencv340-on-ubuntu1604/)).

   1.2.3. Clone the repository into some folder:

   ```
   git clone https://github.com/atenpas/gpd
   ```

   1.2.4. Build the package:

   ```
   cd gpd
   mkdir build && cd build
   cmake ..
   make -j
   sudo make install
   ```

   You can optionally install GPD with `sudo make install` so that it can be used by other projects as a shared library.

   If building the package does not work, try to modify the compiler flags, `CMAKE_CXX_FLAGS`, in the file CMakeLists.txt.

   1.3 You'll need to change path in [line 32](https://github.com/atenpas/gpd/blob/6327f20eabfcba41a05fdd2e2ba408153dc2e958/cfg/ros_eigen_params.cfg#L32) 

2. Clone this repository into the `src` folder of your catkin workspace:

   ```
   cd <location_of_your_workspace>/src
   git clone https://github.com/atenpas/gpd_ros
   ```

3. Build your catkin workspace:

   ```
   cd <location_of_your_workspace>
   catkin build
   ```

## 2) Generate Grasps for a Point Cloud on a ROS Topic

First, you need to modify the config file in your `gpd` folder, e.g., 
`<path_to_gpd>/cfg/ros_eigen_params.cfg`. Search for parameters that have 
absolute file paths and change them to actual paths on your system.

Next, you need to modify the path in the ROS launch file that points to the 
config file that you changed in the previous step, e.g., 
[this line](https://github.com/atenpas/gpd_ros/blob/master/launch/ur5.launch#L18).

Now, you can run GPD as a ROS node. The following command will launch a ROS node
that waits for point clouds on the ROS topic `/cloud_stitched`. Once a point
cloud is received, the node will search the cloud for grasps.

```
roslaunch gpd_ros ur5.launch
```
Call service to get grasps poses
```
rosservice call /detect_grasps/detect_grasps_poses "{}"
```

## 3) Using Advanced Messages

If you want to speed up GPD or look for grasps on a specific object, you should 
use one of these messages: [CloudSamples](https://github.com/atenpas/gpd_ros/blob/master/msg/CloudSamples.msg), [CloudIndexed](https://github.com/atenpas/gpd_ros/blob/master/msg/CloudIndexed.msg). Both of these messages build up on the [CloudSources](https://github.com/atenpas/gpd_ros/blob/master/msg/CloudSources.msg) message that can be used to represent a point cloud whose points were seen by multiple cameras or from multiple viewpoints.

As a typical use case for the `CloudSamples` message, consider a table with a single object on top of it, observed by one camera. The complete point cloud should be put in the message so that GPD can check grasp poses against collisions with the table. Samples in the message should correspond to points on the object so that GPD can search for grasps on the object (and avoids searching for grasps on the table).

## 4) Troubleshooting

If `catkin_make` cannot find `libinference_engine.so`, required by OpenVino, make 
sure that `LD_LIBRARY_PATH` contains the path to that library:

```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:path_to_dldt/inference-engine/bin/intel64/Release/lib/
```
