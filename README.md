# Human and Object Detection with Intel D435 Depth Camera

### Table of contents
1. [Introduction](#introduction)
2. [Installation guide](#installation_guide)
    1. [System setup](#system_setup)
    2. [Installing dependencies](#installing_dependencies)
    3. [Building project](#building_project)
3. [Launching project](#launching_project)

## 1. Introduction <a name="introduction"></a>

Human and object detection with Intel D435 Depth Camera is applied by using `darknet_ros` package. 
- https://github.com/leggedrobotics/darknet_ros

Setup is done according to the video in the following youtube link with the source code in the drive link.
- https://www.youtube.com/watch?v=jxH68h8wzTM
- https://drive.google.com/drive/folders/1UHXizZZQEcFSx9R2IreEWMQO6SBcUyPy

## 2. Installation guide <a name="installation_guide"></a>

### 2.1. System setup <a name="system_setup"></a>
This project is developed for `Ubuntu 20.04` with `ROS Noetic`. Be sure your system has the same configuration. You can check the following links to install them:

 - `Ubuntu 20.04` setup guide link: https://releases.ubuntu.com/bionic/ 
 - `ROS Noetic` setup guide link: https://wiki.ros.org/noetic/Installation/Ubuntu 

 It is required that the `ROS Noetic` should be sourced in each terminal to run the simulation as mentioned in the `ROS Noetic` setup guide link. To do that,
 ```
source /opt/ros/noetic/setup.bash
 ```
should be sourced in each terminal. To make it easier, it can be added to the `.bashrc` file for automatic sourcing. Details can be found in the `ROS Noetic` setup guide link. Sourcing `ROS Noetic` will no longer be mentioned in further steps.

Bonus: If you have a different Linux or ROS distro than the specified requirements, you may prefer to use a Docker container. To do that, check the `docker` folder.

### 2.2. Installing dependencies <a name="installing_dependencies"></a>

To have a clear installation, updating and upgrading your system is recommended.
```
sudo apt-get update
sudo apt-get upgrade
```

For the installation of packages, basic tools are required. 
```
sudo apt install git apt-utils python3-catkin-tools wget libtool
```

Basic ROS packages are required for the project.
```
sudo apt install ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-tf ros-noetic-ddynamic-reconfigure ros-noetic-diagnostic-updater
```

Realsense2 is required for the Intel D435 Camera.
```
sudo apt install ros-noetic-realsense2-camera ros-noetic-realsense2-description
```

Check whether OpenCV version is greater than 4, i.e. OpenCV4 is installed.
```
pkg-config --modversion opencv4
```

If not, install OpenCV4 by following the link below
- https://rodosingh.medium.com/using-cmake-to-build-and-install-opencv-for-python-and-c-in-ubuntu-20-04-6c5881eebd9a

Then, go to `opt/ros/noetic/share/cv_bridge/cmake` and change the `cv_bridgeConfig.cmake` file:
Find line `set(_include_dirs "include;/usr/include;/usr/include/opencv")` and change with `set(_include_dirs "include;/usr/include;/usr/local/include/opencv4")`

### 2.3. Building project <a name="building_project"></a>

Open a terminal (will be mentioned as T1) and create a directory preferably in the `home` location as
```
mkdir -p sabes_ws
cd sabes_ws
```
Clone the project repository (T1)
```
git clone git@github.com:cemkucukgenc/darknet_ros_3D_v1.git
```
To build the project in `sabes_ws/darknet_ros_3D_v1/catkin_ws` (T1)
```
cd darknet_ros_3D_v1/catkin_ws
catkin build
```

## 3. Launching project <a name="launching_project"></a>

To run the project (T1)
```
source /sabes_ws/darknet_ros_3D_v1/catkin_ws/devel/setup.bash
roslaunch launch_inference launch_inference.launch
```
To see the output in the ROS topic (T2)
```
source /sabes_ws/darknet_ros_3D_v1/catkin_ws/devel/setup.bash
rostopic echo /objects_position/message
```

If you encounter issues with running, you can check this link
- https://github.com/leggedrobotics/darknet_ros/issues/381
