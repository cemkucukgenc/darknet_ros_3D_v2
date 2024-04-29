# Human and Object Detection with Intel D435 Depth Camera

### Table of contents
- [Human and Object Detection with Intel D435 Depth Camera](#human-and-object-detection-with-intel-d435-depth-camera)
    - [Table of contents](#table-of-contents)
  - [1. Introduction ](#1-introduction-)
  - [2. Installation guide ](#2-installation-guide-)
    - [2.1. System setup ](#21-system-setup-)
    - [2.2. Installing dependencies ](#22-installing-dependencies-)
    - [2.3. Building project ](#23-building-project-)
  - [3. Launching project ](#3-launching-project-)

## 1. Introduction <a name="introduction"></a>

Human and object detection with Intel D435 Depth Camera is applied by using `gb_visual_detection_3d` package, which build on top of `darknet_ros` package.
- https://github.com/leggedrobotics/darknet_ros (`Main` branch since it is already `Noetic`)
- https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d/tree/noetic (`Noetic` branch)
- https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d_msgs/tree/melodic (`Melodic` branch)
- https://www.youtube.com/watch?v=JvAMI9OxQDU 


## 2. Installation guide <a name="installation_guide"></a>

### 2.1. System setup <a name="system_setup"></a>
This project is developed for `Ubuntu 20.04` with `ROS Noetic`. Be sure your system has the same configuration. You can check the following links to install them:

 - `Ubuntu 20.04` setup guide link: https://releases.ubuntu.com/focal/ 
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
sudo apt install ros-noetic-realsense2-camera ros-noetic-realsense2-description ros-noetic-rgbd-launch
```

### 2.3. Building project <a name="building_project"></a>

Open a terminal (will be mentioned as T1) and create a directory preferably in the `home` location as
```
mkdir -p sabes_ws
cd sabes_ws
```
Clone the project repository (T1)
```
git clone git@github.com:cemkucukgenc/darknet_ros_3D_v2.git
```
To build the project in `sabes_ws/darknet_ros_3D_v2/catkin_ws` (T1)
```
cd darknet_ros_3D_v2/catkin_ws
catkin build
```

## 3. Launching project <a name="launching_project"></a>

To run the camera driver (T1)
```
source /sabes_ws/darknet_ros_3D_v2/catkin_ws/devel/setup.bash
roslaunch realsense2_camera rs_rgbd.launch
```
To run the project (T2)
```
source /sabes_ws/darknet_ros_3D_v2/catkin_ws/devel/setup.bash
roslaunch darknet_ros_3d darknet_ros_3d.launch
```
To see the output in the ROS topic (T3)
```
source /sabes_ws/darknet_ros_3D_v2/catkin_ws/devel/setup.bash
rostopic echo /darknet_ros_3d/bounding_boxes
```
Bonus: Bookmarked issue pages that helped me to debug
- https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d/issues/15
