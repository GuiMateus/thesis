# KUKA LBR iiwa 7 Screw Detection

This repository is ROS Melodic package made to control the screw disassembly KUKA LBR iiwa 7 robot from Aalborg university. This repository is part of a Masters project from group 963 from the Robotics Masters program.

This package allows the launch of a screw detection and segmentation node, and an impedance control strategy used to introduce a peg in the screw.

A small description of each folder is given below:

  - **matlab_scripts**: Consists of MATLAB scripts to run the robot program.
  - **robot_comm**: Handles all the communication of the robot's subsystems.
  - **service**: Consists of services files for robot communication.
  - **tf_virtual**: Creates a virtual environment used for pixel to robot transformations.
  - **vision**: Handles screw detection using YOLOv4 and DeepLabV3+.

## Getting Started

The system contains YOLOv4 and DeepLabV3+. However, YOLOv4 still has to be manually built, for that follow the instructions on the [repo](https://github.com/AlexeyAB/darknet).

OBS: To build darknet you need to get a CMake version bigger than 3.12, which is not compatible with ROS. Do not uninstall the current version installed in the system.


In case of problems with DeepLabV3+, follow the [repo](https://github.com/jfzhang95/pytorch-deeplab-xception).


### Prerequisites

This requires a system setup with ROS. It is recommended to use `Ubuntu 18.04`, `ROS Melodic`, `MATLAB2020b` with `ROS toolbox`, `Instrument Control toolbox`, and the `KUKA Sunrise Toolbox`.

### Creating workspace and cloning the repository

```bash
# create a catkin workspace
mkdir -p catkin_ws/src && cd catkin_ws/src

# Clone the repository from bitbucket.
git clone --recursive git@bitbucket.org:rob9disassembly/rob9.git

# install dependencies
sudo apt update -qq && cd ..
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro melodic -y

#install python catkin tools. Needed for catkin build command
sudo apt-get install python-catkin-tools

# build the workspace
catkin build
```

### Installing dependencies

In order for the two Intel Realsens cameras on the **LH7+** to work the Intel Realsense SDK have to be installed. Go to Intel RealSense's website and [install the Intel RealSense SDK for Linux](https://www.intelrealsense.com/developers/).

### Intel RealSense Camera Configuration

The two Intel Realsense d435 cameras are launched using the `rs_camera.launch` file found in the **realsense2_camera** package. For this launch file the following arguments must be setup for each camera:

```
<arg name="depth_width" default="1280"/>
<arg name="depth_height" default="720"/>
<arg name="enable_depth" default="true"/>

<arg name="infra_width" default="1280"/>
<arg name="infra_height" default="720"/>
<arg name="enable_infra1" default="true"/>
<arg name="enable_infra2" default="true"/>

<arg name="color_width" default="1280"/>
<arg name="color_height" default="720"/>
<arg name="enable_color" default="true"/>

<arg name="enable_pointcloud" default="true"/>
<arg name="enable_sync" default="false"/>
<arg name="align_depth" default="true"/>

<arg name="initial_reset" default="true"/>

```

### Launching The System

In order to launch the camera, communication, and robot the following command must be typed on the terminal inside of the catkin workspace.

```shell
roslaunch tf_virtual main.launch
```

Then, to launch the vision service, the following must be typed.

```shell
rosrun vision main.py
```

# Authors

* **[Guilherme Mateus Martins](mailto:gmateu16@student.aau.dk)**   - *Group member* - [Guilherme Mateus Martins](https://bitbucket.org/%7Bba72de4e-9cb6-4e73-89db-24d4d8f12fe7%7D/)
* **[Jacob Krunderup Sørensen](mailto:jksa16@student.aau.dk)**     - *Group member* - [Jacob Krunderup Sørensen](https://bitbucket.org/%7Bc489c61d-40d9-449d-9964-13a350e237df%7D/)

# Acknowledgements

* Aalborg university
* Dimitris Chrysostomou
* Sebastian Hjorth