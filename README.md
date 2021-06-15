# KUKA LBR iiwa 7 Screw Detection

This repo contains the materials used in the Master's Thesis from Guilherme Mateus at Aalborg University. The pipeline contained in it creates 3D semantical and volumetric reconstructions of environments using Deep Learning. This implementation is done using ROS melodic as a framework of communcation.

A small description of each package is given below:

  - **ontologies**: Handles object ontonlogies.
  - **service**: Consists of services files for system communication.
  - **realsense-ros**: Gathers data using realsense camera.
  - **userInterface**: Provides a GUI for users to control the system.
  - **vision**: Handles screw detection using YOLOv4 and DeepLabV3+.

## Getting Started

The system contains YOLOv4 and DeepLabV3+. However, YOLOv4 still has to be manually built under ```bash catkin_ws/src/release/vision/src/vision/pythonClasses/darknet```, for that follow the instructions on the [repo](https://github.com/AlexeyAB/darknet).

OBS: To build darknet you need to get a CMake version bigger than 3.12, which is not compatible with ROS. Do not uninstall the current version installed in the system, instead use a local CMake version.


In case of problems with DeepLabV3+, follow the [repo](https://github.com/jfzhang95/pytorch-deeplab-xception).


### Prerequisites

This requires a system setup with ROS. It is recommended to use `Ubuntu 18.04` with `ROS Melodic`.

### Creating workspace and cloning the repository

```bash
# create a catkin workspace
mkdir -p catkin_ws/src && cd catkin_ws/src

# Clone the repository from bitbucket.
git clone --recursive https://guimateus@bitbucket.org/guimateus/thesis.git

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

Go to Intel Realsense website and [install the SDK for Linux](https://www.intelrealsense.com/developers/).


### Launching The System

In order to launch the camera, communication, and 3D reconstruction system, type the following on a terminal window.

```shell
roslaunch tf_virtual main.launch
```

Then, to launch the vision service, the following must be typed.

```shell
rosrun vision main.py
```

# Authors

* **[Guilherme Mateus Martins](mailto:gmateu16@student.aau.dk)**   - [Git Profile](https://bitbucket.org/%7Bba72de4e-9cb6-4e73-89db-24d4d8f12fe7%7D/) - [LinkedIn](https://www.linkedin.com/in/guilherme-mateus-346b58b5/)

# Acknowledgements

* Aalborg university
* Dimitris Chrysostomou