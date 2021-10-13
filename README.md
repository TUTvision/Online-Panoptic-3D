# Online Panoptic 3D Reconstruction

Panoptic segmentation model:
[https://github.com/TUTvision/ScanNet-EfficientPS](https://github.com/TUTvision/ScanNet-EfficientPS)

## requirements
- ROS melodic [link]
- Voxblox [link]

## Installation
0. Install ROS Melodic, initialise catkin workspace and build Voxblox-ROS
```
# install ROS
# follow instructions in http://wiki.ros.org/melodic/Installation/Ubuntu
# we recommend the desktop-full version, otherwise you will need to install some additional packages (if build of Voxblox or this package fails, cmake will tell you what's missing)

# initialise catkin
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin config --extend /opt/ros/kinetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel

# Build Voxblox-ROS
cd ~/catkin_ws/src/
git clone https://github.com/ethz-asl/voxblox.git
wstool init . ./voxblox/voxblox_https.rosinstall
wstool update

cd ~/catkin_ws/
catkin build voxblox_ros

```
1. Clone repository to catkin workspace
```
cd ~/catkin_ws/src/
git clone git@github.com:TUTvision/Online-Panoptic-3D.git
```
2. Update submodules (dlib)
```
cd Online-Panoptic-3D
git submodule update --init --recursive
```
2. Build the ROS package
```
cd ~/catkin_ws
catkin build Online-Panoptic-3D
```

## How to run
