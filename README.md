# Online Panoptic 3D Reconstruction

Panoptic segmentation model:
[https://github.com/TUTvision/ScanNet-EfficientPS](https://github.com/TUTvision/ScanNet-EfficientPS)

## requirements
- ROS melodic [link]
- Voxblox [link]

## ROS Installation
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
catkin build online-panoptic-3d
```

## Standalone installation
Currently not supported. However, the tracker library and Voxblox are separate from ROS and thus could be compiled as a standalone project as well with some work. You will need to implement your own data / sensor input pipeline.

## How to run

Currently, input data has to be read from disk, however in the future we plan to add a way to work with real-time sensor streams as well.

### Input data

Currently we only support data saved a directory in a specific format. With some work, however, one could design their own input pipeline to match our panoptic point cloud format.

See our panoptic segmentation model's documentation on how to process ScanNet scenes to this format: [ScanNet-EfficientPS](https://github.com/TUTvision/ScanNet-EfficientPS)

Panoptic segmentations are encoded as 8-bit RGB point clouds:
- First channel (R) contains the confidence score of the points estimated class as a 8-bit unsigned integer: scores in the [0, 1] range are binned to integers in [0, 255] range
- Second channel (G) contains the estimated class of the point in [0, 255] range
- Third channel (B) contains the estimated panoptic instance in [0, 255] range

We provide a separate ROS node ([python/data_publisher_node.py](https://github.com/TUTvision/Online-Panoptic-3D/blob/main/python/data_publisher_node.py)) for publishing these point clouds from a directory with predetermined structure:
```
scan_root
|-- color/ # color images
|  |--  ...
|-- depth/ # depth images
|  |--  ...
|-- pose/  # poses in ScanNet format
|  |--  ...
|-- panoptic/ # images processed with ScanNet-EfficientPS
|  |--  ...
|-- colormask/ # optional, for visualisation only
|  |--  ...
|-- intrinsic_color.txt # camera intrinsics of color sensor
|-- intrinsic_depth.txt # camera intrinsics of depth sensor
```

### Parameters

Voxblox's parameters work as explained in their documentation, and can be given as an input to our package:
[https://voxblox.readthedocs.io/en/latest/pages/The-Voxblox-Node.html](https://voxblox.readthedocs.io/en/latest/pages/The-Voxblox-Node.html)

Subscribed ROS topics:

Published ROS topics:

Parameters to the data publisher node:

Parameters to the reconstruction node:


### Run for a single scan

The easiest way to run the reconstruction node is via roslaunch. We provide a launch file ([launch/reconstruction_from_dir.launch](https://github.com/TUTvision/Online-Panoptic-3D/blob/main/launch/reconstruction_from_dir.launch)) as an example:
```
cd ~/catkin_ws
source devel/setup.bash

roslaunch online-panoptic-3d reconstruct_from_dir.launch
```

This will do the following:
1. Read necessary data in ScanNet format from files and publish them as ROS messages with our publisher node
2. Apply depth_image_proc to form panoptic point clouds from published data
3. Run our panoptic reconstruction node
4. Visualise online results in RViz

After the scan sequence has ended, a mesh (*.ply), id color values (*_label_map.txt), id's and classes associated with voxel center points (*.txt) and average excecution times (*_timings.txt) are saved to the output directory. (see parameters above)

### Run for a directory of scans

You can process multiple scans from a directory with [scripts/run_fusion_for_dir.py](https://github.com/TUTvision/Online-Panoptic-3D/blob/main/scripts/run_fusion_for_dir.py) similar to above instructions. Design a launch file with your desired parameters and give it as an input to the script.

For example:
```
cd ~/catkin_ws/src/Online-Panoptic-3D

python scripts/run_fusion_for_dir.py \\
    --source_dir ~/DATA/ScanNet/tasks/scans_test \\
    --launch_file launch/reconstruction_from_dir.launch
```

### Evaluation of ScanNet results
