# Online Panoptic 3D Reconstruction

![0000_example](https://user-images.githubusercontent.com/35113270/137200642-f6c37be2-f0bf-4a3f-94d4-aecb76cf8f57.gif)

Panoptic segmentation model:
[https://github.com/TUTvision/ScanNet-EfficientPS](https://github.com/TUTvision/ScanNet-EfficientPS)

## requirements
- ROS melodic [link]
- Voxblox [link]

## ROS Installation
0. Install ROS Melodic, initialise catkin workspace and build Voxblox-ROS
```
# install ROS
# follow the instructions in http://wiki.ros.org/melodic/Installation/Ubuntu
# we recommend the desktop-full version, otherwise you will need to install some additional packages
# (if compilation of Voxblox or this package fails, cmake will tell you what's missing)

# initialise catkin
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin config --extend /opt/ros/melodic
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

### Prerequisites

- Process the data to a suitable format. We provide a data publisher node ([python/data_publisher_node.py](https://github.com/TUTvision/Online-Panoptic-3D/blob/main/python/data_publisher_node.py)) for reading data with our specific format (explained at [docs/data.md](https://github.com/TUTvision/Online-Panoptic-3D/blob/main/docs/data.md)) and publishing it as ROS messages.
- Also check the documentation for our ros node parameters at [docs/parameters.md](https://github.com/TUTvision/Online-Panoptic-3D/blob/main/docs/parameters.md)

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

After the scan sequence has ended, a mesh (\*.ply), id color values (\*_label_map.txt), id's and classes associated with voxel center points (\*.txt) and average excecution times (\*_timings.txt) are saved to the output directory. (see [docs/parameters.md](https://github.com/TUTvision/Online-Panoptic-3D/edit/main/docs/parameters.md))

### Run for a directory of scans

You can process multiple scans from a directory with [scripts/run_fusion_for_dir.py](https://github.com/TUTvision/Online-Panoptic-3D/blob/main/scripts/run_fusion_for_dir.py) similar to above instructions. Design a launch file with your desired parameters and give it as an input to the script.

For example:
```
cd ~/catkin_ws/src/Online-Panoptic-3D

python scripts/run_fusion_for_dir.py \
    --source_dir ~/DATA/ScanNet/tasks/scans_test \
    --launch_file launch/reconstruction_from_dir.launch
```

### Evaluation of ScanNet results

1. Generate panoptic ground truth
```
cd ~/catkin_ws/src/Online-Panoptic-3D/scripts

python3 export_panoptic_gt.py \
    --source_dir path/to/scannet/data/root \
    --output_name panoptic_gt.txt \
    -l path/to/ScanNet/scannetv2-labels.combined.tsv \
    -sc path/to/ScanNet/
```
2. Perform nearest neighbour search to transform 3D panoptic labels to ScanNet gt coordinates
```
python3 mesh_labels_to_gt_from_dir.py \
    -i path/to/scannet/data/root \
    -p name_of_reconstruction_output.ply  \
    -l name_of_reconstruction_output_test_color_map.txt  \
    -s gt_panoptic.txt  \
    -o transofrmed_labels_name.txt
```
3. Evaulate panoptic quality accross a directory of scans
 ```
python3 evaluate_pq_from_dir_multithread.py \
    -i path/to/scannet/data/root \
    -p transofrmed_labels_name.txt \
    -gt gt_panoptic.txt \
    -o path/to/results/directory/scannet_pq.txt
```

You can also reformat results for SCanNet evaluation with [scripts/mesh_labels_to_scannet_format_from_dir.py](https://github.com/TUTvision/Online-Panoptic-3D/blob/main/scripts/mesh_labels_to_scannet_format_from_dir.py)
