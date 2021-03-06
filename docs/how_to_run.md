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
