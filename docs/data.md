Currently we only support data saved in a directory in a specific format. With some work, however, one could design their own input pipeline to match our panoptic point cloud format.

See our panoptic segmentation model's documentation on how to process ScanNet scenes to this format: [ScanNet-EfficientPS](https://github.com/TUTvision/ScanNet-EfficientPS)

Panoptic segmentations are encoded as 8-bit RGB point clouds:
- First color channel (R) contains the confidence score of the points estimated class as a 8-bit unsigned integer: scores in the [0, 1] range are binned to integers in [0, 255] range
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
