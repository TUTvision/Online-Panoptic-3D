
All parameters are listed in [../launch/reconstruct_from_dir.launch](https://github.com/TUTvision/Online-Panoptic-3D/blob/main/launch/reconstruct_from_dir.launch).

They can be edited directly in the launch file, or given as command-line arguments, e.g. you can call:
```
roslaunch online-panoptic-3d reconstruct_from_dir.launch \
  data_root:="/home/user/DATA/scannet/scene0000_00"
  output_name:="/home/user/DATA/scannet_reconstruction/scene0000_00_output"
```
## Data publisher node
- data_root 
  - path to directory containing the necessary data (i.e. dirs like color/, depth/ and camera intrinsics)
- data_format
  - format of input data 
    - scannet: our scannet format
    - rtabmap: data exported from rtabmap (needs testing)
  - default: scannet
- publish_rate
  - rate (Hz) at which data is published
  - when 0, will publish as soon as reconstruction node has processed the data
  - when >0, will publish at fixed intervals 
  - default: 0
- publish_interval
  - Interval of published frames. E.g. 10 means only every tenth frame of the data stream is published.
  - For example, if the original video is captured at 30Hz, interval of 10 will result in input rate of 3Hz, meaning one has to process one frame each 333ms for real-time operation
  - default: 10
- reduction_rate
  - percentage of point cloud decimation
  - smaller percentages will speed up processing significantly
  -  e.g. 0.5 means cloud resolution will be reduced by half, i.e. if original images have 1280×720 pixels, the point cloud will have only 640x360 points
  - default: 0.5
    
## online-panoptic-3d node
Voxblox's parameters function as explained in their documentation, and can be given as an input to our reconstruction node:
[https://voxblox.readthedocs.io/en/latest/pages/The-Voxblox-Node.html](https://voxblox.readthedocs.io/en/latest/pages/The-Voxblox-Node.html)


### Tracker parameters
- tracker_type
  - hungarian
    - optimal matching with the hungarian data-association algorithm
  - greedy
    - greedy matching algorithm, faster but not necessarily optimal
  - default: hungarian
- normalise_likelihoods
  - boolean (true/false)
  - if true, match likelihoods are normalised to sum to one accross detections for each target, and association_threshold is set to association_threshold/n_detections
  - only works with hungarian data-association
  - will make the association threshold more flexible
  - default true
- only_process_local_voxels
  - boolean (true/false)
  - when true, only voxels seen in current camera view are taken into account when computing association likelihoods
  - will make association faster and usually more precise as well
  - default: true
- use_local_reference
  - boolean (true/false)
  - when true, only targets seen in current camera view are taken into account when computing association likelihoods
  - when false, processing time will increase with total number of targets
  - should most likely be always true, since targets out of view should not be matched with detections in the camera view anyways
  - default: true
- likelihood_metric
  - how to estimate association likelihood 
    - iou: Intersection over Union between voxel clusters
    - mh: Mean Mahalanobis distance between points in detection and target object represented by a multivariate Gaussian distribution
    - bh: Bhattacharyya distance between detection and target, both represented by a multivariate Gaussian distribution
  - default: iou
- confidence_threshold
  - float in the range (0, 1)
  - threshold below which an object's class is determined as 'void'
  - default: 0.45
- association_threshold
  - float in the range (0, 1)
  - if a match likelihood is bleow this threshold in data-association, the detection will be assigned a new object instance
  - default: 0.25
- weighting_strategy
  - how voxels' instance weights are incremented
    - constant: weights are always incremented by 1
    - distance: weights are incremented based on the voxel's distance from nearest surface
    - tsdf: weights are incremented based on the voxel's tsdf weight
  - default: tsdf

### Outlier rejection parameters
- perform_outlier_rejection
  - boolean (true/false)
  - default: true
- outlier_rejection_method
  - gaussian:
    - faster, less precise 
    - voxel set for a detection is represented by a multivariate Gaussian distribution, and points further than a given threshold from the distribution are discarded
    - distance between point and distribution is determined by squared Mahalanobis distance
  - dbscan
    - slower, but more accurate
    - voxel set is clustered wit the DBSCAN algortihm and the largest cluster is chosen while others are discarded 
  - default: gaussian
- outlier_threshold
  - when the gaussian method is used, this determines the distance threshold 
  - for example, the default value of 6.2514 corresponds to 90% confidence interval
  - default: 6.2514
- cluster_min_points
  - minimum number of points accepted as a cluster (in both methods)
  - default: 10
- dbscan_epsilon
  - epsilon parameter of the DBSCAN algortihm, corresponds to the maximum allowed distance between neighboring points in a cluster
  - e.g. default value of 1.5 is greater than sqrt(2), therefore if a voxel's size is 1x1x1, other voxels connected to it from it's sides, but not only from corners, will be counted to the same cluster
  - larger values will result in less dense clusters
  - default: 1.5

### Data processing parameters -->
- infer_distribution
  - boolean (true/false)
  - if true, a multivariate gaussian is inferred for each detection
  - will automatically be set true if required for association likelihood or outlier rejection
  - default: false
- sample_measurements
  - boolean (true/false)
  - reduce number of points for each detection to speed up processing
  - each detection's points are sampled from a multivariate gaussian distribution inferred from all detected points for said object in the current frame
  - could reduce output quality
  - default="false"/>
- max_samples
  - maximum number of points sampled for each detection (if sample_measurements is true)
  - if threre are less detectied points for an object than this, they will all be used
  - default="1000"/>

### Output parameters -->
- output_name
  - path to desired output directory
  - default: (arg data_root)/3d_panoptic
- visualise_global_ids
  - boolean (true/false)
  - if true, each 'thing' object instance will be given a randomised color
  - if false, each object of the same class are given the same color
  - default: true

### Live Visualisation
- rviz_cfg: path to rviz config file, e.g. [../launch/fusion.rviz](https://github.com/TUTvision/Online-Panoptic-3D/blob/main/launch/fusion.rviz)
