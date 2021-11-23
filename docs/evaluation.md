### Evaluation of ScanNet results

1. Generate panoptic ground truth
```
cd ~/catkin_ws/src/Online-Panoptic-3D/scripts

python3 export_panoptic_gt.py \
    --source_dir path/to/scannet/data/root \
    --output_name gt \
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
    -o transformed_labels_name.txt
```
3. Evaulate panoptic quality accross a directory of scans
 ```
python3 evaluate_pq_from_dir_multithread.py \
    -i path/to/scannet/data/root \
    -p transofrmed_labels_name.txt \
    -gt gt_panoptic.txt \
    -o path/to/results/directory/scannet_pq.txt
```

You can also reformat results for ScanNet evaluation with [scripts/mesh_labels_to_scannet_format_from_dir.py](https://github.com/TUTvision/Online-Panoptic-3D/blob/main/scripts/mesh_labels_to_scannet_format_from_dir.py)
