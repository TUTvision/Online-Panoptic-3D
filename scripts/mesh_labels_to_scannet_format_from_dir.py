
import argparse
from pathlib import Path
import multiprocessing

import mesh_labels_to_scannet_format
from thread_launcher import Thread_launcher

def callback(self,
    scenes,
    panoptic_results_name,
    label_color_file_name,
    output_dir):

    for d in scenes:
        print(f"Processing {d.name} ...")

        panoptic_results = d / Path(panoptic_results_name)
        label_color_file = d / Path(label_color_file_name)
        scannet_file = d / Path(d.name + "_vh_clean_2.ply")

        mesh_labels_to_scannet_format.main(
            Path(d.name), output_dir, panoptic_results, label_color_file, scannet_file)

        print(f"{d.name} processed!")

def main(source_dir, panoptic_results_name, label_color_file_name, output_dir):

    scenes = list(source_dir.glob('scene*'))
    n_scenes = len(scenes)
    scenes.sort( )

    n_cpu = multiprocessing.cpu_count()

    multiThreading = Thread_launcher(
        callback, n_cpu,
        panoptic_results_name = panoptic_results_name,
        label_color_file_name = label_color_file_name,
        output_dir = output_dir)

    multiThreading.launch(scenes)
    multiThreading.join()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument("-i", "--source_dir", type=Path, dest="source_dir",
                        help="Path to ScanNet scans root")
    parser.add_argument("-p", "--panoptic_results", type=Path, dest="panoptic_results_name",
                        help="File name of panoptic voxel labels (.ply)")
    parser.add_argument("-l", "--label_color_file", type=Path, dest="label_color_file_name",
                        help="File name of color mapping of instance id's (*_color_map.txt)")
    parser.add_argument("-o", "--output_dir", type=Path, dest="output_dir",
                        help="Desired output directory")

    args = parser.parse_args()

    main(**vars(args))
