
import argparse
from pathlib import Path
import multiprocessing

import mesh_labels_to_gt
from thread_launcher import Thread_launcher

def callback(
    scenes,
    output_name,
    panoptic_results_name,
    label_color_file_name,
    scannet_name):

    for d in scenes:
        print(f"Processing {d.name} ...")

        output_file = d / Path(output_name)
        panoptic_results = d / Path(panoptic_results_name)
        label_color_file = d / Path(label_color_file_name)
        scannet_file = d / Path(scannet_name)

        mesh_labels_to_gt.main(
            output_file, panoptic_results, label_color_file, scannet_file)

        print(f"{d.name} processed!")

def main(source_dir, panoptic_results_name, label_color_file_name, scannet_name, output_name):

    scenes = list(source_dir.glob('scene*'))
    n_scenes = len(scenes)
    scenes.sort( )

    n_cpu = multiprocessing.cpu_count()

    multiThreading = Thread_launcher(
        callback,
        n_cpu,
        output_name=output_name,
        panoptic_results_name=panoptic_results_name,
        label_color_file_name=label_color_file_name,
        scannet_name=scannet_name)

    multiThreading.launch(scenes)
    multiThreading.join()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument("-i", "--source_dir", type=Path, dest="source_dir",
                        help="Path to ScanNet scans root")
    parser.add_argument("-p", "--panoptic_results", type=Path, dest="panoptic_results_name",
                        help="File name of panoptic mesh (.ply)")
    parser.add_argument("-l", "--label_color_file", type=Path, dest="label_color_file_name",
                        help="File name of color mapping of instance id's (*_color_map.txt)")
    parser.add_argument("-s", "--scannet_file", type=Path, dest="scannet_name",
                        help="File name of scannet panoptic ground truth  (.txt)")
    parser.add_argument("-o", "--output_name", type=Path, dest="output_name",
                        help="name of output file (.txt)")

    args = parser.parse_args()

    main(**vars(args))
