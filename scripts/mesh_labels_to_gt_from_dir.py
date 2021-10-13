
import re
import sys
import math
import threading
import random
import argparse
from pathlib import Path
from collections import OrderedDict

import numpy as np
from scipy.stats import mode
import faiss
from plyfile import PlyData

import mesh_labels_to_gt

class Thread(threading.Thread):
   def __init__(self, callback, args):
      threading.Thread.__init__(self)

      self.callback = callback
      self.args = args

   def run(self):
       self.callback(self.args)

class Thread_launcher():
    def __init__(self,
        panoptic_results_name,
        label_color_file_name,
        scannet_name,
        output_name,
        n_threads = 8):

        self.n_threads = n_threads

        self.panoptic_results_name = panoptic_results_name
        self.label_color_file_name = label_color_file_name
        self.scannet_name = scannet_name
        self.output_name = output_name

        self.threads = []

    def callback(self, scenes):
        for d in scenes:
            print(f"Processing {d.name} ...")

            output_file = d / Path(self.output_name)
            panoptic_results = d / Path(self.panoptic_results_name)
            label_color_file = d / Path(self.label_color_file_name)
            scannet_file = d / Path(self.scannet_name)

            mesh_labels_to_gt.main(
                output_file, panoptic_results, label_color_file, scannet_file)

            print(f"{d.name} processed!")

    def launch(self, scenes):
        n_scenes = len(scenes)
        partition_size = int(math.ceil(n_scenes/self.n_threads))

        last_thread = False

        idx = 0
        for i in range(self.n_threads):
            if idx + partition_size >= n_scenes:
                partition = scenes[idx:]

                last_thread = True
            else:
                partition = scenes[idx : idx + partition_size]

            idx += partition_size

            t = Thread(self.callback, partition)
            t.start()
            self.threads.append(t)

            if last_thread: break

    def join(self):
        for t in self.threads:
            t.join()

        self.threads.clear()

def main(source_dir, panoptic_results_name, label_color_file_name, scannet_name, output_name):

    scenes = list(source_dir.glob('scene*'))
    n_scenes = len(scenes)
    scenes.sort( )

    '''
    for i, d in enumerate(scenes):
        print(f"{i+1}/{n_scenes} {d.name}", end="                           \r")

        output_file = d / Path(output_name)
        panoptic_results = d / Path(panoptic_results_name)
        label_color_file = d / Path(label_color_file_name)
        scannet_file = d / Path(scannet_name)

        mesh_labels_to_gt.main(
            output_file, panoptic_results, label_color_file, scannet_file)
    '''

    multiThreading = Thread_launcher(
        panoptic_results_name, label_color_file_name, scannet_name, output_name)

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
    parser.add_argument("-s", "--scannet_file", type=Path, dest="scannet_name",
                        help="File name of scannet panoptic ground truth  (.txt)")
    parser.add_argument("-o", "--output_name", type=Path, dest="output_name",
                        help="name of output file (.txt)")


    args = parser.parse_args()

    main(**vars(args))
