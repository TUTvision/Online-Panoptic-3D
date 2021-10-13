#!/usr/bin/env python3

import sys
import argparse
from pathlib import Path

import numpy as np
from plyfile import PlyData

STUFF_LABELS = [0, 1, 2] # 0 == void?
THING_LABELS = [3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 16, 24, 28, 33, 34, 36, 39]

def main(output_file,
         label_file,
         instance_file,
         mesh_file):

    if not label_file.exists():
        raise Exception(f"Label file {str(label_file)} not found.")
    if not instance_file.exists():
        raise Exception(f"Label file {str(instance_file)} not found.")
    if not mesh_file.exists():
        raise Exception(f"Mesh file {str(mesh_file)} not found.")

    if not output_file.parent.is_dir():
        output_file.parent.mkdir(parents=True)

    mesh = PlyData.read(str(mesh_file))
    vertices = mesh['vertex'].data

    labels    = np.empty(vertices.size, dtype=np.ushort)
    instances = np.empty(vertices.size, dtype=np.ushort)

    with open(str(label_file), 'r') as f:
        for i, line in enumerate(f):
            try:
                label = int(line)

            except Exception as e:
                print(f"Exception when reading label {line}:")
                print(e)

            # stuff instance id's are set same as labels
            if label in STUFF_LABELS:
                instances[i] = label

            labels[i] = label

    instance_id = 0

    with open(str(instance_file), 'r') as f:
        for i, line in enumerate(f):
            splits = line.split(' ')

            mask_path = Path(splits[0]) # relative to instance_file
            mask_path = instance_file.parent / mask_path

            # ensure instance id isn't a stuff label
            while instance_id in STUFF_LABELS:
                instance_id += 1

            with open(str(mask_path), 'r') as mask:
                for j, val in enumerate(mask):
                    if float(val) > 0 and labels[j] in THING_LABELS:
                        instances[j] = instance_id

            instance_id += 1

    with open(str(output_file), 'w') as f:
        for i, v in enumerate(vertices):

            line = str(v[0])      + " " + \
                   str(v[1])      + " " + \
                   str(v[2])      + " " + \
                   str(instances[i]) + " " + \
                   str(labels[i])    + "\n"

            f.write(line)

    print(f"Wrote panoptic labels to {str(output_file)} succesfully.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument("-o", "--output_file", type=Path, dest="output_file",
                        help="Path to desired output file (.txt)")

    parser.add_argument("-l", "--label_file", type=Path, dest="label_file",
                        help="Path to scannet label ground truth (.txt)")

    parser.add_argument("-i", "--instance_file", type=Path, dest="instance_file",
                        help="Path to scannet instance ground truth  (.txt)")

    parser.add_argument("-m", "--mesh_file", type=Path, dest="mesh_file",
                        help="Path to scannet mesh (*_vh_clean_2.ply)")

    args = parser.parse_args()

    main(**vars(args))
