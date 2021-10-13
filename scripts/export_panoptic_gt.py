
import os
from pathlib import Path
from argparse import ArgumentParser

import numpy as np
from plyfile import PlyData

STUFF_LABELS = [0, 1, 2] # 0 == void
THING_LABELS = [3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 16, 24, 28, 33, 34, 36, 39]

def export_labels_and_instances(scan_root, output_name, label_map_path, scannet_root):

    scan_path = str(scan_root.resolve())

    label_output = scan_root / Path(output_name + "_labels.txt")
    label_output = str(label_output.resolve())
    instance_output = scan_root / Path(output_name + "_instances.txt")
    instance_output = str(instance_output.resolve())

    label_map = str(label_map_path.resolve())

    main_dir = os.getcwd()
    sc_script_dir = scannet_root / Path("BenchmarkScripts/3d_helpers")
    export_script = "export_train_mesh_for_evaluation.py"

    # call the ScanNet scripts from their original directory to avoid having to
    # copy them here ...

    labels_cmd = f"{export_script} \
        --scan_path {scan_path} \
        --output_file {label_output} \
        --label_map_file {label_map} \
        --type label >/dev/null 2>&1"

    intances_cmd = f"{export_script} \
        --scan_path {scan_path} \
        --output_file {instance_output} \
        --label_map_file {label_map} \
        --type instance >/dev/null 2>&1"

    os.chdir(sc_script_dir)
    os.system(f'python {labels_cmd}')
    os.system(f'python {intances_cmd}')
    os.chdir(main_dir)

def export_panoptic_labels(scan_path, output_name):

    mesh_file = next(scan_path.glob('*_vh_clean_2.ply'))

    if not mesh_file.exists():
        raise Exception(f"ScanNet mesh {str(mesh_file)} not found.")

    label_file = next(scan_path.glob('*_labels.txt'))

    if not label_file.exists():
        raise Exception(f"Label file {str(label_file)} not found.")

    instance_file = next(scan_path.glob('*_instances.txt'))

    if not instance_file.exists():
        raise Exception(f"Label file {str(instance_file)} not found.")

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

    output_file = scan_path / Path(output_name + "_panoptic.txt")

    with open(str(output_file), 'w') as f:
        for i, v in enumerate(vertices):

            line = str(v[0])      + " " + \
                   str(v[1])      + " " + \
                   str(v[2])      + " " + \
                   str(instances[i]) + " " + \
                   str(labels[i])    + "\n"

            f.write(line)

def main(source_dir, output_name, label_map_path, scannet_root):

    scenes = list(source_dir.glob('scene*'))
    n_scenes = len(scenes)

    for i, d in enumerate(scenes):
        print(f"{i+1}/{n_scenes} {d.name}", end="                           \r")

        export_labels_and_instances(d, output_name, label_map_path, scannet_root)
        export_panoptic_labels(d, output_name)

if __name__ == '__main__':

    scannet_default = Path("ScanNet")

    parser = ArgumentParser()

    parser.add_argument("-s", "--source_dir", type=Path, dest="source_dir",
                        help="Path to ScanNet scans root")

    parser.add_argument("-o", "--output_name", type=str, dest="output_name",
                        help="Desired output file name (without suffix)")

    parser.add_argument("-l", "--label_map_path", type=Path, dest="label_map_path",
                        help="Path to ScanNet label map file (.tsv)")

    parser.add_argument("-sc", "--scannet_root", type=Path, dest="scannet_root",
                        default=scannet_default, help="Path to ScanNet root folder")

    args = parser.parse_args()

    main(**vars(args))
