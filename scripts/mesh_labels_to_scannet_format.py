
import sys
import random
import argparse
from pathlib import Path
from collections import OrderedDict

import numpy as np
from scipy.stats import mode
import faiss
from plyfile import PlyData

STUFF_LABELS = [0, 1, 2] # 0 == void?
THING_LABELS = [3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 16, 24, 28, 33, 34, 36, 39]

ALL_LABELS = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 16, 24, 28, 33, 34, 36, 39]
EVAL_LABELS = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 16, 24, 28, 33, 34, 36, 39]

EVAL_CLASSES = [
    "wall", "floor", "cabinet", "bed", "chair", "sofa", "table", "door",
    "window", "bookshelf", "picture", "counter", "desk", "curtain",
    "refrigerator", "shower curtain", "toilet", "sink", "bathtub", "other furniture"]

CLASS_COLORS = OrderedDict()
CLASS_COLORS[0]  = (  0,   0,   0) # void
CLASS_COLORS[1]  = (174, 199, 232) # wall
CLASS_COLORS[2]  = (152, 223, 138) # floor
CLASS_COLORS[3]  = ( 31, 119, 180) # cabinet
CLASS_COLORS[4]  = (255, 187, 120) # bed
CLASS_COLORS[5]  = (188, 189,  34) # chair
CLASS_COLORS[6]  = (140,  86,  75) # sofa
CLASS_COLORS[7]  = (255, 152, 150) # table
CLASS_COLORS[8]  = (214,  39,  40) # door
CLASS_COLORS[9]  = (197, 176, 213) # window
CLASS_COLORS[10] = (148, 103, 189) # bookshelf
CLASS_COLORS[11] = (196, 156, 148) # picture
CLASS_COLORS[12] = ( 23, 190, 207) # counter
CLASS_COLORS[14] = (247, 182, 210) # desk
CLASS_COLORS[16] = (219, 219, 141) # curtain
CLASS_COLORS[24] = (255, 127,  14) # refrigerator
CLASS_COLORS[28] = (158, 218, 229) # shower curtain
CLASS_COLORS[33] = ( 44, 160,  44) # toilet
CLASS_COLORS[34] = (112, 128, 144) # sink
CLASS_COLORS[36] = (227, 119, 194) # bathtub
CLASS_COLORS[39] = ( 82,  84, 163) # other furniture


def randRGB(seed=None):
    if seed is not None:
        random.seed(seed)
    r = random.random()*255
    g = random.random()*255
    b = random.random()*255
    rgb = [r, g, b]
    return rgb

def label_from_color(vert_color, seg_colors, label_map):
    color_arr = np.asarray(seg_colors)
    vc_arr = np.asarray(vert_color)

    diff  = np.sum(np.abs(color_arr - vc_arr), axis=1)
    idx   = np.argmin(diff)
    label = label_map[idx]

    return label, idx

def parse_color_map_file(path):
    labels     = []
    instances  = []
    colors     = []

    colors.append((0,0,0))
    labels.append(255)
    instances.append(255)

    with open(str(path), 'r') as f:
        for i, line in enumerate(f):
            splits = line.split(' ')

            try:
                c = ( float(splits[0]), float(splits[1]), float(splits[2]) )
                i = int(splits[3])
                l = int(splits[4])

                colors.append(c)
                labels.append(l)
                instances.append(i)

            except:
                print(f"Data invalid in row {i} of {str(path)}")

    return colors, labels, instances

def parse_panoptic_results(path):

    vertices  = []
    labels    = []
    instances = []

    with open(str(path), 'r') as f:
        for i, line in enumerate(f):
            splits = line.split(' ')

            try:
                v = ( float(splits[0]), float(splits[1]), float(splits[2]) )
                i = int(splits[3])
                l = int(splits[4])

                vertices.append(v)
                labels.append(l)
                instances.append(i)

            except:
                print(f"Data invalid in row {i} of {str(path)}")

    assert len(vertices) == len(labels)
    assert len(vertices) == len(instances)

    vertices  = np.asarray(vertices,  dtype=np.float32)
    labels    = np.asarray(labels,    dtype=np.ushort)
    instances = np.asarray(instances, dtype=np.ushort)

    return vertices, labels, instances

def get_mesh_vertices(mesh_file):

    mesh = PlyData.read(str(mesh_file))
    vertices = []

    for i, v in enumerate(mesh['vertex']):
        vertices.append( (v['x'], v['y'], v['z']) )

    vertices    = np.asarray(vertices, dtype=np.float32)

    return vertices

def parse_panoptic_mesh(mesh_file, label_color_file):

    mesh = PlyData.read(str(mesh_file))

    colors, c_labels, c_instances = parse_color_map_file(label_color_file)

    vertices    = []
    v_labels    = []
    v_instances = []

    for i, v in enumerate(mesh['vertex']):
        color = ( v['red'], v['green'], v['blue'] )
        #print(color)
        instance, idx = label_from_color(color, colors, c_instances)
        label = c_labels[idx]

        vertices.append( (v['x'], v['y'], v['z']) )
        v_labels.append(label)
        v_instances.append(instance)

    vertices    = np.asarray(vertices, dtype=np.float32)
    v_labels    = np.asarray(v_labels,    dtype=np.ushort)
    v_instances = np.asarray(v_instances, dtype=np.ushort)

    return vertices, v_labels, v_instances

def get_segments(vertices, labels, instances):

    segments = {}

    for v, l, i in zip(vertices, labels, instances):

        if i not in segments.keys():
            segments[i] = {'label':l, 'vertices':[]}

        segments[i]['vertices'].append(v)

    for inst, seg in segments.items():
        seg['vertices'] = np.asarray(seg['vertices'], np.float32)

    return segments

def write_semantic_results(output_dir, output_name, labels):

    if not output_dir.is_dir():
        output_dir.mkdir(parents=True)

    file = output_dir / Path(output_name + '.txt')

    with open(str(file), 'w') as f:
        for l in labels:

            line = str(int(l)) + "\n"

            f.write(line)

def write_instance_results(output_dir, output_name, segments, vertices):

    if not output_dir.is_dir():
        output_dir.mkdir(parents=True)

    mask_dir = output_dir / Path("predicted_masks")

    if not mask_dir.is_dir():
        mask_dir.mkdir()

    scene_file = output_dir / Path(output_name + '.txt')

    with open(str(scene_file), 'w') as f:
        for instance, seg in segments.items():

            if seg["label"] not in THING_LABELS:
                continue

            mask_name = Path(f"{output_name}_{instance:03}.txt")

            mask_file = mask_dir / mask_name
            relative_mask_file = mask_file.relative_to(mask_dir.parent)

            with open(str(mask_file), 'w') as m:
                for v in vertices:
                    line = "0\n"

                    diff = seg['vertices'] - v
                    diff = np.abs(diff).sum(axis=1)

                    # vertices should be equal, but just in case of rounding errors
                    # check inside a small radius
                    if np.any(diff < 0.001):
                        line = "1\n"

                    m.write(line)

            line = f"{str(relative_mask_file)} {seg['label']} 1.0\n"
            f.write(line)

def main(output_name, output_dir, panoptic_results, label_color_file, scannet_file):

    if not panoptic_results.exists():
        raise Exception(f"Label file {str(panoptic_results)} not found.")
    if not label_color_file.exists():
        raise Exception(f"Label color mapping {str(label_color_file)} not found.")
    if not scannet_file.exists():
        raise Exception(f"Label file {str(scannet_file)} not found.")

    if not output_dir.is_dir():
        output_dir.mkdir(parents=True)

    output_name = str(output_name.stem)

    gt_vertices = get_mesh_vertices(scannet_file)
    mesh_vertices, mesh_labels, mesh_instances = parse_panoptic_mesh(panoptic_results, label_color_file)

    # search index, Cartesian 3D space, distances are 3D euclidean
    index = faiss.IndexFlatL2(3)

    # add voxels to search index
    index.add(mesh_vertices)

    # get k nearest neighbors for each point in mesh
    k = 10

    D, I = index.search(gt_vertices, k)

    nn_labels = []
    nn_instances = []

    for i, idx in enumerate(I):
        li = mesh_labels[idx][0]
        l = EVAL_LABELS[li] if li < len(EVAL_LABELS) else 0

        nn_labels.append(l)
        nn_instances.append(mesh_instances[idx][0])

    sem_output = output_dir / Path("3d_semantic")
    write_semantic_results(sem_output, output_name, nn_labels)

    inst_output = output_dir / Path("3d_instance")
    segments = get_segments(gt_vertices, nn_labels, nn_instances)
    write_instance_results(inst_output, output_name, segments, gt_vertices)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument("-p", "--panoptic_results", type=Path, dest="panoptic_results",
                        help="Path to panoptic voxel labels (.ply)")

    parser.add_argument("-l", "--label_color_file", type=Path, dest="label_color_file",
                        help="Path to color mapping of instance id's (*_color_map.txt)")

    parser.add_argument("-s", "--scannet_file", type=Path, dest="scannet_file",
                        help="Path to scannet mesh (.ply)")

    parser.add_argument("-n", "--output_name", type=Path, dest="output_name",
                        help="Output file name (no .txt)")

    parser.add_argument("-o", "--output_dir", type=Path, dest="output_dir",
                        help="Output directory")

    args = parser.parse_args()

    main(**vars(args))
