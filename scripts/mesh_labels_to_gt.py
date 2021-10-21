
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
        instance, idx = label_from_color(color, colors, c_instances)
        label = c_labels[idx]

        vertices.append( (v['x'], v['y'], v['z']) )
        v_labels.append(label)
        v_instances.append(instance)

    vertices    = np.asarray(vertices, dtype=np.float32)
    v_labels    = np.asarray(v_labels,    dtype=np.ushort)
    v_instances = np.asarray(v_instances, dtype=np.ushort)

    return vertices, v_labels, v_instances

def write_panoptic_results(file, vertices, instances, labels):

    with open(str(file), 'w') as f:
        for i, v in enumerate(vertices):

            line = str(v[0])      + " " + \
                   str(v[1])      + " " + \
                   str(v[2])      + " " + \
                   str(instances[i]) + " " + \
                   str(labels[i])    + "\n"

            f.write(line)

def generate_panoptic_mesh(mesh_file,
                           mesh_output,
                           labels,
                           instances,
                           label_color_file=None,
                           semantic=False):

    label_colors_from_file = False

    if label_color_file is not None:
        colors, c_labels, c_instances = parse_color_map_file(label_color_file)
    else:
        c_instances = list(np.unique(instances))
        colors = []

        for instance in c_instances:
            colors.append(randRGB(instance))

    mesh = PlyData.read(str(mesh_file))

    for i, inst in enumerate(instances):
        if semantic or labels[instances.index(inst)] in STUFF_LABELS:
            color = CLASS_COLORS[labels[i]]
        else:
            color = colors[c_instances.index(inst)]

        mesh['vertex'][i]['red']   = int(color[0])
        mesh['vertex'][i]['green'] = int(color[1])
        mesh['vertex'][i]['blue']  = int(color[2])

    PlyData(mesh).write(str(mesh_output))

def main(output_file, panoptic_results, label_color_file, scannet_file, mesh_file=None, mesh_output=None):

    if not panoptic_results.exists():
        raise Exception(f"Label file {str(panoptic_results)} not found.")

    if mesh_file is not None:
        if not mesh_file.exists():
            raise Exception(f"Mesh file {str(mesh_file)} not found.")
        if mesh_output is None:
            raise Exception(f"Mesh output not specified.")
        elif not mesh_output.parent.is_dir():
            mesh_output.parent.mkdir(parents=True)

    if not output_file.parent.is_dir():
        output_file.parent.mkdir(parents=True)

    if scannet_file is not None:
        gt_vertices, gt_labels, gt_instances = parse_panoptic_results(scannet_file)
    else:
        gt_vertices = get_mesh_vertices(mesh_file)

    if panoptic_results.suffix == ".ply":
        if label_color_file is None or not label_color_file.exists():
            raise Exception(f"Label color mapping {str(label_color_file)} not found.")

        mesh_vertices, mesh_labels, mesh_instances = parse_panoptic_mesh(panoptic_results, label_color_file)
    else:
        mesh_vertices, mesh_labels, mesh_instances = parse_panoptic_results(panoptic_results)

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

    write_panoptic_results(output_file, gt_vertices, nn_instances, nn_labels)

    if mesh_file is not None:
        generate_panoptic_mesh(
            mesh_file, mesh_output, nn_labels, nn_instances, label_color_file)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument("-p", "--panoptic_results", type=Path, dest="panoptic_results",
                        help="Path to panoptic voxel labels (.ply)")

    parser.add_argument("-l", "--label_color_file", type=Path, dest="label_color_file", default=None,
                        help="Path to color mapping of instance id's (*_color_map.txt)")

    parser.add_argument("-s", "--scannet_file", type=Path, dest="scannet_file", default=None,
                        help="Path to scannet panoptic ground truth  (.txt)")

    parser.add_argument("-o", "--output_file", type=Path, dest="output_file",
                        help="Path to desired output file (.txt)")

    parser.add_argument("-m", "--mesh_file", type=Path, dest="mesh_file", default=None,
                        help="Path to scannet mesh (*_vh_ceal_2.ply), used for visualisation only")

    parser.add_argument("-mo", "--mesh_output", type=Path, dest="mesh_output", default=None,
                        help="Path to output mesh (*.ply), used for visualisation only")

    args = parser.parse_args()

    main(**vars(args))
