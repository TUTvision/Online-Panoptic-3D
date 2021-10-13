
import sys
import random
import argparse
from pathlib import Path

import numpy as np
from scipy.stats import mode
import faiss
from plyfile import PlyData

STUFF_LABELS = [0, 1, 2] # 0 == void?
THING_LABELS = [3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 16, 24, 28, 33, 34, 36, 39]

ALL_LABELS = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 16, 24, 28, 33, 34, 36, 39]

CLASS_COLORS = {
    0 : (  0,   0,   0), # void
    1 : (174, 199, 232), # wall
    2 : (152, 223, 138), # floor
    3 : (31, 119, 180),  # cabinet
    4 : (255, 187, 120), # bed
    5 : (188, 189, 34),  # chair
    6 : (140, 86, 75),   # sofa
    7 : (255, 152, 150), # table
    8 : (214, 39, 40),   # door
    9 : (197, 176, 213), # window
    10 : (148, 103, 189), # bookshelf
    11 : (196, 156, 148), # picture
    12 : (23, 190, 207),  # counter
    14 : (247, 182, 210), # desk
    16 : (219, 219, 141), # curtain
    24 : (255, 127, 14),  # refrigerator
    28 : (158, 218, 229), #  shower curtain
    33 : (44, 160, 44),   # toilet
    34 : (112, 128, 144), #  sink
    36 : (227, 119, 194), #  bathtub
    39 : (82, 84, 163)    #  otherfurn
}

def randRGB(seed=None):
    if seed is not None:
        random.seed(seed)
    r = random.random()*255
    g = random.random()*255
    b = random.random()*255
    rgb = [r, g, b]
    return rgb

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
                           semantic=False):

    mesh = PlyData.read(str(mesh_file))
    #vertices = mesh['vertex']

    for i, inst in enumerate(instances):
        if semantic or inst in STUFF_LABELS:

            color = CLASS_COLORS[labels[i]]

        else:
            color = randRGB(inst)

        mesh['vertex'][i][3] = int(color[0])
        mesh['vertex'][i][4] = int(color[1])
        mesh['vertex'][i][5] = int(color[2])

    PlyData(mesh).write(str(mesh_output))

def main(output_file, panoptic_results, scannet_file, mesh_file=None, mesh_output=None):

    # TODO: for some reason, instancse are not correct at all
    # - possbile other approach: use voxblox mesh instead of voxel grid

    if not panoptic_results.exists():
        raise Exception(f"Label file {str(panoptic_results)} not found.")
    if not scannet_file.exists():
        raise Exception(f"Label file {str(scannet_file)} not found.")
    if not scannet_file.exists():
        raise Exception(f"Label file {str(scannet_file)} not found.")

    if mesh_file is not None:
        if not mesh_file.exists():
            raise Exception(f"Mesh file {str(mesh_file)} not found.")
        if mesh_output is None:
            raise Exception(f"Mesh output not specified.")
        elif not mesh_output.parent.is_dir():
            mesh_output.parent.mkdir(parents=True)

    if not output_file.parent.is_dir():
        output_file.parent.mkdir(parents=True)

    gt_vertices, gt_labels, gt_instances = parse_panoptic_results(scannet_file)
    vx_vertices, vx_labels, vx_instances = parse_panoptic_results(panoptic_results)

    # search index, Cartesian 3D space, distances are 3D euclidean
    index = faiss.IndexFlatL2(3)

    # add voxels to search index
    index.add(vx_vertices)

    # get k nearest neighbors for each point in mesh
    k = 10

    D, I = index.search(gt_vertices, k)

    nn_labels = []
    nn_instances = []

    for i, idx in enumerate(I):
        # weight by distance?
        # or simply choose the nearest neighbor?

        #minfo = mode(vx_labels[idx])
        #li = minfo[0][0]
        li = vx_labels[idx][0]
        l = ALL_LABELS[li] if li < len(ALL_LABELS) - 1 else 0

        nn_labels.append(l)
        nn_instances.append(vx_instances[idx][0])

    write_panoptic_results(output_file, gt_vertices, nn_instances, nn_labels)

    if mesh_file is not None:
        generate_panoptic_mesh(
            mesh_file, mesh_output, nn_labels, nn_instances, True)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument("-p", "--panoptic_results", type=Path, dest="panoptic_results",
                        help="Path to panoptic voxel labels (.txt)")

    parser.add_argument("-s", "--scannet_file", type=Path, dest="scannet_file",
                        help="Path to scannet panoptic ground truth  (.txt)")

    parser.add_argument("-o", "--output_file", type=Path, dest="output_file",
                        help="Path to desired output file (.txt)")

    parser.add_argument("-m", "--mesh_file", type=Path, dest="mesh_file", default=None,
                        help="Path to scannet mesh (*_vh_ceal_2.ply), used for visualisation only")

    parser.add_argument("-mo", "--mesh_output", type=Path, dest="mesh_output", default=None,
                        help="Path to output mesh (*.ply), used for visualisation only")

    args = parser.parse_args()

    main(**vars(args))
