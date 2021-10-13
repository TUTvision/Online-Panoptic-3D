
import re
import sys
import random
import argparse
from pathlib import Path
from collections import OrderedDict

import numpy as np
from scipy.stats import mode
import faiss
from plyfile import PlyData

STUFF_LABELS = [1, 2]
THING_LABELS = [3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 16, 24, 28, 33, 34, 36, 39]
EVAL_LABELS = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 16, 24, 28, 33, 34, 36, 39]
EVAL_CLASSES = [
    "wall", "floor", "cabinet", "bed", "chair", "sofa", "table", "door",
    "window", "bookshelf", "picture", "counter", "desk", "curtain",
    "refrigerator", "shower curtain", "toilet", "sink", "bathtub", "other furniture"]

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

                #if l in EVAL_LABELS and i > 0:
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

def get_segments(vertices, labels, instances):

    segments = {}

    for v, l, i in zip(vertices, labels, instances):

        if i not in segments.keys():
            segments[i] = {'label':l, 'vertices':[]}

        segments[i]['vertices'].append(v)

    for inst, seg in segments.items():
        seg['vertices'] = np.asarray(seg['vertices'], np.float32)

    return segments

def intersection_over_union(V1, V2):

    intersection = 0

    for v2 in V2:
        intersection += (V1 == v2).all(axis=1).any()

    union = V1.shape[0] + V2.shape[0] - intersection

    iou = intersection / union if union > 0 else 0

    return iou

def write_output(string, stream):
    stream.write(string + '\n')
    print(string)

def add_one_to_count(label, counts):
    if label not in counts.keys():
        counts[label] = 1
    else:
        counts[label] += 1

def main(source_dir, prediction_name, ground_truth_name, output_file, threshold=0.5):

    scenes = list(source_dir.glob('scene*'))
    n_scenes = len(scenes)
    scenes.sort( )

    class_tp_counts = {}
    class_fp_counts = {}
    class_fn_counts = {}

    class_tp_iou_sum = {}

    class_gt_counts = {}
    class_det_counts = {}

    for i, d in enumerate(scenes):
        print(f"{i+1}/{n_scenes} {d.name}\n")

        prediction = d / prediction_name
        ground_truth = d / ground_truth_name

        pred_v, pred_l, pred_i = parse_panoptic_results(prediction)
        gt_v, gt_l, gt_i       = parse_panoptic_results(ground_truth)

        pred_segs = get_segments(pred_v, pred_l, pred_i)
        gt_segs   = get_segments(gt_v, gt_l, gt_i )

        gt_overlaps = {}

        print(f"No. pred instances: {len(pred_segs)}")
        print(f"No. gt instances: {len(gt_segs)}")

        print(f"No. pred verts: {pred_v.shape[0]}")
        print(f"No. gt verts: {gt_v.shape[0]}\n")

        for idx, gt_inst in enumerate(gt_i):

            pred_inst = pred_i[idx]

            if gt_inst not in gt_overlaps.keys():
                gt_overlaps[gt_inst] = {pred_inst}
            else:
                gt_overlaps[gt_inst].add(pred_inst)

        iou_map = {}
        matches = {}
        matched_pred_inst = []

        for gt_inst, pred_set in gt_overlaps.items():

            if gt_inst not in iou_map.keys():
                iou_map[gt_inst] = {}

            for pred_inst in pred_set:
                iou = intersection_over_union(
                    gt_segs[gt_inst]['vertices'],
                    pred_segs[pred_inst]['vertices'])

                if iou > 0.5:
                    matches[gt_inst] = pred_inst
                    matched_pred_inst.append(pred_inst)
                    iou_map[gt_inst][pred_inst] = iou

        for idx, gt_inst in enumerate(gt_segs.keys()):

            gt_label = gt_segs[gt_inst]['label']

            add_one_to_count(gt_label, class_gt_counts)

            if gt_inst not in matches.keys(): # false negative
                add_one_to_count(gt_label, class_fn_counts)

                continue

            pred_inst = matches[gt_inst]
            pred_label = pred_segs[pred_inst]['label']

            if gt_label == pred_label: # true positive
                add_one_to_count(gt_label, class_tp_counts)

                if gt_label not in class_tp_iou_sum.keys():
                    class_tp_iou_sum[gt_label] = iou_map[gt_inst][pred_inst]
                else:
                    class_tp_iou_sum[gt_label] += iou_map[gt_inst][pred_inst]

        for idx, pred_inst in enumerate(pred_segs.keys()):

            pred_label = pred_segs[pred_inst]['label']
            add_one_to_count(pred_label, class_det_counts)

            if pred_inst not in matched_pred_inst: # false positive
                add_one_to_count(pred_label, class_fp_counts)

    RQ = {}
    SQ = {}
    PQ = {}

    for i, l in enumerate(EVAL_LABELS):

        if l not in class_tp_counts.keys():
            RQ[l] = 0
            SQ[l] = 0
            PQ[l] = 0

            continue

        n_tp = class_tp_counts[l] if l in class_tp_counts else 0
        n_fp = class_fp_counts[l] if l in class_fp_counts else 0
        n_fn = class_fn_counts[l] if l in class_fn_counts else 0

        if l not in class_tp_counts.keys():
            SQ[l] = 0
        else:
            #mean_iou = class_tp_iou_sum[l] / class_tp_iou_n[l]

            SQ[l] = class_tp_iou_sum[l] / n_tp

        RQ[l] = n_tp / ( n_tp + 0.5*n_fp + 0.5*n_fn)

        PQ[l] = SQ[l]*RQ[l]

    output_fs = open(output_file, 'w', encoding="utf-8")
    write_output("", output_fs)
    write_output(f"{16*' '}    SQ   RQ   PQ  G D", output_fs)

    st_sq_sum = 0
    st_rq_sum = 0
    st_pq_sum = 0

    th_sq_sum = 0
    th_rq_sum = 0
    th_pq_sum = 0

    n_valid = 0

    for i, l in enumerate(EVAL_LABELS):

        sq = SQ[l]
        rq = RQ[l]
        pq = PQ[l]

        if pq > 0:
            n_valid += 1

        if l in STUFF_LABELS:
            st_sq_sum += sq
            st_rq_sum += rq
            st_pq_sum += pq

        elif l in THING_LABELS:
            th_sq_sum += sq
            th_rq_sum += rq
            th_pq_sum += pq

        if l in class_det_counts.keys():
            n_det = class_det_counts[l]
        else:
            n_det = 0

        if l in class_gt_counts.keys():
            n_gt = class_gt_counts[l]
        else:
            n_gt = 0

        output_str = f"{EVAL_CLASSES[i]:>16} | {sq:.2f} {rq:.2f} {pq:.2f} {n_gt:d} {n_det:d}"
        write_output(output_str, output_fs)

    write_output("", output_fs)

    write_output(f"{'MEAN':>16}    SQ   RQ   PQ", output_fs)

    st_sq_mean = st_sq_sum / len(STUFF_LABELS)
    st_rq_mean = st_rq_sum / len(STUFF_LABELS)
    st_pq_mean = st_pq_sum / len(STUFF_LABELS)

    output_str = f"{'STUFF':>16} | {st_sq_mean:.2f} {st_rq_mean:.2f} {st_pq_mean:.2f}"
    write_output(output_str, output_fs)

    th_sq_mean = th_sq_sum / len(THING_LABELS)
    th_rq_mean = th_rq_sum / len(THING_LABELS)
    th_pq_mean = th_pq_sum / len(THING_LABELS)

    output_str = f"{'THINGS':>16} | {th_sq_mean:.2f} {th_rq_mean:.2f} {th_pq_mean:.2f}"
    write_output(output_str, output_fs)

    sq_mean = (st_sq_sum + th_sq_sum) / len(EVAL_LABELS)
    rq_mean = (st_rq_sum + th_rq_sum) / len(EVAL_LABELS)
    pq_mean = (st_pq_sum + th_pq_sum) / len(EVAL_LABELS)

    output_str = f"{'ALL':>16} | {sq_mean:.2f} {rq_mean:.2f} {pq_mean:.2f}"
    write_output(output_str, output_fs)

    write_output("", output_fs)

    output_fs.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument("-i", "--source_dir", type=Path, dest="source_dir",
                        help="Path to ScanNet scans root")

    parser.add_argument("-p", "--prediction", type=str, dest="prediction_name",
                        help="Name of predicted labels of ground truth mesh vertices (.txt)")

    parser.add_argument("-gt", "--ground_truth", type=str, dest="ground_truth_name",
                        help="Name of gt labels of ground truth mesh vertices (*.txt)")

    parser.add_argument("-th", "--threshold", type=float, dest="threshold", default=0.5,
                        help="Match IoU threshold")

    parser.add_argument("-o", "--output_file", type=Path, dest="output_file", default=None,
                        help="Path to a file to which save the output (*.txt)")

    args = parser.parse_args()

    main(**vars(args))
