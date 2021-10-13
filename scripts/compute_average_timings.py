
import re
from pathlib import Path
from argparse import ArgumentParser

import numpy as np

import roslaunch
import rospy
from std_msgs.msg import Bool

from bondpy import bondpy

def parse_timing_file(path):

    times = {}

    with open(str(path), 'r') as f:
        for i, line in enumerate(f):
            str_float = re.findall("\d+\.\d+", line)

            if len(str_float) > 0:
                time = float(str_float[0])
            else:
                continue

            if 'fusion' in line.lower():
                times['fusion'] = time
            elif 'traking' in line.lower() or 'tracking' in line.lower():
                times['tracking'] = time
            elif 'preprocess' in line.lower():
                times['preprocess'] = time
            elif 'association' in line.lower():
                times['association'] = time

    return times

def write_output(string, stream):
    stream.write(string + '\n')
    print(string)

def main(source_dir, timings_name, output_file):

    scenes = list(source_dir.glob('scene*'))
    n_scenes = len(scenes)
    scenes.sort( )

    fusion_times = []
    tracking_times = []
    preprocess_times = []
    association_times = []

    for i, d in enumerate(scenes):
        print(f"{i+1}/{n_scenes} {d.name}", end="                           \r")

        file = d / Path(timings_name)

        times = parse_timing_file(file)

        fusion_times.append(float(times['fusion']))
        tracking_times.append(float(times['tracking']))
        preprocess_times.append(float(times['preprocess']))
        association_times.append(float(times['association']))

    fusion_times = np.asarray(fusion_times)
    tracking_times = np.asarray(tracking_times)
    preprocess_times = np.asarray(preprocess_times)
    association_times = np.asarray(association_times)

    fusion_av = fusion_times.mean()
    tracking_av = tracking_times.mean()
    preprocess_av = preprocess_times.mean()
    association_av = association_times.mean()

    print()
    print()

    with open(str(output_file), 'w') as f:
        text = f"Fusion average: {fusion_av}\n\n" + \
               f"Tracking average: {tracking_av}\n" + \
               f"Preprocess average: {preprocess_av}\n" + \
               f"Association average: {association_av}\n"

        write_output(text, f)

if __name__ == '__main__':

    parser = ArgumentParser()

    parser.add_argument("-s", "--source_dir", type=Path, dest="source_dir",
                        help="Path to ScanNet scans root")

    parser.add_argument("-t", "--timings_name", type=str, dest="timings_name", help="")
    parser.add_argument("-o", "--output_file", type=Path, dest="output_file", help="")

    args = parser.parse_args()

    main(**vars(args))
