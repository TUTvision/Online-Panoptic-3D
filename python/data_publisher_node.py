#!/usr/bin/env python
""" raiviol 4/2021
    leevi.raivio@tuni.fi
"""

from data_loader import Scannet_Loader
from data_loader import Rtabmap_Loader

import sys
import re
from pathlib import Path

import rospy
from rospy import ROSInterruptException
from std_msgs.msg import Bool
from std_srvs.srv import Empty

from bondpy import bondpy

from data_publisher import DataPublisher

def run_loader(pub, rate, interval, loader):

    try:
        scheduler = rospy.Rate(rate) # Hz

        frame = loader.get_frame(interval)

        count = 0
        while frame is not None:
            msg = "load frame no. " + str(count)
            rospy.loginfo(msg)

            r, d, p, m, li, lc = frame

            time = rospy.get_rostime()

            rospy.loginfo("rgb")
            pub.publish_rgb_from_file(r, time)
            rospy.loginfo("depth")
            pub.publish_depth_from_file(d, time)
            rospy.loginfo("mask")
            pub.publish_mask_from_file(m, time)
            rospy.loginfo("label")
            pub.publish_label_from_file(li, lc, time)
            rospy.loginfo("pose")
            pub.publish_pose(p, time)

            scheduler.sleep()
            rospy.loginfo("get frame")
            frame = loader.get_frame(interval)

            count += 1

        loader.done()

    except:
        rospy.logwarn("ERROR IN PUBLISH RUNNER")

def loader_callback(data, args):

    try:
        loader, publisher, interval = args

        frame = loader.get_frame(interval)

        if frame is not None:
            r, d, p, m, li, lc = frame

            time = rospy.get_rostime()

            status = publisher.publish_pose(p, time)

            if not status:
                loader_callback(data, args)
            else:
                publisher.publish_rgb_from_file(r, time)
                publisher.publish_depth_from_file(d, time)
                publisher.publish_mask_from_file(m, time)
                publisher.publish_label_from_file(li, lc, time)

        else:
            loader.done()
    except:
        rospy.logwarn("ERROR IN PUBLISH CALLBACK")

def run_node(path, rate, interval, format, reduction_rate, bond_id):
    rgb_shape = (968, 1296)
    depth_shape = (480, 640)
    world_frame_id = "world"
    camera_frame_id = "camera"

    if format == "scannet":
        loader = Scannet_Loader(path)

    elif format == "rtabmap":
        loader = Rtabmap_Loader(path)
        rgb_shape = (360, 640)
        depth_shape = rgb_shape

    else:
        raise ROSInterruptException("Data format {} not recognised".format(format))

    publisher = CloudPublisher(path, rgb_shape, depth_shape, reduction_rate, world_frame_id, camera_frame_id)

    id = str(bond_id)
    bond = bondpy.Bond("publisher_bond", id)

    if bond_id > 0:
        rospy.loginfo("Start bond "+id)
        bond.start()

    rospy.loginfo("Data root: {}".format(str(path)))

    rospy.loginfo("Iterating images...")

    if rate == 0:
        confirm_subscriber = \
            rospy.Subscriber("voxel_fusion_confirm",
                Bool, loader_callback, (loader, publisher, interval))

        rospy.sleep(5) # for some reason, integrator will not catch this without a small delay
        loader_callback(True, (loader, publisher, interval)) # publish to start fusion
    else:
        run_loader(publisher, rate, interval, loader)

    # check once a second whether fusion is complete
    while not loader.finished:
        rospy.sleep(1)

    # save results
    rospy.wait_for_service('/voxblox_fusion_node/generate_mesh')
    service_call = rospy.ServiceProxy('/voxblox_fusion_node/generate_mesh', Empty)
    resp = service_call()

    rospy.wait_for_service('/voxblox_fusion_node/save_segmentation')
    service_call = rospy.ServiceProxy('/voxblox_fusion_node/save_segmentation', Empty)
    resp = service_call()

    if bond_id > 0:
        rospy.loginfo("Break bond "+id)
        bond.break_bond()

if __name__ == '__main__':

    path = Path(rospy.get_param("/data_publisher/data_root"))
    format = rospy.get_param("/data_publisher/data_format")
    rate = float(rospy.get_param("/data_publisher/publish_rate", 1))
    interval = int(rospy.get_param("/data_publisher/publish_interval", 1))
    reduction_rate = float(rospy.get_param("/data_publisher/reduction_rate", 1))
    bond_id = int(rospy.get_param("/data_publisher/bond_id", -1))

    confirm_pub = rospy.Publisher("/fusion_complete_confirm", Bool, queue_size=1)

    results_name = rospy.get_param("/results_name")
    result_file = Path(results_name + '.txt')

    bmsg = Bool()

    try:
        rospy.init_node('rgbd_publisher', log_level=rospy.INFO)

        # avoid rewriting results, can also be used to continue from
        # last unprocessed scene if inference script crashes
        if not result_file.exists():
            run_node(path, rate, interval, format, reduction_rate, bond_id)

        bmsg.data = True

    except rospy.ROSInterruptException as E:
        rospy.logwarn(E)
        bmsg.data = False

    info = "Confirm fusion done, success = "+str(bmsg.data)
    rospy.loginfo(info)
    confirm_pub.publish(bmsg)

    sys.exit(0)
