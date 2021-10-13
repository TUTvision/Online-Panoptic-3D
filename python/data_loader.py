
import sys
import re
from pathlib import Path

import rospy
from rospy import ROSInterruptException

class Loader_Base():
    def __init__(self, path = None):
        self.reset()

        if path is not None:
            self.load_data(path)

    def reset(self):
        self.rgb_imgs    = []
        self.depth_imgs  = []
        self.poses       = []
        self.label_masks = []
        self.label_imgs  = []
        self.label_cats  = []

        self.running_idx = 0

        self.finished = False

    def get_frame(self, interval = 1):

        if self.running_idx >= len(self.rgb_imgs):
            return None

        output = (self.rgb_imgs[self.running_idx],
                  self.depth_imgs[self.running_idx],
                  self.poses[self.running_idx],
                  self.label_masks[self.running_idx],
                  self.label_imgs[self.running_idx],
                  self.label_cats[self.running_idx])

        self.running_idx += interval;

        return output

    def done(self):
        self.finished = True


class Scannet_Loader(Loader_Base):

    def load_data(self, path):
        rgb_path = path / Path("color")
        if not rgb_path.is_dir():
            raise ROSInterruptException("{} not a directory".format(str(rgb_path)))

        rospy.loginfo("RGB path: {}".format(str(rgb_path)))

        depth_path = path / Path("depth")
        if not depth_path.is_dir():
            raise ROSInterruptException("{} not a directory".format(str(depth_path)))

        rospy.loginfo("Depth path: {}".format(str(depth_path)))

        mask_path = path / Path("colormask")
        if not mask_path.is_dir():
            raise ROSInterruptException("{} not a directory".format(str(mask_path)))

        label_path = path / Path("panoptic")
        if not label_path.is_dir():
            raise ROSInterruptException("{} not a directory".format(str(label_path)))

        rospy.loginfo("Label path: {}".format(str(label_path)))

        pose_path = path / Path("pose")
        if not pose_path.is_dir():
            raise ROSInterruptException("{} not a directory".format(str(pose_path)))

        rospy.loginfo("Pose path: {}".format(str(pose_path)))

        image_files = list(rgb_path.glob('*.png'))
        if len(image_files)<1:
            image_files = list(rgb_path.glob('*.jpg'))
            if len(image_files)<1:
                raise ROSInterruptException("No color images found in {}".format(str(rgb_path)))

        # sort images by time
        image_files.sort(key=lambda fname:int(''.join(re.findall(r'\d+', str(fname)))))

        for rgb_f in image_files:
            name = rgb_f.name

            label_img_f = label_path / name
            if not label_img_f.exists():
                label_img_f = label_path / Path(name.split('.')[0] + '.png')
            if not label_img_f.exists():
                label_img_f = label_path / Path(name.split('.')[0] + '_mask.png')

                if not label_img_f.exists():
                    continue
                    #raise ROSInterruptException("Matching label image for {} not found".format(str(name)))

            depth_f = depth_path / name

            if not depth_f.exists():
                depth_f = depth_path / Path(name.split('.')[0] + '.png')
                if not depth_f.exists():
                    raise ROSInterruptException("Matching depth image for {} not found".format(str(name)))

            label_mask_f = mask_path / name
            if not label_mask_f.exists():
                label_mask_f = mask_path / Path(name.split('.')[0] + '.png')

                if not label_mask_f.exists():
                    raise ROSInterruptException("Matching label mask for {} not found".format(str(name)))

            label_cat_f = label_path / Path(name.split('.')[0] + '.txt')
            if not label_cat_f.exists():
                raise ROSInterruptException("Matching label category file for {} not found".format(str(name)))

            pose_f = pose_path / Path(name.split('.')[0] + '.txt')
            if not pose_f.exists():
                raise ROSInterruptException("Matching pose for {} not found".format(str(name)))

            self.rgb_imgs.append(str(rgb_f.resolve()))
            self.depth_imgs.append(str(depth_f.resolve()))
            self.label_masks.append(str(label_mask_f.resolve()))
            self.label_imgs.append(str(label_img_f.resolve()))
            self.label_cats.append(str(label_cat_f.resolve()))
            self.poses.append(str(pose_f.resolve()))

class Rtabmap_Loader(Loader_Base):

    def load_data(path):
        rgb_path = path / Path("color")
        if not rgb_path.is_dir():
            raise ROSInterruptException("{} not a directory".format(str(rgb_path)))

        rospy.loginfo("RGB path: {}".format(str(rgb_path)))

        depth_path = path / Path("depth")
        if not depth_path.is_dir():
            raise ROSInterruptException("{} not a directory".format(str(depth_path)))

        rospy.loginfo("Depth path: {}".format(str(depth_path)))

        mask_path = path / Path("colormask")
        if not mask_path.is_dir():
            raise ROSInterruptException("{} not a directory".format(str(mask_path)))

        label_path = path / Path("panoptic")
        if not label_path.is_dir():
            raise ROSInterruptException("{} not a directory".format(str(label_path)))

        rospy.loginfo("Label path: {}".format(str(label_path)))

        pose_path = path / Path("poses.txt")
        if not pose_path.exists():
            raise ROSInterruptException("{} not found".format(str(pose_path)))

        rospy.loginfo("Pose path: {}".format(str(pose_path)))

        depth_files = list(depth_path.glob('*.png'))
        if len(depth_files)<1:
            depth_files = list(depth_path.glob('*.jpg'))
            if len(depth_files)<1:
                raise ROSInterruptException("No depth images found in {}".format(str(depth_path)))

        # sort images by time
        depth_files.sort(key=lambda fname:int(''.join(re.findall(r'\d+', str(fname)))))

        for depth_f in depth_files:
            name = depth_f.name

            rgb_f = rgb_path / name
            if not rgb_f.exists():
                rgb_f = rgb_path / Path(name.split('.')[0] + '.png')
                if not rgb_f.exists():
                    raise ROSInterruptException("Matching color image for {} not found".format(str(name)))

            label_mask_f = mask_path / name
            if not label_mask_f.exists():
                label_mask_f = mask_path / Path(name.split('.')[0] + '_mask.png')

                if not label_mask_f.exists():
                    raise ROSInterruptException("Matching label mask for {} not found".format(str(name)))

            label_img_f = label_path / name
            if not label_img_f.exists():
                label_img_f = label_path / Path(name.split('.')[0] + '.png')

                if not label_img_f.exists():
                    raise ROSInterruptException("Matching label image for {} not found".format(str(name)))

            self.rgb_imgs.append(str(rgb_f.resolve()))
            self.depth_imgs.append(str(depth_f.resolve()))
            self.label_masks.append(str(label_mask_f.resolve()))
            self.label_imgs.append(str(label_img_f.resolve()))

        with open(str(pose_path),'r') as pose_fs:
            for idx, line in enumerate(pose_fs):
                self.poses.append(line)
