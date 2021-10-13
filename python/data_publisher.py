
import cv2
import numpy as np
from pathlib import Path

import rospy
from rospy import ROSInterruptException
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import MultiArrayDimension
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
import tf_conversions
import tf2_ros
import tf
from tf.transformations import quaternion_from_matrix

def get_pose_from_file(file, d=4):
    ''' Pose mtx from a scannet pose file (.txt)
    '''

    cam_pose = np.eye(d)

    with open(str(file),'r') as f:
        for i, line in enumerate(f):
            if i > d-1: break
            if len(line) < 1: return None

            line_f   = [float(x) for x in line.replace(',','.').split(' ')[:d]]

            cam_pose[i,:] = line_f

    return cam_pose

def get_pose_from_line(line):
    ''' Pose mtx from a line of rtabmap raw pose export (poses.txt)
    '''

    pose_raw = [float(x) for x in line.replace(',','.').split(' ')]

    cam_pose = np.eye(4)
    cam_pose[0,:] = pose_raw[:4]
    cam_pose[1,:] = pose_raw[4:8]
    cam_pose[2,:] = pose_raw[8:]

    return cam_pose

class DataPublisher():
    def __init__(self,
                 path,
                 rgb_shape,
                 depth_shape,
                 reduction_rate = 1,
                 world_frame_id = "world",
                 camera_frame_id = "camera"):

        rgb_intr_path = path / Path("intrinsic_color.txt")
        depth_intr_path = path / Path("intrinsic_depth.txt")

        if rgb_intr_path.exists():
            self.rgb_intr = get_pose_from_file(rgb_intr_path, 3)
        else:
            pass

        if depth_intr_path.exists():
            self.depth_intr = get_pose_from_file(depth_intr_path, 3)
        else:
            pass

        self.rgb_pub_        = rospy.Publisher('/camera/rgb/img_rect',   Image, queue_size=200)
        self.depth_pub_      = rospy.Publisher('/camera/depth/img_rect', Image, queue_size=200)
        self.label_mask_pub_ = rospy.Publisher('/camera/label/color_rect', Image, queue_size=200)
        self.label_enc_pub_  = rospy.Publisher('/camera/label/enc_rect', Image, queue_size=200)

        self.rgb_info_pub_   = rospy.Publisher('/camera/rgb/camera_info', CameraInfo, queue_size=200)
        self.depth_info_pub_ = rospy.Publisher('/camera/depth/camera_info', CameraInfo, queue_size=200)

        self.tf_pub_ = tf2_ros.TransformBroadcaster()

        self.bridge_ = CvBridge()

        self.rgb_shape_ = rgb_shape
        self.depth_shape_ = depth_shape

        self.reduction_rate_ = reduction_rate

        self.camera_id_ = camera_frame_id
        self.world_id_  = world_frame_id

    def create_img_msg(self, file_path, time, intrinsics=None, type="rgb"):
        img = cv2.imread(file_path, cv2.IMREAD_UNCHANGED)

        if intrinsics is not None:
            img = cv2.resize(
                cv2.undistort(
                    img,
                    intrinsics,
                    None),
                    (self.depth_shape_[1],self.depth_shape_[0])
                )

        encoding = "bgr8"
        if type == "depth":
            encoding = "passthrough"#"mono16"
        elif type == "label":
            encoding = "bgr8"

        img = cv2.resize(img, (int(img.shape[1]*self.reduction_rate_),
                               int(img.shape[0]*self.reduction_rate_)))

        msg_out = self.bridge_.cv2_to_imgmsg(img, encoding=encoding)
        msg_out.header.stamp = time
        msg_out.header.frame_id = self.camera_id_

        return msg_out

    def publish_rgb_from_file(self, file_path, time):
        self.rgb_pub_.publish(
            self.create_img_msg(file_path, time, intrinsics=None)
            )

    def publish_mask_from_file(self, img_file_path, time):
        self.label_mask_pub_.publish(
            self.create_img_msg(img_file_path, time, intrinsics=None, type="rgb")
            )

    def publish_label_from_file(self, img_file_path, cat_file_path, time):
        self.label_enc_pub_.publish(
            self.create_img_msg(img_file_path, time, intrinsics=None, type="label")
            )

    def publish_depth_from_file(self, file_path, time):
        self.depth_pub_.publish(
            self.create_img_msg(file_path, time, intrinsics=None, type="depth")
            )

    def generate_camera_info(self, time, img_shape, frame_id, intrinsics):
        info = CameraInfo()

        r = self.reduction_rate_

        # store info without header
        info.header.stamp = time
        info.header.frame_id = frame_id
        info.width = int(img_shape[1]*r)
        info.height = int(img_shape[0]*r)
        info.distortion_model = 'plumb_bob'
        cx = info.width / 2.0
        cy = info.height / 2.0

        info.D = [0, 0, 0, 0, 0]
        info.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]

        K = intrinsics
        info.K = np.ravel(K, order='C')

        resize_mult = np.array(((r,0,0),(0,r,0),(0,0,1)))

        P = np.eye(3,4)
        P[:3,:3] = np.matmul(resize_mult, K)
        info.P = np.ravel(P, order='C')

        return info

    def publish_pose(self, pose_input, time):

        path = Path(pose_input)

        status = True

        if path.suffix == '.txt':
            pose = get_pose_from_file(path)
        else:
            pose = get_pose_from_line(pose_input)

        if np.isfinite(pose).all() and not np.isnan(pose).any() and pose is not None:

            R = np.eye(4,4)
            R[:3,:3] = pose[:3,:3]

            t = TransformStamped()
            t.header.stamp = time
            t.header.frame_id = self.world_id_
            t.child_frame_id  = self.camera_id_
            t.transform.translation.x = pose[0,3]
            t.transform.translation.y = pose[1,3]
            t.transform.translation.z = pose[2,3]

            quat =  quaternion_from_matrix(R)
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]

            self.tf_pub_.sendTransform(t)

            # simultaneously publish camera metadata
            rgb_info   = self.generate_camera_info(time, self.rgb_shape_, self.camera_id_, self.rgb_intr)
            depth_info = self.generate_camera_info(time, self.depth_shape_, self.camera_id_, self.depth_intr)

            self.rgb_info_pub_.publish(rgb_info)
            self.depth_info_pub_.publish(depth_info)

        else:
            status = False

        return status
