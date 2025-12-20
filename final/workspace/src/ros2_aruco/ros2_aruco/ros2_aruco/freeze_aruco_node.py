#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import cv2
import numpy as np
import math
import os

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from ros2_aruco_interfaces.srv import CustomAruco
from ament_index_python.packages import get_package_share_directory


from tf2_ros import TransformBroadcaster


import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
import math
# import tf_transformations
# from autolab_core import transformations
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from ros2_aruco_interfaces.msg import ArucoMarkers
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as scipyR
import tf2_ros
# ------------------------------------------------------------
ALLOWED_SIZES = (0.05, 0.15)
# ------------------------------------------------------------


def tf_to_matrix(tf_msg):
    t = tf_msg.transform.translation
    r = tf_msg.transform.rotation

    # translation vector
    trans = np.array([t.x, t.y, t.z])

    # quaternion (x, y, z, w)
    quat = np.array([r.x, r.y, r.z, r.w])

    # convert quat → rotation matrix
    R_mat = scipyR.from_quat(quat).as_matrix()

    # build homogeneous transform
    T = np.eye(4)
    T[:3, :3] = R_mat
    T[:3, 3]  = trans
    return T


def quaternion_from_matrix(matrix):
    """Return quaternion from rotation matrix.

    >>> R = rotation_matrix(0.123, (1, 2, 3))
    >>> q = quaternion_from_matrix(R)
    >>> numpy.allclose(q, [0.0164262, 0.0328524, 0.0492786, 0.9981095])
    True

    """
    q = np.empty((4,), dtype=np.float64)
    M = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
    t = np.trace(M)
    if t > M[3, 3]:
        q[3] = t
        q[2] = M[1, 0] - M[0, 1]
        q[1] = M[0, 2] - M[2, 0]
        q[0] = M[2, 1] - M[1, 2]
    else:
        i, j, k = 0, 1, 2
        if M[1, 1] > M[0, 0]:
            i, j, k = 1, 2, 0
        if M[2, 2] > M[i, i]:
            i, j, k = 2, 0, 1
        t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
        q[i] = t
        q[j] = M[i, j] + M[j, i]
        q[k] = M[k, i] + M[i, k]
        q[3] = M[k, j] - M[j, k]
    q *= 0.5 / math.sqrt(t * M[3, 3])
    return q

class FreezeArucoNode(Node):
    def __init__(self):
        super().__init__("freeze_aruco_node")

        # ------------------ parameters ------------------
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("aruco_dictionary", "DICT_5X5_250")
        self.declare_parameter("package_name", "planning")

        self.image_topic = self.get_parameter("image_topic").value
        self.info_topic = self.get_parameter("camera_info_topic").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.package_name = self.get_parameter("package_name").value

        # ------------------ aruco ------------------
        dict_name = self.get_parameter("aruco_dictionary").value
        self.aruco_dict = cv2.aruco.Dictionary_get(getattr(cv2.aruco, dict_name))
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # ------------------ camera info ------------------
        self.info_msg = None
        self.K = None
        self.D = None

        self.info_sub = self.create_subscription(
            CameraInfo,
            self.info_topic,
            self.info_callback,
            qos_profile_sensor_data,
        )


        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self
        )

        # ------------------ image ------------------
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_image_msg = None

        self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        # ------------------ service ------------------
        self.create_service(CustomAruco, "/freeze_aruco", self.handle_freeze)

        self.get_logger().info("Freeze ArUco node ready")

    # ------------------------------------------------------------
    def info_callback(self, msg):
        self.info_msg = msg
        self.K = np.array(msg.k).reshape(3, 3)
        self.D = np.array(msg.d)
        self.get_logger().info("Camera info received")
        self.destroy_subscription(self.info_sub)

    # ------------------------------------------------------------
    def image_callback(self, msg):
        if self.info_msg is None:
            return
        self.latest_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="mono8"
        )
        self.latest_image_msg = msg

    # ------------------------------------------------------------
    def handle_freeze(self, request, response):
        label = request.frame_name.strip().lower()
        marker_size = float(request.marker_size)

        # ---------- validation ----------
        if marker_size not in ALLOWED_SIZES:
            response.success = False
            response.message = "marker_size must be 0.05 or 0.15"
            return response

        if label not in ("arm", "table"):
            response.success = False
            response.message = "frame_name must be 'arm' or 'table'"
            return response

        if (label == "arm" and marker_size != 0.15) or \
           (label == "table" and marker_size != 0.05):
            response.success = False
            response.message = "marker_size incompatible with frame_name"
            return response

        if self.info_msg is None:
            response.success = False
            response.message = "CameraInfo not received"
            return response

        if self.latest_image is None:
            response.success = False
            response.message = "No image received"
            return response

        img = self.latest_image

        # ---------- detect aruco ----------
        corners, ids, _ = cv2.aruco.detectMarkers(
            img, self.aruco_dict, parameters=self.aruco_params
        )

        if ids is None or len(ids) != 1:
            response.success = False
            response.message = "Expected exactly one ArUco marker."
            return response

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, marker_size, self.K, self.D
        )

        rvec = rvecs[0][0]
        tvec = tvecs[0][0]

        if label == "table":

            # ---------- build 4x4 transform ----------
            R_mat, _ = cv2.Rodrigues(rvec)

            T = np.eye(4)
            T[:3, :3] = R_mat
            T[:3, 3] = tvec

            # ---------- save to <pkg_share>/frames ----------
            frames_dir = os.path.join("/home/cc/ee106a/fa25/class/ee106a-aef/ros_workspaces/final/workspace/" "frames")
            os.makedirs(frames_dir, exist_ok=True)

            fname = f"camera_to_ar_marker_table.npy"
            fpath = os.path.join(frames_dir, fname)
            np.save(fpath, T)

            response.success = True
            response.message = f"Saved transform to {fpath}"
            self.get_logger().info(response.message)
            return response
        else:
            # OpenCV (camera → aruco)
            G_cam_to_ar = np.eye(4)
            G_cam_to_ar[:3, :3] = cv2.Rodrigues(np.array(rvec))[0]
            G_cam_to_ar[:3, 3]  = np.array(tvec).reshape(3)

            G_ar_to_cam = np.linalg.inv(G_cam_to_ar)

            # GIVEN fixed transform
            
            G_ar_arm_to_base = np.array([
                [-1, 0, 0, 0.0],
                [ 0, 0, 1, 0.16],
                [ 0, 1, 0, -0.13],
                [ 0, 0, 0, 1.0]
            ])

            G_base_to_ar = np.linalg.inv(G_ar_arm_to_base)

            # TF lookup
            wrist_msg = self.tf_buffer.lookup_transform(
                'wrist_3_link',
                'base_link',
                rclpy.time.Time()
            )

            G_wrist_to_base = tf_to_matrix(wrist_msg)

            G_wrist_to_cam = G_wrist_to_base @ G_base_to_ar @ G_ar_to_cam 

            # ---------- save to <pkg_share>/frames ----------
            frames_dir = os.path.join("/home/cc/ee106a/fa25/class/ee106a-aef/ros_workspaces/final/workspace/" "frames")
            os.makedirs(frames_dir, exist_ok=True)  


            fname = f"wrist_to_cam.npy"
            fpath = os.path.join(frames_dir, fname)
            np.save(fpath, G_wrist_to_cam)
            response.success = True
            response.message = f"Saved transform to {fpath}"
            self.get_logger().info(response.message)
            return response




# ------------------------------------------------------------
def main():
    rclpy.init()
    node = FreezeArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
