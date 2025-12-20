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

# ------------------------------------------------------------
ALLOWED_SIZES = (0.05, 0.15)
# ------------------------------------------------------------


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

        # ------------------ image ------------------
        self.bridge = CvBridge()
        self.latest_image = None

        self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

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

        # ---------- build 4x4 transform ----------
        R_mat, _ = cv2.Rodrigues(rvec)

        T = np.eye(4)
        T[:3, :3] = R_mat
        T[:3, 3] = tvec

        # ---------- save to <pkg_share>/frames ----------
        frames_dir = os.path.join("/home/cc/ee106a/fa25/class/ee106a-aef/ros_workspaces/final/workspace/" "frames")
        os.makedirs(frames_dir, exist_ok=True)

        fname = f"ar_marker_{label}.npy"
        fpath = os.path.join(frames_dir, fname)
        np.save(fpath, T)

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
