import os
import time
import yaml
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import tf2_ros
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as scipyR
# import open3d as o3d
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import struct


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

class ImageCaptureService(Node):
    def __init__(self):
        super().__init__('image_capture_service')

        # -----------------------
        # Parameters
        # -----------------------
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('cloud_topic', '/camera/camera/depth/color/points')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('wrist_frame', 'wrist_3_link')
        self.declare_parameter('output_root', '/home/cc/ee106a/fa25/class/ee106a-aef/ros_workspaces/final/circular_scans/')
        self.declare_parameter('settle_time', 0.25)

        self.image_topic = self.get_parameter('image_topic').value
        self.cloud_topic = self.get_parameter('cloud_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.wrist_frame = self.get_parameter('wrist_frame').value
        self.output_root = os.path.expanduser(
            self.get_parameter('output_root').value
        )
        self.settle_time = self.get_parameter('settle_time').value

        # -----------------------
        # Image subscription
        # -----------------------
        self.bridge = CvBridge()
        self.latest_image = None

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        # -----------------------
        # Point cloud subscription (ADDED)
        # -----------------------
        self.latest_cloud = None
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self.cloud_callback,
            10
        )

        self.i = 0

        # -----------------------
        # TF buffer
        # -----------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self
        )

        # -----------------------
        # Service
        # -----------------------
        self.srv = self.create_service(
            Trigger,
            '/external_step_done',
            self.handle_trigger
        )

        # -----------------------
        # Run index
        # -----------------------
        os.makedirs(self.output_root, exist_ok=True)
        self.run_idx = self._get_next_run_index()

        self.run_dir = os.path.join(
            self.output_root, f'run{self.run_idx}'
        )
        os.makedirs(self.run_dir, exist_ok=True)

        self.get_logger().info(
            f'Image capture service ready. Saving to {self.run_dir}'
        )


    def _get_next_run_index(self):
        existing = []
        for d in os.listdir(self.output_root):
            if d.startswith('run'):
                try:
                    existing.append(int(d.replace('run', '')))
                except ValueError:
                    pass
        return max(existing) + 1 if existing else 0

    # ------------------------------------------------------------
    def image_callback(self, msg: Image):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')

    # ------------------------------------------------------------
    # ADDED: point cloud callback
    def cloud_callback(self, msg: PointCloud2):
        self.latest_cloud = msg

    # ------------------------------------------------------------
    def handle_trigger(self, request, response):
        self.get_logger().info('Capture request received')

        # 1. settling disabled per your code
        # time.sleep(self.settle_time)

        # 2. Check image availability
        if self.latest_image is None:
            response.success = False
            response.message = 'No image received yet'
            self.get_logger().error(response.message)
            return response

        # 3. Check cloud availability (ADDED)
        if self.latest_cloud is None:
            response.success = False
            response.message = 'No point cloud received yet'
            self.get_logger().error(response.message)
            return response

        # 4. TF lookup
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.wrist_frame,
                rclpy.time.Time()
            )
        except Exception as e:
            response.success = False
            response.message = f'TF lookup failed: {e}'
            self.get_logger().error(response.message)
            return response

        wrist_to_camera = np.load("/home/cc/ee106a/fa25/class/ee106a-aef/ros_workspaces/final/workspace/frames/wrist_to_cam.npy")

        # 5. Save image
        image_path = f"{self.run_dir}/image_{self.i}.png"
        cv2.imwrite(image_path, self.latest_image)

        # 6. Save pose (existing code)
        base_to_wrist = tf_to_matrix(tf)
        base_to_camera = base_to_wrist @ wrist_to_camera
        np.save(f"{self.run_dir}/pose_{self.i}.npy", base_to_camera)

        # 7. SAVE POINT CLOUD (NO Open3D)
        try:
            # Convert PointCloud2 → Nx3 numpy
            pts = []
            colors = []
            for p in pc2.read_points(self.latest_cloud, field_names=("x", "y", "z", "rgb"), skip_nans=True):
                x, y, z, rgb = p

                # unpack rgb (float32 stored as uint32)
                rgb_uint32 = struct.unpack('I', struct.pack('f', rgb))[0]

                r = (rgb_uint32 >> 16) & 255
                g = (rgb_uint32 >> 8) & 255
                b = rgb_uint32 & 255

                pts.append([x, y, z])
                colors.append([r/255.0, g/255.0, b/255.0])


            pts = np.array(pts, dtype=np.float32)

            # Transform into base_link frame
            pts_h = np.hstack([pts, np.ones((pts.shape[0], 1))])  # N×4
            pts_base = (base_to_camera @ pts_h.T).T[:, :3]        # N×3

            # Save as cloud_i.npy
            np.save(f"{self.run_dir}/cloud_{self.i}.npy",
                {"xyz": pts_base, "rgb": colors})

            self.get_logger().info(f"Saved cloud_{self.i}.npy")

        except Exception as e:
            self.get_logger().error(f"Failed saving point cloud: {e}")
        # ------------------------------------------------------------

        self.i += 1

        response.success = True
        response.message = f'Saved image, pose, and cloud {self.i}'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ImageCaptureService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
