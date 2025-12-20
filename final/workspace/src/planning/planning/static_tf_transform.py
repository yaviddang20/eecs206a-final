#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from tf2_ros import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation as R
import numpy as np
import time

class ConstantTransformPublisher(Node):
    def __init__(self):
        super().__init__('constant_tf_publisher')
        self.br = StaticTransformBroadcaster(self)

        # Homogeneous transform G_ar->base
        G = np.array([
            [-1, 0, 0, 0.0],
            [ 0, 0, 1, 0.16],
            [ 0, 1, 0, -0.13],
            [ 0, 0, 0, 1.0]
        ])

        r = R.from_matrix(G[:3, :3])
        q = r.as_quat()

        # Create TransformStamped
        self.transform = TransformStamped()
        # ---------------------------
        # TODO: Fill out TransformStamped message
        # --------------------------
        self.transform.child_frame_id = 'base_link'
        self.transform.header.frame_id = 'ar_marker_arm'
        # self.transform.parent_frame_id = 'camera_link'
        my_transform = Transform()
        my_transform.translation.x = G[0, 3]
        my_transform.translation.y = G[1, 3]
        my_transform.translation.z = G[2, 3]
        my_transform.rotation.x = q[0]
        my_transform.rotation.y = q[1]
        my_transform.rotation.z = q[2]
        my_transform.rotation.w = q[3]

        self.transform.transform = my_transform

        self.get_logger().info("Done with tf transform")

        self.timer = self.create_timer(0.05, self.broadcast_tf)

    def broadcast_tf(self):
        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.br.sendTransform(self.transform)

def main():
    rclpy.init()
    node = ConstantTransformPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()