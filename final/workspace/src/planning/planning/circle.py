import math
import sys
import time

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetMotionPlan
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from std_srvs.srv import Trigger

# from tf_transformations import quaternion_multiply
from tf2_ros import Buffer, TransformListener
from control_msgs.action import FollowJointTrajectory
from scipy.spatial.transform import Rotation as scipyR
from rclpy.action import ActionClient

from planning.ik import IKPlanner
import numpy as np
from planning.utils import quaternion_from_euler

def deg_to_rad(deg):
    return deg * math.pi / 180.0

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


def wrap_pi(a):
    return (a + math.pi) % (2*math.pi) - math.pi

class CircularScanNode(Node):
    def __init__(self):
        super().__init__("circular_scan")

        # --- existing setup unchanged ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 1
        )

        self.exec_ac = ActionClient(
            self,
            FollowJointTrajectory,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
        )

        self.radius = 0.2
        self.num_steps = 10
        self.z_offset = 0.4

        self.ik_planner = IKPlanner()

        self.sync_client = self.create_client(Trigger, "/external_step_done")

        self.current_joint_state = None
        self.job_queue = []

        while self.current_joint_state is None:
            self.get_logger().info("Waiting for joint state...")
            rclpy.spin_once(self, timeout_sec=0.1)

        self.build_circle_jobs()
        self.execute_jobs()


    def joint_state_callback(self, msg: JointState):
        self.current_joint_state = msg

    def wait_for_joint_state(self, timeout_sec=5.0):
        """
        Block until we have a current joint state or timeout.
        """
        start = self.get_clock().now()
        while rclpy.ok() and self.current_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - start).nanoseconds > timeout_sec * 1e9:
                self.get_logger().error("Timed out waiting for /joint_states")
                return False
        return True

    def call_sync_service(self):
        """
        Call the external sync Trigger service and wait for response.
        """
        req = Trigger.Request()
        future = self.sync_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error("Sync service call failed.")
            return False

        resp = future.result()
        if not resp.success:
            self.get_logger().warn(
                f"Sync service responded with success=False: {resp.message}"
            )
        else:
            self.get_logger().info(f"Sync service OK: {resp.message}")
        return True

    def _execute_joint_trajectory(self, joint_traj):
        self.get_logger().info("Waiting for controller action server...")
        self.exec_ac.wait_for_server()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_traj

        self.get_logger().info("Sending trajectory to controller...")
        send_future = self.exec_ac.send_goal_async(goal)
        print(send_future)
        send_future.add_done_callback(self._on_goal_sent)

    def _on_goal_sent(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("bonk")
            rclpy.shutdown()
            return

        self.get_logger().info("Executing...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_exec_done)

    def _on_exec_done(self, future):
        try:
            result = future.result().result
            self.get_logger().info("Execution complete.")
            self.execute_jobs()
        except Exception as e:
            self.get_logger().error(f"Execution failed: {e}")

    def build_circle_jobs(self):
        
        G_base_to_ar_table = np.load("/home/cc/ee106a/fa25/class/ee106a-aef/ros_workspaces/final/workspace/frames/base_to_ar_marker_table.npy")
        cx, cy, cz = G_base_to_ar_table[:3, 3]

        self.get_logger().info(f"Circle center: ({cx:.3f}, {cy:.3f}, {cz:.3f})")
 
        r = self.radius
        N = self.num_steps

        current_js = self.current_joint_state

        theta0 = math.atan2(-cy, -cx)
        dtheta = 2 * math.pi / N

        for i in list(range(5)) + list(range(9, 5, -1)):

            theta = theta0 + i * dtheta

            x = cx + r * math.cos(theta)
            y = min(cy + r * math.sin(theta), 0.72)
            z = cz + self.z_offset

            # yaw_center = math.atan2(cy - y, cx - x)
            # yaw = wrap_pi(yaw_center + math.pi) 
            new_theta = i * dtheta
            # if i == 5:
                # new_theta -= deg_to_rad(1)
            qx, qy, qz, qw = quaternion_from_euler(
                deg_to_rad(60.0), math.pi, new_theta
            )
            ik = self.ik_planner.compute_ik(
                current_js, x, y, z, qx=qx, qy=qy, qz=qz, qw=qw
            )
            if ik is None:
                self.get_logger().error("IK failed, aborting.")
                return

            self.job_queue.append(("move", ik))
            self.job_queue.append(("sync", None))

            if i == 4:
                x = 0.0
                y = 0.55
                z = 0.25
                qx, qy, qz, qw = quaternion_from_euler(
                deg_to_rad(0), math.pi, math.pi
            )
                ik = self.ik_planner.compute_ik(
                    current_js, x, y, z, qx=qx, qy=qy, qz=qz, qw=qw
                )
                if ik is None:
                    self.get_logger().error("IK failed, aborting.")
                    return
                current_js = ik
                self.job_queue.append(("move", ik))



                x = 0.0
                y = 0.55
                z = cz + self.z_offset
                qx, qy, qz, qw = quaternion_from_euler(
                deg_to_rad(0), math.pi, math.pi
            )
                ik = self.ik_planner.compute_ik(
                    current_js, x, y, z, qx=qx, qy=qy, qz=qz, qw=qw
                )
                if ik is None:
                    self.get_logger().error("IK failed, aborting.")
                    return
                

                current_js = ik
                self.job_queue.append(("move", ik))



            #     x = 0.0
            #     y = 0.75
            #     z = cz + self.z_offset
            #     qx, qy, qz, qw = quaternion_from_euler(
            #     deg_to_rad(60), math.pi, math.pi
            # )
            #     ik = self.ik_planner.compute_ik(
            #         current_js, x, y, z, qx=qx, qy=qy, qz=qz, qw=qw
            #     )
            #     if ik is None:
            #         self.get_logger().error("IK failed, aborting.")
            #         return
                

            #     current_js = ik
            #     self.job_queue.append(("move", ik))
                # self.job_queue.append(("sync", None))

                qx, qy, qz, qw = quaternion_from_euler(
                deg_to_rad(0), math.pi, 0
            )
                ik = self.ik_planner.compute_ik(
                    current_js, x, y, z, qx=qx, qy=qy, qz=qz, qw=qw
                )
                if ik is None:
                    self.get_logger().error("IK failed, aborting.")
                    return
                current_js = ik
                self.job_queue.append(("move", ik))



            #     new_theta = i * dtheta + deg_to_rad(1)
            #     qx, qy, qz, qw = quaternion_from_euler(
            #     deg_to_rad(60.0), math.pi, new_theta
            # )
            #     ik = self.ik_planner.compute_ik(
            #         current_js, x, y, z, qx=qx, qy=qy, qz=qz, qw=qw
            #     )
            #     if ik is None:
            #         self.get_logger().error("IK failed, aborting.")
            #         return
            # self.job_queue.append(("move", ik))



            current_js = ik
        
            print(x,y,z, new_theta)
        # exit()

    def execute_jobs(self):
        if not self.job_queue:
            self.get_logger().info("All jobs completed.")
            return

        job_type, payload = self.job_queue.pop(0)

        if job_type == "sync":
            req = Trigger.Request()
            time.sleep(0.25)
            future = self.sync_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            self.execute_jobs()
            return

        if job_type == "move":
            traj = self.ik_planner.plan_to_joints(payload)
            if traj is None:
                self.get_logger().error("Planning failed.")
                return
            self._execute_joint_trajectory(traj.joint_trajectory)



def main(args=None):
    time.sleep(1.0)
    rclpy.init(args=args)
    node = CircularScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
