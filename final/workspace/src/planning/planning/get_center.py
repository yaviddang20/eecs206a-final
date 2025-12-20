# ROS Libraries
from std_srvs.srv import Trigger
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PointStamped
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R
from ros2_aruco_interfaces.srv  import CustomAruco
from scipy.spatial.transform import Rotation as scipyR
import math
import time

from planning.ik import IKPlanner
from planning.utils import quaternion_from_euler

import math
import numpy as np

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

class UR7e_GetCenter(Node):
    def __init__(self):
        super().__init__("get_center")

        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 1
        )

        self.exec_ac = ActionClient(
            self,
            FollowJointTrajectory,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
        )

        self.gripper_cli = self.create_client(Trigger, "/toggle_gripper")

        self.current_plan = None
        self.joint_state = None

        self.job_queue = []

        self.ik_planner = IKPlanner()

        self.freeze_client = self.create_client(
            CustomAruco, "/freeze_aruco"
        )


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        while self.joint_state is None:
            self.get_logger().info("Waiting for initial joint state...")
            rclpy.spin_once(self, timeout_sec=3.0)
        self.get_center()

    def joint_state_callback(self, msg: JointState):
        self.joint_state = msg

    def get_center(self):

        shared_x = 0.0
        shared_y = 0.40
        shared_z = 0.25

        qx, qy, qz, qw = quaternion_from_euler(deg_to_rad(60.0), math.pi, 0)
        j1 = self.ik_planner.compute_ik(
            self.joint_state,
            x=shared_x,
            y=shared_y,
            z=shared_z,
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
        )

        self.job_queue.append((j1, 0.6, False))

        self.job_queue.append(('placeholder', 0.0, True))

        self.execute_jobs()


        # --- compute center exactly as before ---
        G_cam_to_ar_table = np.load(
            "/home/cc/ee106a/fa25/class/ee106a-aef/ros_workspaces/final/workspace/frames/camera_to_ar_marker_table.npy"
        )

        G_wrist_to_cam = np.load(
            "/home/cc/ee106a/fa25/class/ee106a-aef/ros_workspaces/final/workspace/frames/wrist_to_cam.npy"
        )
        trans = self.tf_buffer.lookup_transform(
            "base_link", "wrist_3_link", rclpy.time.Time()
        )

        G_base_to_wrist = tf_to_matrix(trans)
        G_base_to_ar_table = G_base_to_wrist @ G_wrist_to_cam @ G_cam_to_ar_table
        np.save("/home/cc/ee106a/fa25/class/ee106a-aef/ros_workspaces/final/workspace/frames/base_to_ar_marker_table.npy", G_base_to_ar_table)

    def execute_jobs(self):
        if not self.job_queue:
            self.get_logger().info("All jobs completed.")
            rclpy.shutdown()
            return

        self.get_logger().info(
            f"Executing job queue, {len(self.job_queue)} jobs remaining."
        )
        next_job, velocity, before_call_freeze = self.job_queue.pop(0)

        if before_call_freeze:
            req = CustomAruco.Request()
            req.frame_name = "table"
            req.marker_size = 0.05
            time.sleep(0.5)

            future = self.freeze_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            result = future.result()
            if result.success:
                print("Freeze successful:", result.message)
            else:
                print("Freeze failed:", result.message)

        if isinstance(next_job, JointState):

            traj = self.ik_planner.plan_to_joints(next_job, velocity=velocity)
            if traj is None:
                self.get_logger().error("Failed to plan to position")
                return

            self.get_logger().info("Planned to position")

            self._execute_joint_trajectory(traj.joint_trajectory)
        elif next_job == "toggle_grip":
            self.get_logger().info("Toggling gripper")
            self._toggle_gripper()
        elif next_job == "placeholder":
            self.get_logger().info("Placeholder job, proceeding to next job.")
            self.execute_jobs()  # Proceed to next job
        else:
            self.get_logger().error(f"Unknown job type: {next_job}")
            self.execute_jobs()  # Proceed to next job

    def _toggle_gripper(self):
        if not self.gripper_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Gripper service not available")
            rclpy.shutdown()
            return

        req = Trigger.Request()
        future = self.gripper_cli.call_async(req)
        # wait for 2 seconds
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        self.get_logger().info("Gripper toggled.")
        self.execute_jobs()  # Proceed to next job

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
            self.execute_jobs()  # Proceed to next job
        except Exception as e:
            self.get_logger().error(f"Execution failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = UR7e_GetCenter()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()
