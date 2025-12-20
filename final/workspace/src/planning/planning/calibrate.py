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
import math
import time

from planning.ik import IKPlanner
from planning.utils import quaternion_from_euler

import math

def deg_to_rad(deg):
    return deg * math.pi / 180.0

class UR7e_Calibrate(Node):
    def __init__(self):
        super().__init__("calibrate")

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

        self.ik_planner = IKPlanner()

        self.freeze_client = self.create_client(
            CustomAruco, "/freeze_aruco"
        )

        self.job_queue = (
            []
        )  # Entries should be of type either JointState or String('toggle_grip')
        while self.joint_state is None:
            self.get_logger().info("Waiting for initial joint state...")
            rclpy.spin_once(self, timeout_sec=3.0)
        self.calibrate()

    def joint_state_callback(self, msg: JointState):
        self.joint_state = msg

    def calibrate(self):

        shared_x = 0.0
        shared_y = 0.75
        shared_z = 0.15

        qx, qy, qz, qw = quaternion_from_euler(deg_to_rad(6.0), math.pi, math.pi / 2)
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


        qx, qy, qz, qw = quaternion_from_euler(deg_to_rad(12.0), math.pi, math.pi)
        j2 = self.ik_planner.compute_ik(
            j1,
            x=shared_x,
            y=shared_y,
            z=shared_z,
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
        )

        self.job_queue.append((j2, 0.6, False))

        qx, qy, qz, qw = quaternion_from_euler(deg_to_rad(6.0), math.pi, math.pi / 2)
        j3 = self.ik_planner.compute_ik(
            j2,
            x=shared_x,
            y=shared_y,
            z=shared_z,
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
        )

        self.job_queue.append((j3, 0.4, True))
    
        qx, qy, qz, qw = quaternion_from_euler(0.0, math.pi, 0)
        j4 = self.ik_planner.compute_ik(
            j3,
            x=shared_x,
            y=shared_y,
            z=shared_z,
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
        )
        self.job_queue.append((j4, 0.6, False))

        self.execute_jobs()

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
            req.frame_name = "arm"
            req.marker_size = 0.15
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
        else:
            self.get_logger().error("Unknown job type.")
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
    node = UR7e_Calibrate()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()
