import rclpy
import time
import numpy as np
from typing import Optional

from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState

from tf2_ros import Buffer, TransformListener
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState

from .base_adapter import RobotAdapter


class SimRobotAdapter(RobotAdapter):

    def __init__(self, node: Node):
        self.node = node

        self.base_frame = 'link1'
        self.ee_frame = 'end_effector_link'

        self.joint_names = [
            'joint1', 'joint2', 'joint3',
            'joint4'
        ]

        self.client = ActionClient(
            node,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        self.ik_client = node.create_client(
            GetPositionIK,
            '/compute_ik'
        )

        self.current_joints = None

        node.create_subscription(
            JointState,
            '/joint_states',
            self.joint_cb,
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

    # ---------------- CONNECT ----------------
    def connect(self):
        self.client.wait_for_server()
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Waiting for IK service...")
        return True

    # ---------------- JOINT STATE ----------------
    def joint_cb(self, msg):
        pos = []
        for j in self.joint_names:
            if j in msg.name:
                pos.append(msg.position[msg.name.index(j)])
        if len(pos) == len(self.joint_names):
            self.current_joints = pos

    # ---------------- IK ----------------
    def compute_ik(self, pose: Pose):
        if self.current_joints is None:
            return None

        req = GetPositionIK.Request()
        req.ik_request.group_name = "arm"

        state = RobotState()
        state.joint_state.name = self.joint_names
        state.joint_state.position = list(self.current_joints)
        req.ik_request.robot_state = state

        req.ik_request.pose_stamped.header.frame_id = self.base_frame
        req.ik_request.pose_stamped.pose = pose

        future = self.ik_client.call_async(req)

        start = self.node.get_clock().now()
        while not future.done():
            if (self.node.get_clock().now() - start).nanoseconds > 5e9:
                return None
            rclpy.spin_once(self.node, timeout_sec=0.1)

        res = future.result()
        if res and res.error_code.val == 1:
            sol = []
            for j in self.joint_names:
                idx = res.solution.joint_state.name.index(j)
                sol.append(res.solution.joint_state.position[idx])
            return sol

        return None

    # ---------------- EXECUTE ----------------
    def execute(self, joints):
        goal = FollowJointTrajectory.Goal()

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        pt = JointTrajectoryPoint()
        pt.positions = list(joints)
        pt.time_from_start.sec = 5

        traj.points = [pt]
        goal.trajectory = traj

        before = np.array(self.current_joints)

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)

        handle = future.result()
        if not handle or not handle.accepted:
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)

        after = np.array(self.current_joints)

        return np.linalg.norm(after - before) > 1e-4

    # ---------------- MOVE ----------------
    def move_to_pose(self, pose: Pose) -> bool:

        x, y, z = pose.position.x, pose.position.y, pose.position.z

        # 🔥 RANDOM BACKOFF
        for _ in range(5):
            offset = np.random.uniform(-0.05, 0.05, 3)
            back = Pose()
            back.position.x = x + offset[0]
            back.position.y = y + offset[1]
            back.position.z = z + offset[2]
            back.orientation.w = 1.0

            j = self.compute_ik(back)
            if j and self.execute(j):
                break

        # 🔥 TARGET
        target_j = self.compute_ik(pose)
        if not target_j:
            return False

        if not self.execute(target_j):
            return False

        time.sleep(1.5)
        return True

    def is_done(self):
        return True

    def wait_motion_complete(self):
        return True

    # ---------------- MEASUREMENT ----------------
    def get_measurement(self):

        pts = []
        rots = []

        for _ in range(20):
            try:
                t = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.ee_frame,
                    rclpy.time.Time()
                )

                pts.append([
                    t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z
                ])

                rots.append([
                    t.transform.rotation.x,
                    t.transform.rotation.y,
                    t.transform.rotation.z,
                    t.transform.rotation.w
                ])

            except:
                pass

            rclpy.spin_once(self.node, timeout_sec=0.05)

        if len(pts) < 5:
            return None

        avg_p = np.mean(pts, axis=0)
        avg_q = np.mean(rots, axis=0)
        avg_q = avg_q / np.linalg.norm(avg_q)

        pose = Pose()
        pose.position.x = avg_p[0]
        pose.position.y = avg_p[1]
        pose.position.z = avg_p[2]

        # ✅ IMPORTANT: keep real orientation
        pose.orientation.x = avg_q[0]
        pose.orientation.y = avg_q[1]
        pose.orientation.z = avg_q[2]
        pose.orientation.w = avg_q[3]

        return pose

    def get_current_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                rclpy.time.Time()
            )

            pose = Pose()
            pose.position.x = t.transform.translation.x
            pose.position.y = t.transform.translation.y
            pose.position.z = t.transform.translation.z
            pose.orientation = t.transform.rotation

            return pose

        except Exception as e:
            self.node.get_logger().error(f"TF error: {e}")
            return None

    def disconnect(self):
        self.node.get_logger().info("Sim adapter disconnected")