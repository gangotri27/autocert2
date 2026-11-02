#!/usr/bin/env python3

import rclpy
import time
import numpy as np
import csv
import random
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState

class FixedJTISO(Node):

    def __init__(self):
        super().__init__('fixed_jt_iso')

        # Use sim time
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        # Clients
        self.client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')

        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

        # Joint state tracking
        self.current_joints = None
        self.create_subscription(
            JointState, '/joint_states', self.joint_cb, 10)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.base_frame = 'link1'
        self.ee_frame = 'end_effector_link'
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5_roll']

        # Experiment
        self.num_reps = 10
        self.output_file = 'iso_results.csv'

        self.get_logger().info("🚀 Fixed JT ISO Test Ready")

    def joint_cb(self, msg):
        pos = []
        for j in self.joint_names:
            if j in msg.name:
                pos.append(msg.position[msg.name.index(j)])
        if len(pos) == len(self.joint_names):
            self.current_joints = pos

    def make_pose(self, x, y, z):
        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = z
        p.orientation.w = 1.0
        return p

    # ---------------- IK ----------------
    def compute_ik(self, pose):
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

        start = self.get_clock().now()
        while not future.done():
            if (self.get_clock().now() - start).nanoseconds > 5e9:
                return None
            rclpy.spin_once(self, timeout_sec=0.1)

        res = future.result()
        if res and res.error_code.val == 1:
            sol = []
            for j in self.joint_names:
                idx = res.solution.joint_state.name.index(j)
                sol.append(res.solution.joint_state.position[idx])
            return sol
        return None

    # ---------------- EXECUTION ----------------
    def execute(self, joints):
        goal = FollowJointTrajectory.Goal()

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        pt = JointTrajectoryPoint()
        pt.positions = list(joints)
        pt.time_from_start.sec = 5  # IMPORTANT

        traj.points = [pt]
        goal.trajectory = traj

        before = np.array(self.current_joints)

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()
        if not handle or not handle.accepted:
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        after = np.array(self.current_joints)

        # 🔥 Motion validation
        moved = np.linalg.norm(after - before) > 1e-4
        if not moved:
            self.get_logger().error("⚠️ Robot did NOT move!")
            return False

        return True

    # ---------------- TF ----------------
    def get_point(self):
        pts = []
        for _ in range(10):
            try:
                t = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.ee_frame,
                    rclpy.time.Time())
                pts.append([
                    t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z
                ])
            except:
                pass
            rclpy.spin_once(self, timeout_sec=0.05)

        if len(pts) < 5:
            return None

        return np.mean(pts, axis=0)

    # ---------------- RUN ----------------
    def run(self):

        self.client.wait_for_server()

        while self.current_joints is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        target = self.make_pose(0.20, 0.0, 0.22)
        cmd = np.array([target.position.x, target.position.y, target.position.z])

        samples = []

        for rep in range(self.num_reps):

            self.get_logger().info(f"Rep {rep+1}")

            # 🔥 RANDOM BACK-OFF
            for _ in range(5):
                offset = np.random.uniform(-0.05, 0.05, 3)
                back = self.make_pose(
                    cmd[0] + offset[0],
                    cmd[1] + offset[1],
                    cmd[2] + offset[2]
                )
                j = self.compute_ik(back)
                if j and self.execute(j):
                    break

            # 🔥 TARGET MOVE
            target_j = self.compute_ik(target)
            if not target_j:
                continue

            if not self.execute(target_j):
                continue

            # Wait a bit
            time.sleep(1.5)

            p = self.get_point()
            if p is not None:
                samples.append(p)
                err = np.linalg.norm(p - cmd) * 1000
                self.get_logger().info(f" → Error: {err:.2f} mm")

        # ---------------- CALC ----------------
        if len(samples) < 3:
            self.get_logger().error("Not enough samples")
            return

        samples = np.array(samples)
        bary = np.mean(samples, axis=0)

        ap = np.linalg.norm(bary - cmd) * 1000
        dists = np.linalg.norm(samples - bary, axis=1)
        rp = (np.mean(dists) + 3*np.std(dists, ddof=1)) * 1000

        self.get_logger().info(f"\nAP: {ap:.3f} mm")
        self.get_logger().info(f"RP: {rp:.3f} mm")


def main():
    rclpy.init()
    node = FixedJTISO()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()