#!/usr/bin/env python3

import rclpy
import time
import numpy as np
import csv
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState

from tf2_ros import Buffer, TransformListener
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState


class AutoCertISO(Node):

    def __init__(self):
        super().__init__('autocert_iso')

        # ---------------- CONFIG ----------------
        self.num_reps = 10
        self.output_file = 'iso_multi_pose_with_orientation.csv'

        self.base_frame = 'link1'
        self.ee_frame = 'end_effector_link'

        self.joint_names = [
            'joint1', 'joint2', 'joint3',
            'joint4', 'joint5_roll'
        ]

        # 🔥 ISO-style distributed poses
        self.poses = [
            (0.20,  0.00, 0.22),
            (0.25,  0.05, 0.20),
            (0.18, -0.05, 0.25),
            (0.22,  0.08, 0.18),
            (0.15, -0.02, 0.30),
        ]

        # ---------------- ROS SETUP ----------------
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')

        self.ik_client = self.create_client(
            GetPositionIK, '/compute_ik')

        self.current_joints = None
        self.create_subscription(
            JointState, '/joint_states',
            self.joint_cb, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self)

        self.get_logger().info("🚀 AutoCert ISO (Position + Orientation) Ready")

        self.init_csv()

    # ---------------- CSV ----------------
    def init_csv(self):
        with open(self.output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'pose_id', 'rep',
                'cmd_x', 'cmd_y', 'cmd_z',
                'meas_x', 'meas_y', 'meas_z',
                'pos_error_mm',
                'ori_error_deg'
            ])

    def log_csv(self, pose_id, rep, cmd, meas, pos_err, ori_err):
        with open(self.output_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                pose_id, rep,
                cmd[0], cmd[1], cmd[2],
                meas[0], meas[1], meas[2],
                pos_err,
                ori_err
            ])

    # ---------------- JOINT STATE ----------------
    def joint_cb(self, msg):
        pos = []
        for j in self.joint_names:
            if j in msg.name:
                pos.append(msg.position[msg.name.index(j)])
        if len(pos) == len(self.joint_names):
            self.current_joints = pos

    # ---------------- POSE ----------------
    def make_pose(self, x, y, z):
        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = z
        p.orientation.x = 0.0
        p.orientation.y = 0.0
        p.orientation.z = 0.0
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
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()
        if not handle or not handle.accepted:
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        after = np.array(self.current_joints)

        moved = np.linalg.norm(after - before) > 1e-4
        return moved

    # ---------------- TF (POSITION + ORIENTATION) ----------------
    def get_pose(self):
        pts = []
        rots = []

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

                rots.append([
                    t.transform.rotation.x,
                    t.transform.rotation.y,
                    t.transform.rotation.z,
                    t.transform.rotation.w
                ])

            except:
                pass

            rclpy.spin_once(self, timeout_sec=0.05)

        if len(pts) < 5:
            return None, None

        return np.mean(pts, axis=0), np.mean(rots, axis=0)

    # ---------------- ORIENTATION ERROR ----------------
    def quat_error_deg(self, q1, q2):
        q1 = np.array(q1) / np.linalg.norm(q1)
        q2 = np.array(q2) / np.linalg.norm(q2)

        dot = np.clip(np.abs(np.dot(q1, q2)), -1.0, 1.0)
        angle = 2 * np.arccos(dot)

        return np.degrees(angle)

    # ---------------- RUN ----------------
    def run(self):

        self.client.wait_for_server()

        while self.current_joints is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        all_results = {}

        for pose_id, (x, y, z) in enumerate(self.poses):

            self.get_logger().info(f"\n🎯 Pose {pose_id}")

            target = self.make_pose(x, y, z)
            cmd = np.array([x, y, z])
            cmd_q = np.array([0, 0, 0, 1])

            samples = []
            ori_samples = []

            for rep in range(self.num_reps):

                self.get_logger().info(f"  Rep {rep+1}")

                # RANDOM BACKOFF
                for _ in range(5):
                    offset = np.random.uniform(-0.05, 0.05, 3)
                    back = self.make_pose(
                        x + offset[0],
                        y + offset[1],
                        z + offset[2]
                    )
                    j = self.compute_ik(back)
                    if j and self.execute(j):
                        break

                # TARGET MOVE
                target_j = self.compute_ik(target)
                if not target_j:
                    continue

                if not self.execute(target_j):
                    continue

                time.sleep(1.5)

                p, q = self.get_pose()
                if p is None:
                    continue

                pos_err = np.linalg.norm(p - cmd) * 1000
                ori_err = self.quat_error_deg(cmd_q, q)

                self.log_csv(pose_id, rep, cmd, p, pos_err, ori_err)

                samples.append(p)
                ori_samples.append(q)

                self.get_logger().info(
                    f"   → Pos: {pos_err:.2f} mm | Ori: {ori_err:.2f} deg"
                )

            if len(samples) < 3:
                continue

            # ---------- POSITION METRICS ----------
            samples = np.array(samples)
            bary = np.mean(samples, axis=0)

            ap = np.linalg.norm(bary - cmd) * 1000
            dists = np.linalg.norm(samples - bary, axis=1)
            rp = (np.mean(dists) + 3*np.std(dists, ddof=1)) * 1000

            # ---------- ORIENTATION METRICS ----------
            ori_samples = np.array(ori_samples)
            ori_samples = np.array([q/np.linalg.norm(q) for q in ori_samples])

            q_mean = np.mean(ori_samples, axis=0)
            q_mean = q_mean / np.linalg.norm(q_mean)

            ao = self.quat_error_deg(cmd_q, q_mean)

            ori_dists = [self.quat_error_deg(q_mean, q) for q in ori_samples]
            ro = np.mean(ori_dists) + 3*np.std(ori_dists, ddof=1)

            self.get_logger().info(
                f"📊 Pose {pose_id} → AP: {ap:.3f} mm | RP: {rp:.3f} mm | AO: {ao:.3f} deg | RO: {ro:.3f} deg"
            )

            all_results[pose_id] = (ap, rp, ao, ro)

        # ---------- FINAL SUMMARY ----------
        if all_results:

            aps = [v[0] for v in all_results.values()]
            rps = [v[1] for v in all_results.values()]
            aos = [v[2] for v in all_results.values()]
            ros = [v[3] for v in all_results.values()]

            self.get_logger().info("\n🏁 FINAL RESULT")
            self.get_logger().info(f"Mean AP: {np.mean(aps):.3f} mm")
            self.get_logger().info(f"Mean RP: {np.mean(rps):.3f} mm")
            self.get_logger().info(f"Mean AO: {np.mean(aos):.3f} deg")
            self.get_logger().info(f"Mean RO: {np.mean(ros):.3f} deg")


def main():
    rclpy.init()
    node = AutoCertISO()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()