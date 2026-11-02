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

class UltimateISOTest(Node):

    def __init__(self):
        super().__init__('ultimate_iso_test')

        # 1. Action Client (Controller)
        self.client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

        # 2. IK Service
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

        # 3. Joint State Subscription
        self.current_joints = None
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.base_frame = 'link1'
        self.ee_frame = 'end_effector_link'
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5_roll']

        self.num_reps = 10
        self.output_file = 'iso_final_results.csv'

        # Home Pose (The "Away" move to reset approach for every rep)
        self.home_pose = self.make_pose(0.12, 0.0, 0.20)

        self.client.wait_for_server()
        self.ik_client.wait_for_service()

        self.poses = [
            self.make_pose(0.20, 0.0, 0.22),
            self.make_pose(0.23, 0.0, 0.22),
            self.make_pose(0.17, 0.0, 0.22),
            self.make_pose(0.20, 0.03, 0.22),
            self.make_pose(0.20, -0.03, 0.22),
        ]
        self.init_csv()

    def joint_cb(self, msg):
        positions = []
        for name in self.joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])
        if len(positions) == len(self.joint_names):
            self.current_joints = positions

    def init_csv(self):
        with open(self.output_file, 'w', newline='') as f:
            csv.writer(f).writerow(['pose_id', 'rep', 'cmd_x', 'cmd_y', 'cmd_z', 'meas_x', 'meas_y', 'meas_z'])

    def make_pose(self, x, y, z):
        p = Pose()
        p.position.x, p.position.y, p.position.z = x, y, z
        p.orientation.w = 1.0
        return p

    def compute_ik(self, pose):
        req = GetPositionIK.Request()
        req.ik_request.group_name = "arm"
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped.header.frame_id = self.base_frame
        req.ik_request.pose_stamped.pose = pose
        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        res = future.result()
        if res and res.error_code.val == 1:
            positions = []
            for name in self.joint_names:
                idx = res.solution.joint_state.name.index(name)
                positions.append(res.solution.joint_state.position[idx])
            return positions
        return None

    def send_trajectory(self, target_joints):
        if self.current_joints is None: return False
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = self.joint_names

        p1 = JointTrajectoryPoint()
        p1.positions = [float(p) for p in self.current_joints]
        p1.time_from_start.nanosec = 0 
        traj.points.append(p1)

        p2 = JointTrajectoryPoint()
        p2.positions = [float(p) for p in target_joints]
        p2.time_from_start.sec = 2 
        traj.points.append(p2)

        goal.trajectory = traj
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        handle = future.result()
        if not handle or not handle.accepted: return False
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future, timeout_sec=5.0)
        return True

    def get_measured_point(self):
        samples = []
        for _ in range(10):
            try:
                t = self.tf_buffer.lookup_transform(self.base_frame, self.ee_frame, rclpy.time.Time())
                samples.append([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
            except: pass
            time.sleep(0.05)
        return np.mean(samples, axis=0) if samples else None

    def run(self):
        summary_ap = []
        summary_rp = []

        for i, target in enumerate(self.poses):
            self.get_logger().info(f"\n--- Testing Pose {i} ---")
            samples = []
            cmd_xyz = np.array([target.position.x, target.position.y, target.position.z])

            for rep in range(self.num_reps):
                # 1. MOVE AWAY (Home)
                home_joints = self.compute_ik(self.home_pose)
                self.send_trajectory(home_joints)
                time.sleep(1.0)

                # 2. MOVE TO TARGET
                self.get_logger().info(f" Rep {rep+1}: Moving to target...")
                joints = self.compute_ik(target)
                if joints and self.send_trajectory(joints):
                    time.sleep(2.0) # Wait to settle
                    meas = self.get_measured_point()
                    if meas is not None:
                        samples.append(meas)
                        with open(self.output_file, 'a') as f:
                            csv.writer(f).writerow([i, rep, cmd_xyz[0], cmd_xyz[1], cmd_xyz[2], meas[0], meas[1], meas[2]])
                else:
                    self.get_logger().error(f" Rep {rep+1}: IK or Motion Failed.")

            # --- CALCULATE ACCURACY & PRECISION FOR THIS POSE ---
            if len(samples) >= 3:
                samples = np.array(samples)
                barycenter = np.mean(samples, axis=0)
                
                # Accuracy (AP): Distance from target to barycenter
                ap = np.linalg.norm(barycenter - cmd_xyz)
                
                # Precision/Repeatability (RP): Statistical spread around barycenter
                dists = np.linalg.norm(samples - barycenter, axis=1)
                rp = np.mean(dists) + 3 * np.std(dists, ddof=1)

                self.get_logger().info(f" >> Accuracy (AP): {ap*1000:.3f} mm")
                self.get_logger().info(f" >> Precision (RP): {rp*1000:.3f} mm")
                summary_ap.append(ap*1000)
                summary_rp.append(rp*1000)

        # --- FINAL SUMMARY ---
        self.get_logger().info("\n" + "="*30)
        self.get_logger().info("FINAL ISO 9283 RESULTS")
        self.get_logger().info(f"Mean Accuracy (AP): {np.mean(summary_ap):.3f} mm")
        self.get_logger().info(f"Mean Precision (RP): {np.mean(summary_rp):.3f} mm")
        self.get_logger().info("="*30)

def main():
    rclpy.init()
    node = UltimateISOTest()
    node.run()
    rclpy.shutdown()

if __name__ == "__main__":
    main()