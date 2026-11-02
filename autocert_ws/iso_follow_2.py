#!/usr/bin/env python3

import rclpy
import time
import numpy as np
import csv
import threading
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState

class SimClockISOTest(Node):

    def __init__(self):
        super().__init__('sim_clock_iso_test')

        # Use simulation time if available
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        self.current_joints = None
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.base_frame = 'link1'
        self.ee_frame = 'end_effector_link' 
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5_roll']

        self.num_reps = 10
        self.output_file = 'iso_results.csv'
        self.home_pose = self.make_pose(0.12, 0.05, 0.15)
        
        self.get_logger().info("Sim-Clock-Synced System Initialized.")

    def joint_cb(self, msg):
        pos = []
        for name in self.joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                pos.append(msg.position[idx])
        if len(pos) == len(self.joint_names):
            self.current_joints = pos

    def make_pose(self, x, y, z):
        p = Pose()
        p.position.x, p.position.y, p.position.z = x, y, z
        p.orientation.w = 1.0
        return p

    def compute_ik(self, pose):
        if self.current_joints is None: return None
        req = GetPositionIK.Request()
        req.ik_request.group_name = "arm"
        state = RobotState()
        state.joint_state.name = self.joint_names
        state.joint_state.position = [float(p) for p in self.current_joints]
        req.ik_request.robot_state = state
        req.ik_request.pose_stamped.header.frame_id = self.base_frame
        req.ik_request.pose_stamped.pose = pose
        
        future = self.ik_client.call_async(req)
        start_t = self.get_clock().now()
        while rclpy.ok() and not future.done():
            if (self.get_clock().now() - start_t).nanoseconds > 5e9: # 5s sim-time
                return None
            time.sleep(0.1)
        res = future.result()
        if res and res.error_code.val == 1:
            sol = []
            for name in self.joint_names:
                idx = res.solution.joint_state.name.index(name)
                sol.append(res.solution.joint_state.position[idx])
            return sol
        return None

    def execute_move(self, target_joints):
        """Waits for completion using Simulation Clock (Lag-Proof)."""
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = self.joint_names

        p = JointTrajectoryPoint()
        p.positions = [float(j) for j in target_joints]
        p.time_from_start.sec = 3 
        traj.points = [p]
        goal.trajectory = traj
        
        send_goal_future = self.client.send_goal_async(goal)
        start_t = self.get_clock().now()
        
        # 1. Wait for Goal Acceptance
        while rclpy.ok() and not send_goal_future.done():
            if (self.get_clock().now() - start_t).nanoseconds > 10e9: 
                return False
            time.sleep(0.1)
        
        handle = send_goal_future.result()
        if not handle.accepted: return False

        # 2. Wait for Completion based on Simulation Time
        res_future = handle.get_result_async()
        start_exec_t = self.get_clock().now()
        while rclpy.ok() and not res_future.done():
            # Wait for 15 seconds of SIMULATION time (allows for huge lag)
            if (self.get_clock().now() - start_exec_t).nanoseconds > 15e9: 
                self.get_logger().error("Sim-Time Execution Timeout!")
                return False
            time.sleep(0.1)
        return True

    def get_measured_point(self):
        samples = []
        for _ in range(10):
            try:
                t = self.tf_buffer.lookup_transform(self.base_frame, self.ee_frame, rclpy.time.Time())
                samples.append([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
            except: pass
            time.sleep(0.1)
        return np.mean(samples, axis=0) if samples else None

    def run_experiment(self):
        try:
            self.get_logger().info("Waiting for data (Ensure Gazebo is unpaused)...")
            self.client.wait_for_server()
            while self.current_joints is None: time.sleep(0.1)
            
            # Simple test pose
            poses = [self.make_pose(0.20, 0.0, 0.22)]
            
            for i, target_pose in enumerate(poses):
                self.get_logger().info(f"--- POSE {i} ---")
                samples = []
                cmd_xyz = np.array([target_pose.position.x, target_pose.position.y, target_pose.position.z])

                for rep in range(self.num_reps):
                    self.get_logger().info(f" Rep {rep+1}: Moving...")
                    
                    # Move to Target directly (Simplified for lag testing)
                    target_j = self.compute_ik(target_pose)
                    if target_j and self.execute_move(target_j):
                        time.sleep(2.0) # Settle
                        p = self.get_measured_point()
                        if p is not None:
                            samples.append(p)
                            err = np.linalg.norm(p - cmd_xyz) * 1000
                            self.get_logger().info(f" Success: {err:.2f}mm")
                    
                    # Then Move Away (to force repeatability check)
                    home_j = self.compute_ik(self.home_pose)
                    if home_j: self.execute_move(home_j)

                if len(samples) >= 3:
                    samples = np.array(samples)
                    bary = np.mean(samples, axis=0)
                    ap = np.linalg.norm(bary - cmd_xyz) * 1000
                    dists = np.linalg.norm(samples - bary, axis=1)
                    rp = (np.mean(dists) + 3 * np.std(dists, ddof=1)) * 1000
                    self.get_logger().info(f" >> AP: {ap:.3f} mm | RP: {rp:.3f} mm")

        except Exception as e:
            self.get_logger().error(f"Experiment Error: {e}")
        finally:
            rclpy.shutdown()

def main():
    rclpy.init()
    node = SimClockISOTest()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=node.run_experiment)
    thread.start()
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()

if __name__ == "__main__":
    main()