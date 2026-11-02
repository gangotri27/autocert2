#!/usr/bin/env python3

import rclpy
import time
import numpy as np
import csv
import random
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R

class UltimateResearchCertifier(Node):

    def __init__(self):
        super().__init__('ultimate_research_certifier')

        # 1. REPRODUCIBILITY SEED
        np.random.seed(42)
        random.seed(42)

        self.client = ActionClient(self, MoveGroup, 'move_group')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.base_frame = 'link1'
        self.ee_frame = 'end_effector_link'
        
        # --- ISO EXPERIMENT SETTINGS ---
        self.num_reps = 10 
        self.cube_center = [0.20, 0.0, 0.22] 
        self.cube_side = 0.06                
        self.pos_stability_thresh = 0.0003    # 0.3mm
        self.ori_stability_thresh = 0.001     # ~0.05 degrees
        self.output_file = 'iso_9283_final_audit.csv'

        self.get_logger().info("💎 Initializing Gold-Standard ISO 9283 Engine...")
        self.client.wait_for_server()

        # Generate 3D Distributed ISO Cube
        self.test_poses = self.generate_3d_iso_cube(self.cube_center, self.cube_side)
        self.results_accumulator = []
        self.init_csv()

    def generate_3d_iso_cube(self, center, L):
        c = np.array(center)
        d = L / 2.0
        # ISO 9283 Poses: Center + 4 vertices distributed across volume
        coords = [
            c,                             # P0: Center
            c + [ d,  d,  d],              # P1: Top-Front-Right
            c + [ d, -d, -d],              # P2: Bottom-Front-Left
            c + [-d, -d,  d],              # P3: Top-Back-Left
            c + [-d,  d, -d]               # P4: Bottom-Back-Right
        ]
        poses = []
        for coord in coords:
            p = Pose()
            p.position.x, p.position.y, p.position.z = coord
            p.orientation.w = 1.0 
            poses.append(p)
        return poses

    def init_csv(self):
        with open(self.output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'pose_id', 'rep', 'success', 'error_code',
                'cmd_x', 'cmd_y', 'cmd_z', 'meas_x', 'meas_y', 'meas_z', 
                'internal_plan_time', 'total_exec_time', 'pos_err_mm', 'ori_err_deg'
            ])

    def move_to_pose(self, pose, tolerance=0.003):
        """Moves with precise error auditing and timing."""
        start_time = time.time()
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.num_planning_attempts = 15
        goal.request.allowed_planning_time = 5.0
        
        pos_con = PositionConstraint()
        pos_con.header.frame_id = self.base_frame
        pos_con.link_name = self.ee_frame
        pos_con.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[tolerance]))
        pos_con.constraint_region.primitive_poses.append(pose)
        
        goal.request.goal_constraints.append(Constraints(position_constraints=[pos_con]))

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        
        if not handle or not handle.accepted:
            return False, -1, 0.0, 0.0

        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        
        result = res_future.result().result
        exec_time = time.time() - start_time
        
        # Extract native planning time from MoveIt (excludes ROS overhead)
        internal_plan_time = result.planning_time
        
        return (result.error_code.val == 1), result.error_code.val, internal_plan_time, exec_time

    def wait_for_se3_stability(self):
        """Monitors both Translational and Angular variance."""
        pos_history, ori_history = [], []
        for _ in range(60): # Max 3 seconds
            try:
                t = self.tf_buffer.lookup_transform(self.base_frame, self.ee_frame, rclpy.time.Time())
                pos_history.append([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
                ori_history.append([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])
                
                if len(pos_history) > 15:
                    # Position Jitter
                    pos_var = np.max(np.var(pos_history[-15:], axis=0))
                    
                    # Angular Jitter (SE3 Stability)
                    rots = R.from_quat(ori_history[-15:])
                    mean_rot = rots.mean()
                    ang_diffs = (mean_rot.inv() * rots).magnitude()
                    max_ang_jitter = np.max(ang_diffs)

                    if pos_var < (self.pos_stability_thresh**2) and max_ang_jitter < self.ori_stability_thresh:
                        return True
            except: pass
            time.sleep(0.05)
        return False

    def get_stabilized_pose(self):
        p_samples, o_samples = [], []
        for _ in range(25):
            try:
                t = self.tf_buffer.lookup_transform(self.base_frame, self.ee_frame, rclpy.time.Time())
                p_samples.append([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
                o_samples.append([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])
            except: pass
            time.sleep(0.04)
        if len(p_samples) < 15: return None
        return np.mean(p_samples, axis=0), R.from_quat(o_samples).mean()

    def run(self):
        stats = []

        for i, target in enumerate(self.test_poses):
            self.get_logger().info(f"--- 🔳 ISO POSE {i} ---")
            p_samples, q_samples = [], []
            cmd_xyz = np.array([target.position.x, target.position.y, target.position.z])
            cmd_r = R.from_quat([target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w])
            pose_successes = 0

            for rep in range(self.num_reps):
                # 1. SAFE RANDOM BACK-OFF (Retry until reachable)
                for _ in range(3): # Max 3 IK attempts for back-off
                    vec = np.random.normal(size=3)
                    vec = (vec / np.linalg.norm(vec)) * 0.05
                    back_p = Pose()
                    back_p.position.x, back_p.position.y, back_p.position.z = cmd_xyz + vec
                    back_p.orientation = target.orientation
                    if self.move_to_pose(back_p, tolerance=0.02)[0]: break

                # 2. MOVE TO TARGET
                self.get_logger().info(f" Rep {rep+1}: Executing...")
                ok, err, p_time, e_time = self.move_to_pose(target, tolerance=0.003)
                
                if ok:
                    self.wait_for_se3_stability()
                    meas = self.get_stabilized_pose()
                    if meas:
                        p_avg, r_avg = meas
                        p_samples.append(p_avg); q_samples.append(r_avg)
                        pose_successes += 1
                        
                        pos_err = np.linalg.norm(p_avg - cmd_xyz) * 1000
                        ori_err = (cmd_r.inv() * r_avg).magnitude() * (180.0 / np.pi)

                        with open(self.output_file, 'a') as f:
                            csv.writer(f).writerow([time.time(), i, rep, 1, err, cmd_xyz[0], cmd_xyz[1], cmd_xyz[2], p_avg[0], p_avg[1], p_avg[2], p_time, e_time, pos_err, ori_err])
                else:
                    self.get_logger().error(f" Move failed with code: {err}")
                    with open(self.output_file, 'a') as f:
                        csv.writer(f).writerow([time.time(), i, rep, 0, err, cmd_xyz[0], cmd_xyz[1], cmd_xyz[2], 0, 0, 0, p_time, e_time, 0, 0])

            # 3. STATISTICAL REDUCTION
            if len(p_samples) >= 3:
                p_samples = np.array(p_samples)
                bary_p = np.mean(p_samples, axis=0)
                ap = np.linalg.norm(bary_p - cmd_xyz)
                rp = np.mean(np.linalg.norm(p_samples - bary_p, axis=1)) + 3 * np.std(np.linalg.norm(p_samples - bary_p, axis=1), ddof=1)
                
                q_stack = R.from_quat([q.as_quat() for q in q_samples])
                bary_q = q_stack.mean()
                aq = (cmd_r.inv() * bary_q).magnitude() * (180.0 / np.pi)
                angles = (bary_q.inv() * q_stack).magnitude() * (180.0 / np.pi)
                rq = np.mean(angles) + 3 * np.std(angles, ddof=1)
                
                success_rate = (pose_successes / self.num_reps) * 100
                self.get_logger().info(f" >> Success: {success_rate}% | AP: {ap*1000:.3f}mm | RP: {rp*1000:.3f}mm")
                stats.append({'ap': ap*1000, 'rp': rp*1000, 'aq': aq, 'rq': rq, 'sr': success_rate})

        # 4. FINAL AUDIT
        self.get_logger().info("\n" + "💎"*25 + "\nFINAL ISO 9283 AUDIT REPORT\n" + f"Accuracy: {np.mean([x['ap'] for x in stats]):.3f}mm / {np.mean([x['aq'] for x in stats]):.3f}°\nRepeatability: {np.mean([x['rp'] for x in stats]):.3f}mm / {np.mean([x['rq'] for x in stats]):.3f}°\nOverall Success Rate: {np.mean([x['sr'] for x in stats]):.1f}%\n" + "💎"*25)

def main():
    rclpy.init(); node = UltimateResearchCertifier()
    try: node.run()
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()