#!/usr/bin/env python3

import rclpy
import time
import numpy as np
import csv
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

class MinimalISOTest(Node):

    def __init__(self):
        super().__init__('minimal_iso_test')

        self.client = ActionClient(self, MoveGroup, 'move_action')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.base_frame = 'link1'
        self.ee_frame = 'end_effector_link'

        self.num_reps = 10
        self.output_file = 'iso_minimal_results.csv'

        self.get_logger().info("🚀 Starting Minimal ISO Test...")
        self.client.wait_for_server()

        # Simple 5 test poses (you can change later)
        self.poses = [
            self.make_pose(0.20, 0.0, 0.22),
            self.make_pose(0.23, 0.0, 0.22),
            self.make_pose(0.17, 0.0, 0.22),
            self.make_pose(0.20, 0.03, 0.22),
            self.make_pose(0.20, -0.03, 0.22),
        ]

        self.init_csv()

    def init_csv(self):
        with open(self.output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'pose_id', 'rep',
                'cmd_x', 'cmd_y', 'cmd_z',
                'meas_x', 'meas_y', 'meas_z',
                'pos_error_mm'
            ])

    def make_pose(self, x, y, z):
        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = z
        p.orientation.w = 1.0
        return p

    def move_to_pose(self, pose, tolerance=0.005):
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 3.0

        pos = PositionConstraint()
        pos.header.frame_id = self.base_frame
        pos.link_name = self.ee_frame

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [tolerance]

        pos.constraint_region.primitives.append(sphere)
        pos.constraint_region.primitive_poses.append(pose)

        goal.request.goal_constraints.append(Constraints(position_constraints=[pos]))

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()

        if not handle or not handle.accepted:
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        return result.error_code.val == 1

    def get_measured_point(self):
        samples = []
        for _ in range(10):
            try:
                t = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.ee_frame,
                    rclpy.time.Time()
                )
                samples.append([
                    t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z
                ])
            except:
                pass
            time.sleep(0.05)

        if not samples:
            return None

        return np.mean(samples, axis=0)

    def run(self):
        final_ap = []
        final_rp = []

        for i, target in enumerate(self.poses):
            self.get_logger().info(f"\n--- Pose {i} ---")

            samples = []
            cmd = np.array([
                target.position.x,
                target.position.y,
                target.position.z
            ])

            for rep in range(self.num_reps):

                # Random back-off (5 cm)
                vec = np.random.normal(size=3)
                vec = (vec / np.linalg.norm(vec)) * 0.05

                backoff = self.make_pose(
                    cmd[0] + vec[0],
                    cmd[1] + vec[1],
                    cmd[2] + vec[2]
                )

                self.move_to_pose(backoff, tolerance=0.01)

                # Move to target
                self.get_logger().info(f" Rep {rep+1}")
                if not self.move_to_pose(target):
                    continue

                time.sleep(1.5)

                meas = self.get_measured_point()
                if meas is not None:
                    samples.append(meas)

                    error = np.linalg.norm(meas - cmd) * 1000

                    with open(self.output_file, 'a') as f:
                        csv.writer(f).writerow([
                            i, rep,
                            cmd[0], cmd[1], cmd[2],
                            meas[0], meas[1], meas[2],
                            error
                        ])

            if len(samples) < 3:
                self.get_logger().error("Not enough samples")
                continue

            samples = np.array(samples)

            barycenter = np.mean(samples, axis=0)

            # Accuracy (AP)
            ap = np.linalg.norm(barycenter - cmd)

            # Repeatability (RP)
            dists = np.linalg.norm(samples - barycenter, axis=1)
            rp = np.mean(dists) + 3 * np.std(dists, ddof=1)

            self.get_logger().info(f" AP: {ap*1000:.3f} mm")
            self.get_logger().info(f" RP: {rp*1000:.3f} mm")

            final_ap.append(ap * 1000)
            final_rp.append(rp * 1000)

        if final_ap:
            self.get_logger().info("\n=== FINAL RESULTS ===")
            self.get_logger().info(f"Mean AP: {np.mean(final_ap):.3f} mm")
            self.get_logger().info(f"Mean RP: {np.mean(final_rp):.3f} mm")


def main():
    rclpy.init()
    node = MinimalISOTest()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()