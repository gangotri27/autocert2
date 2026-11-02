#!/usr/bin/env python3

import rclpy
import time
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

class SimpleISOTest(Node):

    def __init__(self):
        super().__init__('simple_iso_test')

        self.client = ActionClient(self, MoveGroup, 'move_action')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.base_frame = 'link1'
        self.ee_frame = 'end_effector_link'

        self.get_logger().info("Waiting for MoveIt...")
        self.client.wait_for_server()

        # 5 simple ISO-like poses (workspace spread)
        self.poses = [
            self.make_pose(0.20, 0.0, 0.22),
            self.make_pose(0.23, 0.0, 0.22),
            self.make_pose(0.17, 0.0, 0.22),
            self.make_pose(0.20, 0.03, 0.22),
            self.make_pose(0.20, -0.03, 0.22),
        ]

    def make_pose(self, x, y, z):
        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = z
        p.orientation.w = 1.0
        return p

    def move_to_pose(self, pose):
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 3.0
        goal.request.start_state.is_diff = True

        # small tolerance
        pos = PositionConstraint()
        pos.header.frame_id = self.base_frame
        pos.link_name = self.ee_frame

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.005]  # 5 mm

        pos.constraint_region.primitives.append(sphere)
        pos.constraint_region.primitive_poses.append(pose)
        pos.weight = 1.0

        ori = OrientationConstraint()
        ori.header.frame_id = self.base_frame
        ori.link_name = self.ee_frame
        ori.orientation = pose.orientation
        ori.absolute_x_axis_tolerance = 0.1
        ori.absolute_y_axis_tolerance = 0.1
        ori.absolute_z_axis_tolerance = 0.1
        ori.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos)
        constraints.orientation_constraints.append(ori)

        goal.request.goal_constraints.append(constraints)

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        return result.error_code.val == 1

    def get_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            return np.array([
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z
            ])
        except:
            return None

    def run(self):
        all_results = []

        for i, pose in enumerate(self.poses):
            self.get_logger().info(f"\n--- Pose {i} ---")

            samples = []

            for rep in range(5):
                success = self.move_to_pose(pose)

                if not success:
                    self.get_logger().warn("Motion failed")
                    continue

                time.sleep(2.0)  # settling

                p = self.get_pose()
                if p is not None:
                    samples.append(p)
                    self.get_logger().info(f"Rep {rep+1}: {p}")

            samples = np.array(samples)

            mean = np.mean(samples, axis=0)
            distances = np.linalg.norm(samples - mean, axis=1)

            rp = np.mean(distances) + 3 * np.std(distances)

            self.get_logger().info(f"Mean: {mean}")
            self.get_logger().info(f"Repeatability (RP): {rp*1000:.3f} mm")

            all_results.append(rp * 1000)

        self.get_logger().info("\n=== FINAL RESULT ===")
        self.get_logger().info(f"Mean RP: {np.mean(all_results):.3f} mm")


def main():
    rclpy.init()
    node = SimpleISOTest()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()