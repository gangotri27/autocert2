#!/usr/bin/env python3
"""
MoveIt Adapter — Robot-Agnostic Motion Execution via Action Client
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool
import numpy as np

from autocert_interfaces.action import MoveToPose
from .base_adapter import RobotAdapter
from tf2_ros import Buffer, TransformListener

class MoveItAdapter(RobotAdapter):

    def __init__(self, node: Node, params: dict = None):
        self.node = node
        params = params or {}

        self._planning_group = params.get('planning_group', 'arm')
        self._ee_link = params.get('end_effector_link', 'tool0')
        self._ref_frame = params.get('reference_frame', 'base_link')
        self._planner_id = params.get('planner_id', 'RRTConnect')
        self._planning_time = float(params.get('planning_time', 5.0))
        self._vel_scale = float(params.get('velocity_scaling', 0.3))
        self._acc_scale = float(params.get('acceleration_scaling', 0.3))
        self._settling_time = float(params.get('settling_time_s', 2.0))
        self._motion_timeout = float(params.get('motion_timeout_s', 30.0))

        self._connected = False
        self._motion_done = False
        self._motion_succeeded = False

        self._action_client = ActionClient(node, MoveToPose, '/move_to_pose')
        self._motion_done_sub = node.create_subscription(Bool, '/motion_done', self._motion_done_cb, 10)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)

    def connect(self) -> bool:
        self.node.get_logger().info('Waiting for /move_to_pose action server...')
        timeout_sec = 30.0
        start = time.time()
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            if time.time() - start > timeout_sec:
                return False
        self._connected = True
        return True

    def move_to_pose(self, pose: Pose, timeout: float = None):
        if not self._connected: return False

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
        pose_stamped.header.frame_id = self._ref_frame
        pose_stamped.pose = pose

        goal = MoveToPose.Goal()
        goal.target_pose = pose_stamped
        goal.planning_group = self._planning_group
        goal.end_effector_link = self._ee_link
        goal.planner_id = self._planner_id
        goal.planning_time = self._planning_time
        goal.velocity_scaling = self._vel_scale
        goal.acceleration_scaling = self._acc_scale
        goal.execute_immediately = True

        self._motion_done = False
        self._motion_succeeded = False

        send_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=10.0)

        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted: return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=timeout or self._motion_timeout)

        if result_future.result() is None: return False

        action_result = result_future.result().result
        if action_result.success:
            self._motion_succeeded = True
            # CRITICAL FIX: Return a tuple including times to fix the 0.0 CSV issue!
            return (True, getattr(action_result, 'planning_time_ms', 0.0), getattr(action_result, 'execution_time_ms', 0.0))
        else:
            return (False, 0.0, 0.0)

    def wait_motion_complete(self, timeout: float = 30.0) -> bool:
        start = time.time()
        while not self._motion_done and (time.time() - start) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self._motion_done and self._motion_succeeded

    def get_current_pose(self) -> Pose:
        try:
            transform = self._tf_buffer.lookup_transform(self._ref_frame, self._ee_link, rclpy.time.Time())
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
            return pose
        except Exception: return Pose()

    def get_measurement(self) -> Pose:
        positions, orientations = [],[]
        for _ in range(20):
            try:
                t = self._tf_buffer.lookup_transform(self._ref_frame, self._ee_link, rclpy.time.Time())
                positions.append([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
                orientations.append([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])
            except Exception: pass
            rclpy.spin_once(self.node, timeout_sec=0.05)

        if len(positions) < 5: return None

        avg_pos = np.mean(positions, axis=0)
        avg_quat = np.mean(orientations, axis=0)
        avg_quat /= np.linalg.norm(avg_quat)

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = float(avg_pos[0]), float(avg_pos[1]), float(avg_pos[2])
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = float(avg_quat[0]), float(avg_quat[1]), float(avg_quat[2]), float(avg_quat[3])
        return pose

    def disconnect(self):
        self._connected = False

    def _motion_done_cb(self, msg: Bool):
        self._motion_done = True
        self._motion_succeeded = msg.data