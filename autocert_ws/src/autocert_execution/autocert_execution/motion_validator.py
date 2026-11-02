#!/usr/bin/env python3
"""
Motion plan validation utilities.
Checks plans before execution to prevent unsafe motion.
"""

import numpy as np
from typing import List, Optional, Tuple
from geometry_msgs.msg import Pose, PoseStamped


class MotionValidator:
    """
    Validates motion plans and poses before execution.
    Robot-agnostic: operates on workspace bounds and joint limits
    loaded from parameters.
    """

    def __init__(
        self,
        workspace_bounds: dict,
        joint_min_limits: Optional[List[float]] = None,
        joint_max_limits: Optional[List[float]] = None,
    ):
        self._bounds = workspace_bounds
        self._joint_min = np.array(joint_min_limits) if joint_min_limits else None
        self._joint_max = np.array(joint_max_limits) if joint_max_limits else None

    def validate_pose_in_workspace(self, pose: Pose) -> Tuple[bool, str]:
        """
        Check if a Cartesian pose falls within workspace bounds.

        Args:
            pose: Target end-effector pose

        Returns:
            (is_valid, reason_if_invalid)
        """
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z

        x_bounds = self._bounds.get('x', [-2.0, 2.0])
        y_bounds = self._bounds.get('y', [-2.0, 2.0])
        z_bounds = self._bounds.get('z', [-1.0, 3.0])

        if not (x_bounds[0] <= x <= x_bounds[1]):
            return False, f'X={x:.3f} outside bounds {x_bounds}'
        if not (y_bounds[0] <= y <= y_bounds[1]):
            return False, f'Y={y:.3f} outside bounds {y_bounds}'
        if not (z_bounds[0] <= z <= z_bounds[1]):
            return False, f'Z={z:.3f} outside bounds {z_bounds}'

        return True, 'OK'

    def validate_quaternion(self, pose: Pose) -> Tuple[bool, str]:
        """Check quaternion normalization."""
        q = pose.orientation
        norm = np.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        if abs(norm - 1.0) > 0.01:
            return False, f'Quaternion not normalized: norm={norm:.4f}'
        return True, 'OK'

    def validate_joint_limits(
        self, joint_values: List[float]
    ) -> Tuple[bool, str]:
        """Check joint values are within limits."""
        if self._joint_min is None or self._joint_max is None:
            return True, 'No joint limits loaded'

        joints = np.array(joint_values)
        below = joints < self._joint_min
        above = joints > self._joint_max

        if below.any():
            idx = np.where(below)[0].tolist()
            return False, f'Joints {idx} below minimum limits'
        if above.any():
            idx = np.where(above)[0].tolist()
            return False, f'Joints {idx} above maximum limits'

        return True, 'OK'

    def validate_pose(self, pose: Pose) -> Tuple[bool, List[str]]:
        """Run all pose validations. Returns (all_valid, list_of_issues)."""
        issues = []

        ok, msg = self.validate_pose_in_workspace(pose)
        if not ok:
            issues.append(f'Workspace: {msg}')

        ok, msg = self.validate_quaternion(pose)
        if not ok:
            issues.append(f'Quaternion: {msg}')

        return len(issues) == 0, issues