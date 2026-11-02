"""ISO 9283 cube test pose generation"""

import numpy as np
from geometry_msgs.msg import Pose, Quaternion
from typing import List, Tuple
from .base_strategy import PoseStrategy


class ISOCubeStrategy(PoseStrategy):
    """Generate ISO 9283 cube test poses"""
    
    def __init__(self, config: dict):
        super().__init__(config)
        
        # Cube parameters
        self.size_m = config.get('cube_size_mm', 320) / 1000.0
        self.center = config.get('cube_center', [0.0126, -0.0004, 0.1463])
        self.half_size = self.size_m / 2.0
        
        # Orientation
        self.fixed_orientation = config.get('fixed_orientation', [0.0, 0.0, 0.0, 1.0])
        
        # Pose set type
        self.pose_set = config.get('pose_set', 'standard')
        
    def generate_poses(self) -> List[Pose]:
        """Generate ISO cube poses"""
        cx, cy, cz = self.center
        hs = self.half_size
        
        if self.pose_set == 'standard':
            # 5 poses: center, +/-X, +/-Y
            positions = [
                (cx, cy, cz),                    # P1: Center
                (cx + hs, cy, cz),                # P2: +X
                (cx - hs, cy, cz),                # P3: -X
                (cx, cy + hs, cz),                # P4: +Y
                (cx, cy - hs, cz),                # P5: -Y
            ]
        elif self.pose_set == 'extended':
            # 7 poses: add +/-Z
            positions = [
                (cx, cy, cz),                    # P1: Center
                (cx + hs, cy, cz),                # P2: +X
                (cx - hs, cy, cz),                # P3: -X
                (cx, cy + hs, cz),                # P4: +Y
                (cx, cy - hs, cz),                # P5: -Y
                (cx, cy, cz + hs),                # P6: +Z
                (cx, cy, cz - hs),                # P7: -Z
            ]
        else:  # full
            # 27 poses: all combinations of +/- positions
            positions = []
            for dx in [-hs, 0, hs]:
                for dy in [-hs, 0, hs]:
                    for dz in [-hs, 0, hs]:
                        positions.append((cx + dx, cy + dy, cz + dz))
                        
        # Convert to Pose messages
        poses = []
        for x, y, z in positions:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation.x = self.fixed_orientation[0]
            pose.orientation.y = self.fixed_orientation[1]
            pose.orientation.z = self.fixed_orientation[2]
            pose.orientation.w = self.fixed_orientation[3]
            poses.append(pose)
            
        return poses
        
    def get_pose_label(self, index: int) -> str:
        """Get ISO pose label"""
        if self.pose_set == 'standard':
            labels = [
                "P1 (CENTER)",
                "P2 (+X)",
                "P3 (-X)",
                "P4 (+Y)",
                "P5 (-Y)"
            ]
        elif self.pose_set == 'extended':
            labels = [
                "P1 (CENTER)",
                "P2 (+X)",
                "P3 (-X)",
                "P4 (+Y)",
                "P5 (-Y)",
                "P6 (+Z)",
                "P7 (-Z)"
            ]
        else:
            # Full grid
            labels = [f"P{i+1}" for i in range(27)]
            
        return labels[index] if index < len(labels) else f"P{index+1}"
        
    def validate_pose(self, pose: Pose) -> bool:
        """Validate pose (to be implemented with IK)"""
        # In real implementation, call IK service
        return True