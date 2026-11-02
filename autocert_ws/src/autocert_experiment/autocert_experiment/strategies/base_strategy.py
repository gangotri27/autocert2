"""Base class for pose generation strategies"""

from abc import ABC, abstractmethod
from geometry_msgs.msg import Pose
from typing import List, Tuple, Optional


class PoseStrategy(ABC):
    """Abstract base class for pose generation strategies"""
    
    def __init__(self, config: dict):
        self.config = config
        
    @abstractmethod
    def generate_poses(self) -> List[Pose]:
        """Generate list of test poses"""
        pass
        
    @abstractmethod
    def get_pose_label(self, index: int) -> str:
        """Get human-readable label for pose"""
        pass
        
    @abstractmethod
    def validate_pose(self, pose: Pose) -> bool:
        """Validate if pose is reachable"""
        pass