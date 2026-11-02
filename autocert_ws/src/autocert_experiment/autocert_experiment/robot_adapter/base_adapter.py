"""Base robot adapter interface"""

from abc import ABC, abstractmethod
from geometry_msgs.msg import Pose
from typing import Optional


class RobotAdapter(ABC):
    """Abstract base class for robot adapters"""
    
    @abstractmethod
    def connect(self) -> bool:
        """Connect to robot system"""
        pass
        
    @abstractmethod
    def move_to_pose(self, pose: Pose, timeout: float = 10.0) -> bool:
        """Move robot to target pose"""
        pass
        
    @abstractmethod
    def wait_motion_complete(self, timeout: float = 30.0) -> bool:
        """Wait for motion to complete"""
        pass
        
    @abstractmethod
    def get_current_pose(self) -> Optional[Pose]:
        """Get current end-effector pose"""
        pass
        
    @abstractmethod
    def get_measurement(self) -> Optional[Pose]:
        """Get measured pose from external system"""
        pass
        
    @abstractmethod
    def disconnect(self):
        """Disconnect from robot system"""
        pass