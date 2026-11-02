"""ROS2 client interface for AutoCert services"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from autocert_interfaces.srv import (
    BatchFK, 
    GetJointLimits, 
    GetRobotInfo, 
    CheckCollision,
    SampleWorkspace
)
from geometry_msgs.msg import Pose
import numpy as np
import numpy.typing as npt
from typing import List, Optional, Tuple, Dict, Any, Union
import time
from dataclasses import dataclass
import json


@dataclass
class RobotInfo:
    """Robot information data class"""
    robot_name: str
    planning_group: str
    end_effector_link: str
    joint_names: List[str]
    link_names: List[str]
    group_names: List[str]
    joint_min_limits: np.ndarray
    joint_max_limits: np.ndarray
    joint_velocity_limits: np.ndarray
    num_joints: int
    num_links: int
    collision_checking_available: bool
    kinematics_solver_available: bool
    kinematics_solver_name: str
    default_timeout: float


@dataclass
class JointLimits:
    """Joint limits data class"""
    names: List[str]
    min_positions: np.ndarray
    max_positions: np.ndarray
    velocity_limits: np.ndarray
    acceleration_limits: np.ndarray
    planning_group: str
    end_effector_link: str


@dataclass
class SamplingResult:
    """Workspace sampling result data class"""
    positions: np.ndarray
    orientations: np.ndarray
    joint_samples: np.ndarray
    valid_indices: List[int]
    end_effector_link: str
    computation_time_ms: float
    reachability_index: float
    sampling_strategy: str
    robot_name: str


class AutoCertClient(Node):
    """
    Main client for AutoCert services.
    
    Provides robot-agnostic access to workspace sampling,
    kinematics computation, and calibration services.
    """
    
    def __init__(self, node_name: str = 'autocert_client'):
        super().__init__(node_name)
        
        # Use separate callback groups for parallel service calls
        self.cb_group = MutuallyExclusiveCallbackGroup()
        
        # Create service clients
        self.fk_cli = self.create_client(
            BatchFK, 'batch_fk', callback_group=self.cb_group
        )
        self.limits_cli = self.create_client(
            GetJointLimits, 'get_joint_limits', callback_group=self.cb_group
        )
        self.info_cli = self.create_client(
            GetRobotInfo, 'get_robot_info', callback_group=self.cb_group
        )
        self.collision_cli = self.create_client(
            CheckCollision, 'check_collision', callback_group=self.cb_group
        )
        self.sample_cli = self.create_client(
            SampleWorkspace, 'sample_workspace', callback_group=self.cb_group
        )
        
        # Wait for services
        self._wait_for_services()
        
        # Cache robot info
        self._robot_info = None
        self._joint_limits = None
        
        self.get_logger().info('AutoCert client initialized and connected')
    
    def _wait_for_services(self, timeout: float = 10.0):
        """Wait for all required services to be available"""
        services = {
            'batch_fk': self.fk_cli,
            'get_joint_limits': self.limits_cli,
            'get_robot_info': self.info_cli,
            'check_collision': self.collision_cli,
            'sample_workspace': self.sample_cli
        }
        
        start_time = time.time()
        for name, client in services.items():
            while not client.wait_for_service(timeout_sec=0.1):
                if time.time() - start_time > timeout:
                    raise RuntimeError(
                        f"Service {name} not available after {timeout}s"
                    )
                self.get_logger().info(f'Waiting for {name} service...')
        
        self.get_logger().info('All services available')
    
    def get_robot_info(self, force_refresh: bool = False) -> RobotInfo:
        """
        Get comprehensive robot information.
        
        Args:
            force_refresh: Force refresh cached information
            
        Returns:
            RobotInfo dataclass with robot metadata
        """
        if self._robot_info is not None and not force_refresh:
            return self._robot_info
        
        request = GetRobotInfo.Request()
        future = self.info_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is None:
            raise RuntimeError("Failed to get robot info")
        
        response = future.result()
        
        self._robot_info = RobotInfo(
            robot_name=response.robot_name,
            planning_group=response.planning_group,
            end_effector_link=response.end_effector_link,
            joint_names=list(response.joint_names),
            link_names=list(response.link_names),
            group_names=list(response.group_names),
            joint_min_limits=np.array(response.joint_min_limits),
            joint_max_limits=np.array(response.joint_max_limits),
            joint_velocity_limits=np.array(response.joint_velocity_limits),
            num_joints=response.num_joints,
            num_links=response.num_links,
            collision_checking_available=response.collision_checking_available,
            kinematics_solver_available=response.kinematics_solver_available,
            kinematics_solver_name=response.kinematics_solver_name,
            default_timeout=response.default_timeout
        )
        
        return self._robot_info
    
    def get_joint_limits(self, force_refresh: bool = False) -> JointLimits:
        """
        Get joint limits for the robot.
        
        Args:
            force_refresh: Force refresh cached limits
            
        Returns:
            JointLimits dataclass with limit information
        """
        if self._joint_limits is not None and not force_refresh:
            return self._joint_limits
        
        request = GetJointLimits.Request()
        future = self.limits_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is None:
            raise RuntimeError("Failed to get joint limits")
        
        response = future.result()
        
        self._joint_limits = JointLimits(
            names=list(response.joint_names),
            min_positions=np.array(response.min_limits),
            max_positions=np.array(response.max_limits),
            velocity_limits=np.array(response.velocity_limits),
            acceleration_limits=np.array(response.acceleration_limits),
            planning_group=response.planning_group,
            end_effector_link=response.end_effector_link
        )
        
        return self._joint_limits
    
    def compute_fk(
        self,
        joint_samples: npt.NDArray,
        filter_collisions: bool = True,
        timeout_sec: float = 30.0
    ) -> Tuple[np.ndarray, np.ndarray, List[int], float]:
        """
        Compute forward kinematics for batch of joint samples.
        
        Args:
            joint_samples: Array of joint configurations (n_samples x n_joints)
            filter_collisions: Whether to filter out colliding configurations
            timeout_sec: Service call timeout
            
        Returns:
            Tuple of (positions, orientations, valid_indices, computation_time_ms)
        """
        # Validate input
        if len(joint_samples.shape) != 2:
            raise ValueError(f"Expected 2D array, got shape {joint_samples.shape}")
        
        n_samples, n_joints = joint_samples.shape
        
        # Verify joint count matches robot
        info = self.get_robot_info()
        if n_joints != info.num_joints:
            raise ValueError(
                f"Joint count mismatch: expected {info.num_joints}, "
                f"got {n_joints}"
            )
        
        # Prepare request
        request = BatchFK.Request()
        request.joint_samples = joint_samples.flatten().tolist()
        request.num_joints = int(n_joints)
        request.num_samples = int(n_samples)
        request.filter_collisions = filter_collisions
        
        # Call service
        future = self.fk_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        
        if future.result() is None:
            raise RuntimeError('Failed to call batch_fk service')
        
        response = future.result()
        
        # Convert to numpy arrays
        positions = np.array([
            [pose.position.x, pose.position.y, pose.position.z]
            for pose in response.poses
        ])
        
        orientations = np.array([
            [pose.orientation.x, pose.orientation.y, 
             pose.orientation.z, pose.orientation.w]
            for pose in response.poses
        ])
        
        computation_time = response.computation_time_ms[0] if response.computation_time_ms else 0.0
        
        return positions, orientations, list(response.valid_indices), computation_time
    
    def check_collision(
        self,
        joint_positions: npt.NDArray,
        timeout_sec: float = 5.0
    ) -> Tuple[bool, bool, bool]:
        """
        Check collision for a single joint configuration.
        
        Args:
            joint_positions: Joint configuration to check
            timeout_sec: Service call timeout
            
        Returns:
            Tuple of (in_collision, self_collision, environment_collision)
        """
        request = CheckCollision.Request()
        request.joint_positions = joint_positions.flatten().tolist()
        
        future = self.collision_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        
        if future.result() is None:
            raise RuntimeError('Failed to call check_collision service')
        
        response = future.result()
        return response.in_collision, response.self_collision, response.environment_collision
    
    def sample_workspace(
        self,
        num_samples: int,
        strategy: str = 'random',
        enable_collision_checking: bool = True,
        seed_configuration: Optional[np.ndarray] = None,
        timeout_sec: float = 60.0
    ) -> SamplingResult:
        """
        Generate workspace samples.
        
        This is a high-level method that combines sampling and FK computation.
        
        Args:
            num_samples: Number of samples to generate
            strategy: Sampling strategy ('random', 'latin', 'halton', 'grid')
            enable_collision_checking: Whether to check collisions
            seed_configuration: Optional seed configuration
            timeout_sec: Service call timeout
            
        Returns:
            SamplingResult with workspace data
        """
        # Get robot info and limits
        info = self.get_robot_info()
        limits = self.get_joint_limits()
        
        # Import here to avoid circular imports
        from .sampling import create_sampler
        
        # Create sampler and generate samples
        sampler = create_sampler(
            (limits.min_positions, limits.max_positions),
            strategy
        )
        
        joint_samples = sampler.generate_samples(num_samples)
        
        # Compute FK
        start_time = time.time()
        positions, orientations, valid_indices, comp_time = self.compute_fk(
            joint_samples,
            filter_collisions=enable_collision_checking,
            timeout_sec=timeout_sec
        )
        elapsed = (time.time() - start_time) * 1000  # Convert to ms
        
        # Compute reachability index
        reachability = 100.0 * len(valid_indices) / num_samples if num_samples > 0 else 0.0
        
        return SamplingResult(
            positions=positions,
            orientations=orientations,
            joint_samples=joint_samples[valid_indices] if valid_indices else joint_samples,
            valid_indices=valid_indices,
            end_effector_link=info.end_effector_link,
            computation_time_ms=comp_time,
            reachability_index=reachability,
            sampling_strategy=strategy,
            robot_name=info.robot_name
        )
    
    def close(self):
        """Clean up resources"""
        self.destroy_node()


# Alias for backward compatibility
WorkspaceSamplerClient = AutoCertClient