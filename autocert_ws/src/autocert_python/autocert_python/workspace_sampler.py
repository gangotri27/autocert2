"""ROS2 client interface to the C++ WorkspaceSampler"""

import rclpy
from rclpy.node import Node
from autocert_interfaces.srv import BatchFK, GetJointLimits, GetRobotInfo
from geometry_msgs.msg import Pose
import numpy as np
import numpy.typing as npt
from typing import List, Optional, Tuple, Dict, Any
import time


class WorkspaceSamplerClient(Node):
    """ROS2 client for batch FK computation"""
    
    def __init__(self, service_name: str = 'batch_fk'):
        super().__init__('workspace_sampler_client')
        self.service_name = service_name
        
        # Create clients
        self.fk_cli = self.create_client(BatchFK, service_name)
        self.limits_cli = self.create_client(GetJointLimits, 'get_joint_limits')
        self.info_cli = self.create_client(GetRobotInfo, 'get_robot_info')
        
        # Wait for services
        self._wait_for_service(self.fk_cli, 'batch_fk')
        self._wait_for_service(self.limits_cli, 'get_joint_limits')
        self._wait_for_service(self.info_cli, 'get_robot_info')
        
        # Cache robot info
        self._robot_info = None
        self._joint_limits = None
        
        self.get_logger().info('Connected to all AutoCert services')
    
    def _wait_for_service(self, client, service_name: str, timeout: float = 5.0):
        """Wait for a service to become available"""
        start_time = time.time()
        while not client.wait_for_service(timeout_sec=0.1):
            if time.time() - start_time > timeout:
                raise RuntimeError(f"Service {service_name} not available after {timeout}s")
            self.get_logger().info(f'Waiting for {service_name} service...')
    
    def get_robot_info(self, force_refresh: bool = False) -> Dict[str, Any]:
        """
        Get robot information including joint limits
        
        Args:
            force_refresh: Force refresh cached info
            
        Returns:
            Dictionary with robot information
        """
        if self._robot_info is not None and not force_refresh:
            return self._robot_info
        
        request = GetRobotInfo.Request()
        future = self.info_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is None:
            raise RuntimeError("Failed to get robot info")
        
        response = future.result()
        
        self._robot_info = {
            'robot_name': response.robot_name,
            'planning_group': response.planning_group,
            'end_effector_link': response.end_effector_link,
            'joint_names': list(response.joint_names),
            'joint_min_limits': np.array(response.joint_min_limits),
            'joint_max_limits': np.array(response.joint_max_limits),
            'num_joints': response.num_joints,
            'collision_checking_available': response.collision_checking_available
        }
        
        return self._robot_info
    
    def get_joint_limits(self) -> Tuple[npt.NDArray, npt.NDArray, List[str]]:
        """
        Get joint limits directly
        
        Returns:
            Tuple of (min_limits, max_limits, joint_names)
        """
        if self._joint_limits is not None:
            return self._joint_limits
        
        request = GetJointLimits.Request()
        future = self.limits_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is None:
            raise RuntimeError("Failed to get joint limits")
        
        response = future.result()
        
        self._joint_limits = (
            np.array(response.min_limits),
            np.array(response.max_limits),
            list(response.joint_names)
        )
        
        return self._joint_limits
    
    def compute_fk(
        self,
        joint_samples: npt.NDArray,
        timeout_sec: float = 30.0,
        filter_valid: bool = True
    ) -> Tuple[List[Pose], List[int], str]:
        """
        Compute FK for batch of joint samples
        
        Args:
            joint_samples: Array of joint configurations (n_samples x n_joints)
            timeout_sec: Service call timeout
            filter_valid: If True, only return valid (collision-free) poses
            
        Returns:
            Tuple of (poses, valid_indices, end_effector_link)
        """
        # Validate input
        if len(joint_samples.shape) != 2:
            raise ValueError(f"Expected 2D array, got shape {joint_samples.shape}")
        
        n_samples, n_joints = joint_samples.shape
        
        # Verify joint count matches robot
        info = self.get_robot_info()
        if n_joints != info['num_joints']:
            raise ValueError(
                f"Joint count mismatch: expected {info['num_joints']}, "
                f"got {n_joints}"
            )
        
        # Prepare request
        request = BatchFK.Request()
        request.joint_samples = joint_samples.flatten().tolist()
        request.num_joints = int(n_joints)
        request.num_samples = int(n_samples)
        
        # Call service
        future = self.fk_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        
        if future.result() is not None:
            response = future.result()
            
            if filter_valid:
                # Filter to only valid samples
                valid_poses = [response.poses[i] for i in response.valid_indices]
                return valid_poses, response.valid_indices, response.end_effector_link
            else:
                return response.poses, response.valid_indices, response.end_effector_link
        else:
            self.get_logger().error('Service call failed')
            raise RuntimeError('Failed to call batch_fk service')
    
    @staticmethod
    def poses_to_numpy(poses: List[Pose]) -> npt.NDArray:
        """Convert list of ROS poses to numpy array [x, y, z, qx, qy, qz, qw]"""
        data = []
        for pose in poses:
            data.append([
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ])
        return np.array(data)
    
    @staticmethod
    def positions_to_numpy(poses: List[Pose]) -> npt.NDArray:
        """Extract positions from poses as numpy array [x, y, z]"""
        positions = []
        for pose in poses:
            positions.append([
                pose.position.x,
                pose.position.y,
                pose.position.z
            ])
        return np.array(positions)