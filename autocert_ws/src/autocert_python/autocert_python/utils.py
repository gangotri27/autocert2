"""Utility functions for workspace sampling and calibration results"""

import json
import numpy as np
import numpy.typing as npt
from typing import Dict, Any, List, Optional, Tuple, Union
from geometry_msgs.msg import Pose
import datetime
import os
import pandas as pd
from scipy.spatial import ConvexHull
import hashlib


def pose_to_dict(pose: Pose) -> Dict[str, Any]:
    """Convert ROS Pose to dictionary"""
    return {
        'position': {
            'x': pose.position.x,
            'y': pose.position.y,
            'z': pose.position.z
        },
        'orientation': {
            'x': pose.orientation.x,
            'y': pose.orientation.y,
            'z': pose.orientation.z,
            'w': pose.orientation.w
        }
    }


def pose_from_dict(data: Dict[str, Any]) -> Pose:
    """Create ROS Pose from dictionary"""
    pose = Pose()
    pose.position.x = data['position']['x']
    pose.position.y = data['position']['y']
    pose.position.z = data['position']['z']
    pose.orientation.x = data['orientation']['x']
    pose.orientation.y = data['orientation']['y']
    pose.orientation.z = data['orientation']['z']
    pose.orientation.w = data['orientation']['w']
    return pose


def save_results(
    robot_name: str,
    joint_samples: npt.NDArray,
    poses: Union[List[Pose], npt.NDArray],
    sampling_strategy: str,
    filename: Optional[str] = None,
    metadata: Optional[Dict[str, Any]] = None,
    compression: bool = False
) -> str:
    """
    Save workspace sampling results to JSON or compressed numpy format
    
    Args:
        robot_name: Name of the robot
        joint_samples: Array of joint configurations
        poses: List of poses or position array
        sampling_strategy: Strategy used for sampling
        filename: Output filename (auto-generated if None)
        metadata: Additional metadata to save
        compression: Whether to use numpy compression (.npz)
        
    Returns:
        Path to saved file
    """
    if filename is None:
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        hash_input = f"{robot_name}_{timestamp}_{np.random.randint(1000)}"
        file_hash = hashlib.md5(hash_input.encode()).hexdigest()[:8]
        filename = f"workspace_{robot_name}_{timestamp}_{file_hash}"
    
    # Handle poses
    if isinstance(poses, list):
        # Convert ROS poses to arrays
        positions = np.array([[p.position.x, p.position.y, p.position.z] for p in poses])
        orientations = np.array([[p.orientation.x, p.orientation.y, 
                                  p.orientation.z, p.orientation.w] for p in poses])
    elif isinstance(poses, np.ndarray):
        if poses.shape[1] == 3:  # Only positions
            positions = poses
            orientations = np.zeros((len(poses), 4))
        elif poses.shape[1] == 7:  # [x,y,z,qx,qy,qz,qw]
            positions = poses[:, :3]
            orientations = poses[:, 3:]
        else:
            raise ValueError(f"Unexpected poses shape: {poses.shape}")
    else:
        raise TypeError(f"Unsupported poses type: {type(poses)}")
    
    # Prepare metadata
    data = {
        'robot_name': robot_name,
        'timestamp': datetime.datetime.now().isoformat(),
        'sampling_strategy': sampling_strategy,
        'n_samples': len(joint_samples),
        'n_joints': joint_samples.shape[1] if len(joint_samples) > 0 else 0,
        'joint_limits': {
            'min': joint_samples.min(axis=0).tolist() if len(joint_samples) > 0 else [],
            'max': joint_samples.max(axis=0).tolist() if len(joint_samples) > 0 else []
        },
        'workspace_metrics': compute_workspace_metrics(positions) if len(positions) > 0 else {}
    }
    
    if metadata:
        data['metadata'] = metadata
    
    if compression:
        # Save as compressed numpy
        npz_filename = filename if filename.endswith('.npz') else filename + '.npz'
        np.savez_compressed(
            npz_filename,
            joint_samples=joint_samples,
            positions=positions,
            orientations=orientations,
            metadata=np.array([json.dumps(data)])
        )
        print(f"Results saved to: {npz_filename}")
        return npz_filename
    else:
        # Save as JSON
        json_filename = filename if filename.endswith('.json') else filename + '.json'
        
        # Prepare full data for JSON
        json_data = data.copy()
        json_data['joint_samples'] = joint_samples.tolist()
        json_data['positions'] = positions.tolist()
        json_data['orientations'] = orientations.tolist()
        
        with open(json_filename, 'w') as f:
            json.dump(json_data, f, indent=2)
        
        print(f"Results saved to: {json_filename}")
        return json_filename


def load_results(filename: str) -> Dict[str, Any]:
    """
    Load workspace sampling results from JSON or NPZ file
    
    Args:
        filename: Path to JSON or NPZ file
        
    Returns:
        Dictionary containing results with numpy arrays
    """
    if filename.endswith('.npz'):
        # Load from numpy compressed
        data = np.load(filename, allow_pickle=True)
        result = {
            'joint_samples': data['joint_samples'],
            'positions': data['positions'],
            'orientations': data['orientations']
        }
        
        # Load metadata if present
        if 'metadata' in data:
            result.update(json.loads(data['metadata'][0]))
        
        return result
    else:
        # Load from JSON
        with open(filename, 'r') as f:
            data = json.load(f)
        
        # Convert lists back to numpy arrays
        data['joint_samples'] = np.array(data.get('joint_samples', []))
        data['positions'] = np.array(data.get('positions', []))
        data['orientations'] = np.array(data.get('orientations', []))
        
        return data


def extract_positions(poses: Union[List[Pose], npt.NDArray]) -> npt.NDArray:
    """Extract positions from pose list or array"""
    if isinstance(poses, list):
        return np.array([[p.position.x, p.position.y, p.position.z] for p in poses])
    elif isinstance(poses, np.ndarray):
        if poses.shape[1] >= 3:
            return poses[:, :3]
    raise ValueError(f"Cannot extract positions from {type(poses)}")


def extract_orientations(poses: Union[List[Pose], npt.NDArray]) -> npt.NDArray:
    """Extract orientations from pose list or array"""
    if isinstance(poses, list):
        return np.array([[p.orientation.x, p.orientation.y, 
                         p.orientation.z, p.orientation.w] for p in poses])
    elif isinstance(poses, np.ndarray):
        if poses.shape[1] == 7:
            return poses[:, 3:]
        elif poses.shape[1] == 4:
            return poses
    raise ValueError(f"Cannot extract orientations from {type(poses)}")


def compute_workspace_metrics(positions: npt.NDArray) -> Dict[str, float]:
    """
    Compute workspace metrics from point cloud
    
    Args:
        positions: Array of positions (n_points x 3)
        
    Returns:
        Dictionary of metrics
    """
    if len(positions) == 0:
        return {}
    
    metrics = {
        'x_range': float(positions[:, 0].max() - positions[:, 0].min()),
        'y_range': float(positions[:, 1].max() - positions[:, 1].min()),
        'z_range': float(positions[:, 2].max() - positions[:, 2].min()),
        'x_center': float(positions[:, 0].mean()),
        'y_center': float(positions[:, 1].mean()),
        'z_center': float(positions[:, 2].mean()),
        'x_std': float(positions[:, 0].std()),
        'y_std': float(positions[:, 1].std()),
        'z_std': float(positions[:, 2].std()),
        'n_points': len(positions)
    }
    
    # Compute convex hull volume if enough points
    if len(positions) >= 4:
        try:
            hull = ConvexHull(positions)
            metrics['convex_hull_volume'] = float(hull.volume)
            metrics['convex_hull_area'] = float(hull.area)
        except:
            metrics['convex_hull_volume'] = 0.0
            metrics['convex_hull_area'] = 0.0
    
    return metrics


def filter_collision_samples(
    joint_samples: npt.NDArray,
    valid_indices: List[int]
) -> npt.NDArray:
    """Filter joint samples to only valid (collision-free) ones"""
    return joint_samples[valid_indices]


def to_dataframe(
    joint_samples: npt.NDArray,
    positions: npt.NDArray,
    orientations: Optional[npt.NDArray] = None,
    joint_names: Optional[List[str]] = None
) -> pd.DataFrame:
    """
    Convert workspace data to pandas DataFrame for analysis
    
    Args:
        joint_samples: Joint configurations
        positions: End-effector positions
        orientations: End-effector orientations (optional)
        joint_names: Names of joints
        
    Returns:
        DataFrame with all data
    """
    n_samples = len(joint_samples)
    
    # Create column names
    if joint_names is None:
        joint_cols = [f'J{i}' for i in range(joint_samples.shape[1])]
    else:
        joint_cols = joint_names
    
    # Create data dictionary
    data = {}
    for i, name in enumerate(joint_cols):
        data[f'joint_{name}'] = joint_samples[:, i]
    
    data['pos_x'] = positions[:, 0]
    data['pos_y'] = positions[:, 1]
    data['pos_z'] = positions[:, 2]
    
    if orientations is not None:
        if orientations.shape[1] >= 4:
            data['ori_x'] = orientations[:, 0]
            data['ori_y'] = orientations[:, 1]
            data['ori_z'] = orientations[:, 2]
            data['ori_w'] = orientations[:, 3]
    
    return pd.DataFrame(data)


def from_dataframe(df: pd.DataFrame) -> Dict[str, np.ndarray]:
    """
    Extract numpy arrays from DataFrame
    
    Args:
        df: DataFrame with workspace data
        
    Returns:
        Dictionary with joint_samples, positions, orientations
    """
    # Extract joint columns
    joint_cols = [col for col in df.columns if col.startswith('joint_')]
    joint_samples = df[joint_cols].values if joint_cols else np.array([])
    
    # Extract position columns
    positions = df[['pos_x', 'pos_y', 'pos_z']].values
    
    # Extract orientation columns if present
    if all(col in df.columns for col in ['ori_x', 'ori_y', 'ori_z', 'ori_w']):
        orientations = df[['ori_x', 'ori_y', 'ori_z', 'ori_w']].values
    else:
        orientations = None
    
    return {
        'joint_samples': joint_samples,
        'positions': positions,
        'orientations': orientations
    }


def compute_reachability_map(
    positions: npt.NDArray,
    grid_resolution: int = 50,
    method: str = 'density'
) -> Tuple[npt.NDArray, npt.NDArray, npt.NDArray, npt.NDArray]:
    """
    Compute 2D reachability map by projecting workspace
    
    Args:
        positions: Array of positions
        grid_resolution: Resolution of output grid
        method: 'density' or 'binary'
        
    Returns:
        Tuple of (x_grid, y_grid, reachability_map, density_map)
    """
    from scipy.stats import gaussian_kde
    
    # Create grid
    x = positions[:, 0]
    y = positions[:, 1]
    
    x_grid = np.linspace(x.min(), x.max(), grid_resolution)
    y_grid = np.linspace(y.min(), y.max(), grid_resolution)
    xx, yy = np.meshgrid(x_grid, y_grid)
    
    if method == 'density':
        # Use KDE for density estimation
        try:
            kde = gaussian_kde(np.vstack([x, y]))
            positions_grid = np.vstack([xx.ravel(), yy.ravel()])
            density = kde(positions_grid).reshape(xx.shape)
        except:
            # Fallback to histogram
            density, _, _ = np.histogram2d(
                x, y, bins=grid_resolution,
                range=[[x.min(), x.max()], [y.min(), y.max()]]
            )
            density = density.T
    else:
        # Binary reachability (1 if any point in cell)
        hist, _, _ = np.histogram2d(
            x, y, bins=grid_resolution,
            range=[[x.min(), x.max()], [y.min(), y.max()]]
        )
        density = (hist > 0).astype(float)
    
    return x_grid, y_grid, xx, yy, density