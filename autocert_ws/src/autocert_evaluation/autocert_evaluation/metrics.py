"""ISO 9283 metrics computation with outlier handling"""

import pandas as pd
import numpy as np
from typing import Dict, List, Any
from scipy.spatial.transform import Rotation as R

from .iso9283 import ISO9283Metrics
from .outlier_detector import OutlierDetector


class MetricsCalculator:
    """Main metrics calculator with outlier handling"""
    
    def __init__(self, df: pd.DataFrame, outlier_threshold: float = 3.0):
        self.df = df
        self.outlier_detector = OutlierDetector(outlier_threshold)
        self.outliers_removed: Dict[int, List[int]] = {}
        
    def compute_pose_metrics(self, pose_id: int, 
                            remove_outliers: bool = True) -> Dict[str, Any]:
        """Compute all metrics for a single pose"""
        
        # Get pose data
        pose_data = self.df[self.df['pose_id'] == pose_id]
        
        if len(pose_data) == 0:
            return {}
            
        # Detect outliers if requested
        if remove_outliers:
            outliers, stats = self.outlier_detector.detect_pose_outliers(self.df, pose_id)
            if outliers:
                self.outliers_removed[pose_id] = outliers
                pose_data = pose_data[~pose_data['repetition'].isin(outliers)]
                
        # Extract data
        commanded_pos = np.array([
            pose_data['cmd_x'].iloc[0],
            pose_data['cmd_y'].iloc[0],
            pose_data['cmd_z'].iloc[0]
        ])
        
        measured_pos = pose_data[['meas_x', 'meas_y', 'meas_z']].values
        
        commanded_quat = np.array([
            pose_data['cmd_qx'].iloc[0],
            pose_data['cmd_qy'].iloc[0],
            pose_data['cmd_qz'].iloc[0],
            pose_data['cmd_qw'].iloc[0]
        ])
        
        measured_quats = pose_data[['meas_qx', 'meas_qy', 'meas_qz', 'meas_qw']].values
        
        # Compute metrics
        accuracy = ISO9283Metrics.compute_position_accuracy(commanded_pos, measured_pos)
        repeatability = ISO9283Metrics.compute_position_repeatability(measured_pos)
        orient_acc = ISO9283Metrics.compute_orientation_accuracy(commanded_quat, measured_quats)
        orient_rep = ISO9283Metrics.compute_orientation_repeatability(measured_quats)
        
        # Combine results
        result = {
            'pose_id': pose_id,
            'num_samples': len(pose_data),
            'outliers_removed': len(outliers) if remove_outliers else 0,
            **accuracy,
            **repeatability,
            **orient_acc,
            **orient_rep
        }
        
        return result
        
    def compute_all_metrics(self) -> Dict[int, Dict[str, Any]]:
        """Compute metrics for all poses"""
        results = {}
        
        for pose_id in self.df['pose_id'].unique():
            results[int(pose_id)] = self.compute_pose_metrics(pose_id)
            
        return results
        
    def compute_overall_metrics(self) -> Dict[str, float]:
        """Compute overall statistics across all poses"""
        pose_metrics = self.compute_all_metrics()
        
        if not pose_metrics:
            return {}
            
        # Extract metrics
        accuracies = [m['ap_distance_mm'] for m in pose_metrics.values()]
        repeatabilities = [m['rp_distance_mm'] for m in pose_metrics.values()]
        orient_acc = [m['mean_angular_error_deg'] for m in pose_metrics.values()]
        orient_rep = [m['orientation_repeatability_deg'] for m in pose_metrics.values()]
        
        # Compute confidence intervals (95%)
        n = len(accuracies)
        if n > 1:
            acc_ci = 1.96 * np.std(accuracies) / np.sqrt(n)
            rep_ci = 1.96 * np.std(repeatabilities) / np.sqrt(n)
        else:
            acc_ci = rep_ci = 0
            
        return {
            'overall_mean_accuracy_mm': float(np.mean(accuracies)),
            'overall_max_accuracy_mm': float(np.max(accuracies)),
            'overall_min_accuracy_mm': float(np.min(accuracies)),
            'accuracy_95_confidence_mm': float(acc_ci),
            'overall_mean_repeatability_mm': float(np.mean(repeatabilities)),
            'overall_max_repeatability_mm': float(np.max(repeatabilities)),
            'overall_min_repeatability_mm': float(np.min(repeatabilities)),
            'repeatability_95_confidence_mm': float(rep_ci),
            'overall_mean_orientation_accuracy_deg': float(np.mean(orient_acc)),
            'overall_mean_orientation_repeatability_deg': float(np.mean(orient_rep)),
            'num_poses': len(pose_metrics)
        }