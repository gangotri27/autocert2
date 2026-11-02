"""Outlier detection for measurement data"""

import numpy as np
import pandas as pd
from typing import List, Dict, Tuple


class OutlierDetector:
    """Detect outliers using 3-sigma rule and IQR"""
    
    def __init__(self, threshold: float = 3.0):
        self.threshold = threshold
        
    def detect_3sigma(self, values: np.ndarray) -> np.ndarray:
        """Detect outliers using 3-sigma rule"""
        mean = np.mean(values)
        std = np.std(values)
        
        if std == 0:
            return np.zeros_like(values, dtype=bool)
            
        return np.abs(values - mean) > self.threshold * std
        
    def detect_iqr(self, values: np.ndarray) -> np.ndarray:
        """Detect outliers using IQR method"""
        q1 = np.percentile(values, 25)
        q3 = np.percentile(values, 75)
        iqr = q3 - q1
        
        lower_bound = q1 - 1.5 * iqr
        upper_bound = q3 + 1.5 * iqr
        
        return (values < lower_bound) | (values > upper_bound)
        
    def detect_pose_outliers(self, df: pd.DataFrame, 
                            pose_id: int) -> Tuple[List[int], Dict]:
        """
        Detect outliers for a specific pose
        
        Returns:
            List of outlier repetition indices and statistics
        """
        pose_data = df[df['pose_id'] == pose_id]
        
        if len(pose_data) < 3:
            return [], {}
            
        # Extract positions
        positions = pose_data[['meas_x', 'meas_y', 'meas_z']].values
        
        # Compute centroid
        centroid = np.mean(positions, axis=0)
        
        # Compute distances from centroid
        distances = np.linalg.norm(positions - centroid, axis=1)
        
        # Detect outliers
        outlier_mask = self.detect_3sigma(distances)
        
        # Get outlier indices
        outliers = pose_data[outlier_mask]['repetition'].tolist()
        
        # Statistics
        stats = {
            'mean_distance': float(np.mean(distances)),
            'std_distance': float(np.std(distances)),
            'max_distance': float(np.max(distances)),
            'min_distance': float(np.min(distances)),
            'num_outliers': int(np.sum(outlier_mask)),
            'total_samples': len(pose_data)
        }
        
        return outliers, stats