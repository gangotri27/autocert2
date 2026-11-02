"""ISO 9283 metric definitions"""

import numpy as np
from scipy.spatial.transform import Rotation as R
from typing import Dict, Any


class ISO9283Metrics:
    """ISO 9283 metric computation"""
    
    @staticmethod
    def compute_position_accuracy(commanded: np.ndarray, 
                                  measured: np.ndarray) -> Dict[str, float]:
        """
        Compute position accuracy (AP)
        
        AP = distance between commanded position and centroid of measured positions
        """
        centroid = np.mean(measured, axis=0)
        ap = np.linalg.norm(commanded - centroid)
        
        return {
            'ap_x': float(commanded[0] - centroid[0]),
            'ap_y': float(commanded[1] - centroid[1]),
            'ap_z': float(commanded[2] - centroid[2]),
            'ap_distance_m': float(ap),
            'ap_distance_mm': float(ap * 1000.0)
        }
        
    @staticmethod
    def compute_position_repeatability(measured: np.ndarray) -> Dict[str, float]:
        """
        Compute position repeatability (RP)
        
        RP = mean(l_i) + 3 * sigma(l_i)
        where l_i = distance from each point to centroid
        """
        if len(measured) < 2:
            return {}
            
        centroid = np.mean(measured, axis=0)
        distances = np.linalg.norm(measured - centroid, axis=1)
        
        mean_dist = np.mean(distances)
        std_dist = np.std(distances, ddof=1)
        rp = mean_dist + 3 * std_dist
        
        return {
            'rp_distance_m': float(rp),
            'rp_distance_mm': float(rp * 1000.0),
            'mean_distance_mm': float(mean_dist * 1000.0),
            'std_distance_mm': float(std_dist * 1000.0)
        }
        
    @staticmethod
    def compute_orientation_accuracy(cmd_quat: np.ndarray,
                                    meas_quats: np.ndarray) -> Dict[str, float]:
        """
        Compute orientation accuracy
        
        Angular deviation between commanded and measured orientations
        """
        cmd_rot = R.from_quat(cmd_quat)
        
        errors = []
        for q in meas_quats:
            meas_rot = R.from_quat(q)
            rel_rot = cmd_rot.inv() * meas_rot
            errors.append(rel_rot.magnitude())
            
        errors = np.array(errors)
        
        return {
            'mean_angular_error_deg': float(np.mean(errors) * 180.0 / np.pi),
            'max_angular_error_deg': float(np.max(errors) * 180.0 / np.pi),
            'std_angular_error_deg': float(np.std(errors) * 180.0 / np.pi)
        }
        
    @staticmethod
    def compute_orientation_repeatability(meas_quats: np.ndarray) -> Dict[str, float]:
        """
        Compute orientation repeatability
        
        Spread of measured orientations around mean orientation
        """
        if len(meas_quats) < 2:
            return {}
            
        rotations = R.from_quat(meas_quats)
        mean_rotation = rotations.mean()
        
        deviations = []
        for rot in rotations:
            rel_rot = mean_rotation.inv() * rot
            deviations.append(rel_rot.magnitude())
            
        deviations = np.array(deviations)
        
        mean_dev = np.mean(deviations)
        std_dev = np.std(deviations, ddof=1)
        rp = mean_dev + 3 * std_dev
        
        return {
            'orientation_repeatability_deg': float(rp * 180.0 / np.pi),
            'mean_deviation_deg': float(mean_dev * 180.0 / np.pi),
            'std_deviation_deg': float(std_dev * 180.0 / np.pi)
        }