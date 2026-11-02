"""Data validation for ISO 9283 measurements"""

import pandas as pd
import numpy as np
from typing import Tuple, List, Optional


class DataValidator:
    """Validate measurement data against ISO 9283 requirements"""
    
    REQUIRED_COLUMNS = [
        'pose_id', 'repetition', 'timestamp',
        'cmd_x', 'cmd_y', 'cmd_z',
        'cmd_qx', 'cmd_qy', 'cmd_qz', 'cmd_qw',
        'meas_x', 'meas_y', 'meas_z',
        'meas_qx', 'meas_qy', 'meas_qz', 'meas_qw'
    ]
    
    def __init__(self, df: pd.DataFrame):
        self.df = df
        
    def validate_schema(self) -> Tuple[bool, List[str]]:
        """Validate that all required columns exist"""
        missing = []
        for col in self.REQUIRED_COLUMNS:
            if col not in self.df.columns:
                missing.append(col)
                
        return len(missing) == 0, missing
        
    def validate_types(self) -> Tuple[bool, List[str]]:
        """Validate column data types"""
        errors = []
        
        # Check integer columns
        if not pd.api.types.is_integer_dtype(self.df['pose_id']):
            errors.append("pose_id must be integer")
            
        if not pd.api.types.is_integer_dtype(self.df['repetition']):
            errors.append("repetition must be integer")
            
        # Check numeric columns
        numeric_cols = ['cmd_x', 'cmd_y', 'cmd_z', 'meas_x', 'meas_y', 'meas_z']
        for col in numeric_cols:
            if not pd.api.types.is_numeric_dtype(self.df[col]):
                errors.append(f"{col} must be numeric")
                
        return len(errors) == 0, errors
        
    def validate_quaternions(self) -> Tuple[bool, List[str]]:
        """Validate quaternion normalization"""
        errors = []
        
        # Check commanded quaternions
        cmd_norms = np.sqrt(
            self.df['cmd_qx']**2 + self.df['cmd_qy']**2 + 
            self.df['cmd_qz']**2 + self.df['cmd_qw']**2
        )
        
        if (np.abs(cmd_norms - 1.0) > 0.01).any():
            errors.append("Commanded quaternions not normalized")
            
        # Check measured quaternions
        meas_norms = np.sqrt(
            self.df['meas_qx']**2 + self.df['meas_qy']**2 + 
            self.df['meas_qz']**2 + self.df['meas_qw']**2
        )
        
        if (np.abs(meas_norms - 1.0) > 0.01).any():
            errors.append("Measured quaternions not normalized")
            
        return len(errors) == 0, errors
        
    def validate_repetitions(self, expected: int = 30) -> List[int]:
        """Check if each pose has expected repetitions"""
        invalid = []
        for pose_id in self.df['pose_id'].unique():
            count = len(self.df[self.df['pose_id'] == pose_id])
            if count != expected:
                invalid.append(int(pose_id))
        return invalid
        
    def validate_timestamps(self) -> bool:
        """Check if timestamps are monotonic"""
        return np.all(np.diff(self.df['timestamp'].values) >= 0)