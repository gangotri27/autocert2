"""Hash utilities for robot configuration integrity"""

import hashlib
import os
from typing import Optional


def compute_file_hash(filepath: str, algorithm: str = 'sha256') -> Optional[str]:
    """Compute hash of file"""
    if not os.path.exists(filepath):
        return None
        
    hash_obj = hashlib.new(algorithm)
    
    with open(filepath, 'rb') as f:
        for chunk in iter(lambda: f.read(4096), b''):
            hash_obj.update(chunk)
            
    return hash_obj.hexdigest()


def compute_string_hash(content: str, algorithm: str = 'sha256') -> str:
    """Compute hash of string"""
    hash_obj = hashlib.new(algorithm)
    hash_obj.update(content.encode('utf-8'))
    return hash_obj.hexdigest()


def compute_robot_model_hash(urdf_string: str) -> str:
    """Compute hash of robot model"""
    return compute_string_hash(urdf_string)


def compute_srdf_hash(srdf_string: str) -> str:
    """Compute hash of SRDF"""
    return compute_string_hash(srdf_string)