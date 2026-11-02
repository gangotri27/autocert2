"""Configuration loader for AutoCert pipeline"""

import yaml
import os
from typing import Dict, Any, Optional


class ConfigLoader:
    """Load and validate master configuration"""
    
    REQUIRED_FIELDS = [
        'results_dir',
        'robot_metadata_file',
        'limits_file',
        'experiment_config',
        'evaluation_config'
    ]
    
    def __init__(self, config_path: str):
        self.config_path = config_path
        self.config: Dict[str, Any] = {}
        
    def load(self) -> bool:
        """Load configuration from YAML file"""
        if not os.path.exists(self.config_path):
            return False
            
        try:
            with open(self.config_path, 'r') as f:
                self.config = yaml.safe_load(f)
            return self.validate()
        except Exception as e:
            print(f"Failed to load config: {e}")
            return False
            
    def validate(self) -> bool:
        """Validate required fields exist"""
        for field in self.REQUIRED_FIELDS:
            if field not in self.config:
                print(f"Missing required field: {field}")
                return False
        return True
        
    def get(self, key: str, default: Any = None) -> Any:
        """Get configuration value"""
        return self.config.get(key, default)