"""Pose generator factory"""

from typing import Dict, Type
from .strategies.base_strategy import PoseStrategy
from .strategies.iso_cube import ISOCubeStrategy
from .strategies.workspace_sampler import WorkspaceSamplerStrategy


class PoseGenerator:
    """Factory for creating pose strategies"""
    
    STRATEGIES: Dict[str, Type[PoseStrategy]] = {
        'iso_cube': ISOCubeStrategy,
        'workspace_sampler': WorkspaceSamplerStrategy,
    }
    
    @classmethod
    def create(cls, strategy_name: str, config: dict, **kwargs) -> PoseStrategy:
        """Create pose strategy instance"""
        if strategy_name not in cls.STRATEGIES:
            raise ValueError(f"Unknown strategy: {strategy_name}")
            
        strategy_class = cls.STRATEGIES[strategy_name]
        
        # Special case for workspace_sampler needing joint limits
        if strategy_name == 'workspace_sampler' and 'joint_limits' in kwargs:
            return strategy_class(config, kwargs['joint_limits'])
            
        return strategy_class(config)