#!/usr/bin/env python3
"""Builds a MoveIt2 configuration dictionary from YAML parameters."""

import os
import yaml
from typing import Dict, Any, Optional

class MoveItConfigBuilder:

    REQUIRED_KEYS =[
        'planning_group',
        'end_effector_link',
        'reference_frame',
    ]

    def __init__(self, config_path: Optional[str] = None):
        self._config: Dict[str, Any] = {}
        if config_path:
            self.load_from_yaml(config_path)

    def load_from_yaml(self, path: str) -> 'MoveItConfigBuilder':
        path = os.path.expanduser(path)
        if not os.path.exists(path):
            raise FileNotFoundError(f'Config not found: {path}')
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        self._config.update(data)
        return self

    def set(self, key: str, value: Any) -> 'MoveItConfigBuilder':
        self._config[key] = value
        return self

    def validate(self) -> bool:
        missing =[k for k in self.REQUIRED_KEYS if k not in self._config]
        if missing:
            raise ValueError(f'Missing required config keys: {missing}')
        return True

    def build_ompl_config(self) -> Dict[str, Any]:
        planning_group = self._config.get('planning_group', 'arm')
        planner_id = self._config.get('planner_id', 'RRTConnect')
        planner_config_name = f"{planner_id}kConfigDefault"

        return {
            'planning_plugins':['ompl_interface/OMPLPlanner'],
            'request_adapters':[
                'default_planning_request_adapters/ResolveConstraintFrames',
                'default_planning_request_adapters/CheckStartStateBounds',
                'default_planning_request_adapters/CheckStartStateCollision',
            ],
            'response_adapters':[
                'default_planning_response_adapters/ValidateSolution',
                'default_planning_response_adapters/DisplayMotionPath',
            ],
            'start_state_max_bounds_error': 0.1,
            'planner_configs': {
                planner_config_name: {
                    'type': f'geometric::{planner_id}',
                    'range': 0.0,
                }
            },
            planning_group: {
                'default_planner_config': planner_config_name,
                'planner_configs':[planner_config_name],
            },
        }

    def build_moveit_cpp_config(self) -> Dict[str, Any]:
        planner_config_name = f"{self._config.get('planner_id', 'RRTConnect')}kConfigDefault"
        return {
            'planning_scene_monitor_options': {
                'name': "planning_scene_monitor",
                'robot_description': "robot_description",
                'joint_state_topic': "/joint_states",
                'attached_collision_object_topic': "/moveit_cpp/planning_scene_monitor",
                'publish_planning_scene_topic': "/moveit_cpp/publish_planning_scene",
                'monitored_planning_scene_topic': "/moveit_cpp/monitored_planning_scene",
                'wait_for_initial_state_timeout': 10.0,
            },
            'planning_pipelines': {
                'pipeline_names':['ompl']
            },
            'plan_request_params': {
                'planning_attempts': 5,
                'planning_pipeline': 'ompl',
                'planner_id': planner_config_name,
            },
            'ompl': self.build_ompl_config(),
        }

    def get(self, key: str, default: Any = None) -> Any:
        return self._config.get(key, default)

    @property
    def config(self) -> Dict[str, Any]:
        return dict(self._config)