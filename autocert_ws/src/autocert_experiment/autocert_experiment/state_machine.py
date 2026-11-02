"""ISO 9283 experiment state machine"""

from enum import Enum, auto
import time
from typing import Optional


class ExperimentState(Enum):
    """Experiment states"""
    IDLE = auto()
    INITIALIZING = auto()
    LOADING_CONFIG = auto()
    CONNECTING_ROBOT = auto()
    GENERATING_POSES = auto()
    READY = auto()
    STARTING = auto()
    MOVING_TO_POSE = auto()
    WAITING_MOTION = auto()
    STABILIZING = auto()
    MEASURING = auto()
    WAITING_MEASUREMENT = auto()
    LOGGING_RESULT = auto()
    NEXT_REPETITION = auto()
    NEXT_POSE = auto()
    COMPLETED = auto()
    ERROR = auto()


class StateMachine:
    """Experiment state machine"""
    
    def __init__(self):
        self.state = ExperimentState.IDLE
        self.state_start_time = time.time()
        self.transitions = {}
        self._setup_transitions()
        
    def _setup_transitions(self):
        """Setup valid state transitions"""
        self.transitions = {
            ExperimentState.IDLE: [ExperimentState.INITIALIZING],
            ExperimentState.INITIALIZING: [ExperimentState.LOADING_CONFIG, ExperimentState.ERROR],
            ExperimentState.LOADING_CONFIG: [ExperimentState.CONNECTING_ROBOT, ExperimentState.ERROR],
            ExperimentState.CONNECTING_ROBOT: [ExperimentState.GENERATING_POSES, ExperimentState.ERROR],
            ExperimentState.GENERATING_POSES: [ExperimentState.READY, ExperimentState.ERROR],
            ExperimentState.READY: [ExperimentState.STARTING],
            ExperimentState.STARTING: [ExperimentState.MOVING_TO_POSE, ExperimentState.ERROR],
            ExperimentState.MOVING_TO_POSE: [ExperimentState.WAITING_MOTION, ExperimentState.ERROR],
            ExperimentState.WAITING_MOTION: [ExperimentState.STABILIZING, ExperimentState.ERROR],
            ExperimentState.STABILIZING: [ExperimentState.MEASURING, ExperimentState.ERROR],
            ExperimentState.MEASURING: [ExperimentState.WAITING_MEASUREMENT, ExperimentState.ERROR],
            ExperimentState.WAITING_MEASUREMENT: [ExperimentState.LOGGING_RESULT, ExperimentState.ERROR],
            ExperimentState.LOGGING_RESULT: [ExperimentState.NEXT_REPETITION, ExperimentState.NEXT_POSE, ExperimentState.COMPLETED],
            ExperimentState.NEXT_REPETITION: [ExperimentState.MOVING_TO_POSE],
            ExperimentState.NEXT_POSE: [ExperimentState.MOVING_TO_POSE, ExperimentState.COMPLETED],
            ExperimentState.COMPLETED: [ExperimentState.IDLE],
            ExperimentState.ERROR: [ExperimentState.IDLE]
        }
        
    def transition_to(self, new_state: ExperimentState) -> bool:
        """Attempt to transition to new state"""
        if new_state in self.transitions.get(self.state, []):
            self.state = new_state
            self.state_start_time = time.time()
            return True
        return False
        
    def can_transition_to(self, state: ExperimentState) -> bool:
        """Check if transition is possible"""
        return state in self.transitions.get(self.state, [])
        
    def time_in_state(self) -> float:
        """Get time spent in current state"""
        return time.time() - self.state_start_time