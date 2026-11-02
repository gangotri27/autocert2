"""Joint space sampling strategies for workspace generation"""

import numpy as np
from scipy.stats import qmc
from typing import List, Tuple, Optional, Union, Callable
import numpy.typing as npt


class JointSampler:
    """Base class for joint space sampling"""
    
    def __init__(
        self, 
        joint_limits: Tuple[npt.NDArray, npt.NDArray],
        joint_names: Optional[List[str]] = None
    ):
        """
        Initialize sampler with joint limits
        
        Args:
            joint_limits: Tuple of (min_limits, max_limits) arrays
            joint_names: Optional list of joint names
        """
        self.min_limits = np.array(joint_limits[0], dtype=np.float64)
        self.max_limits = np.array(joint_limits[1], dtype=np.float64)
        self.n_joints = len(self.min_limits)
        self.joint_names = joint_names or [f"J{i}" for i in range(self.n_joints)]
        
        # Validate
        if len(self.max_limits) != self.n_joints:
            raise ValueError("min_limits and max_limits must have same length")
        
        if any(self.min_limits >= self.max_limits):
            raise ValueError("min_limits must be less than max_limits")
    
    def random_sampling(self, n_samples: int) -> npt.NDArray:
        """Uniform random sampling within joint limits"""
        samples = np.random.uniform(
            low=self.min_limits,
            high=self.max_limits,
            size=(n_samples, self.n_joints)
        )
        return samples
    
    def latin_hypercube_sampling(self, n_samples: int) -> npt.NDArray:
        """Latin Hypercube sampling for better space coverage"""
        sampler = qmc.LatinHypercube(d=self.n_joints)
        samples = sampler.random(n=n_samples)
        
        # Scale to joint limits
        scaled_samples = qmc.scale(
            samples,
            self.min_limits,
            self.max_limits
        )
        return scaled_samples
    
    def halton_sampling(self, n_samples: int) -> npt.NDArray:
        """Halton sequence sampling for low-discrepancy sequences"""
        sampler = qmc.Halton(d=self.n_joints, scramble=True)
        samples = sampler.random(n=n_samples)
        
        # Scale to joint limits
        scaled_samples = qmc.scale(
            samples,
            self.min_limits,
            self.max_limits
        )
        return scaled_samples
    
    def grid_sampling(self, n_samples: int) -> npt.NDArray:
        """
        Grid-based sampling (approximate).
        Creates a grid with roughly n_samples points.
        """
        # Calculate grid dimensions
        n_per_dim = int(np.ceil(n_samples ** (1.0 / self.n_joints)))
        
        # Create 1D grids for each dimension
        grids = []
        for i in range(self.n_joints):
            grid = np.linspace(self.min_limits[i], self.max_limits[i], n_per_dim)
            grids.append(grid)
        
        # Create meshgrid and reshape
        mesh = np.meshgrid(*grids, indexing='ij')
        samples = np.stack([m.flatten() for m in mesh], axis=1)
        
        # Return exactly n_samples if possible, otherwise truncate
        if len(samples) > n_samples:
            indices = np.random.choice(len(samples), n_samples, replace=False)
            samples = samples[indices]
        
        return samples
    
    def generate_samples(
        self,
        n_samples: int,
        strategy: str = 'random',
        seed: Optional[int] = None
    ) -> npt.NDArray:
        """
        Generate joint samples using specified strategy
        
        Args:
            n_samples: Number of samples to generate
            strategy: Sampling strategy ('random', 'latin', 'halton', 'grid')
            seed: Random seed for reproducibility
            
        Returns:
            Array of joint configurations (n_samples x n_joints)
        """
        if seed is not None:
            np.random.seed(seed)
        
        strategies = {
            'random': self.random_sampling,
            'latin': self.latin_hypercube_sampling,
            'halton': self.halton_sampling,
            'grid': self.grid_sampling
        }
        
        if strategy not in strategies:
            raise ValueError(
                f"Unknown strategy: {strategy}. "
                f"Available: {list(strategies.keys())}"
            )
        
        return strategies[strategy](n_samples)
    
    def generate_biased_samples(
        self,
        n_samples: int,
        bias_center: npt.NDArray,
        bias_strength: float = 1.0,
        strategy: str = 'random'
    ) -> npt.NDArray:
        """
        Generate samples biased toward a specific region
        
        Args:
            n_samples: Number of samples
            bias_center: Center of bias (joint configuration)
            bias_strength: Strength of bias (0 = uniform, >0 = biased)
            strategy: Base sampling strategy
            
        Returns:
            Biased samples
        """
        base_samples = self.generate_samples(n_samples, strategy)
        
        # Apply bias by moving samples toward center
        bias_center = np.array(bias_center)
        if bias_center.shape != (self.n_joints,):
            raise ValueError(f"bias_center must have shape ({self.n_joints},)")
        
        # Linear interpolation toward center
        weights = np.random.beta(1, bias_strength + 1, size=n_samples)
        weights = weights.reshape(-1, 1)
        
        biased_samples = (1 - weights) * base_samples + weights * bias_center
        
        # Ensure within limits
        biased_samples = np.clip(biased_samples, self.min_limits, self.max_limits)
        
        return biased_samples
    
    def get_limits_matrix(self) -> Tuple[npt.NDArray, npt.NDArray]:
        """Get limits as matrices for linear algebra operations"""
        return self.min_limits, self.max_limits
    
    def normalize_samples(self, samples: npt.NDArray) -> npt.NDArray:
        """Normalize samples to [0, 1] range"""
        return (samples - self.min_limits) / (self.max_limits - self.min_limits)
    
    def denormalize_samples(self, normalized_samples: npt.NDArray) -> npt.NDArray:
        """Convert normalized samples back to joint space"""
        return normalized_samples * (self.max_limits - self.min_limits) + self.min_limits
    
    def save_config(self, filename: str):
        """Save sampler configuration to file"""
        config = {
            'joint_names': self.joint_names,
            'min_limits': self.min_limits.tolist(),
            'max_limits': self.max_limits.tolist(),
            'n_joints': self.n_joints
        }
        
        import json
        with open(filename, 'w') as f:
            json.dump(config, f, indent=2)
    
    @classmethod
    def load_config(cls, filename: str) -> 'JointSampler':
        """Load sampler configuration from file"""
        import json
        with open(filename, 'r') as f:
            config = json.load(f)
        
        return cls(
            (np.array(config['min_limits']), np.array(config['max_limits'])),
            config.get('joint_names')
        )


# Convenience functions
def random_sampling(
    joint_limits: Tuple[npt.NDArray, npt.NDArray],
    n_samples: int,
    joint_names: Optional[List[str]] = None
) -> npt.NDArray:
    """Convenience function for random sampling"""
    sampler = JointSampler(joint_limits, joint_names)
    return sampler.random_sampling(n_samples)


def latin_hypercube_sampling(
    joint_limits: Tuple[npt.NDArray, npt.NDArray],
    n_samples: int,
    joint_names: Optional[List[str]] = None
) -> npt.NDArray:
    """Convenience function for Latin Hypercube sampling"""
    sampler = JointSampler(joint_limits, joint_names)
    return sampler.latin_hypercube_sampling(n_samples)


def halton_sampling(
    joint_limits: Tuple[npt.NDArray, npt.NDArray],
    n_samples: int,
    joint_names: Optional[List[str]] = None
) -> npt.NDArray:
    """Convenience function for Halton sequence sampling"""
    sampler = JointSampler(joint_limits, joint_names)
    return sampler.halton_sampling(n_samples)


def grid_sampling(
    joint_limits: Tuple[npt.NDArray, npt.NDArray],
    n_samples: int,
    joint_names: Optional[List[str]] = None
) -> npt.NDArray:
    """Convenience function for grid sampling"""
    sampler = JointSampler(joint_limits, joint_names)
    return sampler.grid_sampling(n_samples)


def create_sampler(
    joint_limits: Tuple[npt.NDArray, npt.NDArray],
    strategy: str = 'random',
    joint_names: Optional[List[str]] = None
) -> JointSampler:
    """Create a sampler with specified default strategy"""
    return JointSampler(joint_limits, joint_names)


# Strategy mapping for easy access
SAMPLING_STRATEGIES = {
    'random': random_sampling,
    'latin': latin_hypercube_sampling,
    'halton': halton_sampling,
    'grid': grid_sampling
}