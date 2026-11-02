"""AutoCert Python Research API - Robot-Agnostic Calibration Framework"""

from .sampling import (
    JointSampler,
    random_sampling,
    latin_hypercube_sampling,
    halton_sampling,
    grid_sampling,
    create_sampler
)

from .client import (
    AutoCertClient,
    WorkspaceSamplerClient,
    RobotInfo,
    JointLimits
)

from .visualization import (
    plot_workspace,
    plot_workspace_comparison,
    animate_workspace,
    plot_joint_distribution,
    plot_reachability_map,
    plot_calibration_results
)

from .utils import (
    save_results,
    load_results,
    extract_positions,
    extract_orientations,
    compute_workspace_metrics,
    filter_collision_samples,
    to_dataframe,
    from_dataframe
)

# from .calibration import (
#     CalibrationEngine,
#     KinematicCalibration,
#     ParameterEstimator,
#     CalibrationResult
# )

__all__ = [
    # Sampling
    'JointSampler',
    'random_sampling',
    'latin_hypercube_sampling',
    'halton_sampling',
    'grid_sampling',
    'create_sampler',
    
    # Client
    'AutoCertClient',
    'WorkspaceSamplerClient',
    'RobotInfo',
    'JointLimits',
    
    # Visualization
    'plot_workspace',
    'plot_workspace_comparison',
    'animate_workspace',
    'plot_joint_distribution',
    'plot_reachability_map',
    'plot_calibration_results',
    
    # Utils
    'save_results',
    'load_results',
    'extract_positions',
    'extract_orientations',
    'compute_workspace_metrics',
    'filter_collision_samples',
    'to_dataframe',
    'from_dataframe',
    
    # Calibration
    # 'CalibrationEngine',
    # 'KinematicCalibration',
    # 'ParameterEstimator',
    # 'CalibrationResult'
]