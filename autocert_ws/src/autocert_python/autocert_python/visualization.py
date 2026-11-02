"""Visualization utilities for workspace point clouds and calibration results"""

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import numpy.typing as npt
from typing import Optional, List, Tuple, Dict, Any
import matplotlib.animation as animation
from matplotlib.patches import FancyBboxPatch
from matplotlib.colors import LinearSegmentedColormap
import seaborn as sns


def set_publication_style():
    """Set matplotlib style for publication-quality figures"""
    plt.style.use('seaborn-v0_8-whitegrid')
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.size'] = 11
    plt.rcParams['axes.labelsize'] = 12
    plt.rcParams['axes.titlesize'] = 12
    plt.rcParams['xtick.labelsize'] = 10
    plt.rcParams['ytick.labelsize'] = 10
    plt.rcParams['legend.fontsize'] = 10
    plt.rcParams['figure.figsize'] = (10, 8)
    plt.rcParams['figure.dpi'] = 100
    plt.rcParams['savefig.dpi'] = 300
    plt.rcParams['savefig.bbox'] = 'tight'


def plot_workspace(
    positions: npt.NDArray,
    title: str = "End-Effector Workspace",
    figsize: Tuple[int, int] = (12, 8),
    alpha: float = 0.3,
    s: float = 1.0,
    color_by: Optional[npt.NDArray] = None,
    cmap: str = 'viridis',
    elev: float = 30,
    azim: float = 45,
    save_path: Optional[str] = None,
    show: bool = True,
    show_axes: bool = True,
    equal_aspect: bool = True
):
    """
    Plot 3D workspace point cloud with professional styling
    
    Args:
        positions: Array of positions (n_points x 3)
        title: Plot title
        figsize: Figure size
        alpha: Point transparency
        s: Point size
        color_by: Optional array for color mapping (e.g., reachability index)
        cmap: Colormap name
        elev: Elevation angle for 3D view
        azim: Azimuth angle for 3D view
        save_path: Path to save figure
        show: Whether to display the plot
        show_axes: Whether to show axes
        equal_aspect: Whether to set equal aspect ratio
    """
    set_publication_style()
    
    fig = plt.figure(figsize=figsize)
    ax = fig.add_subplot(111, projection='3d')
    
    if color_by is not None:
        scatter = ax.scatter(
            positions[:, 0],
            positions[:, 1],
            positions[:, 2],
            c=color_by,
            cmap=cmap,
            alpha=alpha,
            s=s,
            edgecolors='none'
        )
        plt.colorbar(scatter, ax=ax, label='Reachability Index', shrink=0.6)
    else:
        ax.scatter(
            positions[:, 0],
            positions[:, 1],
            positions[:, 2],
            alpha=alpha,
            s=s,
            c='blue',
            edgecolors='none'
        )
    
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_title(title)
    ax.view_init(elev=elev, azim=azim)
    
    if not show_axes:
        ax.set_axis_off()
    
    if equal_aspect and len(positions) > 0:
        # Set equal aspect ratio
        max_range = np.max([
            positions[:, 0].max() - positions[:, 0].min(),
            positions[:, 1].max() - positions[:, 1].min(),
            positions[:, 2].max() - positions[:, 2].min()
        ]) / 2.0
        
        mid_x = (positions[:, 0].max() + positions[:, 0].min()) / 2
        mid_y = (positions[:, 1].max() + positions[:, 1].min()) / 2
        mid_z = (positions[:, 2].max() + positions[:, 2].min()) / 2
        
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Figure saved to: {save_path}")
    
    if show:
        plt.show()
    else:
        plt.close(fig)
    
    return fig, ax


def plot_workspace_comparison(
    positions_list: List[npt.NDArray],
    labels: List[str],
    title: str = "Workspace Comparison",
    figsize: Tuple[int, int] = (15, 5),
    colors: Optional[List[str]] = None,
    alphas: Optional[List[float]] = None,
    save_path: Optional[str] = None
):
    """
    Compare multiple workspaces side by side
    
    Args:
        positions_list: List of position arrays
        labels: Labels for each workspace
        title: Overall title
        figsize: Figure size
        colors: Colors for each workspace
        alphas: Transparency for each workspace
        save_path: Path to save figure
    """
    set_publication_style()
    
    n_plots = len(positions_list)
    fig, axes = plt.subplots(1, n_plots, figsize=figsize, subplot_kw={'projection': '3d'})
    
    if n_plots == 1:
        axes = [axes]
    
    if colors is None:
        colors = plt.cm.tab10(np.linspace(0, 1, n_plots))
    
    if alphas is None:
        alphas = [0.3] * n_plots
    
    for i, (positions, label, color, alpha) in enumerate(zip(positions_list, labels, colors, alphas)):
        ax = axes[i]
        
        ax.scatter(
            positions[:, 0],
            positions[:, 1],
            positions[:, 2],
            c=[color],
            alpha=alpha,
            s=1.0,
            edgecolors='none'
        )
        
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')
        ax.set_title(label)
        ax.view_init(elev=30, azim=45)
        
        # Set consistent limits
        if i == 0:
            all_positions = np.vstack(positions_list)
            max_range = np.max([
                all_positions[:, 0].max() - all_positions[:, 0].min(),
                all_positions[:, 1].max() - all_positions[:, 1].min(),
                all_positions[:, 2].max() - all_positions[:, 2].min()
            ]) / 2.0
            
            mid_x = (all_positions[:, 0].max() + all_positions[:, 0].min()) / 2
            mid_y = (all_positions[:, 1].max() + all_positions[:, 1].min()) / 2
            mid_z = (all_positions[:, 2].max() + all_positions[:, 2].min()) / 2
            
            x_lim = (mid_x - max_range, mid_x + max_range)
            y_lim = (mid_y - max_range, mid_y + max_range)
            z_lim = (mid_z - max_range, mid_z + max_range)
        
        ax.set_xlim(x_lim)
        ax.set_ylim(y_lim)
        ax.set_zlim(z_lim)
    
    plt.suptitle(title)
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    
    plt.show()
    return fig, axes


def animate_workspace(
    positions: npt.NDArray,
    n_frames: int = 100,
    interval: int = 50,
    elev_range: Tuple[float, float] = (0, 90),
    azim_range: Tuple[float, float] = (0, 360),
    title: str = "Workspace Point Cloud",
    save_path: Optional[str] = None,
    figsize: Tuple[int, int] = (10, 8)
):
    """
    Create rotating animation of workspace for presentations
    
    Args:
        positions: Array of positions (n_points x 3)
        n_frames: Number of animation frames
        interval: Time between frames in ms
        elev_range: Range of elevation angles (min, max)
        azim_range: Range of azimuth angles (min, max)
        title: Animation title
        save_path: Path to save animation (GIF or MP4)
        figsize: Figure size
    """
    set_publication_style()
    
    fig = plt.figure(figsize=figsize)
    ax = fig.add_subplot(111, projection='3d')
    
    scatter = ax.scatter(
        positions[:, 0],
        positions[:, 1],
        positions[:, 2],
        alpha=0.3,
        s=1.0,
        c='blue',
        edgecolors='none'
    )
    
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_title(title)
    
    # Set equal aspect ratio
    max_range = np.max([
        positions[:, 0].max() - positions[:, 0].min(),
        positions[:, 1].max() - positions[:, 1].min(),
        positions[:, 2].max() - positions[:, 2].min()
    ]) / 2.0
    
    mid_x = (positions[:, 0].max() + positions[:, 0].min()) / 2
    mid_y = (positions[:, 1].max() + positions[:, 1].min()) / 2
    mid_z = (positions[:, 2].max() + positions[:, 2].min()) / 2
    
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    def update(frame):
        elev = elev_range[0] + (elev_range[1] - elev_range[0]) * frame / n_frames
        azim = azim_range[0] + (azim_range[1] - azim_range[0]) * frame / n_frames
        ax.view_init(elev=elev, azim=azim)
        return scatter,
    
    anim = animation.FuncAnimation(
        fig, update, frames=n_frames, interval=interval, blit=True
    )
    
    if save_path:
        if save_path.endswith('.gif'):
            anim.save(save_path, writer='pillow', fps=1000/interval)
            print(f"Animation saved to: {save_path}")
        else:
            anim.save(save_path, writer='ffmpeg', fps=1000/interval)
            print(f"Animation saved to: {save_path}")
    
    plt.show()
    return anim


def plot_joint_distribution(
    joint_samples: npt.NDArray,
    joint_names: List[str],
    title: str = "Joint Space Distribution",
    figsize: Optional[Tuple[int, int]] = None,
    save_path: Optional[str] = None
):
    """
    Plot joint value distributions with professional styling
    
    Args:
        joint_samples: Array of joint samples (n_samples x n_joints)
        joint_names: List of joint names
        title: Plot title
        figsize: Figure size
        save_path: Path to save figure
    """
    set_publication_style()
    
    n_joints = len(joint_names)
    n_cols = min(3, n_joints)
    n_rows = (n_joints + n_cols - 1) // n_cols
    
    if figsize is None:
        figsize = (15, 4 * n_rows)
    
    fig, axes = plt.subplots(n_rows, n_cols, figsize=figsize)
    axes = axes.flatten() if n_joints > 1 else [axes]
    
    for i in range(n_joints):
        axes[i].hist(
            joint_samples[:, i],
            bins=50,
            alpha=0.7,
            edgecolor='black',
            color='steelblue',
            density=True
        )
        axes[i].axvline(
            np.mean(joint_samples[:, i]),
            color='red',
            linestyle='--',
            linewidth=2,
            label=f'Mean: {np.mean(joint_samples[:, i]):.3f}'
        )
        axes[i].axvline(
            np.median(joint_samples[:, i]),
            color='green',
            linestyle=':',
            linewidth=2,
            label=f'Median: {np.median(joint_samples[:, i]):.3f}'
        )
        axes[i].set_xlabel(f'{joint_names[i]} [rad]')
        axes[i].set_ylabel('Density')
        axes[i].set_title(f'Distribution of {joint_names[i]}')
        axes[i].grid(True, alpha=0.3)
        axes[i].legend(loc='best', fontsize=8)
    
    # Hide unused subplots
    for i in range(n_joints, len(axes)):
        axes[i].set_visible(False)
    
    plt.suptitle(title, fontsize=14, fontweight='bold')
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    
    plt.show()


def plot_reachability_map(
    positions: npt.NDArray,
    reachability_scores: npt.NDArray,
    title: str = "Workspace Reachability Map",
    resolution: int = 50,
    save_path: Optional[str] = None
):
    """
    Create a 2D reachability map by projecting workspace
    
    Args:
        positions: Array of positions (n_points x 3)
        reachability_scores: Reachability scores for each point
        title: Plot title
        resolution: Grid resolution for 2D map
        save_path: Path to save figure
    """
    set_publication_style()
    
    # Create 2D grid in XY plane
    x = positions[:, 0]
    y = positions[:, 1]
    z = positions[:, 2]
    
    # Create grid
    xi = np.linspace(x.min(), x.max(), resolution)
    yi = np.linspace(y.min(), y.max(), resolution)
    xi, yi = np.meshgrid(xi, yi)
    
    # Interpolate reachability scores
    from scipy.interpolate import griddata
    zi = griddata(
        (x, y),
        reachability_scores,
        (xi, yi),
        method='linear',
        fill_value=0
    )
    
    # Create figure with multiple views
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    
    # XY plane
    ax1 = axes[0, 0]
    im1 = ax1.contourf(xi, yi, zi, levels=20, cmap='viridis')
    ax1.scatter(x, y, c=reachability_scores, cmap='viridis', s=1, alpha=0.3)
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_title('XY Projection')
    ax1.set_aspect('equal')
    plt.colorbar(im1, ax=ax1, label='Reachability')
    
    # XZ plane
    ax2 = axes[0, 1]
    zi_xz = griddata((x, z), reachability_scores, (xi, yi), method='linear', fill_value=0)
    im2 = ax2.contourf(xi, zi_xz, levels=20, cmap='viridis')
    ax2.scatter(x, z, c=reachability_scores, cmap='viridis', s=1, alpha=0.3)
    ax2.set_xlabel('X [m]')
    ax2.set_ylabel('Z [m]')
    ax2.set_title('XZ Projection')
    ax2.set_aspect('equal')
    plt.colorbar(im2, ax=ax2, label='Reachability')
    
    # YZ plane
    ax3 = axes[1, 0]
    zi_yz = griddata((y, z), reachability_scores, (xi, yi), method='linear', fill_value=0)
    im3 = ax3.contourf(zi_yz, yi, levels=20, cmap='viridis')
    ax3.scatter(y, z, c=reachability_scores, cmap='viridis', s=1, alpha=0.3)
    ax3.set_xlabel('Y [m]')
    ax3.set_ylabel('Z [m]')
    ax3.set_title('YZ Projection')
    ax3.set_aspect('equal')
    plt.colorbar(im3, ax=ax3, label='Reachability')
    
    # 3D view
    ax4 = axes[1, 1]
    ax4 = fig.add_subplot(224, projection='3d')
    scatter = ax4.scatter(
        x, y, z,
        c=reachability_scores,
        cmap='viridis',
        s=1,
        alpha=0.3
    )
    ax4.set_xlabel('X [m]')
    ax4.set_ylabel('Y [m]')
    ax4.set_zlabel('Z [m]')
    ax4.set_title('3D View')
    plt.colorbar(scatter, ax=ax4, label='Reachability', shrink=0.6)
    
    plt.suptitle(title, fontsize=14, fontweight='bold')
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    
    plt.show()


def plot_calibration_results(
    nominal_poses: npt.NDArray,
    measured_poses: npt.NDArray,
    calibrated_poses: npt.NDArray,
    title: str = "Calibration Results",
    save_path: Optional[str] = None
):
    """
    Visualize calibration results comparing nominal, measured, and calibrated poses
    
    Args:
        nominal_poses: Nominal (expected) poses
        measured_poses: Measured poses (e.g., from sensors)
        calibrated_poses: Poses after calibration
        title: Plot title
        save_path: Path to save figure
    """
    set_publication_style()
    
    fig = plt.figure(figsize=(15, 5))
    
    # Plot 1: Nominal vs Measured
    ax1 = fig.add_subplot(131, projection='3d')
    ax1.scatter(
        nominal_poses[:, 0],
        nominal_poses[:, 1],
        nominal_poses[:, 2],
        c='blue',
        label='Nominal',
        alpha=0.5,
        s=10
    )
    ax1.scatter(
        measured_poses[:, 0],
        measured_poses[:, 1],
        measured_poses[:, 2],
        c='red',
        label='Measured',
        alpha=0.5,
        s=10
    )
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_zlabel('Z [m]')
    ax1.set_title('Nominal vs Measured')
    ax1.legend()
    
    # Plot 2: Error vectors
    ax2 = fig.add_subplot(132, projection='3d')
    for i in range(min(100, len(nominal_poses))):  # Plot subset for clarity
        ax2.plot(
            [nominal_poses[i, 0], measured_poses[i, 0]],
            [nominal_poses[i, 1], measured_poses[i, 1]],
            [nominal_poses[i, 2], measured_poses[i, 2]],
            'k-',
            alpha=0.3
        )
    ax2.scatter(
        nominal_poses[:, 0],
        nominal_poses[:, 1],
        nominal_poses[:, 2],
        c='blue',
        alpha=0.3,
        s=10
    )
    ax2.set_xlabel('X [m]')
    ax2.set_ylabel('Y [m]')
    ax2.set_zlabel('Z [m]')
    ax2.set_title('Error Vectors')
    
    # Plot 3: Nominal vs Calibrated
    ax3 = fig.add_subplot(133, projection='3d')
    ax3.scatter(
        nominal_poses[:, 0],
        nominal_poses[:, 1],
        nominal_poses[:, 2],
        c='blue',
        label='Nominal',
        alpha=0.3,
        s=10
    )
    ax3.scatter(
        calibrated_poses[:, 0],
        calibrated_poses[:, 1],
        calibrated_poses[:, 2],
        c='green',
        label='Calibrated',
        alpha=0.5,
        s=10
    )
    ax3.set_xlabel('X [m]')
    ax3.set_ylabel('Y [m]')
    ax3.set_zlabel('Z [m]')
    ax3.set_title('Nominal vs Calibrated')
    ax3.legend()
    
    plt.suptitle(title, fontsize=14, fontweight='bold')
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    
    plt.show()


# Alias for backward compatibility
def plot_workspace_3d(*args, **kwargs):
    return plot_workspace(*args, **kwargs)