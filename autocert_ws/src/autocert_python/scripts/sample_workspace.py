#!/usr/bin/env python3
"""Complete workspace sampling script for AutoCert - Robot-Agnostic"""

import rclpy
import argparse
import sys
import os
import time
import numpy as np

# Add parent directory to path for imports when running as script
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from autocert_python.client import AutoCertClient
from autocert_python.sampling import SAMPLING_STRATEGIES
from autocert_python.visualization import (
    plot_workspace, 
    plot_joint_distribution,
    plot_reachability_map
)
from autocert_python.utils import save_results, compute_workspace_metrics


def main():
    """Main execution function for workspace sampling"""
    
    parser = argparse.ArgumentParser(
        description='Robot-agnostic workspace sampling for AutoCert',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --samples 10000 --strategy halton --visualize
  %(prog)s --samples 5000 --collision-check --save results.json
  %(prog)s --robot-name my_robot --planning-group arm_group
        """
    )
    
    # Sampling parameters
    parser.add_argument('--samples', type=int, default=5000,
                       help='Number of joint samples to generate')
    parser.add_argument('--strategy', type=str, default='random',
                       choices=['random', 'latin', 'halton', 'grid'],
                       help='Sampling strategy for joint space')
    parser.add_argument('--collision-check', action='store_true',
                       help='Enable collision checking (filters invalid configs)')
    
    # Robot configuration
    parser.add_argument('--robot-name', type=str, default=None,
                       help='Override robot name (optional)')
    parser.add_argument('--planning-group', type=str, default=None,
                       help='Specify planning group (uses default if not specified)')
    parser.add_argument('--end-effector', type=str, default=None,
                       help='Specify end-effector link (auto-detected if not specified)')
    
    # Output options
    parser.add_argument('--visualize', action='store_true',
                       help='Show interactive visualizations')
    parser.add_argument('--save', type=str, default=None,
                       help='Save results to file (auto-generated if not specified)')
    parser.add_argument('--format', type=str, default='json',
                       choices=['json', 'npz'],
                       help='Output format (json or compressed npz)')
    
    # Performance options
    parser.add_argument('--service-timeout', type=float, default=60.0,
                       help='Timeout for service calls in seconds')
    parser.add_argument('--seed', type=int, default=None,
                       help='Random seed for reproducibility')
    parser.add_argument('--verbose', action='store_true',
                       help='Print detailed progress information')
    
    args = parser.parse_args()
    
    # Set random seed
    if args.seed is not None:
        np.random.seed(args.seed)
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create client
        client = AutoCertClient('workspace_sampler')
        
        # Get robot information dynamically
        print("\n" + "="*60)
        print("AUTOCERT WORKSPACE SAMPLING")
        print("="*60)
        
        robot_info = client.get_robot_info()
        
        print(f"\nRobot Information:")
        print(f"  Name: {robot_info.robot_name}")
        print(f"  Planning Group: {robot_info.planning_group}")
        print(f"  End-Effector Link: {robot_info.end_effector_link}")
        print(f"  Number of Joints: {robot_info.num_joints}")
        print(f"  Joint Names: {', '.join(robot_info.joint_names[:5])}...")
        print(f"  Collision Checking Available: {robot_info.collision_checking_available}")
        
        # Override if specified
        if args.planning_group:
            print(f"\n⚠️  Overriding planning group to: {args.planning_group}")
            # Note: In a full implementation, you'd call a service to change the group
        
        # Get joint limits
        limits = client.get_joint_limits()
        
        print(f"\nJoint Limits (first 3 joints):")
        for i, name in enumerate(limits.names[:3]):
            print(f"  {name}: [{limits.min_positions[i]:.4f}, {limits.max_positions[i]:.4f}] rad")
        
        print(f"\nSampling Configuration:")
        print(f"  Strategy: {args.strategy}")
        print(f"  Samples: {args.samples:,}")
        print(f"  Collision Checking: {'Enabled' if args.collision_check else 'Disabled'}")
        
        # Generate samples using our Python sampler
        print(f"\nGenerating {args.samples:,} samples...")
        start_time = time.time()
        
        from autocert_python.sampling import JointSampler
        sampler = JointSampler(
            (limits.min_positions, limits.max_positions),
            limits.names
        )
        
        joint_samples = sampler.generate_samples(
            args.samples, 
            args.strategy,
            seed=args.seed
        )
        
        sampling_time = time.time() - start_time
        print(f"  Sampling completed in {sampling_time:.2f} seconds")
        print(f"  Rate: {args.samples/sampling_time:.1f} samples/second")
        
        # Compute FK
        print(f"\nComputing forward kinematics...")
        fk_start = time.time()
        
        positions, orientations, valid_indices, comp_time = client.compute_fk(
            joint_samples,
            filter_collisions=args.collision_check,
            timeout_sec=args.service_timeout
        )
        
        fk_time = time.time() - fk_start
        
        # Statistics
        valid_count = len(valid_indices)
        invalid_count = args.samples - valid_count if args.collision_check else 0
        reachability = 100.0 * valid_count / args.samples if args.samples > 0 else 0.0
        
        print(f"\nResults:")
        print(f"  FK Computation: {fk_time:.2f} seconds")
        print(f"  Rate: {args.samples/fk_time:.1f} samples/second")
        print(f"  Valid Configurations: {valid_count:,}/{args.samples:,} ({reachability:.1f}%)")
        if args.collision_check:
            print(f"  Invalid/Colliding: {invalid_count:,} ({100*invalid_count/args.samples:.1f}%)")
        
        # Compute workspace metrics
        if len(positions) > 0:
            metrics = compute_workspace_metrics(positions)
            print(f"\nWorkspace Metrics:")
            print(f"  X Range: [{metrics['x_center']-metrics['x_range']/2:.3f}, "
                  f"{metrics['x_center']+metrics['x_range']/2:.3f}] m")
            print(f"  Y Range: [{metrics['y_center']-metrics['y_range']/2:.3f}, "
                  f"{metrics['y_center']+metrics['y_range']/2:.3f}] m")
            print(f"  Z Range: [{metrics['z_center']-metrics['z_range']/2:.3f}, "
                  f"{metrics['z_center']+metrics['z_range']/2:.3f}] m")
            if 'convex_hull_volume' in metrics:
                print(f"  Convex Hull Volume: {metrics['convex_hull_volume']:.4f} m³")
        
        # Save results
        if args.save is not None or args.visualize:
            # Prepare poses array (positions + orientations)
            poses_array = np.hstack([positions, orientations]) if len(orientations) > 0 else positions
            
            # Filter to valid samples if collision checking was enabled
            if args.collision_check and valid_indices:
                valid_joint_samples = joint_samples[valid_indices]
            else:
                valid_joint_samples = joint_samples
                valid_indices = list(range(len(joint_samples)))
            
            # Save to file
            if args.save:
                compression = (args.format == 'npz')
                save_results(
                    robot_name=robot_info.robot_name,
                    joint_samples=valid_joint_samples,
                    poses=poses_array,
                    sampling_strategy=args.strategy,
                    filename=args.save,
                    compression=compression,
                    metadata={
                        'collision_checking': args.collision_check,
                        'end_effector_link': robot_info.end_effector_link,
                        'planning_group': robot_info.planning_group,
                        'valid_count': valid_count,
                        'invalid_count': invalid_count,
                        'reachability': reachability,
                        'sampling_time': sampling_time,
                        'fk_time': fk_time,
                        'seed': args.seed,
                        'metrics': metrics
                    }
                )
        
        # Visualize
        if args.visualize and len(positions) > 0:
            print("\nGenerating visualizations...")
            
            # Plot workspace
            plot_workspace(
                positions,
                title=f"{robot_info.robot_name.capitalize()} Workspace\n"
                      f"({args.strategy} sampling, {valid_count:,} points, "
                      f"{reachability:.1f}% reachable)",
                alpha=0.3,
                s=0.5,
                color_by=np.linspace(0, 1, len(positions)) if args.collision_check else None
            )
            
            # Plot joint distributions for valid samples
            valid_joint_samples = joint_samples[valid_indices] if valid_indices else joint_samples
            plot_joint_distribution(
                valid_joint_samples,
                robot_info.joint_names,
                title=f"Joint Space Distribution - {robot_info.robot_name}"
            )
            
            # Plot reachability map if enough points
            if len(positions) > 100:
                plot_reachability_map(
                    positions,
                    np.ones(len(positions)),  # Placeholder reachability scores
                    title=f"Workspace Reachability Map - {robot_info.robot_name}"
                )
        
        print("\n" + "="*60)
        print("WORKSPACE SAMPLING COMPLETED SUCCESSFULLY")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        if args.verbose:
            import traceback
            traceback.print_exc()
    finally:
        if 'client' in locals():
            client.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()