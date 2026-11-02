#!/usr/bin/env python3
"""Example experiment script for AutoCert"""

import rclpy
import sys
import os
import numpy as np

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from autocert_python.client import AutoCertClient
from autocert_python.sampling import JointSampler
from autocert_python.visualization import plot_workspace_comparison
from autocert_python.utils import save_results, compute_workspace_metrics


def run_comparison_experiment():
    """Run sampling strategy comparison experiment"""
    
    print("\n" + "="*60)
    print("AUTOCERT - Sampling Strategy Comparison Experiment")
    print("="*60)
    
    rclpy.init()
    client = AutoCertClient('experiment_node')
    
    try:
        # Get robot info
        robot_info = client.get_robot_info()
        limits = client.get_joint_limits()
        
        print(f"\nRobot: {robot_info.robot_name}")
        print(f"Joints: {robot_info.num_joints}")
        
        # Parameters
        n_samples = 5000
        strategies = ['random', 'latin', 'halton']
        
        results = {}
        positions_dict = {}
        
        # Test each strategy
        for strategy in strategies:
            print(f"\nTesting {strategy} sampling...")
            
            # Generate samples
            sampler = JointSampler(
                (limits.min_positions, limits.max_positions),
                limits.names
            )
            joint_samples = sampler.generate_samples(n_samples, strategy)
            
            # Compute FK
            positions, orientations, valid_indices, comp_time = client.compute_fk(
                joint_samples,
                filter_collisions=True
            )
            
            # Store results
            valid_count = len(valid_indices)
            reachability = 100.0 * valid_count / n_samples
            
            results[strategy] = {
                'valid_count': valid_count,
                'reachability': reachability,
                'comp_time': comp_time,
                'positions': positions
            }
            
            positions_dict[strategy] = positions
            
            print(f"  Valid: {valid_count}/{n_samples} ({reachability:.1f}%)")
            print(f"  Time: {comp_time:.2f} ms")
        
        # Visualize comparison
        plot_workspace_comparison(
            [results['random']['positions'],
             results['latin']['positions'],
             results['halton']['positions']],
            ['Random', 'Latin Hypercube', 'Halton'],
            title=f"Sampling Strategy Comparison - {robot_info.robot_name}"
        )
        
        # Print summary
        print("\n" + "="*60)
        print("EXPERIMENT SUMMARY")
        print("="*60)
        print(f"{'Strategy':<10} {'Valid':<10} {'Reachability':<15} {'Time (ms)':<10}")
        print("-"*50)
        for strategy in strategies:
            r = results[strategy]
            print(f"{strategy:<10} {r['valid_count']:<10} {r['reachability']:<14.1f}% {r['comp_time']:<10.2f}")
        
        # Save results
        save_results(
            robot_name=robot_info.robot_name,
            joint_samples=np.array([]),  # Not saving all samples for brevity
            poses=positions_dict['halton'],
            sampling_strategy='comparison',
            filename='strategy_comparison.json',
            metadata={'results': results}
        )
        
    finally:
        client.close()
        rclpy.shutdown()


if __name__ == '__main__':
    run_comparison_experiment()