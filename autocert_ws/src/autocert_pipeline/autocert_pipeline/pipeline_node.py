#!/usr/bin/env python3

"""Pipeline orchestrator node for AutoCert"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import argparse
import sys
import os
from datetime import datetime

from .config_loader import ConfigLoader
from .run_manager import RunManager


class PipelineNode(Node):
    """Orchestrate the complete certification pipeline"""
    
    def __init__(self):
        super().__init__('pipeline_node')
        
        # Parameters
        self.declare_parameter('master_config', '')
        self.declare_parameter('robot_mode', 'simulation')
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/pipeline_status', 10)
        
        self.get_logger().info("Pipeline Node initialized")
        
    def run_pipeline(self, config_path: str, robot_mode: str) -> bool:
        """Execute the complete certification pipeline"""
        
        # Load configuration
        self.get_logger().info(f"Loading master config from {config_path}")
        config_loader = ConfigLoader(config_path)
        if not config_loader.load():
            self.get_logger().error("Failed to load master configuration")
            return False
            
        # Get configuration values
        results_dir = config_loader.get('results_dir', '~/autocert_data')
        robot_metadata = config_loader.get('robot_metadata_file', '')
        limits_file = config_loader.get('limits_file', '')
        experiment_config = config_loader.get('experiment_config', '')
        evaluation_config = config_loader.get('evaluation_config', '')
        robot_name = config_loader.get('robot_name', 'robot')
        
        # Initialize run manager
        run_mgr = RunManager(results_dir, robot_name)
        
        # Stage 1: Experiment
        self.get_logger().info(f"Starting experiment (run ID: {run_mgr.run_id})")
        run_mgr.write_log("Starting experiment")
        self.status_pub.publish(String(data=f"EXPERIMENT_START:{run_mgr.run_id}"))
        
        experiment_file = run_mgr.run_experiment(experiment_config, robot_mode)
        if not experiment_file:
            self.get_logger().error("Experiment failed")
            run_mgr.write_log("Experiment FAILED")
            self.status_pub.publish(String(data="EXPERIMENT_FAILED"))
            return False
            
        self.get_logger().info(f"Experiment complete: {experiment_file}")
        run_mgr.write_log(f"Experiment complete: {experiment_file}")
        self.status_pub.publish(String(data=f"EXPERIMENT_COMPLETE:{experiment_file}"))
        
        # Stage 2: Evaluation
        self.get_logger().info("Starting evaluation")
        run_mgr.write_log("Starting evaluation")
        self.status_pub.publish(String(data=f"EVALUATION_START:{run_mgr.run_id}"))
        
        evaluation_file = run_mgr.run_evaluation(evaluation_config)
        if not evaluation_file:
            self.get_logger().error("Evaluation failed")
            run_mgr.write_log("Evaluation FAILED")
            self.status_pub.publish(String(data="EVALUATION_FAILED"))
            return False
            
        self.get_logger().info(f"Evaluation complete: {evaluation_file}")
        run_mgr.write_log(f"Evaluation complete: {evaluation_file}")
        self.status_pub.publish(String(data=f"EVALUATION_COMPLETE:{evaluation_file}"))
        
        # Stage 3: Certification
        self.get_logger().info("Starting certification")
        run_mgr.write_log("Starting certification")
        self.status_pub.publish(String(data=f"CERTIFICATION_START:{run_mgr.run_id}"))
        
        certificate_file = run_mgr.run_certification(robot_metadata, limits_file)
        if not certificate_file:
            self.get_logger().error("Certification failed")
            run_mgr.write_log("Certification FAILED")
            self.status_pub.publish(String(data="CERTIFICATION_FAILED"))
            return False
            
        self.get_logger().info(f"Certification complete: {certificate_file}")
        run_mgr.write_log(f"Certification complete: {certificate_file}")
        self.status_pub.publish(String(data=f"CERTIFICATION_COMPLETE:{certificate_file}"))
        
        # Pipeline complete
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("PIPELINE COMPLETE")
        self.get_logger().info("="*60)
        self.get_logger().info(f"Run ID: {run_mgr.run_id}")
        self.get_logger().info(f"Experiment: {experiment_file}")
        self.get_logger().info(f"Evaluation: {evaluation_file}")
        self.get_logger().info(f"Certificate: {certificate_file}")
        self.get_logger().info(f"PDF Report: {run_mgr.pdf_file}")
        self.get_logger().info("="*60)
        
        run_mgr.write_log("Pipeline completed successfully")
        self.status_pub.publish(String(data=f"PIPELINE_COMPLETE:{run_mgr.run_id}"))
        
        return True


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='AutoCert Pipeline Node')
    parser.add_argument('--config', type=str, required=True,
                       help='Path to master configuration file')
    parser.add_argument('--robot-mode', type=str, default='simulation',
                       choices=['simulation', 'real'],
                       help='Robot mode: simulation or real')
    
    args = parser.parse_args()
    
    node = PipelineNode()
    
    # Run pipeline
    success = node.run_pipeline(args.config, args.robot_mode)
    
    node.destroy_node()
    rclpy.shutdown()
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()