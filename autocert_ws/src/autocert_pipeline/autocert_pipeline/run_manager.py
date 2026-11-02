"""Run management for AutoCert pipeline"""

import os
import subprocess
import sys
from datetime import datetime
from typing import Optional, Tuple


class RunManager:
    """Manage pipeline runs with timestamped directories"""
    
    def __init__(self, base_results_dir: str, robot_name: str):
        self.base_results_dir = os.path.expanduser(base_results_dir)
        self.robot_name = robot_name
        
        # Create timestamped run ID
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_id = f"{self.timestamp}_{robot_name}"
        
        # Create directory structure
        self.experiments_dir = os.path.join(self.base_results_dir, "experiments")
        self.evaluations_dir = os.path.join(self.base_results_dir, "evaluations")
        self.certificates_dir = os.path.join(self.base_results_dir, "certificates")
        self.logs_dir = os.path.join(self.base_results_dir, "pipeline_logs")
        
        os.makedirs(self.experiments_dir, exist_ok=True)
        os.makedirs(self.evaluations_dir, exist_ok=True)
        os.makedirs(self.certificates_dir, exist_ok=True)
        os.makedirs(self.logs_dir, exist_ok=True)
        
        self.experiment_file = os.path.join(self.experiments_dir, f"{self.run_id}.csv")
        self.evaluation_file = os.path.join(self.evaluations_dir, f"{self.run_id}.json")
        self.certificate_file = os.path.join(self.certificates_dir, f"{self.run_id}.json")
        self.pdf_file = os.path.join(self.certificates_dir, f"{self.run_id}.pdf")
        self.log_file = os.path.join(self.logs_dir, f"{self.run_id}.log")
        
    def run_experiment(self, config_file: str, robot_mode: str = "simulation") -> Optional[str]:
        """Run experiment node as subprocess"""
        cmd = [
            sys.executable, "-m", "autocert_experiment.experiment_node",
            "--config", config_file,
            "--output", self.experiment_file,
            "--robot-mode", robot_mode,
            "--run-id", self.run_id
        ]
        
        result = subprocess.run(cmd, capture_output=True, text=True)
        
        if result.returncode == 0 and os.path.exists(self.experiment_file):
            return self.experiment_file
        return None
        
    def run_evaluation(self, config_file: str) -> Optional[str]:
        """Run evaluation node as subprocess"""
        cmd = [
            sys.executable, "-m", "autocert_evaluation.evaluation_node",
            "--config", config_file,
            "--input", self.experiment_file,
            "--output", self.evaluation_file,
            "--run-id", self.run_id
        ]
        
        result = subprocess.run(cmd, capture_output=True, text=True)
        
        if result.returncode == 0 and os.path.exists(self.evaluation_file):
            return self.evaluation_file
        return None
        
    def run_certification(self, robot_metadata_file: str, limits_file: str) -> Optional[str]:
        """Run certification node as subprocess"""
        cmd = [
            sys.executable, "-m", "autocert_certification.certification_node",
            "--input", self.evaluation_file,
            "--robot-metadata", robot_metadata_file,
            "--limits", limits_file,
            "--output", self.certificate_file,
            "--pdf", self.pdf_file,
            "--run-id", self.run_id
        ]
        
        result = subprocess.run(cmd, capture_output=True, text=True)
        
        if result.returncode == 0 and os.path.exists(self.certificate_file):
            return self.certificate_file
        return None
        
    def write_log(self, message: str):
        """Write to pipeline log file"""
        with open(self.log_file, 'a') as f:
            timestamp = datetime.now().isoformat()
            f.write(f"[{timestamp}] {message}\n")