"""Measurement logging for ISO 9283 experiments"""

import csv
import os
from datetime import datetime
from typing import List, Dict
from geometry_msgs.msg import Pose
from autocert_interfaces.msg import PoseMeasurement


class MeasurementLogger:
    """Log measurements to CSV file"""
    
    REQUIRED_FIELDS = [
        'pose_id', 'repetition', 'timestamp',
        'cmd_x', 'cmd_y', 'cmd_z',
        'cmd_qx', 'cmd_qy', 'cmd_qz', 'cmd_qw',
        'meas_x', 'meas_y', 'meas_z',
        'meas_qx', 'meas_qy', 'meas_qz', 'meas_qw',
        'measurement_system', 'measurement_quality'
    ]
    
    def __init__(self, output_file: str, robot_name: str, run_id: str):
        self.output_file = output_file
        self.robot_name = robot_name
        self.run_id = run_id
        self.measurements = []
        
        # Create directory if needed
        os.makedirs(os.path.dirname(output_file), exist_ok=True)
        
        # Initialize CSV with header
        with open(output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(self.REQUIRED_FIELDS)
            
    def log_measurement(self, msg: PoseMeasurement):
        """Log a single measurement"""
        self.measurements.append(msg)
        
        # Append to CSV
        with open(self.output_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                msg.pose_id,
                msg.repetition,
                msg.timestamp.sec + msg.timestamp.nanosec * 1e-9,
                msg.commanded_pose.position.x,
                msg.commanded_pose.position.y,
                msg.commanded_pose.position.z,
                msg.commanded_pose.orientation.x,
                msg.commanded_pose.orientation.y,
                msg.commanded_pose.orientation.z,
                msg.commanded_pose.orientation.w,
                msg.measured_pose.position.x,
                msg.measured_pose.position.y,
                msg.measured_pose.position.z,
                msg.measured_pose.orientation.x,
                msg.measured_pose.orientation.y,
                msg.measured_pose.orientation.z,
                msg.measured_pose.orientation.w,
                msg.measurement_system,
                msg.measurement_quality
            ])
            
    def get_summary(self) -> Dict:
        """Get logging summary"""
        return {
            'output_file': self.output_file,
            'robot_name': self.robot_name,
            'run_id': self.run_id,
            'num_measurements': len(self.measurements),
            'num_poses': len(set(m.pose_id for m in self.measurements))
        }