"""Report generation for certification"""

import json
import os
from datetime import datetime
from typing import Dict, Any, Optional

from .certificate import Certificate
from .limits import CertificationLimits


class ReportGenerator:
    """Generate certification reports"""
    
    def __init__(self, output_dir: str):
        self.output_dir = os.path.expanduser(output_dir)
        os.makedirs(self.output_dir, exist_ok=True)
        
    def generate_from_evaluation(self, evaluation_data: Dict[str, Any],
                                robot_metadata: Dict[str, Any],
                                limits: CertificationLimits) -> Certificate:
        """Generate certificate from evaluation data"""
        
        # Extract data
        overall = evaluation_data.get('overall', {})
        poses = evaluation_data.get('poses', {})
        
        # Create pose results
        pose_results = []
        for pose_id_str, metrics in poses.items():
            pose_id = int(pose_id_str)
            pose_results.append(PoseResult(
                pose_id=pose_id,
                pose_label=self._get_pose_label(pose_id, len(poses)),
                accuracy_mm=metrics.get('ap_distance_mm', 0.0),
                repeatability_mm=metrics.get('rp_distance_mm', 0.0),
                accuracy_x_mm=metrics.get('ap_x', 0.0) * 1000.0,
                accuracy_y_mm=metrics.get('ap_y', 0.0) * 1000.0,
                accuracy_z_mm=metrics.get('ap_z', 0.0) * 1000.0,
                orientation_error_deg=metrics.get('mean_angular_error_deg'),
                orientation_repeatability_deg=metrics.get('orientation_repeatability_deg'),
                num_samples=metrics.get('num_samples', 0),
                outliers_removed=metrics.get('outliers_removed', 0)
            ))
            
        # Get overall metrics
        mean_acc = overall.get('overall_mean_accuracy_mm', 0.0)
        max_acc = overall.get('overall_max_accuracy_mm', 0.0)
        mean_rep = overall.get('overall_mean_repeatability_mm', 0.0)
        max_rep = overall.get('overall_max_repeatability_mm', 0.0)
        mean_orient = overall.get('overall_mean_orientation_accuracy_deg', 0.0)
        mean_orient_rep = overall.get('overall_mean_orientation_repeatability_deg', 0.0)
        acc_ci = overall.get('accuracy_95_confidence_mm', 0.0)
        rep_ci = overall.get('repeatability_95_confidence_mm', 0.0)
        
        # Evaluate status
        status, issues = limits.evaluate(overall)
        
        # Create certificate
        now = datetime.now()
        cert = Certificate(
            metadata=CertificateMetadata(
                certificate_id="",  # Generated later
                issue_date=now.isoformat(),
                valid_until=datetime(now.year + 1, now.month, now.day).isoformat(),
                standard="ISO 9283",
                laboratory=robot_metadata.get('laboratory', 'AutoCert Lab'),
                technician=robot_metadata.get('technician', 'Automated System'),
                dataset_hash=evaluation_data.get('dataset_hash', ''),
                robot_model_hash=robot_metadata.get('model_hash', ''),
                srdf_hash=robot_metadata.get('srdf_hash', ''),
                controller_version=robot_metadata.get('controller_version', 'Unknown'),
                issues=issues
            ),
            robot=RobotUnderTest(
                manufacturer=robot_metadata.get('manufacturer', 'Unknown'),
                model=robot_metadata.get('model', 'Unknown'),
                serial_number=robot_metadata.get('serial_number', 'N/A'),
                software_version=robot_metadata.get('software_version', 'Unknown'),
                payload_kg=robot_metadata.get('payload_kg', 0.0),
                reach_m=robot_metadata.get('reach_m', 0.0)
            ),
            test_conditions=TestConditions(
                ambient_temperature_c=robot_metadata.get('temperature', 20.0),
                relative_humidity_percent=robot_metadata.get('humidity', 50.0),
                measurement_system=robot_metadata.get('measurement_system', 'Unknown'),
                measurement_system_calibration_date=robot_metadata.get('calibration_date', now.isoformat()),
                test_cube_size_mm=evaluation_data.get('test_cube_size_mm', 300.0),
                test_cube_center=evaluation_data.get('test_cube_center', [0, 0, 300]),
                repetitions_per_pose=evaluation_data.get('repetitions_per_pose', 30),
                outlier_threshold=evaluation_data.get('outlier_threshold', 3.0),
                confidence_level=0.95,
                pose_strategy=evaluation_data.get('pose_strategy', 'iso_cube')
            ),
            pose_results=pose_results,
            overall=OverallResults(
                mean_accuracy_mm=mean_acc,
                max_accuracy_mm=max_acc,
                mean_repeatability_mm=mean_rep,
                max_repeatability_mm=max_rep,
                mean_orientation_accuracy_deg=mean_orient,
                mean_orientation_repeatability_deg=mean_orient_rep,
                accuracy_95_confidence_mm=acc_ci,
                repeatability_95_confidence_mm=rep_ci,
                certification_status=status
            )
        )
        
        # Generate ID
        cert.metadata.certificate_id = cert.generate_id()
        
        return cert
        
    def save_json(self, certificate: Certificate, filename: str) -> str:
        """Save certificate as JSON"""
        filepath = os.path.join(self.output_dir, filename)
        with open(filepath, 'w') as f:
            json.dump(certificate.to_dict(), f, indent=2)
        return filepath
        
    def _get_pose_label(self, pose_id: int, total_poses: int) -> str:
        """Get pose label"""
        if total_poses == 5:
            labels = ["P1 (CENTER)", "P2 (+X)", "P3 (-X)", "P4 (+Y)", "P5 (-Y)"]
        elif total_poses == 7:
            labels = ["P1 (CENTER)", "P2 (+X)", "P3 (-X)", "P4 (+Y)", "P5 (-Y)", "P6 (+Z)", "P7 (-Z)"]
        else:
            labels = [f"P{i+1}" for i in range(total_poses)]
            
        return labels[pose_id] if pose_id < len(labels) else f"P{pose_id+1}"