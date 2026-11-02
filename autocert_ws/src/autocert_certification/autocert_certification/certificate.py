"""Certificate data structures"""

from dataclasses import dataclass, asdict
from typing import List, Dict, Any, Optional
from datetime import datetime
import hashlib


@dataclass
class CertificateMetadata:
    """Certificate metadata"""
    certificate_id: str
    issue_date: str
    valid_until: str
    standard: str
    laboratory: str
    technician: str
    dataset_hash: str
    robot_model_hash: str
    srdf_hash: str
    controller_version: str
    issues: List[str]


@dataclass
class RobotUnderTest:
    """Robot information"""
    manufacturer: str
    model: str
    serial_number: str
    software_version: str
    payload_kg: float
    reach_m: float


@dataclass
class TestConditions:
    """Test conditions"""
    ambient_temperature_c: float
    relative_humidity_percent: float
    measurement_system: str
    measurement_system_calibration_date: str
    test_cube_size_mm: float
    test_cube_center: List[float]
    repetitions_per_pose: int
    outlier_threshold: float
    confidence_level: float
    pose_strategy: str


@dataclass
class PoseResult:
    """Results for a single pose"""
    pose_id: int
    pose_label: str
    accuracy_mm: float
    repeatability_mm: float
    accuracy_x_mm: float
    accuracy_y_mm: float
    accuracy_z_mm: float
    orientation_error_deg: Optional[float]
    orientation_repeatability_deg: Optional[float]
    num_samples: int
    outliers_removed: int


@dataclass
class OverallResults:
    """Overall certification results"""
    mean_accuracy_mm: float
    max_accuracy_mm: float
    mean_repeatability_mm: float
    max_repeatability_mm: float
    mean_orientation_accuracy_deg: Optional[float]
    mean_orientation_repeatability_deg: Optional[float]
    accuracy_95_confidence_mm: float
    repeatability_95_confidence_mm: float
    certification_status: str


@dataclass
class Certificate:
    """Complete ISO 9283 certificate"""
    metadata: CertificateMetadata
    robot: RobotUnderTest
    test_conditions: TestConditions
    pose_results: List[PoseResult]
    overall: OverallResults
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary"""
        return {
            'metadata': asdict(self.metadata),
            'robot': asdict(self.robot),
            'test_conditions': asdict(self.test_conditions),
            'pose_results': [asdict(r) for r in self.pose_results],
            'overall': asdict(self.overall)
        }
        
    def to_json(self) -> str:
        """Convert to JSON string"""
        import json
        return json.dumps(self.to_dict(), indent=2)
        
    def generate_id(self) -> str:
        """Generate certificate ID"""
        hash_input = f"{self.robot.manufacturer}{self.robot.model}{self.robot.serial_number}{self.metadata.issue_date}"
        hash_id = hashlib.md5(hash_input.encode()).hexdigest()[:8].upper()
        date_str = self.metadata.issue_date.replace('-', '')[:8]
        return f"AUTOCERT-{date_str}-{hash_id}"