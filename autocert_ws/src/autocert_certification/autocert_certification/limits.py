"""Certification limits for ISO 9283"""

from dataclasses import dataclass
from typing import Dict, Tuple, List


@dataclass
class CertificationLimits:
    """Robot specification limits"""
    
    # Position accuracy (mm)
    accuracy_limit_mm: float = 1.0
    accuracy_warning_mm: float = 0.8
    
    # Position repeatability (mm)
    repeatability_limit_mm: float = 0.1
    repeatability_warning_mm: float = 0.05
    
    # Orientation accuracy (degrees)
    orientation_accuracy_limit_deg: float = 0.5
    orientation_repeatability_limit_deg: float = 0.1
    
    def load_from_dict(self, data: Dict):
        """Load limits from dictionary"""
        if 'accuracy_limit_mm' in data:
            self.accuracy_limit_mm = data['accuracy_limit_mm']
        if 'accuracy_warning_mm' in data:
            self.accuracy_warning_mm = data['accuracy_warning_mm']
        if 'repeatability_limit_mm' in data:
            self.repeatability_limit_mm = data['repeatability_limit_mm']
        if 'repeatability_warning_mm' in data:
            self.repeatability_warning_mm = data['repeatability_warning_mm']
        if 'orientation_accuracy_limit_deg' in data:
            self.orientation_accuracy_limit_deg = data['orientation_accuracy_limit_deg']
        if 'orientation_repeatability_limit_deg' in data:
            self.orientation_repeatability_limit_deg = data['orientation_repeatability_limit_deg']
            
    def evaluate(self, metrics: Dict) -> Tuple[str, List[str]]:
        """
        Evaluate metrics against limits
        
        Returns:
            (status, issues) where status is "PASS", "CONDITIONAL", or "FAIL"
        """
        issues = []
        
        # Check max accuracy
        max_acc = metrics.get('overall_max_accuracy_mm', 0)
        if max_acc > self.accuracy_limit_mm:
            issues.append(f"Max accuracy {max_acc:.3f}mm exceeds limit {self.accuracy_limit_mm:.3f}mm")
            return "FAIL", issues
            
        # Check max repeatability
        max_rep = metrics.get('overall_max_repeatability_mm', 0)
        if max_rep > self.repeatability_limit_mm:
            issues.append(f"Max repeatability {max_rep:.3f}mm exceeds limit {self.repeatability_limit_mm:.3f}mm")
            return "FAIL", issues
            
        # Check orientation
        mean_orient = metrics.get('overall_mean_orientation_accuracy_deg', 0)
        if mean_orient > self.orientation_accuracy_limit_deg:
            issues.append(f"Orientation accuracy {mean_orient:.3f}deg exceeds limit {self.orientation_accuracy_limit_deg:.3f}deg")
            return "FAIL", issues
            
        orient_rep = metrics.get('overall_mean_orientation_repeatability_deg', 0)
        if orient_rep > self.orientation_repeatability_limit_deg:
            issues.append(f"Orientation repeatability {orient_rep:.3f}deg exceeds limit {self.orientation_repeatability_limit_deg:.3f}deg")
            return "FAIL", issues
            
        # Check warnings
        mean_acc = metrics.get('overall_mean_accuracy_mm', 0)
        if mean_acc > self.accuracy_warning_mm:
            issues.append(f"Mean accuracy {mean_acc:.3f}mm exceeds warning threshold {self.accuracy_warning_mm:.3f}mm")
            
        mean_rep = metrics.get('overall_mean_repeatability_mm', 0)
        if mean_rep > self.repeatability_warning_mm:
            issues.append(f"Mean repeatability {mean_rep:.3f}mm exceeds warning threshold {self.repeatability_warning_mm:.3f}mm")
            
        if issues:
            return "CONDITIONAL", issues
            
        return "PASS", []