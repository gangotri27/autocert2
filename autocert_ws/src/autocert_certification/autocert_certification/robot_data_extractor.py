#!/usr/bin/env python3
"""Extract robot data from URDF and SRDF files for certificate generation."""

import os
import re
import xml.etree.ElementTree as ET
from typing import Dict, List, Tuple, Optional, Any
from urdf_parser_py.urdf import URDF
import numpy as np


class RobotDataExtractor:
    """Extract robot specifications from URDF and SRDF files."""
    
    def __init__(self, urdf_file: str, srdf_file: Optional[str] = None):
        """
        Initialize extractor with URDF and optional SRDF files.
        
        Args:
            urdf_file: Path to URDF or xacro file
            srdf_file: Optional path to SRDF file
        """
        self.urdf_file = os.path.expanduser(urdf_file)
        self.srdf_file = os.path.expanduser(srdf_file) if srdf_file else None
        
        self.robot = None
        self.joint_limits = {}
        self.joint_names = []
        self._load_urdf()
        
    def _load_urdf(self):
        """Load and parse URDF file."""
        if self.urdf_file.endswith('.xacro'):
            # Process xacro file
            import subprocess
            cmd = ['xacro', self.urdf_file]
            result = subprocess.run(cmd, capture_output=True, text=True, check=True)
            urdf_string = result.stdout
        else:
            with open(self.urdf_file, 'r') as f:
                urdf_string = f.read()
        
        # Parse URDF
        self.robot = URDF.from_xml_string(urdf_string)
        
        # Extract joint information
        for joint in self.robot.joints:
            if joint.type in ['revolute', 'prismatic', 'continuous']:
                self.joint_names.append(joint.name)
                if joint.limit:
                    self.joint_limits[joint.name] = {
                        'lower': joint.limit.lower if hasattr(joint.limit, 'lower') else None,
                        'upper': joint.limit.upper if hasattr(joint.limit, 'upper') else None,
                        'velocity': joint.limit.velocity if hasattr(joint.limit, 'velocity') else None,
                        'effort': joint.limit.effort if hasattr(joint.limit, 'effort') else None,
                    }
    
    def get_dof_count(self) -> int:
        """Get number of degrees of freedom."""
        return len(self.joint_names)
    
    def get_joint_ranges(self) -> List[Tuple[str, float, float]]:
        """Get joint ranges as list of (name, lower_deg, upper_deg)."""
        ranges = []
        for name, limits in self.joint_limits.items():
            lower = limits.get('lower', 0)
            upper = limits.get('upper', 0)
            if lower is not None and upper is not None:
                ranges.append((name, np.degrees(lower), np.degrees(upper)))
            else:
                ranges.append((name, -180.0, 180.0))
        return ranges
    
    def get_joint_velocities(self) -> List[Tuple[str, float]]:
        """Get joint velocities in degrees/second."""
        velocities = []
        for name, limits in self.joint_limits.items():
            vel = limits.get('velocity', 0)
            if vel:
                velocities.append((name, np.degrees(vel)))
            else:
                velocities.append((name, 275.0))  # Default
        return velocities

    def get_joint_torques(self) -> List[Tuple[str, Optional[float]]]:
        """Get peak torque (Nm) per joint from URDF's <limit effort="...">.

        Returns None for a joint when the URDF doesn't specify an effort
        limit (or specifies 0), so the caller can tell "genuinely
        computed" apart from "we made this number up" and prompt the
        user only for the ones we can't source.
        """
        torques = []
        for name in self.joint_names:
            limits = self.joint_limits.get(name, {})
            effort = limits.get('effort')
            if effort:
                torques.append((name, round(float(effort), 2)))
            else:
                torques.append((name, None))
        return torques

    def get_footprint_mm(self) -> Optional[float]:
        """Estimate mounting footprint (diameter, mm) from the base
        link's collision geometry, if it has one we can read.

        Returns None if the base link has no collision geometry we
        recognize, so the caller knows to fall back to a user-provided
        value instead of a made-up default.
        """
        if not self.robot:
            return None
        base_link_name = self.robot.get_root()
        base_link = next((l for l in self.robot.links if l.name == base_link_name), None)
        if not base_link or not base_link.collision or not base_link.collision.geometry:
            return None

        geom = base_link.collision.geometry
        try:
            if hasattr(geom, 'radius') and geom.radius:
                return round(geom.radius * 2 * 1000, 1)
            if hasattr(geom, 'size') and geom.size:
                # box: diagonal footprint across x/y
                x, y = geom.size[0], geom.size[1]
                return round((x**2 + y**2) ** 0.5 * 1000, 1)
        except (AttributeError, IndexError, TypeError):
            return None
        return None
    
    def get_reach(self) -> float:
        """Calculate maximum reach from URDF."""
        # Find the furthest link from base
        max_distance = 0.0
        
        # Get base link (first link in chain)
        base_link = self.robot.get_root()
        
        def compute_distance(link_name: str, origin: Tuple[float, float, float] = (0, 0, 0)):
            nonlocal max_distance
            distance = np.sqrt(origin[0]**2 + origin[1]**2 + origin[2]**2)
            max_distance = max(max_distance, distance)
            
            # Find all joints connected to this link
            for joint in self.robot.joints:
                if joint.parent == link_name:
                    # Get joint origin
                    if joint.origin:
                        xyz = joint.origin.xyz if joint.origin.xyz else [0, 0, 0]
                        new_origin = (origin[0] + xyz[0], origin[1] + xyz[1], origin[2] + xyz[2])
                        compute_distance(joint.child, new_origin)
        
        compute_distance(base_link)
        return round(max_distance, 3)
    
    def get_payload(self, user_payload: Optional[float] = None) -> float:
        """Get payload from URDF or use user value."""
        if user_payload is not None:
            return user_payload
        
        # Try to find payload in URDF
        for link in self.robot.links:
            if link.inertial and link.inertial.mass:
                # Rough estimate: payload ~ 10-20% of robot mass
                robot_mass = sum(link.inertial.mass for link in self.robot.links if link.inertial)
                return round(robot_mass * 0.15, 2)
        
        return 0.3  # Default
    
    def get_weight(self) -> float:
        """Calculate approximate robot weight from URDF."""
        total_mass = 0.0
        for link in self.robot.links:
            if link.inertial and link.inertial.mass:
                total_mass += link.inertial.mass
        return round(total_mass, 2) if total_mass > 0 else 0.63
    
    def get_material(self) -> str:
        """Get material from URDF or return default."""
        materials = set()
        for link in self.robot.links:
            if link.visual and link.visual.material:
                materials.add(link.visual.material.name)
        if materials:
            return ', '.join(materials)
        return "PLA / Aluminum"
    
    def get_workspace_volume(self, positions: Optional[np.ndarray] = None) -> float:
        """Calculate workspace volume from sampled positions."""
        if positions is not None and len(positions) > 3:
            from scipy.spatial import ConvexHull
            try:
                hull = ConvexHull(positions)
                return round(hull.volume, 4)
            except:
                pass
        
        # Estimate volume as sphere with radius = reach
        reach = self.get_reach()
        return round((4/3) * np.pi * (reach**3), 4)
    
    def get_workspace_surface_area(self, positions: Optional[np.ndarray] = None) -> float:
        """Calculate workspace surface area."""
        if positions is not None and len(positions) > 3:
            from scipy.spatial import ConvexHull
            try:
                hull = ConvexHull(positions)
                return round(hull.area, 3)
            except:
                pass
        
        reach = self.get_reach()
        return round(4 * np.pi * (reach**2), 3)
    
    def get_repeatability(self, measured_poses: Optional[np.ndarray] = None) -> float:
        """Calculate repeatability from measurement data."""
        if measured_poses is not None and len(measured_poses) > 1:
            centroid = np.mean(measured_poses, axis=0)
            distances = np.linalg.norm(measured_poses - centroid, axis=1)
            mean_dist = np.mean(distances)
            std_dist = np.std(distances, ddof=1) if len(distances) > 1 else 0
            rp = mean_dist + 3 * std_dist
            return round(rp * 1000, 2)  # Convert to mm
        
        return 0.2  # Default repeatability in mm
    
    def extract_all_data(self, user_data: Optional[Dict] = None) -> Dict[str, Any]:
        """Extract all robot data for certificate."""
        user_data = user_data or {}
        
        joint_ranges = self.get_joint_ranges()
        joint_vels = self.get_joint_velocities()
        joint_torques = dict(self.get_joint_torques())

        # Format joint table. Joint names are the real URDF joint names
        # (just prettified for display), not a generic "Axis N" label —
        # each robot's actual joint names show up here.
        joint_table = []
        for i, (name, lower, upper) in enumerate(joint_ranges[:6]):  # Limit to first 6 joints
            vel = joint_vels[i][1] if i < len(joint_vels) else 275
            torque_raw = joint_torques.get(name)
            joint_table.append({
                'name': name.replace('_', ' ').title(),
                'range': f"${lower:.0f}^\\circ$ to ${upper:.0f}^\\circ$",
                'range_raw': (lower, upper),
                'speed': f"{vel:.0f}$^\\circ$/s",
                'speed_raw': vel,
                'torque_raw': torque_raw,  # None means URDF didn't specify one — caller should prompt
                'torque': f"{torque_raw:.1f} Nm" if torque_raw else None,
            })
        
        return {
            'robot_name': user_data.get('robot_name', self.robot.name if self.robot else 'Robot'),
            'manufacturer': user_data.get('manufacturer', 'Unknown Manufacturer'),
            'model': user_data.get('model', self.robot.name if self.robot else 'Unknown Model'),
            'serial_number': user_data.get('serial_number', 'N/A'),
            'payload_kg': self.get_payload(user_data.get('payload_kg')),
            'reach_m': self.get_reach(),
            # weight_kg / material are asked interactively (see
            # interactive_builder.py's Physical & Kinematics section),
            # pre-filled with the URDF-computed value as a default — so
            # an explicit user_data value here must win, not be silently
            # discarded in favor of the computed one.
            'weight_kg': user_data.get('weight_kg') or self.get_weight(),
            'material': user_data.get('material') or self.get_material(),
            'dof': self.get_dof_count(),
            'joints': joint_table,
            'max_tcp_speed': user_data.get('max_tcp_speed', 0.3),
            'repeatability_mm': self.get_repeatability(),
            'input_voltage': user_data.get('input_voltage', '12 V'),
            'power_consumption': user_data.get('power_consumption', '35 W'),
            'max_power': user_data.get('max_power', '120 W'),
            'ip_rating': user_data.get('ip_rating', 'IP20'),
            'noise_db': user_data.get('noise_db', '< 60 dB(A)'),
            'communication': user_data.get('communication', 'TTL Dynamixel bus / ROS 2'),
            'footprint': user_data.get('footprint', 'Ø150 mm'),
            'footprint_mm_computed': self.get_footprint_mm(),
            'cable_length': user_data.get('cable_length', '1 m'),
            'workspace_volume_m3': 0.0806,  # Will be updated from sampling
            'workspace_surface_area_m2': 0.919,
            'reachability_coverage': user_data.get('reachability_coverage', 53.9),
            'reachability_density': user_data.get('reachability_density', 0.557),
            'dexterous_volume': user_data.get('dexterous_volume', 0.0388),
            'contact_info': {
                'company': user_data.get('company', 'AutoCert Lab'),
                'phone': user_data.get('phone', '+91-9876543210'),
                'email': user_data.get('email', 'autocert@robotics.com'),
                'website': user_data.get('website', 'www.autocert.com'),
                'country': user_data.get('country', 'India'),
            }
        }