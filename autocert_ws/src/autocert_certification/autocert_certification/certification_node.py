#!/usr/bin/env python3

import os
import sys
import rclpy
import json
import yaml
import argparse
from rclpy.node import Node
from datetime import datetime, timedelta

from .certificate import Certificate, CertificateMetadata, RobotUnderTest, TestConditions, PoseResult, OverallResults
from .limits import CertificationLimits
from .report_generator import ReportGenerator
from .robot_data_extractor import RobotDataExtractor
from .latex_certificate_generator import LatexCertificateGenerator
from .interactive_builder import CertificateInputBuilder, fill_missing_joint_torques
from . import hash_utils

try:
    from .pdf_generator import PDFGenerator
    PDF_AVAILABLE = True
except ImportError:
    PDF_AVAILABLE = False


class CertificationNode(Node):
    def __init__(self):
        super().__init__('certification_node')
        
        # Parameters
        self.declare_parameter('input_file', '')
        self.declare_parameter('output_file', '')
        self.declare_parameter('pdf_file', '')
        self.declare_parameter('robot_metadata', '')
        self.declare_parameter('limits_file', '')
        self.declare_parameter('run_id', 'unknown')
        self.declare_parameter('urdf_file', '')
        self.declare_parameter('srdf_file', '')
        self.declare_parameter('robot_image', '')
        self.declare_parameter('robot_image_2', '')  # Added for 2nd column image
        self.declare_parameter('robot_mode', 'unknown')  # 'simulation' | 'real' | 'unknown'
        
        # Initialize LaTeX generator
        self.latex_generator = LatexCertificateGenerator()
        
        # Initialize robot data extractor (will be set in run)
        self.robot_extractor = None

    def _prompt_user_input(self, field_name: str, default_value: any, description: str = "") -> any:
        """Prompt user for input if value is not provided.

        Requires the certification_node action to be launched with
        emulate_tty=True (see full_pipeline.launch.py) so that a real
        pseudo-terminal is allocated and input() actually receives
        keystrokes from the operator's terminal, even though this node
        was spawned by `ros2 launch` rather than run standalone.

        If no tty is available (e.g. running headless/CI, or launched
        without emulate_tty=True), falls back to the default instead of
        blocking forever — a missing field should never hang an
        automated run.
        """
        import sys
        if not sys.stdin.isatty():
            self.get_logger().warn(
                f"'{field_name}' missing from robot_metadata.yaml and no "
                f"interactive terminal available — using default: {default_value}"
            )
            return default_value

        # Real tty available (emulate_tty=True in the launch file) — prompt for real.
        # Flush everything and use a loud banner so it doesn't get lost among
        # background node log lines (robot_state_publisher / execution_node)
        # still printing to the same screen at this point in the pipeline.
        sys.stdout.flush()
        print(f"\n{'='*60}", flush=True)
        print("INPUT REQUIRED — certification pipeline is paused", flush=True)
        if description:
            print(description, flush=True)
        print('='*60, flush=True)
        prompt = f"Enter {field_name} (default: {default_value}): "
        user_input = input(prompt).strip()
        if user_input:
            if isinstance(default_value, bool):
                return user_input.lower() in ['true', 'yes', '1']
            elif isinstance(default_value, float):
                try:
                    return float(user_input)
                except ValueError:
                    return default_value
            elif isinstance(default_value, int):
                try:
                    return int(user_input)
                except ValueError:
                    return default_value
            else:
                return user_input
        return default_value

    def run_certification(self, input_path, output_path, pdf_path, metadata_path, limits_path, run_id):
        self.get_logger().info(f"Starting Certification for Run ID: {run_id}")

        # 1. Load Evaluation Data
        with open(input_path, 'r') as f:
            eval_data = json.load(f)

        # 2. Load Robot Metadata
        with open(metadata_path, 'r') as f:
            robot_meta = yaml.safe_load(f)

        # 3. Load Limits
        limits = CertificationLimits()
        if os.path.exists(limits_path):
            with open(limits_path, 'r') as f:
                limits_data = yaml.safe_load(f)
            limits.load_from_dict(limits_data)

        # 4. Extract robot data from URDF, then run the certificate-mode
        # menu (text file / manual input / exit) to gather everything
        # else. This loops so the operator can regenerate the
        # certificate as many times as they like before choosing Exit.
        urdf_file = self.get_parameter('urdf_file').value
        srdf_file = self.get_parameter('srdf_file').value

        if urdf_file and os.path.exists(os.path.expanduser(urdf_file)):
            self.get_logger().info(f"Extracting robot data from URDF: {urdf_file}")
            self.robot_extractor = RobotDataExtractor(urdf_file, srdf_file)
        else:
            self.get_logger().warn(f"URDF file not provided or not found: {urdf_file}")
            self.get_logger().warn("Joint table / payload / reach / DOF / footprint will need to come from the metadata file or manual input instead.")
            self.robot_extractor = None

        builder = CertificateInputBuilder(metadata_path, robot_extractor=self.robot_extractor)

        while True:
            cert_fields = builder.run()
            if cert_fields is None:
                self.get_logger().info("Exit selected in certificate mode — no certificate generated this run.")
                return True

            # Merge: robot_meta (for fields the menu doesn't touch, e.g.
            # laboratory/technician/manufacturer/serial_number) + whatever
            # the menu just produced (takes precedence for shared keys).
            merged_meta = {**robot_meta, **cert_fields}

            if self.robot_extractor:
                robot_data = self.robot_extractor.extract_all_data(merged_meta)
                fill_missing_joint_torques(
                    robot_data.get('joints', []),
                    interactive=sys.stdin.isatty()
                )
            else:
                robot_data = {
                    'robot_name': merged_meta.get('robot_name', merged_meta.get('model', 'Robot')),
                    'manufacturer': merged_meta.get('manufacturer', 'Unknown'),
                    'model': merged_meta.get('model', 'Unknown'),
                    'serial_number': merged_meta.get('serial_number', 'N/A'),
                    'payload_kg': merged_meta.get('payload_kg', 0.3),
                    'reach_m': merged_meta.get('reach_m', 0.485),
                    'weight_kg': merged_meta.get('weight_kg', 0.63),
                    'material': merged_meta.get('material', 'PLA / Aluminum'),
                    'dof': merged_meta.get('dof', 6),
                    'repeatability_mm': merged_meta.get('repeatability_mm', 0.2),
                    'joints': [],
                    'contact_info': {
                        'company': merged_meta.get('company', 'AutoCert Lab'),
                        'phone': merged_meta.get('phone', '+91-9876543210'),
                        'email': merged_meta.get('email', 'autocert@robotics.com'),
                        'website': merged_meta.get('website', 'www.autocert.com'),
                        'country': merged_meta.get('country', 'India'),
                    }
                }

            self._finish_certificate(
                input_path, output_path, pdf_path, run_id,
                eval_data, limits, merged_meta, robot_data,
                urdf_file, srdf_file
            )

            if not sys.stdin.isatty():
                return True  # non-interactive: one pass only, no menu to loop on
            again = input("\nGenerate another certificate for this run? (y/n) [n]: ").strip().lower()
            if again not in ('y', 'yes'):
                return True

    def _finish_certificate(self, input_path, output_path, pdf_path, run_id,
                             eval_data, limits, robot_meta, robot_data,
                             urdf_file, srdf_file):
        """Everything after metadata is gathered: pass/fail evaluation,
        integrity hashes, JSON + PDF certificate output. Split out so the
        menu loop in run_certification() can call it once per iteration."""
        # 5. Perform Pass/Fail Evaluation
        overall_stats = eval_data['overall']
        status, issues = limits.evaluate(overall_stats)

        # ── Simulation-provenance guard ──────────────────────────────────
        # robot_mode is passed in from the launch file (the same value the
        # experiment run actually used). If the data behind this
        # certificate came from the simulation override, the certificate
        # must say so explicitly rather than presenting as a hardware
        # validation. This does not block generation (useful for pipeline
        # testing) — it stamps the artifact so it can't be mistaken for
        # evidentiary data.
        robot_mode = self.get_parameter('robot_mode').value
        is_simulated_run = (robot_mode == 'simulation')
        if is_simulated_run:
            status = 'SIMULATED - NOT A VALID CERTIFICATION'
            issues = issues + [
                'Source data was generated with robot_mode=simulation. '
                'Measurements are synthetic and do not reflect real hardware performance.'
            ]

        # ── Real integrity hashes ────────────────────────────────────────
        # Previously these were hardcoded placeholder strings
        # (dataset_hash="SIM_VALIDATED", robot_model_hash="OMX_6DOF",
        # srdf_hash="N/A"). Compute them for real from the actual files
        # used in this run so the certificate's integrity fields mean
        # something.
        dataset_hash = hash_utils.compute_file_hash(input_path) or 'UNAVAILABLE'
        robot_model_hash = (
            hash_utils.compute_file_hash(os.path.expanduser(urdf_file))
            if urdf_file and os.path.exists(os.path.expanduser(urdf_file))
            else 'UNAVAILABLE'
        )
        srdf_hash = (
            hash_utils.compute_file_hash(os.path.expanduser(srdf_file))
            if srdf_file and os.path.exists(os.path.expanduser(srdf_file))
            else 'UNAVAILABLE'
        )

        now = datetime.now()
        
        # Image path/URL falls back to whatever the certificate-mode menu
        # collected (robot_meta['robot_image']) if no --robot-image CLI
        # arg was given.
        robot_image = self.get_parameter('robot_image').value or robot_meta.get('robot_image', '')
        robot_image_2 = self.get_parameter('robot_image_2').value or robot_meta.get('robot_image_2', '')

        # Additional Data specifically for the LaTeX template
        pdf_data = {
            'robot_image': robot_image,
            'robot_image_2': robot_image_2,
            'software_version': robot_meta.get('software_version', '1.0'),
            'year': str(now.year),
            'subtitle': robot_meta.get('subtitle', ''),
            'description': robot_meta.get('description', ''),
            'safety_rating': robot_meta.get('safety_rating', 'Uncertified (Research use only)'),
            'repo_url': robot_meta.get('repo_url', ''),
            'repo_url_short': robot_meta.get('repo_url_short', ''),
            'max_joint_speed': robot_meta.get('max_joint_speed', 275),
            'max_tcp_speed': robot_meta.get('max_tcp_speed', 0.3),
            'mounting_options': robot_meta.get('mounting_options', 'Tabletop or fixed base'),
            'noise_db': robot_meta.get('noise_db', '< 60 dB(A)'),
            'configuration': robot_meta.get('configuration', 'Articulated Manipulator'),
            'key_features': robot_meta.get('key_features', []),
            'modifications': robot_meta.get('modifications', []),
            'applications': robot_meta.get('applications', []),
            'known_limitations': robot_meta.get('known_limitations', []),
            'package_contents': robot_meta.get('package_contents', []),
            'actuator_type': robot_meta.get('actuator_type', 'DYNAMIXEL Smart Servo'),
            'motor_models': robot_meta.get('motor_models', ''),
            'control_mode': robot_meta.get('control_mode', 'Position / Velocity / Current'),
            'actuator_communication': robot_meta.get('actuator_communication', robot_meta.get('communication', 'TTL Multi-Drop Bus')),
            'feedback': robot_meta.get('feedback', 'Absolute Contactless Encoder'),
            'power_supply': robot_meta.get('power_supply', '12 V, 5 A adapter'),
            'digital_io': robot_meta.get('digital_io', 'Available via external controller'),
            'operating_temp': robot_meta.get('operating_temp', '0°C to 45°C'),
            'humidity_range': robot_meta.get('humidity_range', '5% to 85% (non-condensing)'),
            # These are also produced by RobotDataExtractor when a URDF is
            # available (and **robot_data below will take precedence in
            # that case) — set explicitly here too so nothing entered in
            # the certificate-mode menu gets silently dropped when running
            # without a URDF (the fallback robot_data dict doesn't carry
            # these keys).
            'input_voltage': robot_meta.get('input_voltage', '12 V DC'),
            'power_consumption': robot_meta.get('power_consumption', '35 W'),
            'max_power': robot_meta.get('max_power', '120 W'),
            'cable_length': robot_meta.get('cable_length', '1 m'),
            'ip_rating': robot_meta.get('ip_rating', 'IP20'),
            'footprint': robot_meta.get('footprint'),

            # Common params
            'mean_accuracy_mm': overall_stats.get('overall_mean_accuracy_mm', 0),
            'max_accuracy_mm': overall_stats.get('overall_max_accuracy_mm', 0),
            'mean_repeatability_mm': overall_stats.get('overall_mean_repeatability_mm', 0),
            'max_repeatability_mm': overall_stats.get('overall_max_repeatability_mm', 0),
            'mean_orientation_accuracy_deg': overall_stats.get('overall_mean_orientation_accuracy_deg', 0),
            **robot_data
        }
        
        overall_stats['certification_status'] = status
        overall_stats['issues'] = issues
        
        # 7. Build JSON Certificate
        cert = Certificate(
            metadata=CertificateMetadata(
                certificate_id=f"AC-{run_id.upper()}",
                issue_date=now.strftime("%Y-%m-%d"),
                valid_until=(now + timedelta(days=365)).strftime("%Y-%m-%d"),
                standard="ISO 9283:1998",
                laboratory=robot_meta.get('laboratory', 'AutoCert Lab'),
                technician=robot_meta.get('technician', 'AutoCert System'),
                dataset_hash=dataset_hash,
                robot_model_hash=robot_model_hash,
                srdf_hash=srdf_hash,
                controller_version="ROS2_JAZZY",
                issues=issues
            ),
            robot=RobotUnderTest(
                manufacturer=robot_data.get('manufacturer', 'Unknown'),
                model=robot_data.get('model', 'Unknown'),
                serial_number=robot_data.get('serial_number', 'N/A'),
                software_version=robot_meta.get('software_version', '1.0'),
                payload_kg=robot_data.get('payload_kg', 0.5),
                reach_m=robot_data.get('reach_m', 0.38)
            ),
            test_conditions=TestConditions(
                ambient_temperature_c=robot_meta.get('temperature', 22.0),
                relative_humidity_percent=robot_meta.get('humidity', 45),
                measurement_system=robot_meta.get('measurement_system', 'Leica AT960'),
                measurement_system_calibration_date=robot_meta.get('calibration_date', now.strftime('%Y-%m-%d')),
                test_cube_size_mm=float(eval_data.get('test_cube_size_mm', 60.0)),
                # BUG FIX: this used to be hardcoded to [0.2, 0.0, 0.22]
                # (OMX's cube) regardless of what actually ran. Now reads
                # the real value evaluation_node.py threads through from
                # experiment_node.py's run metadata sidecar, falling back
                # to the old default only if that data isn't available
                # (e.g. older runs predating this fix).
                test_cube_center=eval_data.get('test_cube_center', [0.2, 0.0, 0.22]),
                repetitions_per_pose=overall_stats['num_poses'],
                outlier_threshold=3.0,
                confidence_level=0.95,
                pose_strategy="iso_cube"
            ),
            pose_results=[],  
            overall=OverallResults(
                mean_accuracy_mm=overall_stats.get('overall_mean_accuracy_mm', 0),
                max_accuracy_mm=overall_stats.get('overall_max_accuracy_mm', 0),
                mean_repeatability_mm=overall_stats.get('overall_mean_repeatability_mm', 0),
                max_repeatability_mm=overall_stats.get('overall_max_repeatability_mm', 0),
                mean_orientation_accuracy_deg=overall_stats.get('overall_mean_orientation_accuracy_deg', 0),
                mean_orientation_repeatability_deg=overall_stats.get('overall_mean_orientation_repeatability_deg', 0),
                accuracy_95_confidence_mm=0.05,
                repeatability_95_confidence_mm=0.02,
                certification_status=status
            )
        )

        for pid, pdata in eval_data.get('poses', {}).items():
            cert.pose_results.append(PoseResult(
                pose_id=int(pid),
                pose_label=f"P{pid}",
                accuracy_mm=pdata.get('ap_distance_mm', 0),
                repeatability_mm=pdata.get('rp_distance_mm', 0),
                accuracy_x_mm=pdata.get('ap_x', 0) if 'ap_x' in pdata else 0,
                accuracy_y_mm=pdata.get('ap_y', 0) if 'ap_y' in pdata else 0,
                accuracy_z_mm=pdata.get('ap_z', 0) if 'ap_z' in pdata else 0,
                orientation_error_deg=pdata.get('ao_deg', 0),
                orientation_repeatability_deg=pdata.get('ro_deg', 0),
                num_samples=pdata.get('num_samples', 0),
                outliers_removed=0
            ))

        with open(output_path, 'w') as f:
            json.dump(cert.to_dict(), f, indent=4)
        self.get_logger().info(f"Certificate JSON saved: {output_path}")

        # 10. Output LaTeX PDF Report
        self.get_logger().info("Generating LaTeX PDF certificate...")
        tex_path = os.path.splitext(pdf_path)[0] + ".tex"
        success = self.latex_generator.generate_pdf(pdf_data, pdf_path, tex_output_path=tex_path)

        if success:
            self.get_logger().info(f"Professional LaTeX PDF Report generated: {pdf_path}")
            self.get_logger().info(f"LaTeX source saved: {tex_path}")
        else:
            self.get_logger().error("Failed to generate LaTeX PDF. Ensure pdflatex is installed.")
        
        # Legacy fallback if pdflatex failed
        if PDF_AVAILABLE and not success:
            pdf_gen = PDFGenerator()
            pdf_gen.generate(cert, pdf_path.replace('.pdf', '_legacy.pdf'))
            self.get_logger().info(f"Legacy PDF Report generated: {pdf_path.replace('.pdf', '_legacy.pdf')}")
        
        self.get_logger().info(f"--- CERTIFICATION RESULT: {status} ---")
        if issues:
            self.get_logger().warn(f"Issues: {', '.join(issues)}")
        
        return True


def main():
    rclpy.init()
    parser = argparse.ArgumentParser(description='AutoCert Certification Node')
    parser.add_argument('--input', required=True, help='Input evaluation JSON file')
    parser.add_argument('--output', required=True, help='Output certificate JSON file')
    parser.add_argument('--pdf', required=True, help='Output PDF certificate file')
    parser.add_argument('--robot-metadata', required=True, help='Robot metadata YAML file')
    parser.add_argument('--limits', required=True, help='Limits YAML file')
    parser.add_argument('--run-id', default='run_021', help='Run identifier')
    parser.add_argument('--urdf', default='', help='URDF or xacro file path')
    parser.add_argument('--srdf', default='', help='SRDF file path')
    parser.add_argument('--robot-image', default='', help='Robot primary image file path')
    parser.add_argument('--robot-image-2', default='', help='Robot secondary image file path')
    parser.add_argument('--robot-mode', default='unknown', help="Mode the source run used: 'simulation' or 'real'")
    
    import sys
    from rclpy.utilities import remove_ros_args
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])

    node = CertificationNode()
    
    # Set additional parameters
    if args.urdf:
        node.set_parameters([rclpy.parameter.Parameter('urdf_file', rclpy.Parameter.Type.STRING, args.urdf)])
    if args.srdf:
        node.set_parameters([rclpy.parameter.Parameter('srdf_file', rclpy.Parameter.Type.STRING, args.srdf)])
    if args.robot_image:
        node.set_parameters([rclpy.parameter.Parameter('robot_image', rclpy.Parameter.Type.STRING, args.robot_image)])
    if args.robot_image_2:
        node.set_parameters([rclpy.parameter.Parameter('robot_image_2', rclpy.Parameter.Type.STRING, args.robot_image_2)])
    node.set_parameters([rclpy.parameter.Parameter('robot_mode', rclpy.Parameter.Type.STRING, args.robot_mode)])

    node.run_certification(
        args.input, args.output, args.pdf,
        args.robot_metadata, args.limits, args.run_id
    )
    rclpy.shutdown()


if __name__ == '__main__':
    main()