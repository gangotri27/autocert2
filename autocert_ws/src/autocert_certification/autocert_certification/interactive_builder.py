#!/usr/bin/env python3
"""Interactive / text-file certificate metadata builder.

Implements the certificate-mode menu:
  1) Load fields from a YAML metadata file        (non-interactive)
  2) Enter fields manually, one at a time          (interactive)
  3) Exit

Design rule followed throughout: fields that can be *derived* — payload,
reach, DOF, repeatability, and per-joint name/range/speed from the
URDF/SRDF, plus orientation & positional accuracy from the evaluation
run — are never prompted for, in either mode. They're filled in
silently by RobotDataExtractor / the evaluation stats. Everything else
is asked for in mode 2, pre-filled with the best default available
(a computed value when we have one, otherwise the previous static
default or whatever's already in the metadata file) so the user can
just hit Enter to accept it and only type something when they want to
change it.

Requires a real interactive terminal (see certification_node.py /
run_autocert_pipeline.sh's docstring for why). If stdin isn't a tty,
every prompt silently falls back to its default instead of blocking.
"""
import os
import sys
import yaml
from typing import Any, Dict, List, Optional


def _tty_ok() -> bool:
    return sys.stdin.isatty()


def _print_header(title: str):
    print(f"\n{'=' * 60}")
    print(title)
    print('=' * 60)


def _ask(prompt: str, default: Optional[str] = None) -> Optional[str]:
    if not _tty_ok():
        return default
    suffix = f" [{default}]" if default not in (None, '') else ""
    val = input(f"{prompt}{suffix}: ").strip()
    return val if val else default


def _ask_float(prompt: str, default: Optional[float] = None) -> Optional[float]:
    val = _ask(prompt, str(default) if default is not None else None)
    if val in (None, ''):
        return default
    try:
        return float(val)
    except (TypeError, ValueError):
        return default


def _ask_yesno(prompt: str, default: bool = False) -> bool:
    val = _ask(f"{prompt} (y/n)", 'y' if default else 'n')
    return str(val).strip().lower() in ('y', 'yes', 'true', '1')


def _ask_list(label: str, existing: Optional[List[str]] = None) -> List[str]:
    """Ask how many items the user wants, then collect each one by hand.
    No hardcoded cap — the user decides the count (0 skips the section)."""
    existing = existing or []
    if existing:
        print(f"  ({len(existing)} existing {label} found — enter the same count to keep/edit them, or 0 to clear)")
    count_str = _ask(f"How many {label} do you want to enter?", str(len(existing)))
    try:
        count = int(count_str)
    except (TypeError, ValueError):
        count = 0
    if count <= 0:
        return []
    items = []
    singular = label[:-1] if label.endswith('s') else label
    for i in range(count):
        default = existing[i] if i < len(existing) else None
        item = _ask(f"  {singular} #{i + 1}", default)
        if item:
            items.append(item)
    return items


class CertificateInputBuilder:
    """Drives the certificate-mode menu and produces the metadata dict
    consumed by certification_node.py to build the LaTeX pdf_data.
    """

    def __init__(self, robot_meta_path: str, robot_extractor=None):
        self.robot_meta_path = robot_meta_path
        self.extractor = robot_extractor

    def run(self) -> Optional[Dict[str, Any]]:
        """Runs the menu loop until the user picks Generate (returns the
        merged field dict) or Exit (returns None)."""
        while True:
            _print_header("CERTIFICATE MODE")
            print("  1) Load fields from a YAML text file")
            print("  2) Enter fields manually, one at a time")
            print("  3) Exit")
            choice = _ask("Enter choice (1/2/3)", "1")

            if choice == '1':
                data = self._load_from_file()
                if data is not None:
                    return data
                # bad path — loop back to the menu instead of crashing
            elif choice == '2':
                return self._collect_interactively()
            elif choice == '3':
                return None
            else:
                print("Invalid choice — please enter 1, 2, or 3.")

    # ------------------------------------------------------------------
    def _load_from_file(self) -> Optional[Dict[str, Any]]:
        path = _ask("Path to metadata YAML file", self.robot_meta_path)
        expanded = os.path.expanduser(path) if path else ''
        if not path or not os.path.exists(expanded):
            print(f"File not found: {path}")
            return None
        with open(expanded) as f:
            meta = yaml.safe_load(f) or {}
        return self._finalize_defaults(meta)

    def _collect_interactively(self) -> Dict[str, Any]:
        # Seed from the existing metadata file (if any) so re-running
        # doesn't force retyping every field from scratch.
        meta: Dict[str, Any] = {}
        if self.robot_meta_path and os.path.exists(os.path.expanduser(self.robot_meta_path)):
            with open(os.path.expanduser(self.robot_meta_path)) as f:
                meta = yaml.safe_load(f) or {}

        _print_header("Header")
        meta['robot_name'] = _ask("Robot name", meta.get('robot_name') or meta.get('model', 'Robot'))
        meta['model'] = _ask("Model", meta.get('model', 'Model-X'))
        meta['subtitle'] = _ask("Subtitle / edition line (leave blank to omit)", meta.get('subtitle', ''))
        meta['description'] = _ask("Short description paragraph", meta.get('description', ''))
        meta['robot_image'] = _ask("Image 1 (pitch photo) — file path or URL", meta.get('robot_image', ''))
        meta['robot_image_2'] = _ask("Image 2 (hardware photo) — file path or URL", meta.get('robot_image_2', ''))

        _print_header("Key Features")
        meta['key_features'] = _ask_list('key features', meta.get('key_features'))

        _print_header("Modifications vs. Original")
        has_mods = _ask_yesno("Include a Modifications vs. Original section?", bool(meta.get('modifications')))
        meta['modifications'] = _ask_list('modifications', meta.get('modifications')) if has_mods else []

        _print_header("Applications")
        meta['applications'] = _ask_list('applications', meta.get('applications'))

        _print_header("Performance")
        meta['max_tcp_speed'] = _ask_float("Max TCP speed (m/s)", meta.get('max_tcp_speed', 0.3))
        meta['max_joint_speed'] = _ask_float("Max joint speed (deg/s)", meta.get('max_joint_speed', 275))
        meta['mounting_options'] = _ask("Mounting options", meta.get('mounting_options', 'Tabletop or fixed base'))
        meta['noise_db'] = _ask("Noise level", meta.get('noise_db', '< 60 dB(A)'))
        # Orientation & positional accuracy come from the evaluation run — never asked here.

        _print_header("General Specification")
        meta['configuration'] = _ask("Configuration (e.g. Articulated Manipulator)",
                                      meta.get('configuration', 'Articulated Manipulator'))
        # Payload / Reach / DOF / Repeatability come from URDF + evaluation — never asked here.
        footprint_default = meta.get('footprint')
        if not footprint_default and self.extractor:
            computed = self.extractor.get_footprint_mm()
            if computed:
                footprint_default = f"Ø{computed:.0f} mm"
        meta['footprint'] = _ask("Footprint", footprint_default or 'Ø150 mm')

        # Joint names / ranges / speeds come from the URDF. Torque is filled
        # in separately by fill_missing_joint_torques() once the extractor's
        # joint table exists, since the joint list itself isn't known here.

        _print_header("Actuators")
        meta['actuator_type'] = _ask("Actuator type", meta.get('actuator_type', 'DYNAMIXEL Smart Servo'))
        meta['motor_models'] = _ask("Motor model(s)", meta.get('motor_models', ''))
        meta['control_mode'] = _ask("Control mode", meta.get('control_mode', 'Position / Velocity / Current'))
        meta['actuator_communication'] = _ask(
            "Communication (actuator bus)",
            meta.get('actuator_communication', meta.get('communication', 'TTL Multi-Drop Bus'))
        )
        meta['feedback'] = _ask("Feedback", meta.get('feedback', 'Absolute Contactless Encoder'))

        _print_header("Electrical Specifications")
        meta['input_voltage'] = _ask("Input voltage", meta.get('input_voltage', '12 V DC'))
        meta['power_supply'] = _ask("Power supply", meta.get('power_supply', '12 V, 5 A adapter'))
        meta['power_consumption'] = _ask("Typical power", meta.get('power_consumption', '35 W'))
        meta['max_power'] = _ask("Maximum power", meta.get('max_power', '120 W'))
        meta['digital_io'] = _ask("Digital I/O", meta.get('digital_io', 'Available via external controller'))

        _print_header("Physical & Kinematics")
        material_default = meta.get('material') or (self.extractor.get_material() if self.extractor else 'PLA / Aluminum')
        meta['material'] = _ask("Materials", material_default)
        meta['cable_length'] = _ask("Cable length", meta.get('cable_length', '1 m'))
        weight_default = meta.get('weight_kg') or (self.extractor.get_weight() if self.extractor else 0.63)
        meta['weight_kg'] = _ask_float("Total weight (kg)", weight_default)
        meta['ip_rating'] = _ask("IP Rating", meta.get('ip_rating', 'IP20'))
        meta['operating_temp'] = _ask("Operating temperature range", meta.get('operating_temp', '0°C to 45°C'))
        # NOTE: this is intentionally 'humidity_range' (a display string
        # for the datasheet, e.g. "5% to 85% (non-condensing)"), not
        # 'humidity' — that key is already used elsewhere for the
        # numeric ambient test-condition humidity (e.g. 45) recorded in
        # the certificate JSON. Reusing 'humidity' here would silently
        # overwrite that value.
        meta['humidity_range'] = _ask("Humidity range", meta.get('humidity_range', '5% to 85% (non-condensing)'))

        # Safety & Compliance and Control System sections are intentionally
        # not asked here — they stay generic/unchanged per spec.

        _print_header("Known Limitations")
        meta['known_limitations'] = _ask_list('known limitations', meta.get('known_limitations'))

        _print_header("Documentation & Resources")
        meta['repo_url'] = _ask("GitHub repository URL", meta.get('repo_url', ''))

        _print_header("Package Contents")
        meta['package_contents'] = _ask_list('package contents items', meta.get('package_contents'))

        return self._finalize_defaults(meta)

    # ------------------------------------------------------------------
    def _finalize_defaults(self, meta: Dict[str, Any]) -> Dict[str, Any]:
        """Fills in anything still missing (used by text-file mode, and
        as a safety net after interactive mode) WITHOUT prompting."""
        meta.setdefault('subtitle', '')
        meta.setdefault('description', '')
        meta.setdefault('robot_image', '')
        meta.setdefault('robot_image_2', '')
        meta.setdefault('key_features', [])
        meta.setdefault('modifications', [])
        meta.setdefault('applications', [])
        meta.setdefault('max_tcp_speed', 0.3)
        meta.setdefault('max_joint_speed', 275)
        meta.setdefault('mounting_options', 'Tabletop or fixed base')
        meta.setdefault('noise_db', '< 60 dB(A)')
        meta.setdefault('configuration', 'Articulated Manipulator')
        meta.setdefault('footprint', None)
        meta.setdefault('actuator_type', 'DYNAMIXEL Smart Servo')
        meta.setdefault('motor_models', '')
        meta.setdefault('control_mode', 'Position / Velocity / Current')
        meta.setdefault('actuator_communication', meta.get('communication', 'TTL Multi-Drop Bus'))
        meta.setdefault('feedback', 'Absolute Contactless Encoder')
        meta.setdefault('power_supply', '12 V, 5 A adapter')
        meta.setdefault('digital_io', 'Available via external controller')
        meta.setdefault('operating_temp', '0°C to 45°C')
        meta.setdefault('humidity_range', '5% to 85% (non-condensing)')
        meta.setdefault('known_limitations', [])
        meta.setdefault('package_contents', [])
        meta.setdefault('repo_url', '')
        return meta


def fill_missing_joint_torques(joints: List[Dict[str, Any]], interactive: bool) -> List[Dict[str, Any]]:
    """Backfills torque for any joint whose URDF didn't specify an
    effort limit (torque_raw is None). Only prompts per-joint when
    `interactive` is True and a real tty is available; otherwise falls
    back to the old static default so non-interactive / text-file runs
    never hang.
    """
    for j in joints:
        if j.get('torque_raw') is not None:
            continue  # came straight from the URDF — leave it alone
        if interactive and _tty_ok():
            val = _ask_float(f"Peak torque for joint '{j['name']}' (Nm, not in URDF)", 4.1)
        else:
            val = 4.1
        j['torque_raw'] = val
        j['torque'] = f"{val:.1f} Nm"
    return joints