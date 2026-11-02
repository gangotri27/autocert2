"""
moveit.launch.py
================
Launches ONLY robot_state_publisher.
move_group is intentionally NOT started — MoveItPy inside execution_node
is the sole planning/execution stack.

If used as a standalone helper, this file just starts RSP.
All MoveIt planning config is handled by execution.launch.py.

NoAliasDumper applied here for consistency (RSP params are a separate
YAML so aliasing is less likely, but the fix is applied everywhere
for safety).
"""

import os
import subprocess
import tempfile
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ══════════════════════════════════════════════════════════════════════════════
# NoAliasDumper
# ══════════════════════════════════════════════════════════════════════════════

class NoAliasDumper(yaml.Dumper):
    """yaml.Dumper that never emits anchors or aliases."""
    def ignore_aliases(self, data):
        return True


def _write_yaml(data: dict) -> str:
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', delete=False, prefix='moveit_cfg_'
    )
    yaml.dump(data, tmp, Dumper=NoAliasDumper, default_flow_style=False)
    tmp.flush()
    return tmp.name


# ── helpers ────────────────────────────────────────────────────────────────

def load_robot_description(urdf_file: str, xacro_args: str = '') -> str:
    urdf_file = os.path.expanduser(urdf_file)
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f'URDF/xacro not found: {urdf_file}')
    if urdf_file.endswith('.xacro'):
        cmd = ['xacro', urdf_file]
        if xacro_args:
            cmd.extend(xacro_args.split())
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        if not result.stdout:
            raise RuntimeError(f'xacro produced empty output: {urdf_file}')
        return result.stdout
    with open(urdf_file, 'r') as f:
        return f.read()


# ── launch setup ───────────────────────────────────────────────────────────

def launch_setup(context, *args, **kwargs):
    urdf_file  = LaunchConfiguration('urdf_file').perform(context)
    xacro_args = LaunchConfiguration('xacro_args').perform(context)

    robot_description = load_robot_description(urdf_file, xacro_args)

    if not robot_description:
        raise RuntimeError(
            f'robot_description is empty — check urdf_file: {urdf_file}'
        )

    rsp_params = _write_yaml({
        'robot_state_publisher': {
            'ros__parameters': {
                'robot_description': robot_description,
                'publish_frequency': 50.0,
                'use_sim_time':      False,
            }
        }
    })

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[rsp_params],
    )

    # move_group intentionally NOT launched.
    # Reason: MoveItPy (inside execution_node) handles all planning.
    # Launching move_group in parallel causes:
    #   1. Dual publisher collision on node name 'execution_node'
    #   2. ParameterTypeException in move_group (expects string_array,
    #      gets string from wrong YAML scope)
    #   3. Duplicate competing planning scene monitors

    return [rsp_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('urdf_file',         default_value=''),
        DeclareLaunchArgument('srdf_file',         default_value=''),
        DeclareLaunchArgument('xacro_args',        default_value=''),
        DeclareLaunchArgument('planning_group',    default_value='arm'),
        DeclareLaunchArgument('end_effector_link', default_value='wrist_roll_link'),
        DeclareLaunchArgument('reference_frame',   default_value='link1'),
        DeclareLaunchArgument('robot_name',        default_value='robot'),
        DeclareLaunchArgument('controllers_file',  default_value=''),
        DeclareLaunchArgument('kinematics_file',   default_value=''),
        OpaqueFunction(function=launch_setup),
    ])