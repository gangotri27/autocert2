import os
import subprocess
import tempfile
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

class NoAliasDumper(yaml.Dumper):
    def ignore_aliases(self, data):
        return True

def _write_yaml(data: dict) -> str:
    tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False, prefix='exec_node_')
    yaml.dump(data, tmp, Dumper=NoAliasDumper, default_flow_style=False)
    tmp.flush()
    return tmp.name

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
            raise RuntimeError(f'xacro produced empty output for: {urdf_file}')
        return result.stdout
    with open(urdf_file, 'r') as f:
        return f.read()

def load_file(path: str) -> str:
    path = os.path.expanduser(path)
    if not path or not os.path.exists(path):
        return ''
    with open(path, 'r') as f:
        return f.read()

def launch_setup(context, *args, **kwargs):
    urdf_file      = LaunchConfiguration('urdf_file').perform(context)
    srdf_file      = LaunchConfiguration('srdf_file').perform(context)
    xacro_args     = LaunchConfiguration('xacro_args').perform(context)
    planning_group = LaunchConfiguration('planning_group').perform(context)
    ee_link        = LaunchConfiguration('end_effector_link').perform(context)
    ref_frame      = LaunchConfiguration('reference_frame').perform(context)
    planner_id     = LaunchConfiguration('planner_id').perform(context)
    planning_time  = LaunchConfiguration('planning_time').perform(context)
    vel_scale      = LaunchConfiguration('velocity_scaling').perform(context)
    acc_scale      = LaunchConfiguration('acceleration_scaling').perform(context)

    robot_description          = load_robot_description(urdf_file, xacro_args)
    robot_description_semantic = load_file(srdf_file)

    if not robot_description:
        raise RuntimeError(f'robot_description is empty — check urdf_file: {urdf_file}')

    exec_node_yaml = _write_yaml({
        'execution_node': {
            'ros__parameters': {
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'planning_group': planning_group,
                'end_effector_link': ee_link,
                'reference_frame': ref_frame,
                'planner_id': planner_id,
                'planning_time': float(planning_time),
                'velocity_scaling': float(vel_scale),
                'acceleration_scaling': float(acc_scale),
            }
        }
    })

    execution_node = Node(
        package='autocert_execution',
        executable='execution_node',
        name='execution_node',
        output='screen',
        parameters=[exec_node_yaml],
    )

    return[execution_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('urdf_file',  default_value=''),
        DeclareLaunchArgument('srdf_file',  default_value=''),
        DeclareLaunchArgument('xacro_args', default_value=''),
        DeclareLaunchArgument('planning_group',       default_value='arm'),
        DeclareLaunchArgument('end_effector_link',    default_value='wrist_roll_link'),
        DeclareLaunchArgument('reference_frame',      default_value='link1'),
        DeclareLaunchArgument('planner_id',           default_value='RRTConnect'),
        DeclareLaunchArgument('planning_time',        default_value='5.0'),
        DeclareLaunchArgument('velocity_scaling',     default_value='0.3'),
        DeclareLaunchArgument('acceleration_scaling', default_value='0.3'),
        OpaqueFunction(function=launch_setup),
    ])