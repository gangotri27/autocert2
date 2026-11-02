"""Launch file for AutoCert experiment node"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description"""
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(
                get_package_share_directory('autocert_experiment'),
                'config', 'experiment_config.yaml'
            ),
            description='Path to experiment configuration file'
        ),
        
        DeclareLaunchArgument(
            'output_file',
            default_value='',
            description='Path to output CSV file'
        ),
        
        DeclareLaunchArgument(
            'robot_mode',
            default_value='simulation',
            choices=['simulation', 'real'],
            description='Robot mode: simulation or real'
        ),

        DeclareLaunchArgument(
            'run_id',
            default_value='run_001',
            description='Run identifier for experiment output'
        ),
        
        Node(
            package='autocert_experiment',
            executable='experiment_node',
            name='experiment_node',
            output='screen',
            arguments=[
                '--config', LaunchConfiguration('config_file'),
                '--output', LaunchConfiguration('output_file'),
                '--run-id', LaunchConfiguration('run_id'),
            ],
            parameters=[{
                'robot_mode': LaunchConfiguration('robot_mode')
            }]
        ),
    ])