"""Launch file for AutoCert pipeline"""

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
            'master_config',
            default_value=os.path.join(
                get_package_share_directory('autocert_pipeline'),
                'config', 'master_config.yaml'
            ),
            description='Path to master configuration file'
        ),
        
        DeclareLaunchArgument(
            'robot_mode',
            default_value='simulation',
            choices=['simulation', 'real'],
            description='Robot mode: simulation or real'
        ),
        
        Node(
            package='autocert_pipeline',
            executable='pipeline_node',
            name='pipeline_node',
            output='screen',
            parameters=[{
                'master_config': LaunchConfiguration('master_config'),
                'robot_mode': LaunchConfiguration('robot_mode')
            }]
        ),
    ])