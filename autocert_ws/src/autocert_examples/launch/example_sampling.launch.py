"""Example launch file for Panda robot workspace sampling"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for Panda example"""
    
    # Get package directories
    panda_description_dir = get_package_share_directory('franka_description')
    autocert_launch_dir = get_package_share_directory('autocert_launch')
    
    # Path to Panda URDF
    panda_urdf = os.path.join(
        panda_description_dir,
        'robots',
        'panda_arm.urdf'
    )
    
    # Include the main sampling pipeline
    sampling_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            autocert_launch_dir,
            '/launch/sampling_pipeline.launch.py'
        ]),
        launch_arguments={
            'robot_name': 'panda',
            'planning_group': 'panda_arm',
            'end_effector_link': 'panda_hand_tcp',
            'enable_collision_checking': 'true',
            'robot_description_file': panda_urdf
        }.items()
    )
    
    return LaunchDescription([
        sampling_pipeline
    ])