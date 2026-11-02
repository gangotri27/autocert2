"""Example launch file for Panda robot workspace sampling with explicit group selection"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for Panda example"""
    
    # Include the main sampling pipeline with explicit Panda configuration
    sampling_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('autocert_launch'),
            '/launch/sampling_pipeline.launch.py'
        ]),
        launch_arguments={
            'robot_name': 'panda',
            'robot_description_file': '/opt/ros/jazzy/share/moveit_resources_panda_description/urdf/panda.urdf',
            'srdf_file': '/opt/ros/jazzy/share/moveit_resources_panda_moveit_config/config/panda.srdf',
            'planning_group': 'panda_arm',  # Explicit planning group for Panda
            'end_effector_link': 'panda_link8'  # Explicit end-effector
        }.items()
    )
    
    # Load Panda description
    panda_description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(
                os.path.join(
                    get_package_share_directory('franka_description'),
                    'robots',
                    'panda_arm.urdf'
                )
            ).read()
        }]
    )
    
    return LaunchDescription([
        panda_description,
        sampling_pipeline
    ])