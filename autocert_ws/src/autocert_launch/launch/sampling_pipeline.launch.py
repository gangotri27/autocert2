"""Launch file for AutoCert workspace sampling pipeline"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    robot_description_file = LaunchConfiguration('robot_description_file')
    srdf_file = LaunchConfiguration('srdf_file')

    robot_description = ParameterValue(
        Command(['xacro ', robot_description_file]),
        value_type=str
    )

    robot_description_semantic = ParameterValue(
        Command(['xacro ', srdf_file]),
        value_type=str
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'robot_description_file',
            description='Absolute path to URDF or XACRO file'
        ),

        DeclareLaunchArgument(
            'srdf_file',
            description='Absolute path to SRDF file'
        ),

        DeclareLaunchArgument(
            'robot_name',
            default_value='robot'
        ),

        DeclareLaunchArgument(
            'planning_group',
            default_value=''
        ),

        DeclareLaunchArgument(
            'end_effector_link',
            default_value=''
        ),

        DeclareLaunchArgument(
            'enable_collision_checking',
            default_value='true'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description
            }]
        ),

        Node(
            package='autocert_core',
            executable='batch_fk_server',
            name='batch_fk_server',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'robot_description_param': 'robot_description',
                'robot_name': LaunchConfiguration('robot_name'),
                'planning_group': LaunchConfiguration('planning_group'),
                'end_effector_link': LaunchConfiguration('end_effector_link'),
                'enable_collision_checking': LaunchConfiguration('enable_collision_checking')
            }]
        )

    ])