#!/usr/bin/env python3
"""
Dynamically loads URDF/SRDF and publishes robot_description
to the ROS2 parameter server. Fully robot-agnostic.
"""

import os
import subprocess
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import String


class RobotDescriptionLoader(Node):
    """
    Loads URDF (optionally via xacro) and SRDF and publishes them
    as ROS2 parameters. All downstream nodes read from these parameters.
    """

    def __init__(self):
        super().__init__('robot_description_loader')

        # Declare parameters — everything is injected, nothing hardcoded
        self.declare_parameter(
            'urdf_file', '',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Absolute path to robot URDF or xacro file'
            )
        )
        self.declare_parameter(
            'srdf_file', '',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Absolute path to robot SRDF file'
            )
        )
        self.declare_parameter(
            'xacro_args', '',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Additional xacro arguments as key:=value pairs (space separated)'
            )
        )
        self.declare_parameter('robot_name', 'robot')

        urdf_file = self.get_parameter('urdf_file').value
        srdf_file = self.get_parameter('srdf_file').value
        xacro_args = self.get_parameter('xacro_args').value
        self.robot_name = self.get_parameter('robot_name').value

        if not urdf_file:
            self.get_logger().fatal('urdf_file parameter is required!')
            raise RuntimeError('urdf_file not provided')

        # Load URDF/xacro
        robot_description = self._load_urdf(urdf_file, xacro_args)
        robot_description_semantic = ''
        if srdf_file and os.path.exists(srdf_file):
            robot_description_semantic = self._load_srdf(srdf_file)
        else:
            self.get_logger().warn('No SRDF file provided — planning groups must be defined elsewhere')

        # Publish as parameters so MoveIt nodes can read them
        self.declare_parameter('robot_description', robot_description)
        self.declare_parameter('robot_description_semantic', robot_description_semantic)

        # Also publish topics for monitoring
        self._urdf_pub = self.create_publisher(String, '/robot_description_raw', 1)
        self._srdf_pub = self.create_publisher(String, '/robot_description_semantic_raw', 1)

        # Latched-style: publish once + timer
        self._urdf_msg = String(data=robot_description)
        self._srdf_msg = String(data=robot_description_semantic)
        self._pub_timer = self.create_timer(2.0, self._publish_descriptions)

        self.get_logger().info(
            f'Robot description loaded for: {self.robot_name}\n'
            f'  URDF: {urdf_file}\n'
            f'  SRDF: {srdf_file}'
        )

    def _load_urdf(self, urdf_file: str, xacro_args: str) -> str:
        """Load URDF, processing through xacro if needed."""
        urdf_file = os.path.expanduser(urdf_file)
        if not os.path.exists(urdf_file):
            raise FileNotFoundError(f'URDF file not found: {urdf_file}')

        if urdf_file.endswith('.xacro'):
            cmd = ['xacro', urdf_file]
            if xacro_args:
                cmd.extend(xacro_args.split())
            result = subprocess.run(cmd, capture_output=True, text=True, check=True)
            return result.stdout
        else:
            with open(urdf_file, 'r') as f:
                return f.read()

    def _load_srdf(self, srdf_file: str) -> str:
        """Load SRDF from file."""
        srdf_file = os.path.expanduser(srdf_file)
        with open(srdf_file, 'r') as f:
            return f.read()

    def _publish_descriptions(self):
        """Publish descriptions periodically."""
        self._urdf_pub.publish(self._urdf_msg)
        self._srdf_pub.publish(self._srdf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotDescriptionLoader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()