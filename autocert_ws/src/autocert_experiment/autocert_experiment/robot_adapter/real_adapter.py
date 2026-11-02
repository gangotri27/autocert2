"""Real robot adapter with measurement system integration"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool
from autocert_interfaces.msg import PoseMeasurement
import time
from .base_adapter import RobotAdapter


class RealRobotAdapter(RobotAdapter):
    """Real robot adapter using ROS topics"""
    
    def __init__(self, node: Node):
        self.node = node
        self.connected = False
        
        # Publishers
        self.target_pose_pub = node.create_publisher(
            PoseStamped, '/target_pose', 10
        )
        
        # Subscribers
        self.motion_done = False
        self.motion_done_sub = node.create_subscription(
            Bool, '/motion_done', self.motion_done_callback, 10
        )
        
        self.last_measurement = None
        self.measurement_sub = node.create_subscription(
            PoseMeasurement, '/pose_measurement', self.measurement_callback, 10
        )
        
    def motion_done_callback(self, msg: Bool):
        """Handle motion completion signal"""
        self.motion_done = msg.data
        
    def measurement_callback(self, msg: PoseMeasurement):
        """Handle pose measurement"""
        self.last_measurement = msg.measured_pose
        
    def connect(self) -> bool:
        """Connect to real robot"""
        self.node.get_logger().info("Connecting to real robot...")
        self.connected = True
        return True
        
    def move_to_pose(self, pose: Pose, timeout: float = 10.0) -> bool:
        """Send target pose to robot"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.node.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link'
        pose_msg.pose = pose
        
        self.motion_done = False
        self.target_pose_pub.publish(pose_msg)
        return True
        
    def wait_motion_complete(self, timeout: float = 30.0) -> bool:
        """Wait for motion completion signal"""
        start_time = time.time()
        while not self.motion_done and (time.time() - start_time) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.motion_done
        
    def get_current_pose(self) -> Pose:
        """Get current pose from robot (not measurement)"""
        # In real implementation, would use TF
        return Pose()
        
    def get_measurement(self) -> Pose:
        """Get latest measurement from external system"""
        self.last_measurement = None
        # Wait for measurement
        start_time = time.time()
        while self.last_measurement is None and (time.time() - start_time) < 5.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.last_measurement
        
    def disconnect(self):
        """Disconnect from real robot"""
        self.connected = False