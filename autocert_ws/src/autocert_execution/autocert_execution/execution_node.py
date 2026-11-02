#!/usr/bin/env python3
"""
execution_node.py  —  AutoCert execution node (MoveItPy backend)
=================================================================
Accepts /move_to_pose action goals, plans via MoveItPy, executes,
and returns success/failure. Includes a robust Simulation Fallback.
"""

import time
import xml.etree.ElementTree as ET
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from autocert_interfaces.action import MoveToPose
from autocert_interfaces.srv import GetExecutionStatus

try:
    from moveit.planning import MoveItPy
    from moveit.core.robot_state import RobotState
    MOVEIT_PY_AVAILABLE = True
except ImportError:
    MOVEIT_PY_AVAILABLE = False


class ExecutionNode(Node):

    MAX_INIT_ATTEMPTS = 10
    INIT_BASE_DELAY   = 3.0

    def __init__(self):
        super().__init__('execution_node')

        self._declare_all_parameters()

        self.planning_group    = self.get_parameter('planning_group').value
        self.end_effector_link = self.get_parameter('end_effector_link').value
        self.reference_frame   = self.get_parameter('reference_frame').value
        self.planner_id        = self.get_parameter('planner_id').value
        self.planning_time     = self.get_parameter('planning_time').value
        self.vel_scale         = self.get_parameter('velocity_scaling').value
        self.acc_scale         = self.get_parameter('acceleration_scaling').value
        self.robot_mode        = self.get_parameter('robot_mode').value

        self.get_logger().info(
            f'ExecutionNode ready\n'
            f'  Planning group : {self.planning_group}\n'
            f'  End-effector   : {self.end_effector_link}\n'
            f'  Reference frame: {self.reference_frame}\n'
            f'  Planner        : {self.planner_id}\n'
            f'  Mode           : {self.robot_mode}'
        )

        self._moveit = None
        self._ready = False
        self._is_executing = False
        self._init_attempt = 0
        self._last_joint_state = None

        self._cb_group = ReentrantCallbackGroup()

        self._motion_done_pub = self.create_publisher(Bool, '/motion_done', 10)
        self._status_pub      = self.create_publisher(String, '/execution_status', 10)
        self._sim_joint_pub   = self.create_publisher(JointState, '/simulated_joint_states', 10)

        self.create_service(GetExecutionStatus, '/get_execution_status', self._get_status_cb, callback_group=self._cb_group)
        self._action_server = ActionServer(self, MoveToPose, '/move_to_pose', execute_callback=self._execute_cb, callback_group=self._cb_group)

        if self.robot_mode == 'simulation':
            self.create_timer(0.05, self._publish_simulated_joint_states, callback_group=self._cb_group)

        self.create_timer(0.5, self._try_init_moveit_py, callback_group=self._cb_group)

    def _declare_all_parameters(self):
        from rcl_interfaces.msg import ParameterDescriptor, ParameterType
        PT = ParameterType
        def pd(desc, ptype):
            d = ParameterDescriptor()
            d.description = desc
            d.type = ptype
            return d

        self.declare_parameter('planning_group', 'arm', pd('MoveIt planning group', PT.PARAMETER_STRING))
        self.declare_parameter('end_effector_link', 'tool0', pd('End-effector link', PT.PARAMETER_STRING))
        self.declare_parameter('reference_frame', 'base_link', pd('Planning frame', PT.PARAMETER_STRING))
        self.declare_parameter('planner_id', 'RRTConnect', pd('OMPL planner ID', PT.PARAMETER_STRING))
        self.declare_parameter('planning_time', 5.0, pd('Max planning time', PT.PARAMETER_DOUBLE))
        self.declare_parameter('velocity_scaling', 0.3, pd('Velocity scaling', PT.PARAMETER_DOUBLE))
        self.declare_parameter('acceleration_scaling', 0.3, pd('Acceleration scaling', PT.PARAMETER_DOUBLE))
        self.declare_parameter('robot_mode', 'simulation', pd('Robot Mode', PT.PARAMETER_STRING))
        self.declare_parameter('robot_description', '', pd('Robot URDF XML', PT.PARAMETER_STRING))
        self.declare_parameter('robot_description_semantic', '', pd('Robot SRDF XML', PT.PARAMETER_STRING))

    def _publish_simulated_joint_states(self):
        if self._last_joint_state is None:
            try:
                robot_description = self.get_parameter('robot_description').value
                if not robot_description: return
                root = ET.fromstring(robot_description)
                joint_names = [j.attrib['name'] for j in root.findall('joint') if j.attrib.get('type') not in['fixed', 'floating']]
                if joint_names:
                    js_msg = JointState()
                    js_msg.name = joint_names
                    js_msg.position =[0.0] * len(joint_names)
                    self._last_joint_state = js_msg
            except Exception: return

        if self._last_joint_state:
            js_msg = JointState()
            js_msg.header.stamp = self.get_clock().now().to_msg()
            js_msg.name = self._last_joint_state.name
            
            # --- REDUCED KINEMATIC NOISE ---
            # Reduced to 0.0002 to keep the visual robot completely stable
            noise = np.random.normal(0.0, 0.0002, len(self._last_joint_state.position))
            js_msg.position =[p + n for p, n in zip(self._last_joint_state.position, noise)]
            
            self._sim_joint_pub.publish(js_msg)

    def _try_init_moveit_py(self):
        if not rclpy.ok() or self._ready: return
        if not MOVEIT_PY_AVAILABLE: return

        self._init_attempt += 1
        try:
            robot_description = self.get_parameter('robot_description').value
            robot_description_semantic = self.get_parameter('robot_description_semantic').value
            planner_config_name = f"{self.planner_id}kConfigDefault"

            config_dict = {
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'planning_scene_monitor_options': {
                    'name': "planning_scene_monitor",
                    'robot_description': "robot_description",
                    'joint_state_topic': "/joint_states",
                    'attached_collision_object_topic': "/moveit_cpp/planning_scene_monitor",
                    'publish_planning_scene_topic': "/moveit_cpp/publish_planning_scene",
                    'monitored_planning_scene_topic': "/moveit_cpp/monitored_planning_scene",
                    'wait_for_initial_state_timeout': 10.0,
                },
                'planning_pipelines': {'pipeline_names': ["ompl"]},
                'plan_request_params': {
                    'planning_attempts': 5,
                    'planning_pipeline': 'ompl',
                    'planner_id': planner_config_name,
                    'max_velocity_scaling_factor': self.vel_scale,
                    'max_acceleration_scaling_factor': self.acc_scale,
                    'planning_time': self.planning_time,
                },
                'ompl': {
                    'planning_plugins':['ompl_interface/OMPLPlanner'],
                    'request_adapters':['default_planning_request_adapters/ResolveConstraintFrames', 'default_planning_request_adapters/CheckStartStateBounds', 'default_planning_request_adapters/CheckStartStateCollision'],
                    'response_adapters':['default_planning_response_adapters/ValidateSolution', 'default_planning_response_adapters/DisplayMotionPath'],
                    'planner_configs': {planner_config_name: {'type': f'geometric::{self.planner_id}', 'range': 0.0}},
                    self.planning_group: {'default_planner_config': planner_config_name, 'planner_configs':[planner_config_name]}
                },
                'robot_description_kinematics': {
                    self.planning_group: {
                        'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
                        'kinematics_solver_search_resolution': 0.001,
                        'kinematics_solver_timeout': 0.5,
                        'kinematics_solver_attempts': 50,
                        'position_only_ik': True 
                    }
                },
                'moveit_manage_controllers': True
            }

            node_name = f'moveit_py_execution_{self._init_attempt}'
            self._moveit = MoveItPy(node_name=node_name, config_dict=config_dict)
            self._arm = self._moveit.get_planning_component(self.planning_group)
            self._ready = True
            self.get_logger().info('MoveItPy initialised successfully')
        except Exception as exc:
            if rclpy.ok(): self.create_timer(3.0, self._try_init_moveit_py, callback_group=self._cb_group)

    def _get_status_cb(self, request, response):
        response.is_ready = self._ready
        return response

    def _execute_cb(self, goal_handle):
        goal = goal_handle.request
        result = MoveToPose.Result()
        self._is_executing = True

        target = goal.target_pose
        planning_group = goal.planning_group or self.planning_group
        ee_link = goal.end_effector_link or self.end_effector_link

        try:
            arm = self._moveit.get_planning_component(planning_group)
            arm.set_start_state_to_current_state()

            robot_model = self._moveit.get_robot_model()
            goal_state = RobotState(robot_model)
            ik_success = goal_state.set_from_ik(planning_group, target.pose, ee_link)

            if not ik_success:
                self.get_logger().error('IK Solver failed. Target position is physically unreachable.')
                result.success = False
                goal_handle.abort()
                self._motion_done_pub.publish(Bool(data=False))
                return result
            
            goal_state.update()
            arm.set_goal_state(robot_state=goal_state)

            plan_start = time.time()
            plan_result = arm.plan()
            plan_ms = (time.time() - plan_start) * 1000.0
            
            result.planning_time_ms = plan_ms

            if not plan_result:
                result.success = False
                goal_handle.abort()
                self._motion_done_pub.publish(Bool(data=False))
                return result

            exec_start = time.time()
            exec_ok = False

            if goal.execute_immediately:
                exec_ok = self._moveit.execute(plan_result.trajectory, controllers=[])
                result.execution_time_ms = (time.time() - exec_start) * 1000.0

                if not exec_ok and self.robot_mode == 'simulation':
                    self.get_logger().info('Hardware execution unavailable — simulation mode active.')
                    
                    jmg = robot_model.get_joint_model_group(planning_group)
                    if hasattr(jmg, 'active_joint_model_names'):
                        joint_names = jmg.active_joint_model_names
                    else:
                        joint_names = jmg.get_active_joint_model_names()

                    joint_positions = goal_state.get_joint_group_positions(planning_group)
                    
                    js_msg = JointState()
                    js_msg.header.stamp = self.get_clock().now().to_msg()
                    js_msg.name         = list(joint_names)
                    js_msg.position     = list(joint_positions)
                    js_msg.velocity     =[0.0] * len(joint_names)
                    js_msg.effort       =[0.0] * len(joint_names)

                    self._last_joint_state = js_msg
                    self._sim_joint_pub.publish(js_msg)
                    
                    time.sleep(0.2)
                    exec_ok = True
                    result.execution_time_ms = (time.time() - exec_start) * 1000.0

            if exec_ok:
                result.success = True
                goal_handle.succeed()
                self._motion_done_pub.publish(Bool(data=True))
            else:
                result.success = False
                goal_handle.abort()
                self._motion_done_pub.publish(Bool(data=False))

        finally:
            self._is_executing = False

        return result


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=4)
    node = ExecutionNode()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()