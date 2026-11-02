# my_repo

```bash
ros2 run ros_gz_sim create -name cube  -x 0.4 -y 0 -z 0.05 -file ~/omx_ws/src/omx_demos/cube.sdf
```

# Step 1: The MoveIt Setup Assistant (Configuration Package)

## You cannot write the config manually; you must use the GUI tool.

## 1. Launch the Assistant:

```bash

export LIBGL_ALWAYS_SOFTWARE=1

ros2 launch moveit_setup_assistant setup_assistant.launch.py

```

## 2. Load URDF:

### Click Create New MoveIt Configuration.
### Browse to the open_manipulator_x_description/urdf/open_manipulator_x.urdf.xacro (or your specific .urdf file).
### Click Load Files.

## 3. Self-Collisions:

### Click Generate Collision Matrix.

## 4. Virtual Joints:

### Add a Virtual Joint.
### Name: virtual_joint
### Child Link: base_link (or link1 depending on your URDF, usually link1 is the base for OMX).
### Parent Frame: world
### Type: fixed

## 5. Planning Groups:

### Group 1: Name: arm
#### -> Kinematics Solver: kdl_kinematics_plugin/KDLKinematicsPlugin
#### -> Add Kin. Chain: Base=link1, Tip=link5 (or end_effector_link).
### Group 2: Name: gripper
#### -> Kinematics Solver: None
#### -> Add Joints: gripper_left_joint, gripper_right_joint.

## 6. Robot Poses (SRDF):

### Move the sliders to a "Home" position (all zeros usually). Name it home. Click Save Pose.
### Move sliders to a "Ready" position. Name it ready. Save Pose.
### Create pick and place similarly.
### Also create for gripper -> close and open.

## 7. ros2_control:

### Click Add Interface.
### Select ros2_control.
### Click Auto Detect (if available) or manually add FollowJointTrajectory for the arm joints.

## 8. End Effectors:

### Click Add End Effector.
### Name: gripper_ee (or just gripper_hand)
### End Effector Group: Select gripper.
### Parent Link: Select link5 .
### Parent Group: Select arm (optional, but good practice).
### Click Save.

## 9. ROS 2 Controllers:

### Click the Auto Detect button.

## 10. Author Information: 

### Enter your name/email.

## 11. Configuration Files:

### Save Path: ~/ros2_ws/src/openmanipulator_x_moveit_config (Create this folder).
### Click Generate Package.

# Step 2: Configure Controllers (ros2_controllers.yaml)

## The Setup Assistant generates a generic controller file. You usually need to tweak it to match the hardware interface or simulation.

## Open src/openmanipulator_x_moveit_config/config/ros2_controllers.yaml. Ensure it looks like this (names must match your URDF joints):

```yaml
# This config file is used by ros2_control
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController


    gripper_controller:
      type: position_controllers/GripperActionController


    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

arm_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    allow_nonzero_velocity_at_trajectory_end: true
gripper_controller:
  ros__parameters:
    joints:
      - gripper_left_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    allow_nonzero_velocity_at_trajectory_end: true
```

## Edit moveit_controllers.yaml

```yaml
# MoveIt uses this configuration for controller management
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - arm_controller
    - gripper_controller

  arm_controller:
    # Explicitly tell MoveIt the action namespace
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - joint1
      - joint2
      - joint3
      - joint4

  gripper_controller:
    # IMPORTANT: This must match the action namespace of your driver
    # For GripperActionController, it is usually "gripper_cmd"
    action_ns: gripper_cmd
    
    # IMPORTANT: Change type to GripperCommand to match your driver
    type: GripperCommand
    
    default: true
    joints:
      - gripper_left_joint
```

## Edit open_manipulator_x.urdf.xacro:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="open_manipulator_x">
    <xacro:arg name="initial_positions_file" default="initial_positions.yaml" />

    <!-- 1. Import Geometry (Links/Joints only) -->
    <xacro:include filename="$(find open_manipulator_description)/urdf/open_manipulator_x/open_manipulator_x_arm.urdf.xacro" />

    <!-- 2. Import Control (The file we just fixed in Step 1) -->
    <!-- We use $(find openmanipulator_x_moveit_config) to make sure it finds the local file -->
    <xacro:include filename="$(find openmanipulator_x_moveit_config)/config/open_manipulator_x.ros2_control.xacro" />

    <!-- 3. Create World Link -->
    <link name="world"/>
    <joint name="world_fixed" type="fixed">
        <parent link="world"/>
        <child link="link1"/>
    </joint>

    <!-- 4. Instantiate Robot and Control -->
    <xacro:open_manipulator_x prefix="" />
    <xacro:open_manipulator_x_ros2_control name="MockSystem" initial_positions_file="$(arg initial_positions_file)"/>
</robot>
```

## Fix open_manipulator_x.ros2_control.xacro:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:macro name="open_manipulator_x_ros2_control" params="name initial_positions_file">
        <xacro:property name="initial_positions" value="${xacro.load_yaml(initial_positions_file)['initial_positions']}"/>

        <ros2_control name="${name}" type="system">
            <hardware>
                <!-- FORCE MOCK HARDWARE -->
                <plugin>mock_components/GenericSystem</plugin>
            </hardware>
            <joint name="joint1">
                <command_interface name="position"/>
                <state_interface name="position">
                  <param name="initial_value">${initial_positions['joint1']}</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>
            <joint name="joint2">
                <command_interface name="position"/>
                <state_interface name="position">
                  <param name="initial_value">${initial_positions['joint2']}</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>
            <joint name="joint3">
                <command_interface name="position"/>
                <state_interface name="position">
                  <param name="initial_value">${initial_positions['joint3']}</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>
            <joint name="joint4">
                <command_interface name="position"/>
                <state_interface name="position">
                  <param name="initial_value">${initial_positions['joint4']}</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>
            <joint name="gripper_left_joint">
                <command_interface name="position"/>
                <state_interface name="position">
                  <param name="initial_value">${initial_positions['gripper_left_joint']}</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>
        </ros2_control>
    </xacro:macro>
</robot>
```

# Step 3: The Bring-up Launch File

## Create a file named demo.launch.py inside openmanipulator_x_moveit_config/launch/.

```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. Define the path EXPLICITLY
    pkg_share = FindPackageShare("openmanipulator_x_moveit_config")
    urdf_file_path = PathJoinSubstitution([pkg_share, "config", "open_manipulator_x.urdf.xacro"])

    # 2. Process the Xacro file manually
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ", urdf_file_path,
        " initial_positions_file:=", 
        PathJoinSubstitution([pkg_share, "config", "initial_positions.yaml"])
    ])
    
    robot_description = {"robot_description": robot_description_content}

    # 3. Load other MoveIt configs
    moveit_config = (
        MoveItConfigsBuilder("open_manipulator_x", package_name="openmanipulator_x_moveit_config")
        .robot_description(file_path="config/open_manipulator_x.urdf.xacro")
        .robot_description_semantic(file_path="config/open_manipulator_x.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # 4. Create Nodes

    # --- CRITICAL NODE: ROBOT STATE PUBLISHER ---
    # This is what you were missing. It publishes the /robot_description topic.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    # Controller Manager
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution([pkg_share, "config", "ros2_controllers.yaml"])
        ],
        output="screen",
    )

    # Move Group
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", PathJoinSubstitution([pkg_share, "config", "moveit.rviz"])],
        parameters=[
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    # Spawners
    spawners = []
    for controller in ["joint_state_broadcaster", "arm_controller", "gripper_controller"]:
        spawners.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "--controller-manager", "/controller_manager"],
            )
        )

    # Ensure robot_state_publisher_node is in this list!
    nodes_to_start = [robot_state_publisher_node, ros2_control_node, run_move_group_node, rviz_node] + spawners
    return LaunchDescription(nodes_to_start)
```

# Step 4: The C++ Demo Node

## Create a new package (e.g., omx_moveit_demo) or add to your existing workspace.

```bash
cd ~/omx_ws/src
ros2 pkg create omx_moveit_demo --build-type ament_cmake --license Apache-2.0 --dependencies rclcpp moveit_ros_planning_interface
touch ~/omx_ws/src/omx_moveit_demo/src/simple_move.cpp
```

## File: src/simple_move.cpp

```cpp
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("omx_moveit_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("omx_moveit_demo");

  // We spin up a thread so the node can process MoveIt messages in the background
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // 1. Setup ARM Interface
  // "arm" is the group name we defined in the Setup Assistant
  moveit::planning_interface::MoveGroupInterface arm_group(node, "arm");
  arm_group.setMaxVelocityScalingFactor(1.0); // Full speed
  arm_group.setMaxAccelerationScalingFactor(1.0);

  // 2. Setup GRIPPER Interface
  // "gripper" is the group name we defined in the Setup Assistant
  moveit::planning_interface::MoveGroupInterface gripper_group(node, "gripper");
  gripper_group.setMaxVelocityScalingFactor(1.0);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // --- STEP 1: Go to HOME ---
  RCLCPP_INFO(LOGGER, "Planning to HOME...");
  arm_group.setNamedTarget("home");
  
  if (arm_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    arm_group.execute(my_plan);
    RCLCPP_INFO(LOGGER, "Executed HOME.");
  } else {
    RCLCPP_ERROR(LOGGER, "Failed to plan to HOME.");
  }

  // --- STEP 2: OPEN Gripper ---
  RCLCPP_INFO(LOGGER, "Opening Gripper...");
  // Try named target first, if you defined "open" in setup assistant
  // If "open" doesn't exist, you can set joint targets manually:
  // gripper_group.setJointValueTarget({0.019}); 
  gripper_group.setNamedTarget("open"); 
  gripper_group.move(); // move() plans and executes in one step

  // --- STEP 3: Go to PICK ---
  // Ensure you created a pose named "pick" in Setup Assistant, otherwise use "ready"
  RCLCPP_INFO(LOGGER, "Planning to READY...");
  arm_group.setNamedTarget("ready"); 
  arm_group.move();

  // --- STEP 4: CLOSE Gripper ---
  RCLCPP_INFO(LOGGER, "Closing Gripper...");
  gripper_group.setNamedTarget("close");
  gripper_group.move();

  // ... after closing gripper ...
  
  // --- STEP 5: Go to PLACE ---
  RCLCPP_INFO(LOGGER, "Planning to PLACE...");
  arm_group.setNamedTarget("place"); // Make sure "place" exists in your SRDF
  arm_group.move();

  RCLCPP_INFO(LOGGER, "Demo Complete. Shutting down.");
  rclcpp::shutdown();
  return 0;
}
```

## Configure CMakeLists.txt

```txt
cmake_minimum_required(VERSION 3.8)
project(omx_moveit_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Create the executable
add_executable(simple_move src/simple_move.cpp)
ament_target_dependencies(simple_move rclcpp moveit_ros_planning_interface)

# Install the executable so ros2 run can find it
install(TARGETS simple_move
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

## Configure package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>omx_moveit_demo</name>
  <version>0.0.0</version>
  <description>MoveIt 2 Demo for OpenManipulator</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- These are the critical dependencies -->
  <depend>rclcpp</depend>
  <depend>moveit_ros_planning_interface</depend>
  <depend>moveit_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

# Step 5: Create a Launch File for the Code

```bash
touch src/openmanipulator_x_moveit_config/launch/run_cpp_node.launch.py
```

## Add the Content

### Open src/openmanipulator_x_moveit_config/launch/run_cpp_node.launch.py and paste this code.
### This script does three things:
### -> Loads the URDF (Geometry).
### -> Loads the SRDF (Groups/Poses).
### -> Runs your simple_move executable with those parameters.

```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. Define the path EXPLICITLY (Same as your demo.launch.py)
    pkg_share = FindPackageShare("openmanipulator_x_moveit_config")
    urdf_file_path = PathJoinSubstitution([pkg_share, "config", "open_manipulator_x.urdf.xacro"])

    # 2. Process the Xacro file manually to get the URDF
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ", urdf_file_path,
        " initial_positions_file:=", 
        PathJoinSubstitution([pkg_share, "config", "initial_positions.yaml"])
    ])
    
    robot_description = {"robot_description": robot_description_content}

    # 3. Load the SRDF and other configs using MoveItConfigsBuilder
    moveit_config = (
        MoveItConfigsBuilder("open_manipulator_x", package_name="openmanipulator_x_moveit_config")
        .robot_description(file_path="config/open_manipulator_x.urdf.xacro")
        .robot_description_semantic(file_path="config/open_manipulator_x.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    # 4. Run the C++ Node with the parameters
    return LaunchDescription([
        Node(
            package="omx_moveit_demo", # Your C++ package
            executable="simple_move",  # Your C++ executable
            name="omx_moveit_demo_node",
            output="screen",
            parameters=[
                robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
            ],
        )
    ])
```

# Step 6: Build and Run

```bash
cd ~/omx_ws
colcon build --packages-select omx_moveit_demo
colcon build --packages-select openmanipulator_x_moveit_config
source install/setup.bash
ros2 launch openmanipulator_x_moveit_config demo.launch.py
ros2 launch openmanipulator_x_moveit_config run_cpp_node.launch.py
```
