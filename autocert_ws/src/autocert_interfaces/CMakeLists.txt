cmake_minimum_required(VERSION 3.8)
project(autocert_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  "msg/PoseMeasurement.msg"
)

set(srv_files
  "srv/BatchFK.srv"
  "srv/GetJointLimits.srv"
  "srv/GetRobotInfo.srv"
  "srv/CheckCollision.srv"
  "srv/SampleWorkspace.srv"
  "srv/GetExecutionStatus.srv" 
)

set(action_files
  "action/MoveToPose.action"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES geometry_msgs builtin_interfaces action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
