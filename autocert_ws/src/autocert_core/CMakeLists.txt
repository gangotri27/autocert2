cmake_minimum_required(VERSION 3.8)
project(autocert_core)

# Enable C++20 for std::span support
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(autocert_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml-cpp REQUIRED)

# Create core library
add_library(autocert_core SHARED
  src/workspace_sampler.cpp
  src/collision_checker.cpp
)

target_include_directories(autocert_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(autocert_core
  rclcpp
  moveit_ros_planning
  moveit_core
  geometry_msgs
  autocert_interfaces
  tf2
  tf2_geometry_msgs
  pluginlib
  yaml-cpp
)

target_link_libraries(autocert_core
  ${EIGEN3_LIBRARIES}
  yaml-cpp
)

# Create executable
add_executable(batch_fk_server src/batch_fk_server.cpp)
ament_target_dependencies(batch_fk_server
  rclcpp
  moveit_ros_planning
  moveit_core
  autocert_interfaces
)
target_link_libraries(batch_fk_server autocert_core)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}
)

# Install libraries
install(
  TARGETS autocert_core batch_fk_server
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(autocert_core)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
