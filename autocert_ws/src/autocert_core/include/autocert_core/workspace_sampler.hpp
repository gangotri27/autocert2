#ifndef AUTOCERT_CORE__WORKSPACE_SAMPLER_HPP_
#define AUTOCERT_CORE__WORKSPACE_SAMPLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <mutex>
#include <atomic>
#include <thread>
#include <vector>
#include <memory>

namespace autocert_core
{

struct RobotInfo
{
  std::string robot_name;
  std::string planning_group;
  std::string end_effector_link;
  std::vector<std::string> joint_names;
  std::vector<std::string> link_names;
  std::vector<std::string> group_names;
  std::vector<double> joint_min_limits;
  std::vector<double> joint_max_limits;
  std::vector<double> joint_velocity_limits;
  size_t num_joints;
  size_t num_links;
};

struct BatchFKResult
{
  std::vector<geometry_msgs::msg::Pose> poses;
  std::vector<int> valid_indices;
  double reachability_index;
  double computation_time_ms;
};

class WorkspaceSampler
{
public:
  explicit WorkspaceSampler(
    const rclcpp::Node::SharedPtr& node,
    const std::string& robot_description_param);
  ~WorkspaceSampler();

  // Planning group management
  void setPlanningGroup(const std::string& group_name);
  void setEndEffectorLink(const std::string& link_name);
  void setCollisionChecking(bool enable);
  
  // Robot information
  RobotInfo getRobotInfo() const;
  std::pair<std::vector<double>, std::vector<double>> getJointLimits() const;
  std::string getEndEffectorLink() const { return end_effector_link_; }
  
  // Kinematics computation
  geometry_msgs::msg::Pose computeFK(const std::vector<double>& joint_values);
  BatchFKResult computeBatchFK(
    const std::vector<std::vector<double>>& joint_samples,
    bool filter_collisions);
  bool isStateValid(const std::vector<double>& joint_values);

private:
  struct ThreadLocalData
  {
    std::unique_ptr<moveit::core::RobotState> robot_state;
    planning_scene::PlanningScenePtr planning_scene;
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    uint64_t version;
  };

  ThreadLocalData* getThreadLocalData();
  std::string findDefaultEndEffector() const;
  void initializeJointLimits();

  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelConstPtr robot_model_;
  std::shared_ptr<robot_model_loader::RobotModelLoader> model_loader_;
  planning_scene::PlanningSceneConstPtr base_planning_scene_;
  const moveit::core::JointModelGroup* joint_model_group_;
  
  std::vector<double> joint_min_limits_;
  std::vector<double> joint_max_limits_;
  std::vector<double> joint_velocity_limits_;
  std::vector<std::string> joint_names_;
  std::string end_effector_link_;
  std::string planning_group_;
  bool collision_checking_enabled_;
  
  std::atomic<uint64_t> state_version_{0};
  mutable std::mutex mutex_;
  mutable RobotInfo cached_info_;
  mutable bool info_cached_{false};
};

}  // namespace autocert_core

#endif