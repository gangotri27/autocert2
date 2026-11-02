#include "autocert_core/workspace_sampler.hpp"
#include <moveit/robot_state/conversions.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>

namespace autocert_core
{

WorkspaceSampler::WorkspaceSampler(
  const rclcpp::Node::SharedPtr& node,
  const std::string& robot_description_param)
  : node_(node)
  , joint_model_group_(nullptr)
  , collision_checking_enabled_(false)
{
  auto logger = node_->get_logger();
  RCLCPP_INFO(logger, "Initializing WorkspaceSampler with param: %s",
              robot_description_param.c_str());

  try {
    model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
      node_, robot_description_param, true);
    robot_model_ = model_loader_->getModel();

    if (!robot_model_)
    {
      RCLCPP_ERROR(logger, "Failed to load robot model");
      throw std::runtime_error("Failed to load robot model");
    }

    base_planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);

    const auto& group_names = robot_model_->getJointModelGroupNames();
    if (group_names.empty())
    {
      RCLCPP_ERROR(logger, "No joint model groups found");
      throw std::runtime_error("No joint model groups found");
    }

    planning_group_ = group_names[0];
    joint_model_group_ = robot_model_->getJointModelGroup(planning_group_);
    
    if (!joint_model_group_)
    {
      RCLCPP_ERROR(logger, "Failed to get joint model group");
      throw std::runtime_error("Failed to get joint model group");
    }

    end_effector_link_ = findDefaultEndEffector();
    initializeJointLimits();
    state_version_++;

    RCLCPP_INFO(logger,
      "WorkspaceSampler initialized:\n"
      "  Planning group: %s\n"
      "  End-effector: %s\n"
      "  Joints: %zu",
      planning_group_.c_str(),
      end_effector_link_.c_str(),
      joint_names_.size());

  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Initialization failed: %s", e.what());
    throw;
  }
}

WorkspaceSampler::~WorkspaceSampler() = default;

WorkspaceSampler::ThreadLocalData* WorkspaceSampler::getThreadLocalData()
{
  thread_local static std::unique_ptr<ThreadLocalData> thread_data;
  thread_local static uint64_t last_version = 0;
  
  uint64_t current_version = state_version_.load(std::memory_order_acquire);

  if (!thread_data || last_version != current_version)
  {
    auto new_data = std::make_unique<ThreadLocalData>();
    new_data->robot_state = std::make_unique<moveit::core::RobotState>(robot_model_);
    new_data->robot_state->setToDefaultValues();
    
    if (base_planning_scene_) {
      new_data->planning_scene = base_planning_scene_->diff();
    }
    
    new_data->collision_request.contacts = false;
    new_data->collision_request.max_contacts = 1;
    new_data->collision_request.max_contacts_per_pair = 1;
    new_data->collision_request.verbose = false;
    new_data->version = current_version;
    
    thread_data = std::move(new_data);
    last_version = current_version;
  }

  return thread_data.get();
}

void WorkspaceSampler::setPlanningGroup(const std::string& group_name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto* group = robot_model_->getJointModelGroup(group_name);
  
  if (!group) {
    RCLCPP_ERROR(node_->get_logger(),
      "Planning group '%s' not found", group_name.c_str());
    return;
  }

  planning_group_ = group_name;
  joint_model_group_ = group;
  end_effector_link_ = findDefaultEndEffector();
  info_cached_ = false;
  initializeJointLimits();
  state_version_++;

  RCLCPP_INFO(node_->get_logger(),
    "Switched to planning group: %s (version: %llu)",
    group_name.c_str(), state_version_.load());
}

void WorkspaceSampler::setEndEffectorLink(const std::string& link_name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (robot_model_->hasLinkModel(link_name)) {
    end_effector_link_ = link_name;
    info_cached_ = false;
    state_version_++;
    RCLCPP_INFO(node_->get_logger(), "End effector set to: %s", link_name.c_str());
  } else {
    RCLCPP_ERROR(node_->get_logger(),
      "Link '%s' not found", link_name.c_str());
  }
}

void WorkspaceSampler::setCollisionChecking(bool enable)
{
  collision_checking_enabled_ = enable;
}

RobotInfo WorkspaceSampler::getRobotInfo() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (info_cached_) {
    return cached_info_;
  }

  RobotInfo info;
  info.robot_name = robot_model_->getName();
  info.planning_group = planning_group_;
  info.end_effector_link = end_effector_link_;
  info.joint_names = joint_names_;
  info.joint_min_limits = joint_min_limits_;
  info.joint_max_limits = joint_max_limits_;
  info.joint_velocity_limits = joint_velocity_limits_;
  info.num_joints = joint_names_.size();

  const auto& link_models = robot_model_->getLinkModels();
  info.link_names.reserve(link_models.size());
  for (const auto& link : link_models) {
    info.link_names.push_back(link->getName());
  }
  info.num_links = info.link_names.size();
  info.group_names = robot_model_->getJointModelGroupNames();

  cached_info_ = info;
  info_cached_ = true;
  return info;
}

std::pair<std::vector<double>, std::vector<double>> WorkspaceSampler::getJointLimits() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return {joint_min_limits_, joint_max_limits_};
}

geometry_msgs::msg::Pose WorkspaceSampler::computeFK(const std::vector<double>& joint_values)
{
  if (!joint_model_group_) {
    throw std::runtime_error("No planning group set");
  }

  if (joint_values.size() != joint_model_group_->getActiveVariableCount()) {
    throw std::runtime_error("Invalid number of joint values");
  }

  auto* local_data = getThreadLocalData();
  auto* state = local_data->robot_state.get();

  state->setJointGroupPositions(joint_model_group_, joint_values);
  state->update();

  const Eigen::Isometry3d& end_effector_state =
    state->getGlobalLinkTransform(end_effector_link_);

  geometry_msgs::msg::Pose pose;
  pose.position.x = end_effector_state.translation().x();
  pose.position.y = end_effector_state.translation().y();
  pose.position.z = end_effector_state.translation().z();

  Eigen::Quaterniond quat(end_effector_state.rotation());
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();

  return pose;
}

BatchFKResult WorkspaceSampler::computeBatchFK(
  const std::vector<std::vector<double>>& joint_samples,
  bool filter_collisions)
{
  BatchFKResult result;
  auto start_time = std::chrono::steady_clock::now();

  result.poses.reserve(joint_samples.size());
  result.valid_indices.reserve(joint_samples.size());

  auto* local_data = getThreadLocalData();
  auto* state = local_data->robot_state.get();

  for (size_t i = 0; i < joint_samples.size(); ++i) {
    const auto& sample = joint_samples[i];

    state->setJointGroupPositions(joint_model_group_, sample);
    state->update();

    const Eigen::Isometry3d& end_effector_state =
      state->getGlobalLinkTransform(end_effector_link_);

    geometry_msgs::msg::Pose pose;
    pose.position.x = end_effector_state.translation().x();
    pose.position.y = end_effector_state.translation().y();
    pose.position.z = end_effector_state.translation().z();

    Eigen::Quaterniond quat(end_effector_state.rotation());
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();

    result.poses.push_back(std::move(pose));

    bool is_valid = true;
    if (filter_collisions && collision_checking_enabled_) {
      if (local_data->planning_scene) {
        local_data->collision_result.clear();
        local_data->planning_scene->checkCollision(
          local_data->collision_request,
          local_data->collision_result,
          *state
        );
        is_valid = !local_data->collision_result.collision;
      }
    }

    if (is_valid) {
      result.valid_indices.push_back(static_cast<int>(i));
    }
  }

  auto end_time = std::chrono::steady_clock::now();
  result.computation_time_ms = std::chrono::duration<double, std::milli>(
    end_time - start_time).count();
  
  if (!joint_samples.empty()) {
    result.reachability_index = (result.valid_indices.size() * 100.0) / joint_samples.size();
  }

  return result;
}

bool WorkspaceSampler::isStateValid(const std::vector<double>& joint_values)
{
  if (!collision_checking_enabled_) {
    return true;
  }

  auto* local_data = getThreadLocalData();
  auto* state = local_data->robot_state.get();

  state->setJointGroupPositions(joint_model_group_, joint_values);
  state->update();

  if (local_data->planning_scene) {
    local_data->collision_result.clear();
    local_data->planning_scene->checkCollision(
      local_data->collision_request,
      local_data->collision_result,
      *state
    );
    return !local_data->collision_result.collision;
  }

  return true;
}

std::string WorkspaceSampler::findDefaultEndEffector() const
{
  if (!joint_model_group_) {
    return "";
  }

  const auto& link_names = joint_model_group_->getLinkModelNames();
  if (!link_names.empty()) {
    return link_names.back();
  }
  return "";
}

void WorkspaceSampler::initializeJointLimits()
{
  joint_min_limits_.clear();
  joint_max_limits_.clear();
  joint_velocity_limits_.clear();
  joint_names_.clear();

  if (!joint_model_group_) return;

  const auto& joints = joint_model_group_->getJointModels();
  for (const auto& joint : joints)
  {
    if (joint->getType() == moveit::core::JointModel::REVOLUTE ||
        joint->getType() == moveit::core::JointModel::PRISMATIC)
    {
      const auto& bounds = joint->getVariableBounds();
      if (!bounds.empty()) {
        joint_min_limits_.push_back(bounds[0].min_position_);
        joint_max_limits_.push_back(bounds[0].max_position_);
        joint_velocity_limits_.push_back(bounds[0].max_velocity_);
        joint_names_.push_back(joint->getName());
      }
    }
  }
}

}  // namespace autocert_core