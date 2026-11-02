#include "autocert_core/collision_checker.hpp"
#include <moveit/robot_state/robot_state.h>

namespace autocert_core
{

CollisionChecker::CollisionChecker(
  std::shared_ptr<planning_scene::PlanningScene> planning_scene,
  const std::string& group_name)
  : planning_scene_(planning_scene)
  , group_name_(group_name)
{
  robot_state_ = std::make_unique<moveit::core::RobotState>(
    planning_scene_->getRobotModel());
  joint_model_group_ =
    planning_scene_->getRobotModel()->getJointModelGroup(group_name);
}

bool CollisionChecker::checkState(const std::vector<double>& joint_positions)
{
  robot_state_->setJointGroupPositions(joint_model_group_, joint_positions);
  robot_state_->update();
  return !planning_scene_->isStateColliding(*robot_state_, group_name_);
}

std::vector<bool> CollisionChecker::batchCheck(
  const std::vector<std::vector<double>>& joint_samples)
{
  std::vector<bool> results;
  results.reserve(joint_samples.size());
  
  for (const auto& sample : joint_samples) {
    results.push_back(checkState(sample));
  }
  
  return results;
}

}  // namespace autocert_core