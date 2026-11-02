#ifndef AUTOCERT_CORE__COLLISION_CHECKER_HPP_
#define AUTOCERT_CORE__COLLISION_CHECKER_HPP_

#include <moveit/planning_scene/planning_scene.h>
#include <vector>
#include <string>
#include <memory>

namespace autocert_core
{

class CollisionChecker
{
public:
  explicit CollisionChecker(
    std::shared_ptr<planning_scene::PlanningScene> planning_scene,
    const std::string& group_name);

  bool checkState(const std::vector<double>& joint_positions);
  std::vector<bool> batchCheck(
    const std::vector<std::vector<double>>& joint_samples);

private:
  std::shared_ptr<planning_scene::PlanningScene> planning_scene_;
  std::unique_ptr<moveit::core::RobotState> robot_state_;
  const moveit::core::JointModelGroup* joint_model_group_;
  std::string group_name_;
};

}  // namespace autocert_core

#endif