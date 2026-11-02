#include <rclcpp/rclcpp.hpp>
#include <autocert_interfaces/srv/batch_fk.hpp>
#include <autocert_interfaces/srv/get_joint_limits.hpp>
#include <autocert_interfaces/srv/get_robot_info.hpp>
#include <autocert_interfaces/srv/check_collision.hpp>
#include <autocert_interfaces/srv/sample_workspace.hpp>
#include "autocert_core/workspace_sampler.hpp"

#include <memory>
#include <string>
#include <Eigen/Core>
#include <atomic>
#include <chrono>
#include <algorithm>

namespace autocert
{

class BatchFKServer : public rclcpp::Node
{
public:

  BatchFKServer() : Node("batch_fk_server")
  {
    // CRITICAL: declare robot_description so launch parameters are accepted
    this->declare_parameter<std::string>("robot_description", "");

    this->declare_parameter<std::string>("robot_description_param", "robot_description");
    this->declare_parameter<std::string>("planning_group", "");
    this->declare_parameter<std::string>("end_effector_link", "");
    this->declare_parameter<bool>("enable_collision_checking", true);
    this->declare_parameter<int>("service_threads", 4);
    this->declare_parameter<bool>("enable_performance_logging", false);
    this->declare_parameter<int>("max_samples_per_request", 100000);
    this->declare_parameter<int>("response_chunk_size", 20000);

    RCLCPP_INFO(this->get_logger(),
                "BatchFKServer created in namespace '%s' - waiting for parameters...",
                this->get_namespace());

    init_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&BatchFKServer::initialize, this));
  }

  void initialize()
  {
    init_timer_->cancel();

    RCLCPP_INFO(this->get_logger(), "Initializing BatchFKServer...");

    try
    {
      std::string robot_description_param =
          this->get_parameter("robot_description_param").as_string();

      RCLCPP_INFO(this->get_logger(),
                  "Looking for robot description in parameter: '%s'",
                  robot_description_param.c_str());

      std::string robot_description_xml;

      if (!this->get_parameter(robot_description_param, robot_description_xml) ||
          robot_description_xml.empty())
      {
        RCLCPP_ERROR(this->get_logger(),
                     "Parameter '%s' not found or empty!",
                     robot_description_param.c_str());

        throw std::runtime_error("Robot description parameter not found");
      }

      RCLCPP_INFO(this->get_logger(),
                  "Robot description loaded (%zu chars)",
                  robot_description_xml.length());

      std::string planning_group =
          this->get_parameter("planning_group").as_string();

      bool enable_collision =
          this->get_parameter("enable_collision_checking").as_bool();

      max_samples_per_request_ =
          this->get_parameter("max_samples_per_request").as_int();

      response_chunk_size_ =
          this->get_parameter("response_chunk_size").as_int();

      enable_performance_logging_ =
          this->get_parameter("enable_performance_logging").as_bool();

      sampler_ = std::make_unique<autocert_core::WorkspaceSampler>(
          shared_from_this(),
          robot_description_param);

      if (!planning_group.empty())
        sampler_->setPlanningGroup(planning_group);

      std::string end_effector_link =
          this->get_parameter("end_effector_link").as_string();

      if (!end_effector_link.empty())
        sampler_->setEndEffectorLink(end_effector_link);

      sampler_->setCollisionChecking(enable_collision);

      RCLCPP_INFO(this->get_logger(), "WorkspaceSampler initialized successfully");

      auto info = sampler_->getRobotInfo();

      RCLCPP_INFO(this->get_logger(),
                  "Robot: '%s' | Group: '%s' | Joints: %zu | Links: %zu",
                  info.robot_name.c_str(),
                  info.planning_group.c_str(),
                  info.num_joints,
                  info.num_links);

      createServices();

      RCLCPP_INFO(this->get_logger(),
                  "BatchFKServer fully initialized and ready");
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Initialization failed: %s",
                   e.what());
    }
  }

  size_t getTotalRequests() const { return total_requests_.load(); }
  size_t getTotalSamplesProcessed() const { return total_samples_processed_.load(); }

private:

  void createServices()
  {
    RCLCPP_INFO(this->get_logger(), "Creating services...");

    fk_srv_ = this->create_service<autocert_interfaces::srv::BatchFK>(
        "batch_fk",
        std::bind(&BatchFKServer::handleBatchFK, this,
                  std::placeholders::_1, std::placeholders::_2));

    limits_srv_ = this->create_service<autocert_interfaces::srv::GetJointLimits>(
        "get_joint_limits",
        std::bind(&BatchFKServer::handleGetLimits, this,
                  std::placeholders::_1, std::placeholders::_2));

    info_srv_ = this->create_service<autocert_interfaces::srv::GetRobotInfo>(
        "get_robot_info",
        std::bind(&BatchFKServer::handleGetInfo, this,
                  std::placeholders::_1, std::placeholders::_2));

    collision_srv_ = this->create_service<autocert_interfaces::srv::CheckCollision>(
        "check_collision",
        std::bind(&BatchFKServer::handleCheckCollision, this,
                  std::placeholders::_1, std::placeholders::_2));

    sample_srv_ = this->create_service<autocert_interfaces::srv::SampleWorkspace>(
        "sample_workspace",
        std::bind(&BatchFKServer::handleSampleWorkspace, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "All services created and ready");
  }

  void handleBatchFK(
      const std::shared_ptr<autocert_interfaces::srv::BatchFK::Request> request,
      std::shared_ptr<autocert_interfaces::srv::BatchFK::Response> response)
  {
    if (!sampler_)
    {
      response->error_code = 4;
      response->error_message = "Sampler not initialized";
      return;
    }

    response->error_code = 0;

    if (request->num_samples > response_chunk_size_)
    {
      response->error_code = 1;
      response->error_message = "Request too large";
      return;
    }

    if (request->num_samples > max_samples_per_request_)
    {
      response->error_code = 2;
      response->error_message = "Request exceeds absolute maximum";
      return;
    }

    size_t expected_size =
        static_cast<size_t>(request->num_samples) * request->num_joints;

    if (request->joint_samples.size() != expected_size)
    {
      response->error_code = 3;
      response->error_message = "Invalid input size";
      return;
    }

    const double *data_ptr = request->joint_samples.data();
    const size_t num_joints = request->num_joints;
    const size_t num_samples = request->num_samples;

    std::vector<double> joint_values(num_joints);

    response->poses.reserve(num_samples);
    response->valid_indices.reserve(num_samples);

    auto start_time = std::chrono::steady_clock::now();

    for (size_t i = 0; i < num_samples; ++i)
    {
      for (size_t j = 0; j < num_joints; ++j)
        joint_values[j] = data_ptr[i * num_joints + j];

      auto pose = sampler_->computeFK(joint_values);
      response->poses.push_back(pose);

      bool is_valid = true;

      if (request->filter_collisions)
        is_valid = sampler_->isStateValid(joint_values);

      if (is_valid)
        response->valid_indices.push_back(static_cast<int>(i));
    }

    auto end_time = std::chrono::steady_clock::now();

    double computation_time =
        std::chrono::duration<double, std::milli>(end_time - start_time).count();

    response->end_effector_link = sampler_->getEndEffectorLink();
    response->computation_time_ms.push_back(computation_time);

    total_requests_++;
    total_samples_processed_ += num_samples;
  }

  void handleGetLimits(
      const std::shared_ptr<autocert_interfaces::srv::GetJointLimits::Request>,
      std::shared_ptr<autocert_interfaces::srv::GetJointLimits::Response> response)
  {
    auto limits = sampler_->getJointLimits();
    auto info = sampler_->getRobotInfo();

    response->joint_names = info.joint_names;
    response->min_limits = limits.first;
    response->max_limits = limits.second;
    response->planning_group = info.planning_group;
    response->end_effector_link = info.end_effector_link;
    response->num_joints = static_cast<int32_t>(info.num_joints);
    response->velocity_limits = info.joint_velocity_limits;
  }

  void handleGetInfo(
      const std::shared_ptr<autocert_interfaces::srv::GetRobotInfo::Request>,
      std::shared_ptr<autocert_interfaces::srv::GetRobotInfo::Response> response)
  {
    auto info = sampler_->getRobotInfo();

    response->robot_name = info.robot_name;
    response->planning_group = info.planning_group;
    response->end_effector_link = info.end_effector_link;
    response->joint_names = info.joint_names;
    response->link_names = info.link_names;
    response->group_names = info.group_names;
    response->joint_min_limits = info.joint_min_limits;
    response->joint_max_limits = info.joint_max_limits;
    response->joint_velocity_limits = info.joint_velocity_limits;
    response->num_joints = static_cast<int32_t>(info.num_joints);
    response->num_links = static_cast<int32_t>(info.num_links);
  }

  void handleCheckCollision(
      const std::shared_ptr<autocert_interfaces::srv::CheckCollision::Request> request,
      std::shared_ptr<autocert_interfaces::srv::CheckCollision::Response> response)
  {
    std::vector<double> joint_positions(
        request->joint_positions.begin(),
        request->joint_positions.end());

    bool is_valid = sampler_->isStateValid(joint_positions);

    response->in_collision = !is_valid;
    response->environment_collision = !is_valid;
  }

  void handleSampleWorkspace(
      const std::shared_ptr<autocert_interfaces::srv::SampleWorkspace::Request>,
      std::shared_ptr<autocert_interfaces::srv::SampleWorkspace::Response> response)
  {
    response->reachability_index = 0.0;
    response->workspace_volume = 0.0;

    RCLCPP_WARN_ONCE(this->get_logger(),
                     "SampleWorkspace placeholder - use Python client");
  }

  rclcpp::TimerBase::SharedPtr init_timer_;

  std::unique_ptr<autocert_core::WorkspaceSampler> sampler_;

  rclcpp::Service<autocert_interfaces::srv::BatchFK>::SharedPtr fk_srv_;
  rclcpp::Service<autocert_interfaces::srv::GetJointLimits>::SharedPtr limits_srv_;
  rclcpp::Service<autocert_interfaces::srv::GetRobotInfo>::SharedPtr info_srv_;
  rclcpp::Service<autocert_interfaces::srv::CheckCollision>::SharedPtr collision_srv_;
  rclcpp::Service<autocert_interfaces::srv::SampleWorkspace>::SharedPtr sample_srv_;

  std::atomic<size_t> total_requests_{0};
  std::atomic<size_t> total_samples_processed_{0};

  int max_samples_per_request_{100000};
  int response_chunk_size_{20000};
  bool enable_performance_logging_{false};
};

} 

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<autocert::BatchFKServer>();

  int service_threads = 4;

  try
  {
    service_threads = node->get_parameter("service_threads").as_int();
  }
  catch (...)
  {
  }

  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(),
      service_threads);

  executor.add_node(node);

  RCLCPP_INFO(node->get_logger(),
              "Batch FK server running with multi-threaded executor");

  executor.spin();

  RCLCPP_INFO(node->get_logger(),
              "Shutting down. Processed %zu total requests, %zu samples",
              node->getTotalRequests(),
              node->getTotalSamplesProcessed());

  rclcpp::shutdown();
  return 0;
}