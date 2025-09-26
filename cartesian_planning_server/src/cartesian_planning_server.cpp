// Copyright 2023 Alex Arbogast
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <urdf/model.h>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <cartesian_planner/cartesian_planner.h>
#include <cartesian_planning_msgs/srv/plan_cartesian_trajectory.hpp>

static const std::string NAME = "cartesian_planning_server";

namespace cartesian_planning_server
{

using PlanCartesianTrajectory =
    cartesian_planning_msgs::srv::PlanCartesianTrajectory;

using JointState = sensor_msgs::msg::JointState;

std::vector<double> joint_state_to_vector(const JointState& js,
                                          const std::vector<std::string>& names)
{
  std::unordered_map<std::string, size_t> name_to_index;
  for (size_t i = 0; i < js.name.size(); ++i)
  {
    name_to_index[js.name[i]] = i;
  }

  std::vector<double> result(names.size(), 0.0);
  for (size_t i = 0; i < names.size(); ++i)
  {
    size_t idx = name_to_index[names[i]];
    result[i] = js.position[idx];
  }
  return result;
}

class CartesianPlanningServer : public rclcpp::Node
{
public:
  CartesianPlanningServer() : Node(NAME)
  {
    this->declare_parameter("position_threshold", 0.0005);
    this->declare_parameter("rotation_threshold", 0.01);
    this->declare_parameter("max_sampling_step", 0.05);
    this->declare_parameter("max_step_iterations", 200);
    this->declare_parameter("damping", 0.0);
    this->declare_parameter("robot_base_link", "");
    this->declare_parameter("end_effector_link", "");
    this->declare_parameter("joints", std::vector<std::string>{});
    this->declare_parameter("robot_description", "");

    if (!init())
    {
      RCLCPP_FATAL(this->get_logger(),
                   "Failed to initialize CartesianPlanningServer node");
      rclcpp::shutdown();
    }
  }

private:
  bool init()
  {
    position_threshold_ = this->get_parameter("position_threshold").as_double();
    rotation_threshold_ = this->get_parameter("rotation_threshold").as_double();
    max_sampling_step_ = this->get_parameter("max_sampling_step").as_double();
    max_step_iterations_ = this->get_parameter("max_step_iterations").as_int();
    damping_ = this->get_parameter("damping").as_double();

    std::string robot_description =
        this->get_parameter("robot_description").as_string();
    std::string robot_base_link =
        this->get_parameter("robot_base_link").as_string();
    std::string end_effector_link =
        this->get_parameter("end_effector_link").as_string();
    joint_names_ = this->get_parameter("joints").as_string_array();

    if (robot_description.empty())
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Parameter 'robot_description' is required but not set");
      return false;
    }

    // Build kinematic chain
    KDL::Tree robot_tree;
    KDL::Chain robot_chain;
    urdf::Model robot_model;
    if (!robot_model.initString(robot_description))
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to initialize urdf model from 'robot_description'");
      return false;
    }
    if (!kdl_parser::treeFromUrdfModel(robot_model, robot_tree))
    {
      RCLCPP_FATAL(this->get_logger(),
                   "Failed to parse KDL tree from urdf model");
      return false;
    }
    if (!robot_tree.getChain(robot_base_link, end_effector_link, robot_chain))
    {
      RCLCPP_FATAL(this->get_logger(),
                   "Failed to build kinematic chain from '%s' to '%s'. "
                   "Make sure these links exist in the URDF.",
                   robot_base_link.c_str(), end_effector_link.c_str());
      return false;
    }

    // Initialize planner
    planner_ =
        std::make_unique<cartesian_planner::CartesianPlanner>(robot_chain);

    // Setup service
    plan_trajectory_service_ = this->create_service<PlanCartesianTrajectory>(
        NAME + "/plan_cartesian_trajectory",
        std::bind(&CartesianPlanningServer::planCartesianTrajectoryService,
                  this, std::placeholders::_1, std::placeholders::_2));
    return true;
  }

  void planCartesianTrajectoryService(
      const std::shared_ptr<PlanCartesianTrajectory::Request> req,
      std::shared_ptr<PlanCartesianTrajectory::Response> res)
  {
    cartesian_planner::CartesianPlanningRequest request;
    request.position_threshold = position_threshold_;
    request.max_sampling_step = max_sampling_step_;
    request.max_step_iterations = max_step_iterations_;
    request.damping = damping_;
    request.max_linear_velocity = req->max_linear_velocity;
    request.max_angular_velocity = req->max_angular_velocity;
    request.scaling = static_cast<cartesian_planner::Order>(req->scaling);

    std::vector<double> q_start =
        joint_state_to_vector(req->start_state, joint_names_);

    Eigen::Map<const Eigen::VectorXd> start_state(q_start.data(),
                                                  q_start.size());
    request.start_state = start_state;

    // Path
    Eigen::Isometry3d pose;
    for (const auto& point : req->path)
    {
      tf2::fromMsg(point, pose);
      request.path.push_back(pose);
    }

    // Plan
    cartesian_planner::CartesianPlanningResponse response;
    planner_->planCartesianTrajectory(request, response);

    if (!response.error_code)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to plan Cartesian trajectory. "
                   "Planner returned with error code: %d",
                   response.error_code.val);
    }

    res->error_code = response.error_code;
    res->trajectory.points = response.joint_trajectory;
    res->trajectory.header.stamp = this->get_clock()->now();
    res->trajectory.joint_names = joint_names_;
  }

  rclcpp::Service<PlanCartesianTrajectory>::SharedPtr plan_trajectory_service_;

  std::vector<std::string> joint_names_;
  std::unique_ptr<cartesian_planner::CartesianPlanner> planner_;

  /* Planner request parameters */
  double position_threshold_;
  double rotation_threshold_;
  double max_sampling_step_;  // sec
  int max_step_iterations_;
  double damping_;
};

}  // namespace cartesian_planning_server

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node =
      std::make_shared<cartesian_planning_server::CartesianPlanningServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
