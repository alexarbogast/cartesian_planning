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

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <eigen_conversions/eigen_msg.h>

#include <urdf/model.h>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <cartesian_planner/cartesian_planner.h>
#include <cartesian_planning_msgs/PlanCartesianTrajectory.h>

static const std::string NAME = "cartesian_planning_server";

namespace cartesian_planning_server
{

#define LOAD_ROS_PARAM(nh, param_name, variable)                               \
  if (!nh.getParam(param_name, variable))                                      \
  {                                                                            \
    ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() << "/"             \
                                       << param_name                           \
                                       << " from the parameter server.");      \
    return false;                                                              \
  }

class CartesianPlanningServer
{
public:
  CartesianPlanningServer(ros::NodeHandle& nh) : nh_(nh)
  {
    nh_.getParam("error_threshold", error_threshold_);
    nh_.getParam("max_eef_step", max_eef_step_);
    nh_.getParam("max_step_iterations", max_step_iterations_);
  }

  bool init()
  {
    /* Read robot paramters from parameter server */
    std::string robot_description;
    std::string robot_base_link, end_effector_link;
    if (!ros::param::search("robot_description", robot_description))
    {
      ROS_ERROR(
          "Searched enclosing namespaces for robot_description but "
          "nothing found");
      return false;
    }
    LOAD_ROS_PARAM(nh_, robot_description, robot_description);
    LOAD_ROS_PARAM(nh_, "robot_base_link", robot_base_link);
    LOAD_ROS_PARAM(nh_, "end_effector_link", end_effector_link);
    LOAD_ROS_PARAM(nh_, "joints", joint_names_);

    /* Build kinematic chain */
    KDL::Tree robot_tree;
    KDL::Chain robot_chain;
    urdf::Model robot_model;
    if (!robot_model.initString(robot_description))
    {
      ROS_ERROR("Failed to initialize urdf model from 'robot_description'");
      return false;
    }
    if (!kdl_parser::treeFromUrdfModel(robot_model, robot_tree))
    {
      ROS_FATAL("Failed to parse KDL tree from  urdf model");
      return false;
    }
    if (!robot_tree.getChain(robot_base_link, end_effector_link, robot_chain))
    {
      ROS_FATAL_STREAM("Failed to build kinematic chain from '"
                       << robot_base_link << "' to '" << end_effector_link
                       << "'. Make sure these links exist in the URDF.");
      return false;
    }

    /* Initialize planner */
    planner_ =
        std::make_unique<cartesian_planner::CartesianPlanner>(robot_chain);

    /* Setup planning service */
    plan_trajectory_service_ = nh_.advertiseService(
        "plan_cartesian_trajectory",
        &CartesianPlanningServer::planCartesianTrajectoryService, this);
    return true;
  }

private:
  bool planCartesianTrajectoryService(
      cartesian_planning_msgs::PlanCartesianTrajectory::Request& req,
      cartesian_planning_msgs::PlanCartesianTrajectory::Response& res)
  {
    cartesian_planner::CartesianPlanningRequest request;
    request.error_threshold = error_threshold_;
    request.max_eef_step = max_eef_step_;
    request.max_step_iterations = max_step_iterations_;
    request.max_velocity_threshold = req.velocity;

    auto& q_start = req.start_state.position;
    Eigen::Map<Eigen::VectorXd> start_state(q_start.data(), q_start.size());
    request.start_state = start_state;

    Eigen::Isometry3d pose;
    for (auto& point : req.path)
    {
      tf::poseMsgToEigen(point, pose);
      request.path.push_back(pose);
    }

    cartesian_planner::CartesianPlanningResponse response;
    planner_->planCartesianTrajectory(request, response);

    if (!response.error_code)
    {
      ROS_ERROR_STREAM("Failed to plan Cartesian trajectory. "
                       << "Planner returned with error code: "
                       << response.error_code);
    }

    res.error_code = response.error_code;
    res.trajectory.points = response.joint_trajectory;
    res.trajectory.header.stamp = ros::Time(0.0);
    res.trajectory.joint_names = joint_names_;
    return true;
  }

  ros::NodeHandle nh_;
  ros::ServiceServer plan_trajectory_service_;

  std::vector<std::string> joint_names_;
  std::unique_ptr<cartesian_planner::CartesianPlanner> planner_;

  /* Planner request parameters */
  double error_threshold_ = 0.0005;
  double max_eef_step_ = 0.001;
  int max_step_iterations_ = 200;
};

}  // namespace cartesian_planning_server

int main(int argc, char** argv)
{
  ros::init(argc, argv, NAME);
  ros::NodeHandle nh(NAME);

  cartesian_planning_server::CartesianPlanningServer server(nh);
  if (!server.init())
  {
    ROS_ERROR_NAMED(NAME, "Failed to initialize server");
    return 1;
  }

  ros::spin();
  return 0;
}
