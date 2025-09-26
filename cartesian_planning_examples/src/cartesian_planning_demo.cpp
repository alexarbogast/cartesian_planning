#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/types.hpp>
#include <sensor_msgs/msg/detail/joint_state__struct.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <cartesian_planning_msgs/srv/plan_cartesian_trajectory.hpp>
#include <cartesian_planning_msgs/msg/error_codes.hpp>

using namespace std::chrono_literals;

const std::string NAME = "cartesian_planning_demo";
const std::map<std::string, double> HOME = {
  { "joint1", 0.0 },   { "joint2", -1.125 }, { "joint3", 2.275 },
  { "joint4", -1.15 }, { "joint5", 1.571 },  { "joint6", 0.0 }
};

class CartesianPlanningDemo : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using PlanCartesianTrajectory =
      cartesian_planning_msgs::srv::PlanCartesianTrajectory;

  CartesianPlanningDemo() : Node(NAME)
  {
    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
        this, "joint_trajectory_controller/follow_joint_trajectory");

    planning_client_ = this->create_client<PlanCartesianTrajectory>(
        "cartesian_planning_server/plan_cartesian_trajectory");

    RCLCPP_INFO(this->get_logger(),
                "Waiting for plan_cartesian_trajectory server...");
    planning_client_->wait_for_service();

    RCLCPP_INFO(this->get_logger(),
                "Waiting for follow_trajectory_action server...");
    action_client_->wait_for_action_server();

    RCLCPP_INFO(this->get_logger(), "Ready to plan!");
  }

  void move_home()
  {
    sensor_msgs::msg::JointState start_state;
    bool received = rclcpp::wait_for_message(start_state, shared_from_this(),
                                             "/joint_states");
    if (!received)
    {
      RCLCPP_ERROR(this->get_logger(), "failed to get /joint_states");
      return;
    }

    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = start_state.name;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    for (const auto& joint : goal_msg.trajectory.joint_names)
    {
      point.positions.push_back(HOME.at(joint));
    }
    point.velocities = std::vector<double>(HOME.size(), 0.0);
    point.accelerations = std::vector<double>(HOME.size(), 0.0);
    point.time_from_start = rclcpp::Duration(2s);
    goal_msg.trajectory.points.push_back(point);

    auto send_goal_options =
        rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

    send_goal_options.result_callback = [this](const auto& result) {
      RCLCPP_INFO(this->get_logger(), "Goal finished with code %i",
                  static_cast<int>(result.code));
    };

    auto goal_future =
        action_client_->async_send_goal(goal_msg, send_goal_options);

    if (rclcpp::spin_until_future_complete(shared_from_this(), goal_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "FollowJointTrajectory: failed sending goal");
    }

    auto goal_handle_ = goal_future.get();
    if (!goal_handle_)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "FollowJointTrajectory client: failed sending goal");
    }
  }

  void run()
  {
    move_home();

    sensor_msgs::msg::JointState start_state;
    bool received = rclcpp::wait_for_message(start_state, shared_from_this(),
                                             "/joint_states");
    if (!received)
    {
      RCLCPP_ERROR(this->get_logger(), "failed to get /joint_states");
      return;
    }

    auto request = std::make_shared<PlanCartesianTrajectory::Request>();
    request->start_state = start_state;

    std::vector<std::array<double, 3>> path = {
      { 0.7, 0.3, 0.1 },  { 0.7, -0.3, 0.1 }, { 0.7, -0.3, 0.6 },
      { 0.3, -0.3, 0.6 }, { 0.3, -0.3, 0.1 }, { 0.3, 0.3, 0.1 },
      { 0.3, 0.3, 0.6 },  { 0.7, 0.3, 0.6 },  { 0.7, 0.3, 0.1 }
    };

    geometry_msgs::msg::Pose pose;
    pose.orientation.x = 0.5;
    pose.orientation.y = 0.5;
    pose.orientation.z = 0.5;
    pose.orientation.w = 0.5;

    for (auto& point : path)
    {
      pose.position.x = point[0];
      pose.position.y = point[1];
      pose.position.z = point[2];
      request->path.push_back(pose);
    }

    request->max_linear_velocity = 0.200;
    request->max_angular_velocity = 1.0;
    request->scaling = cartesian_planning_msgs::srv::PlanCartesianTrajectory::
        Request::SCALING_FIRST;

    // Plan Cartesian trajectory
    auto future = planning_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(shared_from_this(), future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "PlanCartesianTrajectory: goal was rejected by server");
      return;
    }

    auto response = future.get();
    if (response->error_code.val !=
        cartesian_planning_msgs::msg::ErrorCodes::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to plan Cartesian trajectory. Error code: %d",
                   response->error_code.val);
      return;
    }

    // Send joint trajectory to action server
    FollowJointTrajectory::Goal goal_msg;
    goal_msg.trajectory = response->trajectory;

    auto goal_future = action_client_->async_send_goal(goal_msg);

    if (rclcpp::spin_until_future_complete(shared_from_this(), goal_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "FollowJointTrajectory: failed sending goal");
    }

    auto goal_handle_ = goal_future.get();
    if (!goal_handle_)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "FollowJointTrajectory client: failed sending goal");
    }
  }

private:
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp::Client<PlanCartesianTrajectory>::SharedPtr planning_client_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CartesianPlanningDemo>();
  node->run();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
