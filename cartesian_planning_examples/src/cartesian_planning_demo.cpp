#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>

#include <cartesian_planning_msgs/ErrorCodes.h>
#include <cartesian_planning_msgs/PlanCartesianTrajectory.h>

const std::string NAME = "cartesian_planning_demo";
const std::vector<double> HOME = { 0.0, -1.125, 2.275, -1.15, 1.571, 0.0 };

class CartesianPlanningDemo
{
public:
  CartesianPlanningDemo(ros::NodeHandle& nh) : nh_(nh)
  {
    action_client_ = std::make_unique<TrajClient>(
        "position_trajectory_controller/follow_joint_trajectory", true);

    planning_client_ =
        nh_.serviceClient<cartesian_planning_msgs::PlanCartesianTrajectory>(
            "cartesian_planning_server/plan_cartesian_trajectory");

    ROS_INFO("Waiting for plan_cartesian_trajectory server...");
    planning_client_.waitForExistence();

    ROS_INFO("Waiting for follow_trajectory_action server...");
    action_client_->waitForServer();

    ROS_INFO("Ready to plan!");
  }

  void move_home()
  {
    control_msgs::FollowJointTrajectoryGoal goal;
    auto start_state = ros::topic::waitForMessage<sensor_msgs::JointState>(
        "/joint_states", nh_);
    goal.trajectory.joint_names = start_state->name;

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = HOME;
    point.velocities = std::vector<double>(6, 0.0);
    point.accelerations = std::vector<double>(6, 0.0);
    point.time_from_start = ros::Duration(2);
    goal.trajectory.points.push_back(point);
    action_client_->sendGoal(goal);
    action_client_->waitForResult();
  }

  void run()
  {
    move_home();
    cartesian_planning_msgs::PlanCartesianTrajectory srv;

    /* Set start state */
    auto start_state = ros::topic::waitForMessage<sensor_msgs::JointState>(
        "/joint_states", nh_);
    srv.request.start_state = *start_state;

    std::vector<std::array<double, 3>> path = {
      { 0.7, 0.3, 0.1 },  { 0.7, -0.3, 0.1 }, { 0.7, -0.3, 0.6 },
      { 0.3, -0.3, 0.6 }, { 0.3, -0.3, 0.1 }, { 0.3, 0.3, 0.1 },
      { 0.3, 0.3, 0.6 },  { 0.7, 0.3, 0.6 },  { 0.7, 0.3, 0.1 }
    };

    /* Create path */
    geometry_msgs::Pose pose;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.707107;
    pose.orientation.z = 0.707107;
    pose.orientation.w = 0.0;

    for (auto& point : path)
    {
      pose.position.x = point[0];
      pose.position.y = point[1];
      pose.position.z = point[2];
      srv.request.path.push_back(pose);
    }

    /* Set velocity */
    srv.request.max_linear_velocity = 0.200;
    srv.request.max_angular_velocity = 1.0;
    srv.request.scaling =
        cartesian_planning_msgs::PlanCartesianTrajectoryRequest::SCALING_FIRST;
    planning_client_.call(srv);

    if (srv.response.error_code.val !=
        cartesian_planning_msgs::ErrorCodes::SUCCESS)
    {
      ROS_ERROR_STREAM("Failed to plan Cartesian trajectory. "
                       << "Planning service returned with ERROR_CODE: "
                       << srv.response.error_code.val);
      return;
    }

    /* Send trajectory to action server */
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = srv.response.trajectory;
    action_client_->sendGoal(goal);
  }

private:
  typedef actionlib::SimpleActionClient<
      control_msgs::FollowJointTrajectoryAction>
      TrajClient;

  ros::NodeHandle nh_;
  ros::ServiceClient planning_client_;
  std::unique_ptr<TrajClient> action_client_;

  std::vector<std::string> joint_names_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, NAME);
  ros::NodeHandle nh;

  CartesianPlanningDemo demo(nh);
  demo.run();
  return 0;
}
