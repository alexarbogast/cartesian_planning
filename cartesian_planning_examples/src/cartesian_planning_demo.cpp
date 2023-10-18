#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>

#include <cartesian_planning_server/PlanCartesianTrajectory.h>

static const std::string NAME = "cartesian_planning_demo";

class CartesianPlanningDemo
{
public:
  CartesianPlanningDemo(ros::NodeHandle& nh) : nh_(nh)
  {
    action_client_ = std::make_unique<TrajClient>(
        "position_trajectory_controller/follow_joint_trajectory", true);

    planning_client_ =
        nh_.serviceClient<cartesian_planning_server::PlanCartesianTrajectory>(
            "cartesian_planning_server/plan_cartesian_trajectory");

    ROS_INFO("Waiting for plan_cartesian_trajectory server...");
    planning_client_.waitForExistence();

    ROS_INFO("Waiting for follow_trajectory_action server...");
    action_client_->waitForServer();

    ROS_INFO("Ready to plan!");
  }

  void run()
  {
    cartesian_planning_server::PlanCartesianTrajectory srv;

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
    srv.request.velocity = 0.200;
    planning_client_.call(srv);
    if (!srv.response.success)
    {
      ROS_ERROR("Failed to plan cartesian trajectory");
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
