#pragma once

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarray.hpp>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <cartesian_planner/utility.h>
#include <cartesian_planner/error_codes.h>

namespace cartesian_planner
{
struct CartesianPlanningRequest
{
  VectorND start_state;
  std::vector<Pose> path;

  double error_threshold = 0.0005;
  double max_eef_step = 0.001;
  double max_velocity_threshold = 0.1;
  unsigned int max_step_iterations = 200;
};

struct CartesianPlanningResponse
{
  bool success = false;
  ErrorCode error_code;

  std::vector<trajectory_msgs::JointTrajectoryPoint> joint_trajectory;
};

class CartesianPlanner
{
public:
  CartesianPlanner(const KDL::Chain& chain);

  bool planCartesianTrajectory(CartesianPlanningRequest& request,
                               CartesianPlanningResponse& response);

private:
  Pose getEndEffectorPose(const KDL::JntArray& q) const;
  KDL::Jacobian getJacobian(const KDL::JntArray& q) const;

  Vector6D poseError(const Pose& p1, const Pose& p2) const;

  unsigned int n_joints_;
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
};

}  // namespace cartesian_planner
