#include <cartesian_planner/cartesian_planner.h>
#include <cartesian_planner/utility.h>

#include <eigen_conversions/eigen_kdl.h>
#include <kdl/jntarrayvel.hpp>

namespace cartesian_planner
{
CartesianPlanner::CartesianPlanner(const KDL::Chain& chain) : chain_(chain)
{
  n_joints_ = chain_.getNrOfJoints();
  fk_pos_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
  jacobian_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
}

bool CartesianPlanner::planCartesianTrajectory(
    CartesianPlanningRequest& request, CartesianPlanningResponse& response)
{
  if (request.start_state.rows() != n_joints_)
  {
    response.success = false;
    response.error_code = ErrorCode::INVALID_ROBOT_STATE;
    return false;
  }

  KDL::JntArrayVel state(n_joints_);
  state.q.data = request.start_state;
  state.qdot.data = VectorND::Zero(n_joints_);

  // Add starting pose to path if we start farther than threshold
  Pose start_pose = getEndEffectorPose(state.q);
  double initial_trans_error =
      (start_pose.translation() - request.path[0].translation()).norm();
  if (initial_trans_error > request.error_threshold)
  {
    request.path.insert(request.path.begin(), start_pose);
  }

  if (request.path.size() < 2)
  {
    response.success = true;
    return true;
  }

  // Initialize a joint trajectory point
  trajectory_msgs::JointTrajectoryPoint joint_state;
  joint_state.positions.resize(n_joints_);
  joint_state.velocities.resize(n_joints_);

  Pose current_pose;
  KDL::Jacobian jacobian(n_joints_);
  double time_from_start = 0.0;

  // Plan the joint trajectory for each segment
  for (auto it = request.path.begin(); it != --request.path.end(); ++it)
  {
    const Pose& start_pose = *it;
    const Pose& end_pose = *std::next(it);

    Eigen::Quaterniond start_quaternion(start_pose.linear());
    Eigen::Quaterniond end_quaternion(end_pose.linear());

    Vector3D direction = end_pose.translation() - start_pose.translation();
    double distance = direction.norm();
    unsigned int n_steps = ceil(distance / request.max_eef_step);
    double segment_time_step =
        (distance / request.max_velocity_threshold) / n_steps;

    for (size_t i = 0; i < n_steps; ++i)
    {
      double s = i / (double)n_steps;

      Eigen::Isometry3d target(start_quaternion.slerp(s, end_quaternion));
      target.translation() =
          s * end_pose.translation() + (1 - s) * start_pose.translation();

      // iterate to find joint position
      size_t j = 0;
      for (; j < request.max_step_iterations; ++j)
      {
        Pose current_pose = getEndEffectorPose(state.q);
        Vector6D error = poseError(current_pose, target);

        if (error.head<3>().norm() < request.error_threshold)
        {
          break;
        }
        jacobian = getJacobian(state.q);

        KDL::JntArray dq(n_joints_);
        dq.data = dampedPinv(jacobian.data, 0.1) * error;
        state.q.data += dq.data;
      }
      if (j == request.max_step_iterations)
      {
        response.success = false;
        response.error_code = ErrorCode::MAX_ITERATIONS;
        return false;
      }

      // find joint velocity
      Vector6D cart_cmd;
      cart_cmd << request.max_velocity_threshold * (direction / distance),
          Vector3D::Zero();
      jacobian = getJacobian(state.q);
      state.qdot.data = dampedPinv(jacobian.data, 0.1) * cart_cmd;

      // add joint state to response
      for (int k = 0; k < n_joints_; ++k)
      {
        joint_state.positions[k] = state.q(k);
        joint_state.velocities[k] = state.qdot(k);
      }
      joint_state.time_from_start = ros::Duration(time_from_start);
      response.joint_trajectory.push_back(joint_state);
      time_from_start += segment_time_step;
    }
  }

  response.success = true;
  response.error_code = ErrorCode::SUCCESS;
  return true;
}

Pose CartesianPlanner::getEndEffectorPose(const KDL::JntArray& q) const
{
  KDL::Frame pose_kdl;
  fk_pos_solver_->JntToCart(q, pose_kdl);

  Pose current_pose;
  tf::transformKDLToEigen(pose_kdl, current_pose);
  return current_pose;
}

KDL::Jacobian CartesianPlanner::getJacobian(const KDL::JntArray& q) const
{
  KDL::Jacobian jacobian(n_joints_);
  jacobian_solver_->JntToJac(q, jacobian);
  return jacobian;
}

Vector6D CartesianPlanner::poseError(const Pose& p1, const Pose& p2) const
{
  Vector3D trans_error = p2.translation() - p1.translation();

  Eigen::Quaterniond orient1(p1.rotation());
  Eigen::Quaterniond orient2(p2.rotation());
  Vector3D orient_error = (orient2 * orient1.inverse()).vec();

  Vector6D error;
  error << trans_error, orient_error;
  return error;
}

}  // namespace cartesian_planner
