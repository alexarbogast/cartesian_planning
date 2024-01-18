#include <cartesian_planner/cartesian_planner.h>
#include <eigen_conversions/eigen_kdl.h>

namespace cartesian_planner
{
CartesianPlanner::CartesianPlanner(const KDL::Chain& chain) : chain_(chain)
{
  n_joints_ = chain_.getNrOfJoints();
  current_positions_.data = VectorND::Zero(n_joints_);
  current_velocities_.data = VectorND::Zero(n_joints_);
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

  // Add starting pose to path if we start farther than threshold
  current_positions_.data = request.start_state;
  Pose start_pose = getEndEffectorPose();
  double trans_error = translationError(start_pose, request.path[0]).norm();
  if (trans_error > request.error_threshold)
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

  Pose current_pose = getEndEffectorPose();
  KDL::Jacobian jacobian = getJacobian();
  double time_from_start = 0.0;
  static const Matrix6D identity = Matrix6D::Identity();

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
        current_pose = getEndEffectorPose();
        if (translationError(current_pose, target).norm() <
            request.error_threshold)
        {
          break;
        }

        jacobian = getJacobian();
        Vector6D error = poseError(current_pose, target);

        KDL::JntArray dq(n_joints_);
        dq.data =
            jacobian.data.transpose() *
            (jacobian.data * jacobian.data.transpose() + 0.1 * 0.1 * identity)
                .inverse() *
            error;

        current_positions_.data += dq.data;
      }
      if (j == request.max_step_iterations)
      {
        response.success = false;
        response.error_code = ErrorCode::MAX_ITERATIONS;
        return false;
      }

      // find joint velocity
      Vector6D vel_cmd;
      vel_cmd << request.max_velocity_threshold * (direction / distance),
          Vector3D::Zero();
      jacobian = getJacobian();
      current_velocities_.data =
          jacobian.data.transpose() *
          (jacobian.data * jacobian.data.transpose() + 0.1 * 0.1 * identity)
              .inverse() *
          vel_cmd;

      // add joint state to response
      for (int k = 0; k < n_joints_; ++k)
      {
        joint_state.positions[k] = current_positions_(k);
        joint_state.velocities[k] = current_velocities_(k);
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

Pose CartesianPlanner::getEndEffectorPose() const
{
  KDL::Frame pose_kdl;
  fk_pos_solver_->JntToCart(current_positions_, pose_kdl);

  Pose current_pose;
  tf::transformKDLToEigen(pose_kdl, current_pose);
  return current_pose;
}

KDL::Jacobian CartesianPlanner::getJacobian() const
{
  KDL::Jacobian jacobian(n_joints_);
  jacobian_solver_->JntToJac(current_positions_, jacobian);
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

inline Vector3D CartesianPlanner::translationError(const Pose& p1,
                                                   const Pose& p2) const
{
  return p2.translation() - p1.translation();
}

}  // namespace cartesian_planner
