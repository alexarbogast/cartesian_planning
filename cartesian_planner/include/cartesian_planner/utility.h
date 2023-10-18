#pragma once

#include <Eigen/Dense>

namespace cartesian_planner
{
  typedef Eigen::Matrix<double, 6, 1> Vector6D;
  typedef Eigen::Matrix<double, 3, 1> Vector3D;
  typedef Eigen::VectorXd VectorND;
  typedef Eigen::Matrix<double, 6, 6> Matrix6D;
  typedef Eigen::Isometry3d Pose;
} // namespace cartesian_planner
