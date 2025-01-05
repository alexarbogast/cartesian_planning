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

#pragma once

#include <Eigen/Dense>

namespace cartesian_planner
{
typedef Eigen::Matrix<double, 6, 1> Vector6D, Twist;
typedef Eigen::Matrix<double, 3, 1> Vector3D;
typedef Eigen::VectorXd VectorND;
typedef Eigen::Matrix<double, 6, 6> Matrix6D;
typedef Eigen::MatrixXd MatrixND;
typedef Eigen::Isometry3d Pose;
typedef Eigen::Quaterniond Quaternion;

/**
 * @brief Find the left pseudoinverse of a matrix
 *
 * Returns the left Moore-Penrose pseudoinverse of a "tall" (more rows than
 * columns) matrix with linearly independent columns.
 *
 * @param matrix the matrix on which to perform the pseudoinverse
 * @returns the left pseudoinverse of "matrix"
 */
MatrixND leftPinv(const MatrixND& matrix);

/**
 * @brief Find the right pseudoinverse of a matrix
 *
 * Returns the right Moore-Penrose pseudoinverse of a "wide" (more columns than
 * rows) matrix with linearly independent rows.
 *
 * @param matrix the matrix on which to perform the pseudoinverse
 * @returns the right pseudoinverse of "matrix"
 */
MatrixND rightPinv(const MatrixND& matrix);

/**
 * @brief Find the damped pseudoinverse of a matrix
 *
 * @param matrix the matrix on which to perform the pseudoinverse
 * @param alpha the damping factor between 0 and 1
 * @returns the damped pseudoinverse of "matrix"
 */
MatrixND dampedPinv(const MatrixND& matrix, double alpha);

}  // namespace cartesian_planner
