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

#include <cartesian_planner/utility.h>

namespace cartesian_planner
{

MatrixND leftPinv(const MatrixND& matrix)
{
  return (matrix.transpose() * matrix).inverse() * matrix.transpose();
}

MatrixND rightPinv(const MatrixND& matrix)
{
  return matrix.transpose() * (matrix * matrix.transpose()).inverse();
}

MatrixND dampedPinv(const MatrixND& matrix, double alpha)
{
  MatrixND identity = MatrixND::Identity(matrix.rows(), matrix.rows());
  return matrix.transpose() *
         (matrix * matrix.transpose() + alpha * alpha * identity).inverse();
}

}  // namespace cartesian_planner
