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

#include <cartesian_planner/path.h>

namespace cartesian_planner
{

LinearPath::LinearPath(const Pose& start, const Pose& end)
  : start_(start), end_(end)
{
}

Pose LinearPath::evaluate(double s) const
{
  const Vector3D& trans1 = start_.translation();
  const Vector3D& trans2 = end_.translation();

  Quaternion orient1(start_.linear());
  Quaternion orient2(end_.linear());

  Pose result = Pose::Identity();
  result.translation() = trans1 + s * (trans2 - trans1);
  result.linear() = orient1.slerp(s, orient2).toRotationMatrix();
  return result;
}

Twist LinearPath::derivative(double s, double s_dot) const
{
  const Vector3D& trans1 = start_.translation();
  const Vector3D& trans2 = end_.translation();

  // TODO: Rotation derivative
  Twist twist;
  twist << (trans2 - trans1) * s_dot, Vector3D::Zero();
  return twist;
}

double LinearPath::length() const
{
  return (start_.translation() - end_.translation()).norm();
}

}  // namespace cartesian_planner
