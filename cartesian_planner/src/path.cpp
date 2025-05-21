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
/*=========================== Linear Path ==================================*/
LinearPath::LinearPath(const Pose& start, const Pose& end)
  : start_(start), end_(end)
{
}

Pose LinearPath::evaluate(double s) const
{
  s = std::clamp(s, 0.0, 1.0);

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
  s = std::clamp(s, 0.0, 1.0);

  Twist twist;
  twist.head<3>() = (end_.translation() - start_.translation()) * s_dot;

  Quaternion orient1(start_.linear());
  Quaternion orient2(end_.linear());
  Quaternion q = orient1.slerp(s, orient2);

  AngleAxis aa(orient1.inverse() * orient2);
  twist.tail<3>() = q * (aa.axis() * aa.angle() * s_dot);
  return twist;
}

double LinearPath::length() const
{
  return (start_.translation() - end_.translation()).norm();
}

/*=========================== Cubic B-spline ===============================*/
const CubicBSplinePath::KnotVectorType CubicBSplinePath::UNIFORM_KNOTS =
    (CubicBSplinePath::KnotVectorType() << -3, -2, -1, 0, 1, 2, 3, 4)
        .finished();

CubicBSplinePath::CubicBSplinePath(const ControlPointVectorType& control_points,
                                   const Quaternion& start_orient,
                                   const Quaternion& end_orient)
  : spline_(CubicBSplinePath::UNIFORM_KNOTS, control_points)
  , start_orient_(start_orient)
  , end_orient_(end_orient)
{
}

Pose CubicBSplinePath::evaluate(double s) const
{
  s = std::clamp(s, 0.0, 1.0);

  Pose result;
  result.translation() = spline_(s);
  result.linear() = start_orient_.slerp(s, end_orient_).matrix();
  return result;
}

Twist CubicBSplinePath::derivative(double s, double s_dot) const
{
  // TODO: find derivative
  Twist result = Twist::Zero();
  result << spline_.derivatives<1>(s).col(1) * s_dot, Vector3D::Zero();
  return result;
}

double CubicBSplinePath::length() const
{
  // TODO: calculate path length!
  return 0.0;
}

}  // namespace cartesian_planner
