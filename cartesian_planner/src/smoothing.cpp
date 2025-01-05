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

#include <cartesian_planner/smoothing.h>
#include <cartesian_planner/path.h>

namespace cartesian_planner
{

double angle_between(const Eigen::VectorXd& v1, const Eigen::VectorXd& v2)
{
  double dotp = v1.dot(v2) / (v1.norm() * v2.norm());
  dotp = std::max(-1.0, std::min(dotp, 1.0));
  return acos(dotp);
}

double entry_length(const Vector3D& v0, const Vector3D& v1, const Vector3D& v2,
                    double tol)
{
  return 3.0 * tol / cos(angle_between(v0 - v1, v2 - v1) / 2.0);
}

std::vector<CubicBSplinePath::Ptr>
continuous_cubic_splines(const std::vector<Vector3D>& control_points)
{
  std::size_t n_splines = control_points.size() - 3;
  std::vector<CubicBSplinePath::Ptr> result;
  result.reserve(n_splines);

  for (std::size_t i = 0; i < n_splines; ++i)
  {
    Eigen::Matrix<double, 3, 4> cps;
    cps << control_points[i], control_points[i + 1], control_points[i + 2],
        control_points[i + 3];
    result.emplace_back(std::make_shared<CubicBSplinePath>(cps));
  }
  return result;
}

std::vector<CubicBSplinePath::Ptr>
smooth_cubic_bspline(const std::vector<Pose>& points, double tol)
{
  /*
   * n_cp_max = 3 * path.size() - 2
   * n_splines_max = 3 * path.size() - 5
   */
  std::vector<Vector3D> cp;
  cp.reserve(3 * points.size() - 2);

  // blend first segment
  const Vector3D& p0 = points[0].translation();
  const Vector3D& p1 = points[1].translation();
  const Vector3D& p2 = points[2].translation();

  Vector3D v0, v1, v2, v3;
  double l_pk = entry_length(p0, p1, p2, tol);

  if ((p1 - p0).norm() > l_pk)
  {
    v1 = p0;
    v3 = p1;
    v2 = p1 + l_pk * (p0 - p1).normalized();
    v0 = 2 * v1 - v2;
    cp.insert(cp.end(), { v0, v1, v2, v3 });
  }
  else
  {
    v1 = p0;
    v2 = p1;
    v0 = 2 * v1 - v2;
    cp.insert(cp.end(), { v0, v1, v2 });
  }

  for (std::size_t i = 1; i < points.size() - 2; ++i)
  {
    const Vector3D& pk_m1 = points[i - 1].translation();
    const Vector3D& pk = points[i].translation();
    const Vector3D& pk_p1 = points[i + 1].translation();
    const Vector3D& pk_p2 = points[i + 2].translation();

    double dist = (pk_p1 - pk).norm();
    double l_pk_p1 = entry_length(pk, pk_p1, pk_p2, tol);
    Vector3D uk_kp1 = (pk_p1 - pk).normalized();

    if (dist > (l_pk + l_pk_p1))
    {
      cp.insert(cp.end(),
                { pk + uk_kp1 * l_pk, pk_p1 - uk_kp1 * l_pk_p1, pk_p1 });
    }
    else if (dist > std::min(l_pk, l_pk_p1))
    {
      double le_pk = l_pk / (l_pk + l_pk_p1) * dist;
      cp.insert(cp.end(), { pk + uk_kp1 * le_pk, pk_p1 });
    }
    else
    {
      cp.push_back(pk_p1);
    }

    l_pk = l_pk_p1;
  }

  // blend last segment
  double last_seg_len =
      ((points.end() - 1)->translation() - (points.end() - 2)->translation())
          .norm();

  if (last_seg_len > l_pk)
  {
    v2 = (points.end() - 2)->translation() +
         l_pk * ((points.end() - 1)->translation() -
                 (points.end() - 2)->translation())
                    .normalized();
    cp.push_back(v2);
  }
  else
  {
    v2 = (points.end() - 2)->translation();
  }

  v1 = points.back().translation();
  v0 = 2 * v1 - v2;
  cp.insert(cp.end(), { v1, v0 });
  return continuous_cubic_splines(cp);
}

}  // namespace cartesian_planner