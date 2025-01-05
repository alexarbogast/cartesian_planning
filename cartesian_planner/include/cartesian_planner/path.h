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

#include <cartesian_planner/utility.h>

namespace cartesian_planner
{

/**
 * An interface for a generic Cartesian path
 *
 * A path maps a sampling parameters "s" to pose in SE(3).
 * A path sampled at a time-scaling s(t) gives a trajectory.
 *
 * p(s): [0, 1] -> SE(3),
 * p(s(t)): [0, T] -> SE(3)
 */
class CartesianPath
{
public:
  virtual ~CartesianPath() = default;

  virtual Pose evaluate(double s) const = 0;
  virtual Twist derivative(double s, double s_dot) const = 0;
  virtual double length() const = 0;
};

/**
 * A linear path in SE(3) beginning at the pose "start" and ending at the pose
 * "end"
 */
class LinearPath : public CartesianPath
{
public:
  LinearPath(const Pose& start, const Pose& end);

  virtual Pose evaluate(double s) const override;
  virtual Twist derivative(double s, double s_dot) const override;
  virtual double length() const override;

private:
  Pose start_;
  Pose end_;
};

}  // namespace cartesian_planner
