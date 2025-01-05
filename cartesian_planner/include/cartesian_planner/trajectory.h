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

#include <cartesian_planner/path.h>
#include <cartesian_planner/time_scaling.h>

#include <memory>

namespace cartesian_planner
{
class CartesianTrajectory
{
public:
  CartesianTrajectory(const std::shared_ptr<CartesianPath>& path, double tf,
                      Order order = Order::FIFTH);

  Pose evaluate(double t) const;
  Twist derivate(double t) const;
  inline double tf() const { return tf_; };

private:
  double tf_;
  std::shared_ptr<CartesianPath> path_;
  std::shared_ptr<Polynomial> scaling_;
  std::shared_ptr<Polynomial> scaling_deriv_;
};

}  // namespace cartesian_planner
