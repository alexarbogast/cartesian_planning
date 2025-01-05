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

#include <cartesian_planner/time_scaling.h>
#include <cmath>
#include <numeric>

namespace cartesian_planner
{

Polynomial::Polynomial(const std::vector<double>& coeffs) : coeffs_(coeffs) {}

double Polynomial::evaluate(double x) const
{
  return std::accumulate(
      coeffs_.rbegin(), coeffs_.rend(), 0.0,
      [x](double acc, double coef) { return acc * x + coef; });
}

Polynomial Polynomial::derivate() const
{
  if (degree() < 1)
    return Polynomial({ 0.0 });

  std::vector<double> new_coeffs(degree());
  for (std::size_t i = 1; i < coeffs_.size(); ++i)
  {
    new_coeffs[i - 1] = coeffs_[i] * i;
  }
  return Polynomial(new_coeffs);
}

Polynomial first_order_scaling(double tf)
{
  return Polynomial({ 0.0, 1 / tf });
}

Polynomial third_order_scaling(double tf)
{
  return Polynomial({ 0.0, 0.0, 3 / pow(tf, 2), -2 / pow(tf, 3) });
}

Polynomial fifth_order_scaling(double tf)
{
  return Polynomial(
      { 0.0, 0.0, 0.0, 10 / pow(tf, 3), -15 / pow(tf, 4), 6 / pow(tf, 5) });
}

ScalingFunc getScaling(Order o)
{
  switch (o)
  {
    case Order::FIRST:
      return [](double tf) { return first_order_scaling(tf); };
    case Order::THIRD:
      return [](double tf) { return third_order_scaling(tf); };
    case Order::FIFTH:
      return [](double tf) { return fifth_order_scaling(tf); };
    default:
      return nullptr;
  }
}

}  // namespace cartesian_planner
