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

#include <cartesian_planner/error_codes.h>

namespace cartesian_planner
{

const char*
ErrorCode::toString(const cartesian_planning_msgs::ErrorCodes& error_code)
{
  switch (error_code.val)
  {
    case 0:
      return "NOT INITIALIZE";
    case cartesian_planning_msgs::ErrorCodes::SUCCESS:
      return "SUCCESS";
    case cartesian_planning_msgs::ErrorCodes::INVALID_ROBOT_STATE:
      return "INVALID_ROBOT_STATE";
    case cartesian_planning_msgs::ErrorCodes::MAX_ITERATIONS:
      return "MAX_ITERATIONS";
    default:
      return "UNKNOWN";
  }
}

std::ostream& operator<<(std::ostream& out, const ErrorCode& e)
{
  return out << ErrorCode::toString(e);
}

}  // namespace cartesian_planner
