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

#include <cartesian_planning_msgs/ErrorCodes.h>

namespace cartesian_planner
{

class ErrorCode : public cartesian_planning_msgs::ErrorCodes
{
public:
  static const char*
  toString(const cartesian_planning_msgs::ErrorCodes& error_code);

  ErrorCode(int code = 0);
  ErrorCode(const cartesian_planning_msgs::ErrorCodes& code);

  explicit inline operator bool() const
  {
    return val == cartesian_planning_msgs::ErrorCodes::SUCCESS;
  }

  explicit inline operator std::string() const { return toString(*this); }

  inline bool operator==(const int c) const { return val == c; }

  inline bool operator!=(const int c) const { return val != c; }
};

std::ostream& operator<<(std::ostream& out, const ErrorCode& e);

}  // namespace cartesian_planner
