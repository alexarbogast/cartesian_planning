#pragma once

#include <cartesian_planning_msgs/ErrorCodes.h>

namespace cartesian_planner
{

class ErrorCode : public cartesian_planning_msgs::ErrorCodes
{
public:
  static const char*
  toString(const cartesian_planning_msgs::ErrorCodes& error_code);

  ErrorCode(int code = 0)
  {
    val = code;
  }

  ErrorCode(const cartesian_planning_msgs::ErrorCodes& code)
  {
    val = code.val;
  }

  explicit operator bool() const
  {
    return val == cartesian_planning_msgs::ErrorCodes::SUCCESS;
  }

  explicit operator std::string() const
  {
    return toString(*this);
  }

  bool operator==(const int c) const
  {
    return val == c;
  }

  bool operator!=(const int c) const
  {
    return val != c;
  }
};

std::ostream& operator<<(std::ostream& out, const ErrorCode& e);

}  // namespace cartesian_planner
