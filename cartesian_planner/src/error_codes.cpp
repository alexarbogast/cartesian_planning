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
