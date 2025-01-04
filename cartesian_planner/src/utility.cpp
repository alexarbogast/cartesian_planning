#include <cartesian_planner/utility.h>

namespace cartesian_planner
{

MatrixND leftPinv(const MatrixND& matrix)
{
  return (matrix.transpose() * matrix).inverse() * matrix.transpose();
}

MatrixND rightPinv(const MatrixND& matrix)
{
  return matrix.transpose() * (matrix * matrix.transpose()).inverse();
}

MatrixND dampedPinv(const MatrixND& matrix, double alpha)
{
  MatrixND identity = MatrixND::Identity(matrix.rows(), matrix.rows());
  return matrix.transpose() *
         (matrix * matrix.transpose() + alpha * alpha * identity).inverse();
}

}  // namespace cartesian_planner
