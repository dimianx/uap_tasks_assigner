#ifndef UAP_TA_TASKS_ASSIGNER_HPP_
#define UAP_TA_TASKS_ASSIGNER_HPP_

#include <unordered_map>
#include <vector>
#include <utility>
#include <limits>

#include <Eigen/Core>

namespace uap_ta
{

class UAPTasksAssigner
{
public:
  // Default constructor
  UAPTasksAssigner() = default;

  // Main function to assign tasks based on the cost matrix
  std::unordered_map<int, std::pair<int, std::vector<int>>> assign(Eigen::MatrixXi& cost_matrix);

private:
  // Draws the minimum number of lines to cover all zeros in the matrix
  Eigen::MatrixXi drawMinLines(const Eigen::MatrixXi& cost_matrix);

  // Calculates the maximum number of zeros in a row or column and indicates the direction
  int hvMax(const Eigen::MatrixXi& cost_matrix, int row, int col);

  // Clears neighbors in the matrix to help with line drawing
  void clearNeighbours(Eigen::MatrixXi& max_zeros, Eigen::MatrixXi& lines, int row, int col);

  // Calculates the total number of lines drawn
  int calcSumOfLines(const Eigen::MatrixXi& lines);

  // Finds intersecting points of lines for optimization
  std::vector<std::pair<int, int>> findIntersectingPoints(const Eigen::MatrixXi& lines);
};

Eigen::MatrixXi UAPTasksAssigner::drawMinLines(const Eigen::MatrixXi& cost_matrix)
{
  Eigen::MatrixXi max_zeros = Eigen::MatrixXi::Zero(cost_matrix.rows(), cost_matrix.cols());
  Eigen::MatrixXi lines = Eigen::MatrixXi::Zero(cost_matrix.rows(), cost_matrix.cols());

  for (int row = 0; row < cost_matrix.rows(); ++row) 
    for (int col = 0; col < cost_matrix.cols(); ++col) 
      if (cost_matrix(row, col) == 0)
        max_zeros(row, col) = hvMax(cost_matrix, row, col);

  for (int row = 0; row < cost_matrix.rows(); ++row) 
    for (int col = 0; col < cost_matrix.cols(); ++col)
      if (std::abs(max_zeros(row, col)) > 0) 
        clearNeighbours(max_zeros, lines, row, col);

  return lines;
}

int UAPTasksAssigner::hvMax(const Eigen::MatrixXi& cost_matrix, int row, int col)
{
  int vertical = 0;
  int horizontal = 0;

  for (int i = 0; i < cost_matrix.cols(); ++i)
    if (cost_matrix(row, i) == 0)
      ++horizontal;

  for (int i = 0; i < cost_matrix.rows(); ++i)
    if (cost_matrix(i, col) == 0)
      ++vertical;

  return vertical > horizontal ? vertical : -horizontal;
}

void UAPTasksAssigner::clearNeighbours(Eigen::MatrixXi& max_zeros, 
                                       Eigen::MatrixXi& lines, 
                                       int row, 
                                       int col)
{
  if (max_zeros(row, col) > 0)
  {
    for (int i = 0; i < max_zeros.rows(); ++i)
    {
      if (max_zeros(i, col) > 0)
        max_zeros(i, col) = 0;
      
      lines(i, col) = 1;
    }
  }
  else
  {
    for (int i = 0; i < max_zeros.cols(); ++i)
    {
      if (max_zeros(row, i) > 0)
        max_zeros(row, i) = 0;
      
      lines(row, i) = 1;
    }
  }

  max_zeros(row, col) = 0;
  lines(row, col) = 1;
}

int UAPTasksAssigner::calcSumOfLines(const Eigen::MatrixXi& lines)
{
  if (lines.isOnes()) return lines.rows();

  Eigen::VectorXi col_sum = lines.colwise().sum();
  Eigen::VectorXi row_sum = lines.rowwise().sum();

  return (col_sum.array() == lines.rows()).count() +
         (row_sum.array() == lines.cols()).count();          
}

std::vector<std::pair<int, int>> UAPTasksAssigner::findIntersectingPoints(const Eigen::MatrixXi& lines) 
{
  std::vector<std::pair<int, int>> indices;

  for (int row = 0; row < lines.rows(); ++row) 
  {
    for (int col = 0; col < lines.cols(); ++col) 
    {
      if (lines(row, col) == 1) 
      {
        bool is_horizontal_line = true;
        bool is_vertical_line = true;

        for (int k = 0; k < lines.cols(); ++k) 
        {
          if (lines(row, k) != 1) 
          {
            is_horizontal_line = false;
            break;
          }
        }

        for (int k = 0; k < lines.rows(); ++k) 
        {
          if (lines(k, col) != 1) 
          {
            is_vertical_line = false;
            break;
          }
        }

        if (is_horizontal_line && is_vertical_line) 
          indices.emplace_back(row, col);
      }
    }
  }

  return indices;
}

std::unordered_map<int, std::pair<int, std::vector<int>>> UAPTasksAssigner::assign(Eigen::MatrixXi& cost_matrix)
{
  if (cost_matrix.cols() < cost_matrix.rows())
    throw std::invalid_argument("Number of tasks cannot be less than the number of agents.");

  auto cost_matrix_orig = cost_matrix;

  std::unordered_map<int, std::pair<int, std::vector<int>>> assignments = {};
  for (int i = 0; i < cost_matrix.rows(); ++i)
    assignments.insert({i, {}});

  Eigen::VectorXi min_col_coeffs = cost_matrix.colwise().minCoeff();
  for (int i = 0; i < cost_matrix.cols(); ++i)
    cost_matrix.col(i) -= Eigen::VectorXi::Constant(cost_matrix.rows(), min_col_coeffs(i));

  Eigen::VectorXi min_row_coeffs = cost_matrix.rowwise().minCoeff();
  for (int i = 0; i < cost_matrix.rows(); ++i)
      cost_matrix.row(i) -= Eigen::RowVectorXi::Constant(cost_matrix.cols(), min_row_coeffs(i));
      
  Eigen::MatrixXi lines = drawMinLines(cost_matrix);
  int num_lines = calcSumOfLines(lines);

  while (num_lines != cost_matrix.rows())
  {
    int min_cost = std::numeric_limits<int>::max();

    for (int row = 0; row < lines.rows(); row++) 
      for (int col = 0; col < lines.cols(); col++) 
        if (lines(row, col) == 0 && cost_matrix(row, col) < min_cost)
          min_cost = cost_matrix(row, col);

    for (int row = 0; row < lines.rows(); row++) 
      for (int col = 0; col < lines.cols(); col++) 
        if (lines(row, col) == 0)
          cost_matrix(row, col) -= min_cost;

    std::vector<std::pair<int, int>> intersections = findIntersectingPoints(lines);
    for (auto kv : intersections)
      cost_matrix(kv.first, kv.second) += min_cost;

    lines = drawMinLines(cost_matrix);
    num_lines = calcSumOfLines(lines);
  }


  while ((cost_matrix.array() == 0).any()) 
  {
    for (int i = 0; i < cost_matrix.rows(); i++) 
    {
      auto row = cost_matrix.row(i);
      int num_zeros = (row.array() == 0).count();
      std::pair<int, int> assignment = {-1, -1};

      if (num_zeros == 1 || num_zeros > 1) 
      {
        int min_row_cost = std::numeric_limits<int>::max();

        for (int j = 0; j < row.size(); j++) 
        {
          if (row(j) == 0) 
          {
            if (num_zeros == 1 || cost_matrix_orig(i, j) < min_row_cost) 
            {
              min_row_cost = cost_matrix_orig(i, j);
              assignment = {i, j};
            }
          }
        }
      }

      if (assignment != std::pair<int, int>{-1, -1}) 
      {
        cost_matrix(assignment.first, assignment.second) = -1;

        for (int k = 0; k < cost_matrix.rows(); k++) 
        {
          if (cost_matrix(k, assignment.second) == 0)
            cost_matrix(k, assignment.second) = -1;
        }

        assignments.at(assignment.first).first += cost_matrix_orig(assignment.first, assignment.second);
        assignments.at(assignment.first).second.push_back(assignment.second);
      }
    }
  }

  return assignments;
}

} // namespace uap_ta
#endif  // UAP_TA_TASKS_ASSIGNER_HPP_
