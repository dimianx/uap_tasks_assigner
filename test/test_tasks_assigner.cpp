#include "uap_ta/tasks_assigner.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>
class UAPTasksAssignerTest : public ::testing::Test 
{
protected:
  uap_ta::UAPTasksAssigner assigner;
};

TEST_F(UAPTasksAssignerTest, AssignTasksSquared) 
{
  Eigen::MatrixXi cost_matrix(4, 4);
  cost_matrix << 10, 10, 8, 7,
                 7, 5, 10, 9,
                 4, 3, 10, 2,
                 1, 2, 10, 3;

  auto result = assigner.assign(cost_matrix);

  std::unordered_map<int, std::pair<int, std::vector<int>>> expected_assignments = {
    {0, {8, {2}}},
    {1, {5, {1}}},
    {2, {2, {3}}},
    {3, {1, {0}}}
  };

  EXPECT_EQ(result.size(), expected_assignments.size());

  for (const auto& [task, assignment] : result) 
  {
    EXPECT_EQ(assignment.first, expected_assignments[task].first);
    EXPECT_EQ(assignment.second, expected_assignments[task].second);
  }
}

TEST_F(UAPTasksAssignerTest, AssignTasksSingleTask) 
{
  Eigen::MatrixXi cost_matrix(1, 1);
  cost_matrix << 5;

  auto result = assigner.assign(cost_matrix);

  std::unordered_map<int, std::pair<int, std::vector<int>>> expected_assignments = {
    {0, {5, {0}}}
  };

  EXPECT_EQ(result.size(), expected_assignments.size());

  for (const auto& [task, assignment] : result) 
  {
    EXPECT_EQ(assignment.first, expected_assignments[task].first);
    EXPECT_EQ(assignment.second, expected_assignments[task].second);
  }
}

TEST_F(UAPTasksAssignerTest, AssignTasksUnequalMatrix) 
{
  Eigen::MatrixXi cost_matrix(2, 3);
  cost_matrix << 5, 8, 7,
                 6, 2, 4;

  auto result = assigner.assign(cost_matrix);

  std::unordered_map<int, std::pair<int, std::vector<int>>> expected_assignments = {
    {0, {5, {0}}},
    {1, {6, {1, 2}}}
  };

  EXPECT_EQ(result.size(), expected_assignments.size());

  for (const auto& [task, assignment] : result) 
  {
    EXPECT_EQ(assignment.first, expected_assignments[task].first);
    EXPECT_EQ(assignment.second, expected_assignments[task].second);
  }
}