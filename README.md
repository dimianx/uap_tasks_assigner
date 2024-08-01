# UAPTasksAssigner

UAPTasksAssigner is a header-only C++ library designed for solving unbalanced assignment problem. 
The library supports solving the unbalanced assignment problem where the number of tasks does not equal the number of resources:

$$
\text{min} \sum_{i=1}^{m} \sum_{j=1}^{n} C_{ij} X_{ij}
$$

Subject to:
1. Each task must be assigned to exactly one resource:
```math
   \sum_{j=0}^{n} X_{ij} \geq 1, \quad i = 0, 1, 2, \dots, m
```

2. Each resource can be assigned to at most one task:
```math
\sum_{i=0}^{m} X_{ij} = 1, \quad j = 0, 1, 2, \dots, n
```

3. The decision variables are binary:
```math
   X_{ij} \in \{0, 1\}
```

where:
-  $C_{ij}$  represents the cost of assigning task \( i \) to resource \( j \).
-  $X_{ij}$  is a binary variable that is 1 if task \( i \) is assigned to resource \( j \), and 0 otherwise.


## Installation
### Prerequisites
- C++17 or later
- Eigen (version 3.3 or later)

## Usage
To use the UAPTasksAssigner, include the header and create an instance of the class. Define a cost matrix and call the `assign` method to get the optimal assignments of tasks to resources.
The cost matrix should be of size \( N \times M \), where \( N \) is the number of agents (resources) and \( M \) is the number of tasks. The `assign` method returns a `std::unordered_map<int, std::pair<int, std::vector<int>>>` where:
- The key (int) is the agent number.
- The value is a `std::pair` consisting of:
  - The total cost for the agent as the first element.
  - A vector of task numbers assigned to the agent as the second element.

### Example

```cpp
#include <iostream>

#include <Eigen/Core>

#include "uap_ta/tasks_assigner.hpp"

int main() 
{
  Eigen::MatrixXi cost_matrix(60, 130);
  for (int i = 0; i < 60; ++i)
    for (int j = 0; j < 130; ++j) 
      cost_matrix(i, j) = (i * 130 + j + 1) % 30 + 1;


  uap_ta::UAPTasksAssigner assigner;
  auto assignments = assigner.assign(cost_matrix);

  for (const auto& [agent, details] : assignments) 
  {
    int total_cost = details.first;
    const std::vector<int>& tasks = details.second;

    std::cout << "Agent " << agent << ": Total Cost = " << total_cost << ", Tasks = [";
    for (size_t i = 0; i < tasks.size(); ++i) 
    {
      std::cout << tasks[i];
      if (i < tasks.size() - 1) std::cout << ", ";
    }
    
    std::cout << "]" << std::endl;
  }

  return 0;
}
```

## References
- Kumar, Avanish. "A modified method for solving the unbalanced assignment problems." Applied mathematics and computation 176.1 (2006): 76-82.
