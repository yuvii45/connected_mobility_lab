// Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef THIRD_PARTY__ASSOCIATION_FUNCTIONS_HPP_
#define THIRD_PARTY__ASSOCIATION_FUNCTIONS_HPP_

#include <iostream>
#include <algorithm>

#include "hungarian_algorithm.hpp"

namespace indoor_positioning_system
{

constexpr static auto DynamicSize = Eigen::Dynamic;
constexpr static auto NoChange = Eigen::NoChange;

template<int N, int M>
using Matrix = Eigen::Matrix<float, N, M>;

using DynamicMatrix = Eigen::Matrix<float, DynamicSize, DynamicSize>;

template<int N>
using SquareMatrix = Matrix<N, N>;

using CostMatrix = Matrix<DynamicSize, DynamicSize>;

using AssignmentMatrix = Eigen::Matrix<int, DynamicSize, DynamicSize>;

inline void hungarian(const CostMatrix & cost_matrix, AssignmentMatrix & assignment_matrix)
{
  const auto n_rows = cost_matrix.rows();
  const auto n_cols = cost_matrix.cols();

  // Resize and initialize assignment matrix
  assignment_matrix.resize(n_rows, n_cols);
  assignment_matrix.setZero();

  // No assignment if matrix is empty
  if (!(n_rows > 0 && n_cols > 0)) {
    return;
  }

  // Create and solve assignment problem with hungarian algorithm
  auto problem = HungarianAlgorithm<float>(cost_matrix);
  problem.SolveAssignmentProblem();

  // Set assignment matrix with solution
  problem.GetAssignmentMatrix(assignment_matrix);
}

}  // namespace indoor_positioning_system

#endif  // THIRD_PARTY__ASSOCIATION_FUNCTIONS_HPP_
