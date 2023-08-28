#pragma once

#include <iostream>
#include <utility>
#include <vector>

using namespace std;

/*
 * @brief:
 * This class solve an optimization problem:
 * Y
 * |
 * |                       P(x1, y1)  P(x2, y2)
 * |            P(x0, y0)                       ... P(x(k-1), y(k-1))
 * |P(start)
 * |
 * |________________________________________________________ X
 *
 *
 * Given an initial set of points from 0 to k-1,  The goal is to find a set of
 * points which makes the line P(start), P0, P(1) ... P(k-1) "smooth".
 */

class FemPosDeviationSmoother {
 public:
  bool Solve(const std::vector<std::pair<double, double>>& raw_point2d,
             const std::vector<double>& bounds, std::vector<double>* opt_x,
             std::vector<double>* opt_y);

 private:
  bool QpWithOsqp(const std::vector<std::pair<double, double>>& raw_point2d,
                  const std::vector<double>& bounds, std::vector<double>* opt_x,
                  std::vector<double>* opt_y);
};
