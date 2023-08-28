#include "ppp.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <vector>

namespace {
constexpr double dt = 50;                     // time: s.
constexpr double turn_heading_threshold = 1.2;// About 80 degree.
constexpr double turn_heading_limit = 4.2;
constexpr double static_point_threshold = 0.2;//
}// namespace

bool pointInRange(int point, const std::vector<vector<int>> &ranges) {
    for (const auto &range: ranges) {
        if (range[0] <= point && point <= range[1]) {
            return true;
        }
    }
    return false;
}

void PathPointsProcessor::ProcessPathPoints(
        const std::vector<std::pair<double, double>> &raw_path_point2d) {
    // Smooth path points with APOLLO DiscretePoints Smoother.
    // For more information, visit:
    // https://zhuanlan.zhihu.com/p/371585754
    std::vector<std::pair<double, double>> smoothed_path_point2d;
    if (!SmoothPathPoints(raw_path_point2d, &smoothed_path_point2d)) {
        std::cout << "Failed to smooth path points." << std::endl;
        return;
    }

    std::vector<std::pair<double, double>> rescaled_path_point2d;// rescaled points
    int n = 5;                                                   // scale factor
    std::vector<std::pair<double, double>> decimated_data = RescalePoints(smoothed_path_point2d, n);

    // Initialize path points.
    std::vector<PathPoint *> path_points;
    for (const auto rescaled_path_point: decimated_data) {
        PathPoint *path_point = new PathPoint;
        path_point->x = rescaled_path_point.first;
        path_point->y = rescaled_path_point.second;
        path_points.push_back(path_point);
    }

    // Evaluate path points, calculate path point's velocity and heading.
    EvaluatePathPoints(path_points);

    // Find turning points.
    std::vector<int> turning_points_index;
    FindTurningPathPoints(path_points, &turning_points_index);
    for (auto i: turning_points_index) {
        std::cout << "turning_id: " << i << std::endl;
    }

    // Find stop points.
    std::vector<vector<int>> stop_points_index;
    FindStoppingPathPoints(path_points, &stop_points_index);
    for (int i = 0; i < stop_points_index.size(); ++i) {
        std::cout << "stop_point: " << stop_points_index[i].front() << ": "
                  << stop_points_index[i].back() << std::endl;
    }
    std::vector<PointOrRange> output;
    output = CombinePoints(stop_points_index, turning_points_index);

    for (auto &element: output) {
        if (element.isRange) {
            // If it's a range, multiply each end of the range by 5
            for (auto &end: element.range) {
                end *= 5;
            }
            std::cout << "(" << element.range[0] << ", " << element.range[1] << ")";
        } else {
            // If it's a point, multiply the point by 5
            element.point *= 5;
            std::cout << " " << element.point << " ";
        }
    }

    return;
}

bool PathPointsProcessor::SmoothPathPoints(
        const std::vector<std::pair<double, double>> &raw_path_point2d,
        std::vector<std::pair<double, double>> *smoothed_path_point2d) {
    // Path_points' size should bigger than 2.
    if (raw_path_point2d.size() <= 2) {
        std::cout << "Path points size less than 2." << std::endl;
        return false;
    }

    constexpr double point_bound = 0.5;
    std::vector<double> box_bounds(raw_path_point2d.size(), point_bound);

    // fix front and back points to avoid end states deviate from the original
    // position.
    box_bounds.front() = 0.0;
    box_bounds.back() = 0.0;

    // box contraints on pos are used in fem pos smoother, thus shrink the
    // bounds by 1.0 / sqrt(2.0)
    const double box_ratio = 1.0 / std::sqrt(2.0);
    for (auto &bound: box_bounds) {
        bound *= box_ratio;
    }

    std::vector<double> opt_x;
    std::vector<double> opt_y;
    FemPosDeviationSmoother smoother;
    bool status = smoother.Solve(raw_path_point2d, box_bounds, &opt_x, &opt_y);

    if (!status) {
        std::cout << "Fem Pos reference line smoothing failed" << std::endl;
        return false;
    }

    if (opt_x.size() < 2 || opt_y.size() < 2) {
        std::cout << "Return by fem pos smoother is wrong. Size smaller than 2 "
                  << std::endl;
        return false;
    }

    for (int i = 0; i < opt_x.size(); ++i) {
        smoothed_path_point2d->emplace_back(opt_x[i], opt_y[i]);
    }

    return true;
}

std::vector<std::pair<double, double>> PathPointsProcessor::RescalePoints(
        const std::vector<std::pair<double, double>> &raw_path_point2d,
        int n) {

    std::vector<std::pair<double, double>> result;

    for (int i = 0; i < raw_path_point2d.size(); i += n) {
        result.push_back(raw_path_point2d[i]);
    }

    return result;
}


void PathPointsProcessor::EvaluatePathPoints(
        std::vector<PathPoint *> path_points) {
    for (int i = 0; i < path_points.size(); ++i) {
        // Set path point's index.
        path_points[i]->index = i + 1;

        // Skip evaluate the first and the last path point.
        if (i == 0 || i == (path_points.size() - 1)) {
            continue;
        }

        // Calculate path point's heading.
        const double theta = atan2((path_points[i + 1]->y - path_points[i]->y),
                                   (path_points[i + 1]->x - path_points[i]->x)) -
                             atan2((path_points[i]->y - path_points[i - 1]->y),
                                   (path_points[i]->x - path_points[i - 1]->x));
        path_points[i]->heading_change = std::abs(theta);

        // Calculate path point's vel.
        path_points[i]->vel =
                std::sqrt((path_points[i + 1]->x - path_points[i - 1]->x) *
                                  (path_points[i + 1]->x - path_points[i - 1]->x) +
                          (path_points[i + 1]->y - path_points[i - 1]->y) *
                                  (path_points[i + 1]->y - path_points[i - 1]->y)) /
                (2 * dt);
    }
}

void PathPointsProcessor::FindTurningPathPoints(
        std::vector<PathPoint *> path_points,
        std::vector<int> *turning_points_index) {
    for (std::size_t i = 0; i < path_points.size() - 2; ++i) {
        const auto *path_point = path_points[i];
        if (path_point->heading_change > turn_heading_threshold && path_point->heading_change < turn_heading_limit) {
            turning_points_index->emplace_back(path_point->index);
        }
    }
}


void PathPointsProcessor::FindStoppingPathPoints(
        std::vector<PathPoint *> path_points,
        std::vector<vector<int>> *stop_points_index) {
    int stop_points_unit_index = 0;
    std::vector<int> stop_points_unit;
    for (int i = 1; i < path_points.size() - 1; ++i) {
    std:
        cout << "Point: " << i << " Path Point velocity: " << path_points[i]->vel << std::endl;
        //std:cout << "Point: " << i << " Path Point HC: " << path_points[i]->heading_change << std::endl;
        if (path_points[i]->vel <= static_point_threshold &&
            path_points[i - 1]->vel > static_point_threshold) {
            stop_points_unit.push_back(i);
        }

        if (path_points[i]->vel <= static_point_threshold &&
            path_points[i + 1]->vel > static_point_threshold) {
            if (i == path_points.size() - 2 && stop_points_unit.empty()) {
                // If the last point is a stopping point and no starting point has been added,
                // then we skip this point.
                continue;
            }
            stop_points_unit.push_back(i);
            stop_points_unit_index++;
        }

        if (stop_points_unit.size() == 2 && (stop_points_unit[0] != stop_points_unit[1])) {
            stop_points_index->push_back(stop_points_unit);
            stop_points_unit.clear();
        }
    }
    // Check if there is a starting point without a stopping point in the stop_points_unit.
    if (stop_points_unit.size() == 1) {
        stop_points_unit.clear();
    }
}

std::vector<PointOrRange> PathPointsProcessor::CombinePoints(
        const std::vector<vector<int>> &stopPoints,
        const std::vector<int> &turningPoints) {
    std::vector<PointOrRange> output;

    // Add stop points to output
    for (const auto &stopPoint: stopPoints) {
        output.push_back(PointOrRange{true, 0, stopPoint});
    }

    // Add turning points to output if they are not within stop point ranges
    for (const auto &point: turningPoints) {
        if (!pointInRange(point, stopPoints)) {
            output.push_back(PointOrRange{false, point, {}});
        }
    }

    // Sort the output
    std::sort(output.begin(), output.end(), [](const PointOrRange &a, const PointOrRange &b) {
        if (!a.isRange && !b.isRange) {
            return a.point < b.point;
        } else if (!a.isRange && b.isRange) {
            return a.point < b.range[0];
        } else if (a.isRange && !b.isRange) {
            return a.range[0] < b.point;
        } else {
            return a.range[0] < b.range[0];
        }
    });

    return output;
}

void PathPointsProcessor::Process(std::string &data_input) {
    std::vector<std::pair<double, double>> raw_path_point2d;
    std::ifstream gt_file(data_input);
    std::string line;
    int idx = 1;
    while (std::getline(gt_file, line)) {
        std::istringstream iss(line);
        std::vector<float> values;

        while (std::getline(iss, line, ',')) {
            values.push_back(std::stof(line));
        }

        raw_path_point2d.emplace_back(make_pairs(values[0], values[1]));

        idx++;
    }
    ProcessPathPoints(raw_path_point2d);
}
