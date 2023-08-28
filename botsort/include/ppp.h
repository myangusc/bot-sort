#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>


#include "DataType.h"
#include "fem_pos_deviation_smoother.h"

using namespace std;

struct PathPoint {
    double x = 0.0;
    double y = 0.0;
    double vel = 0.0;
    double heading_change = 0.0;
    bool is_stop = false;
    int index = -1;
};

struct PointOrRange {
    bool isRange;
    int point;
    std::vector<int> range;
};

class PathPointsProcessor {
public:
    static void Process(std::string &data_input);

private:
    static void ProcessPathPoints(
            const std::vector<std::pair<double, double>> &raw_path_point2d);
    static bool SmoothPathPoints(
            const std::vector<std::pair<double, double>> &raw_path_point2d,
            std::vector<std::pair<double, double>> *smoothed_path_point2d);

    static std::vector<std::pair<double, double>> RescalePoints(
            const std::vector<std::pair<double, double>> &raw_path_point2d,
            int n);

    static void EvaluatePathPoints(std::vector<PathPoint *> path_points);

    static void FindTurningPathPoints(std::vector<PathPoint *> path_points,
                                      std::vector<int> *turning_points_index);

    static void FindStoppingPathPoints(
            std::vector<PathPoint *> path_points,
            std::vector<vector<int>> *stop_points_index);

    static std::vector<PointOrRange> CombinePoints(
            const std::vector<vector<int>> &stopPoints,
            const std::vector<int> &turningPoints);
};
