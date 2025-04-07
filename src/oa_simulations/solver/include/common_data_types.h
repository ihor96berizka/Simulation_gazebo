#pragma once

#include <vector>

namespace Solver
{
struct DistanceSensorData
{
    double angle;  // in radians.
    double distance;
};

struct Obstacle
{
    std::vector<double> distances;
    std::vector<double> angles;
    double averageDistance;
    double averageAngle;
    double a;
};

// params should be set acording to turtlebot3 burger specs:
// https://emanual.robotis.com/docs/en/platform/turtlebot3/features/
struct SolverParams
{
    static constexpr double _thresholdDistance = 1.; // minimum distance to object.
    static constexpr double _w_robot = 0.2; // robot width in meters. turtlebot3 burger w = 0.18
    static constexpr double _distance_sensor_range = 6; // maximum range of distance sensor, in meters.
    static constexpr double _teta_goal = (0); // angle to goal point.
    static constexpr double _gamma = 0.06; // see eq (11)
    inline static double _local_heading{_teta_goal};
};

struct Forces
{
    std::vector<DistanceSensorData> repulsiveFieldData;
    std::vector<DistanceSensorData> attrFieldData;
    std::vector<DistanceSensorData> totalFieldData;
};
}