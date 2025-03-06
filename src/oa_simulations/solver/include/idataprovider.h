#pragma once

#include <vector>
#include <cmath>

#include "common_data_types.h"

namespace Solver
{
constexpr size_t kMaxNumberOfSamples{360};

constexpr double DegreesToRadians(double degrees) {
    return degrees * (M_PI / 180);
}

constexpr double RadiansToDegrees(double radians) {
    return radians * (180.0 / M_PI);
}

constexpr long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

class IDataProvider
{
public:
/*
*   @brief Fetch data from some source.
*   This should be blocking call.
*/
    virtual std::vector<DistanceSensorData> getSample() = 0;
};

}//  namespace Solver

