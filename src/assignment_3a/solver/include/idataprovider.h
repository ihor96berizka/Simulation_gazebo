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

