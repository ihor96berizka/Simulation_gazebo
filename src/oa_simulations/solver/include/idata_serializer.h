#pragma once

#include "common_data_types.h"

namespace Solver
{
class ISerializer
{
public:
    virtual void serializeData(const Forces& forces, const std::vector<DistanceSensorData> lidar_data) = 0;
    virtual ~ISerializer() = default;
};
}
