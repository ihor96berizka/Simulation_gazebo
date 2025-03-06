#pragma once

#include "common_data_types.h"

namespace Solver
{
class ISerializer
{
public:
    virtual void serializeForces(const Forces& forces) = 0;
    virtual ~ISerializer() = default;
};
}
