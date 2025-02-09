#pragma once

#include "common_data_types.h"

namespace Solver
{
class ISerializer
{
    virtual void serializeForces(const Forces& forces) = 0;
};
}
