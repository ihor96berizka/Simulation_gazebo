#pragma once

#include "idata_serializer.h"

class Serializer : public Solver::ISerializer
{
    void serializeForces(const Solver::Forces& forces) override;
};
