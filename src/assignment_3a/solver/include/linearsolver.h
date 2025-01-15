#pragma once

#include "isolver.h"

namespace Solver
{

class LinearSolver : public ISolver
{
public:
    int calculateHeadingAngle(int teta_goal) override;

private:
    std::vector<DistanceSensorData> calculateRepulsiveField() override;
    std::vector<DistanceSensorData> calculateAttractiveField(int teta_goal) override;
};

}
