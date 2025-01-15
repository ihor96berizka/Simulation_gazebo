#ifndef LAPLACESOLVER_H
#define LAPLACESOLVER_H

#include "isolver.h"
#include <iostream>

namespace Solver
{

class LaplaceSolver : public ISolver
{
public:
    LaplaceSolver()
    {
        std::cout << "--------------Initialized Laplace solver---------------\n";
    }
    int calculateHeadingAngle(int teta_goal) override;

private:
    std::vector<DistanceSensorData> calculateRepulsiveField() override;
    std::vector<DistanceSensorData> calculateAttractiveField(int teta_goal) override;
};

}

#endif // LAPLACESOLVER_H
