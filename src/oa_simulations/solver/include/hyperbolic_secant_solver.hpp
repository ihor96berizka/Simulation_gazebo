#ifndef HYPERBOLIC_SECANT_SOLVER_H
#define HEPERBOLIC_SECANT_SOLVER_H

#include "isolver.h"
#include <iostream>
#include <fstream>

namespace Solver
{

class HyperbolicSecantSolver : public ISolver
{
public:
HyperbolicSecantSolver()
    {
        std::cout << "--------------Initialized HyperbolicSecantSolver solver---------------\n";
        _output_stream = std::make_unique<std::ofstream>("hyperbolic_secant_exec_time.txt", std::ios::app);

    }
    ~HyperbolicSecantSolver()
    {
        _output_stream->close();
        std::cout << "====Dumping exec time to file completed....\n";
    }
    int calculateHeadingAngle(int teta_goal) override;

private:
    std::vector<DistanceSensorData> calculateRepulsiveField() override;
    std::vector<DistanceSensorData> calculateAttractiveField(int teta_goal) override;

    std::unique_ptr<std::ofstream> _output_stream;
};

}

#endif // HEPERBOLIC_SECANT_SOLVER_H
