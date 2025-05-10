#ifndef LAPLACESOLVER_H
#define LAPLACESOLVER_H

#include "isolver.h"
#include <iostream>
#include <fstream>

namespace Solver
{

class LaplaceSolver : public ISolver
{
public:
    LaplaceSolver()
    {
        std::cout << "--------------Initialized Laplace solver---------------\n";
        _output_stream = std::make_unique<std::ofstream>("laplace_exec_time.txt", std::ios::app);

    }
    ~LaplaceSolver()
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

#endif // LAPLACESOLVER_H
