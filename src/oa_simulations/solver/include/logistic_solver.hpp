#ifndef LOGISTIC_SOLVER_H
#define LOGISTIC_SOLVER_H

#include "isolver.h"
#include <iostream>
#include <fstream>

namespace Solver
{

class LogisticSolver : public ISolver
{
public:
LogisticSolver()
    {
        std::cout << "--------------Initialized LogisticSolver solver---------------\n";
        _output_stream = std::make_unique<std::ofstream>("logistic_exec_time.txt", std::ios::app);

    }
    ~LogisticSolver()
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

#endif // LOGISTIC_SOLVER_H
