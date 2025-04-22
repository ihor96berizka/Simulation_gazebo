#pragma once
#include "idataprovider.h"
#include "isolver.h"

#include <fstream>
#include <iostream>

namespace Solver
{

class GussianSolver : public ISolver
{
public:

    GussianSolver()
    {
        std::cout << "--------------Initialized Gauss solver---------------\n";
        _output_stream = std::make_unique<std::ofstream>("gauss_exec_time.txt", std::ios::app);

    }
    ~GussianSolver()
    {
        _output_stream->close();
        std::cout << "====Dumping exec time to file completed....\n";
    }
    void init(std::unique_ptr<IDataProvider> dataProvider);
    std::vector<DistanceSensorData> getSensorData(); 
    int calculateHeadingAngle(int teta_goal) override;

private:
    std::vector<std::vector<DistanceSensorData>> getRepulsiceComponents();
    std::vector<DistanceSensorData> calculateRepulsiveField() override;
    std::vector<DistanceSensorData> calculateAttractiveField(int teta_goal) override;

    std::unique_ptr<std::ofstream> _output_stream;
};

} //namespace Solver
