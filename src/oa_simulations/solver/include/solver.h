#pragma once
#include "idataprovider.h"
#include "isolver.h"

namespace Solver
{

class GussianSolver : public ISolver
{
public:
    void init(std::unique_ptr<IDataProvider> dataProvider);
    std::vector<DistanceSensorData> getSensorData(); 
    int calculateHeadingAngle(int teta_goal) override;

private:
    std::vector<std::vector<DistanceSensorData>> getRepulsiceComponents();
    std::vector<DistanceSensorData> calculateRepulsiveField() override;
    std::vector<DistanceSensorData> calculateAttractiveField(int teta_goal) override;
};

} //namespace Solver
