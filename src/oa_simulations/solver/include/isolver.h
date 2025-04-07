#pragma once

#include "idataprovider.h"
#include "idata_serializer.h"

#include <memory>
#include <string>
#include <vector>

namespace Solver
{
/*
* Usage: Create instance of Solver.
* Flow:
*   init(provider)
*   calculateHeadingAngle()
* calculateHeadingAngle() will wait for new chuck of data and then perform calculations.
* It should be called in a working loop in user code.
*/
class ISolver
{
public:
    void init(std::unique_ptr<IDataProvider> dataProvider,
              std::unique_ptr<ISerializer> serializer);
    std::vector<DistanceSensorData> getSensorData();
    Forces getForces();
    virtual int calculateHeadingAngle(int teta_goal) = 0;
    void serializeToFile();

protected:
    std::vector<DistanceSensorData> _distanceSensorData;
    Forces _forces;
    std::unique_ptr<IDataProvider> _dataProvider;
    std::unique_ptr<ISerializer> _serializer;

    std::vector<Obstacle> findObstacles();
    void enlargeObstacles(std::vector<Obstacle>& obstacles, const double w_robot);
    void calculateObstaclesAverages(std::vector<Obstacle> &obstacles);
    virtual std::vector<DistanceSensorData> calculateRepulsiveField() = 0;
    virtual std::vector<DistanceSensorData> calculateAttractiveField(int teta_goal) = 0;
    void calculateForces(int teta_goal);
    std::vector<DistanceSensorData> calculateTotalField(const std::vector<DistanceSensorData>& repulsive,
                                                        const std::vector<DistanceSensorData>& attractive);
};
}
