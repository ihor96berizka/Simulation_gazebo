#include "solver.h"

#include <cmath>
#include <numeric>
#include <algorithm>
#include <iostream>

namespace {
double calculate_val(double theta, double Teta_k, double sigma, double A) 
{
    double underExp = std::pow(Teta_k - theta, 2) / (2 * std::pow(sigma, 2));
    return A * std::exp(-underExp);
}
}
namespace Solver
{
    
std::vector<std::vector<DistanceSensorData> > GussianSolver::getRepulsiceComponents()
{
    return {};
}

int GussianSolver::calculateHeadingAngle(int teta_goal)
{
    std::cout << "===============calculateHeadingAngle=====" << std::endl;
    _distanceSensorData = _dataProvider->getSample();
    std::cout << "===============calculateGForces=====" << std::endl;
    calculateForces(teta_goal);
    auto angle = (std::min_element(std::begin(_forces.totalFieldData), std::end(_forces.totalFieldData),
                            [](const DistanceSensorData& lhs, const DistanceSensorData& rhs)
           {
               return lhs.distance < rhs.distance;
           })->angle);
    std::cout << "---------lib-------Safe angle: " << angle << "----------------\n";
    return angle;
}


std::vector<DistanceSensorData> GussianSolver::calculateRepulsiveField()
{
    //  find obstacles in distance sensors data.
    auto obstacles = findObstacles();

    //calculate d[k] and phi[k] - for (6)
    calculateObstaclesAverages(obstacles);

    // (6) Phi[k] in rads
    enlargeObstacles(obstacles, SolverParams::_w_robot);

    // (9)
    for (size_t k = 0; k < obstacles.size(); ++k)
    {
        double d = SolverParams::_distance_sensor_range - (obstacles[k].averageDistance);
        obstacles[k].a =  d * std::exp(0.5);
        //qInfo() << "A[" << k << "]=" << obstacles[k].a;
    }

    // (10)
    std::vector<DistanceSensorData> repulsiveFieldData(_distanceSensorData.size(), {0, 0});

    for (size_t i = 0; i < obstacles.size(); ++i)
    {
        std::cout << "Average angle g: " << RadiansToDegrees(obstacles[i].averageAngle) << std::endl;
        int midIdx = obstacles[i].angles.size() / 2;
        double Teta_k = (obstacles[i].angles[midIdx]);  //center angle of the obstacle
        double sigma = (RadiansToDegrees(obstacles[i].averageAngle / 2.0));
        double A = obstacles[i].a;

        std::cout << "sigma/: " << sigma << std::endl;
        std::cout << "teta[0]: " << Teta_k << std::endl;
        std::cout << "A[k]: " << obstacles[i].a << std::endl;
        // For each function (Teta_k[i], sigma[i], A[i]), compute the values for all theta_values
        for (int j = 0; j < _distanceSensorData.size(); ++j)
        {
            double val = calculate_val(_distanceSensorData[j].angle, Teta_k, sigma, A);
            repulsiveFieldData[j].distance += val;  // Accumulate the result
            repulsiveFieldData[j].angle = _distanceSensorData[j].angle;
        }
    }

    return repulsiveFieldData;
}

std::vector<DistanceSensorData> GussianSolver::calculateAttractiveField(int teta_goal)
{
    std::vector<DistanceSensorData> attrFieldData;
    for (size_t i = 0; i < _distanceSensorData.size(); ++i) // distance sensor data is used, cause it holds angles.
    {
        double value = SolverParams::_gamma * std::abs((teta_goal - _distanceSensorData[i].angle));
        attrFieldData.push_back({_distanceSensorData[i].angle, value});
    }

    return attrFieldData;
}

}  //namespace
