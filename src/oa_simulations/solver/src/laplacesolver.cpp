#include "laplacesolver.h"

#include <cmath>
#include <numeric>
#include <algorithm>

#include <iostream>

namespace
{
double calculate_val(double theta, double Teta_k, double sigma, double A) 
{
    //double underExp = std::pow(Teta_k - theta, 2) / (2 * std::pow(sigma, 2));
    double underExp = -(std::sqrt(2) * std::abs(Teta_k - theta)
                    /
                    sigma);
    return A * std::exp(std::sqrt(2)) * std::exp(underExp);
}
}

namespace Solver
{

int LaplaceSolver::calculateHeadingAngle(int teta_goal)
{
    _distanceSensorData = _dataProvider->getSample();
    calculateForces(teta_goal);
    return std::min_element(std::begin(_forces.totalFieldData), std::end(_forces.totalFieldData),
                            [](const DistanceSensorData& lhs, const DistanceSensorData& rhs)
           {
               return lhs.distance < rhs.distance;
           })->angle;
}

std::vector<DistanceSensorData> LaplaceSolver::calculateRepulsiveField()
{
    //  find obstacles in distance sensors data.
    auto obstacles = findObstacles();

    //calculate d[k] and phi[k] - for (6)
    calculateObstaclesAverages(obstacles);

    enlargeObstacles(obstacles, SolverParams::_w_robot);

    // (9)
    for (size_t k = 0; k < obstacles.size(); ++k)
    {
        double d = SolverParams::_distance_sensor_range - (obstacles[k].averageDistance);
        obstacles[k].a =  d * std::exp(std::sqrt(2));
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

        //std::cout << "sigma/: " << sigma << std::endl;
        //std::cout << "teta[0]: " << Teta_k << std::endl;
        //std::cout << "A[k]: " << obstacles[i].a << std::endl;
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

std::vector<DistanceSensorData> LaplaceSolver::calculateAttractiveField(int teta_goal)
{
    std::vector<DistanceSensorData> attrFieldData;
    for (size_t i = 0; i < _distanceSensorData.size(); ++i) // distance sensor data is used, cause it holds angles.
    {
        double value = SolverParams::_gamma * (std::sqrt(2) / 0.5) * std::abs(DegreesToRadians(teta_goal - _distanceSensorData[i].angle));
        attrFieldData.push_back({_distanceSensorData[i].angle, value});
    }

    return attrFieldData;
}

}  //namespace
