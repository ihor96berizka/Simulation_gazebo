#include "logistic_solver.hpp"

#include <cmath>
#include <numeric>
#include <algorithm>

#include <iostream>
#include <chrono>
#include <fstream>

namespace
{
double sech(double x)
{
    return 1.0 / std::cosh(x);
}

double calculate_val(double theta, double Teta_k, double sigma, double A) 
{
    double underExp =  M_PI * (Teta_k - theta) / ( 2 * sqrt(3) * sigma);
    double y = sech(underExp);
    return A * y * y;
}
}

namespace Solver
{

int LogisticSolver::calculateHeadingAngle(int teta_goal)
{
    _distanceSensorData = _dataProvider->getSample();

    auto start = std::chrono::high_resolution_clock::now();

    calculateForces(teta_goal);
    int safe_angle = std::min_element(std::begin(_forces.totalFieldData), std::end(_forces.totalFieldData),
                            [](const DistanceSensorData& lhs, const DistanceSensorData& rhs)
            {
                return lhs.distance < rhs.distance;
            })->angle;

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Execution time: " << duration.count() << " microseconds" << std::endl;
    
    (*_output_stream) << duration.count() << std::endl;

    return safe_angle;
}

std::vector<DistanceSensorData> LogisticSolver::calculateRepulsiveField()
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
        double y = sech(M_PI / (2 * std::sqrt(3) ));
        obstacles[k].a =  d * pow(y, 2.0);
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

std::vector<DistanceSensorData> LogisticSolver::calculateAttractiveField(int teta_goal)
{
    std::vector<DistanceSensorData> attrFieldData;
    for (size_t i = 0; i < _distanceSensorData.size(); ++i) // distance sensor data is used, cause it holds angles.
    {
        double value = SolverParams::_gamma * std::abs(DegreesToRadians(teta_goal - _distanceSensorData[i].angle));
        attrFieldData.push_back({_distanceSensorData[i].angle, value});
    }

    return attrFieldData;
}
}  //namespace
