#include "data_serializer.hpp"

#include <iostream>

namespace
{
inline std::string kAngleKey{"angle"};
inline std::string kMagnitudeKey{"magnitude"};
inline std::string kRepulsiveKey{"repulsive"};
inline std::string kAttractiveKey{"attractive"};
inline std::string kTotalKey{"total"};
inline std::string kForcesKey{"forces"};
}
/**
 *
 * filename.json example
 *
 * forces object
 * {
 *  "repulsive": [
 *          {
 *              "angle": 0,
 *              "magnitude" : 1
 *          },
 *          {
 *              "angle": 0,
 *              "magnitude" : 1
 *          },
 *          ...
 *      ],
 *   "attractive": [],
 *   "total" : []
 * }
 *
 * main json object:
 * {
 *  "forces" = [
 *          forces_object1,
 *          ...
 *      ]
 * }
 */

Serializer::Serializer(const std::string& filename) : _filename{filename}
{
    _output_stream = std::make_unique<std::ofstream>(filename);
}
Serializer::~Serializer()
{
    std::cout << "====Dumping data to json....\n";
    main_json_obj[kForcesKey] = json_forces_array;
    (*_output_stream) << std::setw(4) << main_json_obj << std::endl;
    _output_stream->close();
    std::cout << "====Dumping data to json completed....\n";
}
void Serializer::serializeForces(const Solver::Forces& forces)
{
    const auto& repulsive = forces.repulsiveFieldData;
    const auto& attractive = forces.attrFieldData;
    const auto& total = forces.totalFieldData;

    // {main json obj}
    json json_obj;
    //auto json_forces_array = json::array();

    /**populate json with data */
    auto repulsive_force_array = json::array();
    for (const auto& item : repulsive)
    {
        json data;
        data[kAngleKey] = item.angle;
        data[kMagnitudeKey] = item.distance;
        repulsive_force_array.push_back(data);
    }
    json_obj[kRepulsiveKey] = repulsive_force_array;

    auto attractive_force_array = json::array();
    for (const auto& item : attractive)
    {
        json data;
        data[kAngleKey] = item.angle;
        data[kMagnitudeKey] = item.distance;
        attractive_force_array.push_back(data);
    }
    json_obj[kAttractiveKey] = attractive_force_array;

    auto total_force_array = json::array();
    for (const auto& item : total)
    {
        json data;
        data[kAngleKey] = item.angle;
        data[kMagnitudeKey] = item.distance;
        total_force_array.push_back(data);
    }
    json_obj[kTotalKey] = total_force_array;

    std::cout << "Adding new force item...\n";
    json_forces_array.push_back(json_obj);
    std::cout << "done...\n";
}