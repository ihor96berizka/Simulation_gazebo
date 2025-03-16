#pragma once

#include <fstream>
#include <memory>

#include "json.hpp"

#include "idata_serializer.h"

using json = nlohmann::json;

class Serializer : public Solver::ISerializer
{
public:
    Serializer(const std::string& filename);
    ~Serializer();
    void serializeData(const Solver::Forces& forces, const std::vector<Solver::DistanceSensorData> lidar_data) override;

private:
    std::string _filename;
    std::unique_ptr<std::ofstream> _output_stream;
    json main_json_obj;
    json::array_t json_forces_array = json::array();
    json::array_t json_lidar_array = json::array();
};
