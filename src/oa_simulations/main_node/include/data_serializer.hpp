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
    void serializeForces(const Solver::Forces& forces) override;

private:
    std::string _filename;
    std::unique_ptr<std::ofstream> _output_stream;
    json main_json_obj;
    json::array_t json_forces_array = json::array();
};
