#include "ros_data_provider.hpp"
#include "isolver.h"

#include <iostream>
#include <fstream>

template <typename T>
void circular_shift(std::vector<T>& vec, int N) {
    int size = vec.size();
    if (size == 0) return; // Handle empty vector case

    N = (N % size + size) % size; // Normalize N to avoid negative shifts
    std::rotate(vec.begin(), vec.begin() + N, vec.end());
}

void dumpSensorData(const std::vector<Solver::DistanceSensorData> &msgs)
{
    
}

RosDataProvider::RosDataProvider(tools::ThreadSafeQueue<std::vector<float>>& queue) : queue_ref{queue}
{}

std::vector<Solver::DistanceSensorData> RosDataProvider::getSample()
{
    RCLCPP_INFO(rclcpp::get_logger("test_logger"), "==== RosDataProvider::getSample() begin. pop msg =====");
    //std::cout << "==== RosDataProvider::getSample() begin. pop msg =====" << std::endl;
    std::vector<float> raw_msg;
    queue_ref.pop(raw_msg);

    RCLCPP_INFO(rclcpp::get_logger("test_logger"), "Repacking messages.....");
    //std::cout << "Repacking messages.....\n";

    std::vector<Solver::DistanceSensorData> converted_msgs;
    converted_msgs.reserve(raw_msg.size());
    
    // there are 180 samples. each sample corresponds to 1 degree angle. 
    for (size_t angle = 0; angle < raw_msg.size(); ++angle)
    {
        if (raw_msg[angle] == std::numeric_limits<float>::infinity())
        {
            //RCLCPP_INFO(rclcpp::get_logger("test_logger"), "Repacking messages.....");
            //std::cout << "----replace inf val...." << std::endl;
            raw_msg[angle] = Solver::SolverParams::_distance_sensor_range;
        }
        auto mapped_angle = Solver::map(angle, 0, 360, -180, 180);
        converted_msgs.push_back({static_cast<double>(mapped_angle), raw_msg[angle]});
    }

    //std::cout << "Converted: " << converted_msgs.size() << " items\n";
    //RCLCPP_INFO(rclcpp::get_logger("test_logger"), "Converted %lu items", converted_msgs.size());
    //std::ofstream file("lidar_data.txt", std::ios::app);
    //circular_shift(converted_msgs, 180);

    //for (const auto& [angle, dist]: converted_msgs)
    {
        //std::cout  << angle << "|" << dist << std::endl;
        //RCLCPP_INFO(rclcpp::get_logger("test_logger"), "Angle: %f,  dist: %f", angle, dist);
        //file << angle << " " << dist << std::endl;
    }

    std::cout << "Done.....\n";
    RCLCPP_INFO(rclcpp::get_logger("test_logger"), "Done Repacking messages.....");

    return converted_msgs;
}