#ifndef ROS_DATA_PROVIDER_HPP
#define ROS_DATA_PROVIDER_HPP

#include "idataprovider.h"
#include "thread_safe_queue.hpp"

#include "rclcpp/rclcpp.hpp"

class RosDataProvider : public Solver::IDataProvider
{
public:
    RosDataProvider(tools::ThreadSafeQueue<std::vector<float>>& queue);
    std::vector<Solver::DistanceSensorData> getSample() override;

private:
    tools::ThreadSafeQueue<std::vector<float>>& queue_ref;    
};

#endif // ROS_DATA_PROVIDER_HPP