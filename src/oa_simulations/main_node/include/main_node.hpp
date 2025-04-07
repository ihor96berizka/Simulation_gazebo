#ifndef MAIN_NODE_HPP
#define MAIN_NODE_HPP

#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "thread_safe_queue.hpp"
#include "isolver.h"
#include "ros_data_provider.hpp"
#include "jthread.h"


using std::placeholders::_1;

class MainSwcNode : public rclcpp::Node
{

public:
    MainSwcNode();
    ~MainSwcNode();
    void init();
private:
    
    void lidarSensorCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr shutdown_timer_;

    std::pair<double, double> last_position_;  // Store last position (x, y)
    std::atomic<double> total_distance_;
    // queue of lidar scans to be processed
    tools::ThreadSafeQueue<std::vector<float>> queue_;

    //std::vector<float> latest_lidar_sample;
    //mutable std::mutex lidar_data_mtx_;

    std::unique_ptr<tools::jthread> processing_thread_;
    std::unique_ptr<Solver::ISolver> solver_;
    double teta_goal = Solver::SolverParams::_teta_goal;

    size_t step_counter = 0;
};

#endif // MAIN_NODE_HPP