#include "main_node.hpp"

#include "ros_data_provider.hpp"
#include "solver.h"
#include "linearsolver.h"
#include "laplacesolver.h"

#include <cmath>
#include <fstream>

constexpr float DegreesToRadians(float degrees) {
    return degrees * (M_PI / 180.0);
}

MainSwcNode::MainSwcNode(): Node("main_swc_node")
{
}

MainSwcNode::~MainSwcNode()
{
    processing_thread_->request_stop();
}

void MainSwcNode::init()
{

    std::cout <<  " ============ MainSwcNode::init() begin ======== " << std::endl;
    RCLCPP_INFO(this->get_logger(), " ============ MainSwcNode::init() begin ======== ");
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

    lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/my_lidar", 1,
      std::bind(&MainSwcNode::lidarSensorCallback, this,
                std::placeholders::_1));

    auto dataProvider = std::make_unique<RosDataProvider>(queue_);
    solver_ = //std::make_unique<Solver::LinearSolver>();
            //std::make_unique<Solver::GussianSolver>();
            std::make_unique<Solver::LaplaceSolver>();
    solver_->init(std::move(dataProvider));

    std::atomic<int> teta_goal = 0;
    RCLCPP_INFO(this->get_logger(), " ============ MainSwcNode::init() start processing thread ======== ");
    processing_thread_ = std::make_unique<tools::jthread>([this, teta = &teta_goal]()
    {
            RCLCPP_INFO(this->get_logger(), " ======calculate angle begin ====== ");
            int angle = solver_->calculateHeadingAngle(teta->load());
            RCLCPP_INFO(this->get_logger(), "Safe angle: %d", angle);
            //std::cout << "Safe angle: " << angle << std::endl;
            /*
            * send angle to actuator
            */
            teta->store( teta->load() - angle);
            RCLCPP_INFO(this->get_logger(), "new teta_goal angle: %d", teta);

            //Solver::SolverParams::_local_heading = Solver::SolverParams::_teta_goal + angle;

            
            RCLCPP_INFO(this->get_logger(), " ====== send command to actuator ====== ");
            auto command_message = std::make_unique<geometry_msgs::msg::Twist>();
            command_message->linear.x = 0.025;
            command_message->angular.z = DegreesToRadians(angle);
            publisher_->publish(std::move(command_message));
            RCLCPP_INFO(this->get_logger(), " ====== command is sent to actuator ====== ");
        
    });
    RCLCPP_INFO(this->get_logger(), " ============ MainSwcNode::init() end ======== ");
}

void MainSwcNode::lidarSensorCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "lidar scan received");
    
    for (int idx = 0; idx < 180; ++idx)
    {
        RCLCPP_INFO(this->get_logger(), "angle: %d; dist: %f", idx-90, msg->ranges[idx]);
    }
    queue_.push(msg->ranges);
    RCLCPP_INFO(this->get_logger(), "lidar scan pushed to queue");
}
