#include "main_node.hpp"

#include "ros_data_provider.hpp"
#include "data_serializer.hpp"
#include "solver.h"
#include "linearsolver.h"
#include "laplacesolver.h"

#include <cmath>
#include <fstream>

using namespace std::chrono_literals;

constexpr float DegreesToRadians(float degrees) {
    return degrees * (M_PI / 180.0);
}

MainSwcNode::MainSwcNode(): Node("robot_controller")
{
}

MainSwcNode::~MainSwcNode()
{
    processing_thread_->request_stop();
}

void MainSwcNode::init()
{
    std::this_thread::sleep_for(15s);
    std::cout <<  " ============ MainSwcNode::init() begin ======== " << std::endl;
    RCLCPP_INFO(this->get_logger(), " ============ MainSwcNode::init() begin ======== ");
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

    lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 1,
      std::bind(&MainSwcNode::lidarSensorCallback, this,
                std::placeholders::_1));

    auto dataProvider = std::make_unique<RosDataProvider>(queue_);
    auto dataSerializer = std::make_unique<Serializer>("dataRos.json");
    solver_ = //std::make_unique<Solver::LinearSolver>();
            //std::make_unique<Solver::GussianSolver>();
            std::make_unique<Solver::LaplaceSolver>();
    solver_->init(std::move(dataProvider), std::move(dataSerializer));


    /*auto command_message = std::make_unique<geometry_msgs::msg::Twist>();
    command_message->linear.x = 0.0;
    command_message->angular.z = DegreesToRadians(30);
    publisher_->publish(std::move(command_message));
    std::this_thread::sleep_for(1s);
*/ 
    RCLCPP_INFO(this->get_logger(), " ============ MainSwcNode::init() start processing thread ======== ");
    
    RCLCPP_INFO(this->get_logger(), " ======init teta: %f ====== ", teta_goal);
    processing_thread_ = std::make_unique<tools::jthread>([this]()
    {
        //std::this_thread::sleep_for(500ms);
            RCLCPP_INFO(this->get_logger(), " ======calculate angle begin ====== ");
            RCLCPP_INFO(this->get_logger(), " ======calculate angle teta: %f ====== ", teta_goal);
            //auto mapped_teta_goal = Solver::map(teta_goal, 0, 360, -180, 180);
            //RCLCPP_INFO(this->get_logger(), " ======calculate angle mapped teta: %f ====== ", mapped_teta_goal);
            int angle = solver_->calculateHeadingAngle(Solver::SolverParams::_teta_goal);

            RCLCPP_INFO(this->get_logger(), "Safe angle: %d", angle);
            //std::cout << "Safe angle: " << angle << std::endl;
            //auto unmapped_angle = Solver::map(angle, -180, 180, 0, 360);
            //RCLCPP_INFO(this->get_logger(), "Unmapped Safe angle: %d", unmapped_angle);
        //  send angle to actuator
            
            //teta_goal = teta_goal - angle;
            //RCLCPP_INFO(this->get_logger(), "new teta_goal angle: %f", teta_goal);

            //Solver::SolverParams::_local_heading = Solver::SolverParams::_teta_goal + angle;

            
            RCLCPP_INFO(this->get_logger(), " ====== send command to actuator ====== ");
            auto command_message = std::make_unique<geometry_msgs::msg::Twist>();
            command_message->linear.x = 0.1;
            command_message->angular.z = DegreesToRadians(angle);
            publisher_->publish(std::move(command_message));
            RCLCPP_INFO(this->get_logger(), " ====== command is sent to actuator ====== ");

            // serialize data to file
            solver_->serializeToFile();

            std::this_thread::sleep_for(1500ms);

            
            if (angle != 0)
            {
                int safe_angle_rollback = angle * -1;
                std::cout << "Safe angle rollback: " << safe_angle_rollback << std::endl;
                RCLCPP_INFO(this->get_logger(), " ====== Restore original heading command to actuator ====== ");
                auto restore_heading_message = std::make_unique<geometry_msgs::msg::Twist>();
                restore_heading_message->linear.x = 0.0005;
            
                restore_heading_message->angular.z = DegreesToRadians(safe_angle_rollback);
                publisher_->publish(std::move(restore_heading_message));
                std::this_thread::sleep_for(1500ms);
            }
    });
    RCLCPP_INFO(this->get_logger(), " ============ MainSwcNode::init() end ======== ");
    
}

void MainSwcNode::lidarSensorCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    //RCLCPP_INFO(this->get_logger(), "lidar scan received");
    
    /*for (int idx = 0; idx < 180; ++idx)
    {
        RCLCPP_INFO(this->get_logger(), "angle: %d; dist: %f", idx-90, msg->ranges[idx]);
    }*/
    queue_.push(msg->ranges);

    //std::lock_guard lock{lidar_data_mtx_};
    //latest_lidar_sample = msg->ranges;
   // RCLCPP_INFO(this->get_logger(), "lidar scan pushed to queue");
}

