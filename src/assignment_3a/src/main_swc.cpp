#include "main_node.hpp"


int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  auto avoider = std::make_shared<MainSwcNode>();
  avoider->init();
  rclcpp::spin(avoider);
  rclcpp::shutdown();

  return 0;
}