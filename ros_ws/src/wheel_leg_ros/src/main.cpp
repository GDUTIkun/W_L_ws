#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "wheel_leg_ros/controller_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wheel_leg_ros::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
