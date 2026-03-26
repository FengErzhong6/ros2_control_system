#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "manus_system/manus_calibration_node.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<manus_system::ManusCalibrationNode>());
  rclcpp::shutdown();
  return 0;
}

