#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "manus_system/manus_raw_publisher_node.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<manus_system::ManusRawPublisherNode>());
  rclcpp::shutdown();
  return 0;
}

