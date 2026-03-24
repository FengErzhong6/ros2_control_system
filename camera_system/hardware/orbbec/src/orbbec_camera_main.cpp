#include "camera_system/orbbec_camera_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<camera_system::OrbbecCameraNode>();
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
