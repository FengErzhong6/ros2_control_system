#pragma once

#include <chrono>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

#include "manus_system/msg/manus_glove_raw.hpp"
#include "manus_system/msg/manus_glove_raw_array.hpp"

namespace manus_system {

class ManusGripperNode : public rclcpp::Node {
public:
  explicit ManusGripperNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  struct FingertipPair {
    const manus_system::msg::ManusRawNode *thumb_tip{nullptr};
    const manus_system::msg::ManusRawNode *middle_tip{nullptr};
  };

  struct SideCommandState {
    double last_command{-1.0};
    std::chrono::steady_clock::time_point last_publish_time{};
  };

  void declareAndLoadParameters();
  void onGlovesRaw(const manus_system::msg::ManusGloveRawArray::SharedPtr msg);
  void onTeleopState(const std_msgs::msg::String::SharedPtr msg);
  void processGlove(const manus_system::msg::ManusGloveRaw &glove);
  FingertipPair findFingertips(const manus_system::msg::ManusGloveRaw &glove) const;
  double computeDistance(
      const manus_system::msg::ManusRawNode &first,
      const manus_system::msg::ManusRawNode &second) const;
  double mapDistanceToCommand(double distance) const;
  bool shouldPublishCommand(const std::string &side, double command) const;
  void publishCommand(
      const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr &publisher,
      const std::string &side,
      double command);
  static std::string toLowerCopy(std::string value);
  static std::string normalizeTeleopState(std::string value);

  std::string input_topic_;
  std::string left_output_topic_;
  std::string right_output_topic_;
  std::string teleop_state_topic_;
  double lower_threshold_distance_{0.03};
  double upper_threshold_distance_{0.10};
  double command_epsilon_{0.01};
  double publish_rate_hz_{50.0};
  bool require_enabled_state_{true};
  std::string latest_teleop_state_{"UNKNOWN"};

  rclcpp::Subscription<manus_system::msg::ManusGloveRawArray>::SharedPtr gloves_raw_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr teleop_state_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_command_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_command_publisher_;
  SideCommandState left_command_state_;
  SideCommandState right_command_state_;
};

}  // namespace manus_system
