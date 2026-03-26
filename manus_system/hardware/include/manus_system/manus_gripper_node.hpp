#pragma once

#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

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

  void declareAndLoadParameters();
  void onGlovesRaw(const manus_system::msg::ManusGloveRawArray::SharedPtr msg);
  void processGlove(const manus_system::msg::ManusGloveRaw &glove);
  FingertipPair findFingertips(const manus_system::msg::ManusGloveRaw &glove) const;
  double computeDistance(
      const manus_system::msg::ManusRawNode &first,
      const manus_system::msg::ManusRawNode &second) const;
  double mapDistanceToCommand(double distance) const;
  void publishCommand(
      const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr &publisher,
      const std::string &side,
      double command) const;
  static std::string toLowerCopy(std::string value);

  std::string input_topic_;
  std::string left_output_topic_;
  std::string right_output_topic_;
  double lower_threshold_distance_{0.03};
  double upper_threshold_distance_{0.10};

  rclcpp::Subscription<manus_system::msg::ManusGloveRawArray>::SharedPtr gloves_raw_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_command_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_command_publisher_;
};

}  // namespace manus_system
