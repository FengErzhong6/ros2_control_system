#include "manus_system/manus_gripper_node.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace manus_system {

namespace {

constexpr char kThumbChain[] = "thumb";
constexpr char kMiddleChain[] = "middle";
constexpr char kTipJoint[] = "tip";
constexpr char kLeftSide[] = "left";
constexpr char kRightSide[] = "right";

}  // namespace

ManusGripperNode::ManusGripperNode(const rclcpp::NodeOptions &options)
: Node("manus_gripper_node", options)
{
  declareAndLoadParameters();

  gloves_raw_subscription_ = create_subscription<manus_system::msg::ManusGloveRawArray>(
      input_topic_,
      rclcpp::QoS(10),
      std::bind(&ManusGripperNode::onGlovesRaw, this, std::placeholders::_1));

  left_command_publisher_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(left_output_topic_, 10);
  right_command_publisher_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(right_output_topic_, 10);

  RCLCPP_INFO(
      get_logger(),
      "MANUS gripper node ready. input=%s, left_output=%s, right_output=%s, "
      "lower_threshold=%.4f m, upper_threshold=%.4f m",
      input_topic_.c_str(),
      left_output_topic_.c_str(),
      right_output_topic_.c_str(),
      lower_threshold_distance_,
      upper_threshold_distance_);
}

void ManusGripperNode::declareAndLoadParameters()
{
  input_topic_ = declare_parameter<std::string>(
      "input_topic", "/manus_raw_publisher_node/gloves_raw");
  left_output_topic_ = declare_parameter<std::string>(
      "left_output_topic", "/gripper_L_controller/commands");
  right_output_topic_ = declare_parameter<std::string>(
      "right_output_topic", "/gripper_R_controller/commands");
  lower_threshold_distance_ = declare_parameter<double>("lower_threshold_distance", 0.03);
  upper_threshold_distance_ = declare_parameter<double>("upper_threshold_distance", 0.10);

  if (upper_threshold_distance_ <= lower_threshold_distance_) {
    throw std::runtime_error(
        "Parameter 'upper_threshold_distance' must be greater than "
        "'lower_threshold_distance'.");
  }
}

void ManusGripperNode::onGlovesRaw(
    const manus_system::msg::ManusGloveRawArray::SharedPtr msg)
{
  if (msg == nullptr) {
    return;
  }

  for (const auto &glove : msg->gloves) {
    processGlove(glove);
  }
}

void ManusGripperNode::processGlove(const manus_system::msg::ManusGloveRaw &glove)
{
  const std::string normalized_side = toLowerCopy(glove.side);
  if (normalized_side != kLeftSide && normalized_side != kRightSide) {
    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Ignoring glove %u with unsupported side '%s'.",
        glove.glove_id,
        glove.side.c_str());
    return;
  }

  const FingertipPair fingertips = findFingertips(glove);
  if (fingertips.thumb_tip == nullptr || fingertips.middle_tip == nullptr) {
    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Skipping %s glove %u because thumb TIP or middle TIP was not found.",
        normalized_side.c_str(),
        glove.glove_id);
    return;
  }

  const double distance = computeDistance(*fingertips.thumb_tip, *fingertips.middle_tip);
  const double command = mapDistanceToCommand(distance);

  if (normalized_side == kLeftSide) {
    publishCommand(left_command_publisher_, normalized_side, command);
  } else {
    publishCommand(right_command_publisher_, normalized_side, command);
  }
}

ManusGripperNode::FingertipPair ManusGripperNode::findFingertips(
    const manus_system::msg::ManusGloveRaw &glove) const
{
  FingertipPair fingertips;

  for (const auto &raw_node : glove.raw_nodes) {
    const std::string joint_type = toLowerCopy(raw_node.joint_type);
    if (joint_type != kTipJoint) {
      continue;
    }

    const std::string chain_type = toLowerCopy(raw_node.chain_type);
    if (chain_type == kThumbChain) {
      fingertips.thumb_tip = &raw_node;
    } else if (chain_type == kMiddleChain) {
      fingertips.middle_tip = &raw_node;
    }
  }

  return fingertips;
}

double ManusGripperNode::computeDistance(
    const manus_system::msg::ManusRawNode &first,
    const manus_system::msg::ManusRawNode &second) const
{
  const double dx = first.pose.position.x - second.pose.position.x;
  const double dy = first.pose.position.y - second.pose.position.y;
  const double dz = first.pose.position.z - second.pose.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double ManusGripperNode::mapDistanceToCommand(double distance) const
{
  if (distance <= lower_threshold_distance_) {
    return 0.0;
  }

  if (distance >= upper_threshold_distance_) {
    return 1.0;
  }

  return (distance - lower_threshold_distance_) /
         (upper_threshold_distance_ - lower_threshold_distance_);
}

void ManusGripperNode::publishCommand(
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr &publisher,
    const std::string &side,
    double command) const
{
  if (publisher == nullptr) {
    return;
  }

  std_msgs::msg::Float64MultiArray msg;
  msg.data.push_back(command);
  publisher->publish(std::move(msg));

  RCLCPP_DEBUG_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Published gripper command for %s: %.4f",
      side.c_str(),
      command);
}

std::string ManusGripperNode::toLowerCopy(std::string value)
{
  std::transform(
      value.begin(),
      value.end(),
      value.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

}  // namespace manus_system

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<manus_system::ManusGripperNode>());
  rclcpp::shutdown();
  return 0;
}
