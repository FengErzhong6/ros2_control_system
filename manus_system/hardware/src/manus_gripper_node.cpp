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
  teleop_state_subscription_ = create_subscription<std_msgs::msg::String>(
      teleop_state_topic_,
      rclcpp::QoS(1).transient_local(),
      std::bind(&ManusGripperNode::onTeleopState, this, std::placeholders::_1));

  left_command_publisher_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(left_output_topic_, 10);
  right_command_publisher_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(right_output_topic_, 10);

  RCLCPP_INFO(
      get_logger(),
      "MANUS gripper node ready. input=%s, left_output=%s, right_output=%s, "
      "teleop_state_topic=%s, require_enabled_state=%s, publish_rate=%.1f Hz, "
      "command_epsilon=%.4f, lower_threshold=%.4f m, upper_threshold=%.4f m",
      input_topic_.c_str(),
      left_output_topic_.c_str(),
      right_output_topic_.c_str(),
      teleop_state_topic_.c_str(),
      require_enabled_state_ ? "true" : "false",
      publish_rate_hz_,
      command_epsilon_,
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
  teleop_state_topic_ = declare_parameter<std::string>(
      "teleop_state_topic", "/tracker_teleop_controller/teleop_state");
  lower_threshold_distance_ = declare_parameter<double>("lower_threshold_distance", 0.03);
  upper_threshold_distance_ = declare_parameter<double>("upper_threshold_distance", 0.10);
  command_epsilon_ = declare_parameter<double>("command_epsilon", 0.01);
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);
  require_enabled_state_ = declare_parameter<bool>("require_enabled_state", true);

  if (upper_threshold_distance_ <= lower_threshold_distance_) {
    throw std::runtime_error(
        "Parameter 'upper_threshold_distance' must be greater than "
        "'lower_threshold_distance'.");
  }
  if (publish_rate_hz_ <= 0.0) {
    throw std::runtime_error("Parameter 'publish_rate_hz' must be > 0.");
  }
  if (command_epsilon_ < 0.0) {
    throw std::runtime_error("Parameter 'command_epsilon' must be >= 0.");
  }
}

void ManusGripperNode::onGlovesRaw(
    const manus_system::msg::ManusGloveRawArray::SharedPtr msg)
{
  if (msg == nullptr) {
    return;
  }

  if (require_enabled_state_ && latest_teleop_state_ != "ENABLED") {
    return;
  }

  for (const auto &glove : msg->gloves) {
    processGlove(glove);
  }
}

void ManusGripperNode::onTeleopState(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg == nullptr) {
    return;
  }
  latest_teleop_state_ = normalizeTeleopState(msg->data);
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

  if (!shouldPublishCommand(normalized_side, command)) {
    return;
  }

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

bool ManusGripperNode::shouldPublishCommand(const std::string &side, double command) const
{
  const SideCommandState *state = nullptr;
  if (side == kLeftSide) {
    state = &left_command_state_;
  } else if (side == kRightSide) {
    state = &right_command_state_;
  } else {
    return false;
  }

  const auto now = std::chrono::steady_clock::now();
  const auto min_interval = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  const bool interval_elapsed = (now - state->last_publish_time) >= min_interval;
  const bool command_changed =
      state->last_command < 0.0 ||
      std::fabs(command - state->last_command) >= command_epsilon_;
  return interval_elapsed || command_changed;
}

void ManusGripperNode::publishCommand(
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr &publisher,
    const std::string &side,
    double command)
{
  if (publisher == nullptr) {
    return;
  }

  std_msgs::msg::Float64MultiArray msg;
  msg.data.push_back(command);
  publisher->publish(std::move(msg));

  auto *state = (side == kLeftSide) ? &left_command_state_ : &right_command_state_;
  state->last_command = command;
  state->last_publish_time = std::chrono::steady_clock::now();

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

std::string ManusGripperNode::normalizeTeleopState(std::string value)
{
  const auto pipe_pos = value.find('|');
  if (pipe_pos != std::string::npos) {
    value = value.substr(0, pipe_pos);
  }

  value.erase(0, value.find_first_not_of(" \t"));
  const auto last_non_space = value.find_last_not_of(" \t");
  if (last_non_space == std::string::npos) {
    return "UNKNOWN";
  }
  value.erase(last_non_space + 1);

  std::transform(
      value.begin(),
      value.end(),
      value.begin(),
      [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
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
