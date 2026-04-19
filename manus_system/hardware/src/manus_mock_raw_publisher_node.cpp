#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "manus_system/msg/manus_glove_raw.hpp"
#include "manus_system/msg/manus_glove_raw_array.hpp"
#include "manus_system/msg/manus_raw_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace manus_system {

class ManusMockRawPublisherNode : public rclcpp::Node
{
public:
  explicit ManusMockRawPublisherNode(const rclcpp::NodeOptions & options)
  : Node("manus_raw_publisher_node", options)
  {
    publish_rate_hz_ = declare_parameter<double>("publish_rate", 30.0);
    pinch_distance_ = declare_parameter<double>("pinch_distance", 0.08);
    frame_id_ = declare_parameter<std::string>("frame_id", "manus_mock");

    if (publish_rate_hz_ <= 0.0) {
      throw std::runtime_error("Parameter 'publish_rate' must be > 0.");
    }
    if (pinch_distance_ < 0.0) {
      throw std::runtime_error("Parameter 'pinch_distance' must be >= 0.");
    }

    publisher_ = create_publisher<manus_system::msg::ManusGloveRawArray>("~/gloves_raw", 10);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ManusMockRawPublisherNode::publishSample, this));

    RCLCPP_INFO(
      get_logger(),
      "Mock MANUS raw publisher ready. output=%s, publish_rate=%.1f Hz, pinch_distance=%.3f m",
      publisher_->get_topic_name(),
      publish_rate_hz_,
      pinch_distance_);
  }

private:
  manus_system::msg::ManusRawNode makeTipNode(
    uint32_t node_id,
    std::string chain_type,
    double x,
    double y,
    double z) const
  {
    manus_system::msg::ManusRawNode node;
    node.node_id = node_id;
    node.parent_node_id = 0U;
    node.joint_type = "tip";
    node.chain_type = std::move(chain_type);
    node.pose.position.x = x;
    node.pose.position.y = y;
    node.pose.position.z = z;
    node.pose.orientation.w = 1.0;
    return node;
  }

  manus_system::msg::ManusGloveRaw makeGlove(
    uint32_t glove_id,
    const std::string & side,
    double y_offset) const
  {
    manus_system::msg::ManusGloveRaw glove;
    glove.glove_id = glove_id;
    glove.side = side;
    glove.raw_nodes.reserve(2);
    glove.raw_nodes.push_back(makeTipNode(1U, "thumb", 0.00, y_offset, 0.00));
    glove.raw_nodes.push_back(makeTipNode(2U, "middle", pinch_distance_, y_offset, 0.00));
    glove.raw_node_count = static_cast<decltype(glove.raw_node_count)>(glove.raw_nodes.size());
    return glove;
  }

  void publishSample()
  {
    manus_system::msg::ManusGloveRawArray message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = frame_id_;
    message.gloves.reserve(2);
    message.gloves.push_back(makeGlove(1U, "left", 0.05));
    message.gloves.push_back(makeGlove(2U, "right", -0.05));
    publisher_->publish(std::move(message));
  }

  double publish_rate_hz_{30.0};
  double pinch_distance_{0.08};
  std::string frame_id_;
  rclcpp::Publisher<manus_system::msg::ManusGloveRawArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace manus_system

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<manus_system::ManusMockRawPublisherNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
