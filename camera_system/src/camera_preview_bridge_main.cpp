#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace camera_system {
namespace {

size_t bytesPerPixel(const std::string &encoding)
{
  if (encoding == "rgb8" || encoding == "bgr8") {
    return 3U;
  }
  if (encoding == "mono8") {
    return 1U;
  }
  throw std::runtime_error(
      "Unsupported preview bridge encoding '" + encoding +
      "'. Supported values: rgb8, bgr8, mono8.");
}

class CameraPreviewBridge : public rclcpp::Node {
public:
  explicit CameraPreviewBridge(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
  : Node("camera_preview_bridge", options)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/image_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/preview/image_raw");
    output_width_ = declare_parameter<int64_t>("output_width", 0);
    output_height_ = declare_parameter<int64_t>("output_height", 0);
    publish_rate_ = declare_parameter<double>("publish_rate", 20.0);
    skip_when_no_subscribers_ = declare_parameter<bool>("skip_when_no_subscribers", true);

    if (input_topic_.empty() || output_topic_.empty()) {
      throw std::runtime_error("input_topic and output_topic must not be empty.");
    }
    if (input_topic_ == output_topic_) {
      throw std::runtime_error("input_topic and output_topic must differ for preview bridging.");
    }
    if (output_width_ < 0 || output_height_ < 0) {
      throw std::runtime_error("output_width and output_height must be non-negative.");
    }
    if (publish_rate_ <= 0.0) {
      throw std::runtime_error("publish_rate must be positive.");
    }

    auto qos = rclcpp::SensorDataQoS();
    qos.keep_last(1);

    publisher_ = create_publisher<sensor_msgs::msg::Image>(output_topic_, qos);
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
        input_topic_,
        qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { onImage(std::move(msg)); });
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / publish_rate_)),
        [this]() { onPublishTimer(); });

    RCLCPP_INFO(
        get_logger(),
        "Preview bridge ready. input=%s output=%s output_size=%ux%u publish_rate=%.2fHz "
        "skip_when_no_subscribers=%s",
        input_topic_.c_str(),
        output_topic_.c_str(),
        output_width_ > 0 ? static_cast<uint32_t>(output_width_) : 0U,
        output_height_ > 0 ? static_cast<uint32_t>(output_height_) : 0U,
        publish_rate_,
        skip_when_no_subscribers_ ? "true" : "false");
  }

private:
  void onImage(sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(latest_image_mutex_);
    latest_image_ = std::move(msg);
    ++latest_generation_;
  }

  void onPublishTimer()
  {
    if (skip_when_no_subscribers_ &&
        publisher_->get_subscription_count() == 0U &&
        publisher_->get_intra_process_subscription_count() == 0U) {
      return;
    }

    sensor_msgs::msg::Image::ConstSharedPtr latest_image;
    uint64_t latest_generation = 0U;
    {
      std::lock_guard<std::mutex> lock(latest_image_mutex_);
      latest_image = latest_image_;
      latest_generation = latest_generation_;
    }

    if (!latest_image || latest_generation == last_published_generation_) {
      return;
    }

    auto preview = buildPreview(*latest_image);
    if (!preview) {
      return;
    }

    last_published_generation_ = latest_generation;
    publisher_->publish(std::move(preview));
  }

  sensor_msgs::msg::Image::UniquePtr buildPreview(const sensor_msgs::msg::Image &input) const
  {
    const size_t bpp = bytesPerPixel(input.encoding);
    const size_t min_step = static_cast<size_t>(input.width) * bpp;
    const size_t min_data_size = static_cast<size_t>(input.step) * input.height;
    if (input.step < min_step) {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Dropped preview frame on %s due to invalid step=%u for width=%u encoding=%s.",
          input_topic_.c_str(),
          input.step,
          input.width,
          input.encoding.c_str());
      return nullptr;
    }
    if (input.data.size() < min_data_size) {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Dropped preview frame on %s due to invalid data size=%zu expected>=%zu.",
          input_topic_.c_str(),
          input.data.size(),
          min_data_size);
      return nullptr;
    }

    const uint32_t preview_width =
        output_width_ > 0 ? static_cast<uint32_t>(output_width_) : input.width;
    const uint32_t preview_height =
        output_height_ > 0 ? static_cast<uint32_t>(output_height_) : input.height;
    auto output = std::make_unique<sensor_msgs::msg::Image>();
    output->header = input.header;
    output->height = preview_height;
    output->width = preview_width;
    output->encoding = input.encoding;
    output->is_bigendian = input.is_bigendian;
    output->step = static_cast<sensor_msgs::msg::Image::_step_type>(preview_width * bpp);
    output->data.resize(static_cast<size_t>(output->step) * preview_height);

    if (input.width == preview_width &&
        input.height == preview_height &&
        input.step == output->step) {
      std::memcpy(output->data.data(), input.data.data(), output->data.size());
      return output;
    }

    for (uint32_t y = 0; y < preview_height; ++y) {
      const uint32_t src_y = std::min<uint32_t>(
          static_cast<uint32_t>((static_cast<uint64_t>(y) * input.height) / preview_height),
          input.height - 1U);
      const auto *src_row = input.data.data() + static_cast<size_t>(src_y) * input.step;
      auto *dst_row = output->data.data() + static_cast<size_t>(y) * output->step;
      for (uint32_t x = 0; x < preview_width; ++x) {
        const uint32_t src_x = std::min<uint32_t>(
            static_cast<uint32_t>((static_cast<uint64_t>(x) * input.width) / preview_width),
            input.width - 1U);
        std::memcpy(
            dst_row + static_cast<size_t>(x) * bpp,
            src_row + static_cast<size_t>(src_x) * bpp,
            bpp);
      }
    }

    return output;
  }

  std::string input_topic_;
  std::string output_topic_;
  int64_t output_width_{0};
  int64_t output_height_{0};
  double publish_rate_{20.0};
  bool skip_when_no_subscribers_{true};

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  mutable std::mutex latest_image_mutex_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
  uint64_t latest_generation_{0U};
  uint64_t last_published_generation_{0U};
};

}  // namespace
}  // namespace camera_system

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<camera_system::CameraPreviewBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
