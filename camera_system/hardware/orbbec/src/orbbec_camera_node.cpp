#include "camera_system/orbbec_camera_node.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace camera_system {
namespace {

rclcpp::Time to_ros_time(const std::shared_ptr<ob::Frame> &frame, const rclcpp::Clock &clock)
{
  const uint64_t global_us = frame->getGlobalTimeStampUs();
  if (global_us != 0U) {
    return rclcpp::Time(static_cast<int64_t>(global_us) * 1000, RCL_SYSTEM_TIME);
  }

  const uint64_t system_us = frame->getSystemTimeStampUs();
  if (system_us != 0U) {
    return rclcpp::Time(static_cast<int64_t>(system_us) * 1000, RCL_SYSTEM_TIME);
  }

  return clock.now();
}

}  // namespace

OrbbecCameraNode::OrbbecCameraNode(const rclcpp::NodeOptions &options)
: Node("orbbec_camera_node", options)
{
  declareAndLoadParameters();

  auto qos = rclcpp::SensorDataQoS();
  qos.keep_last(1);
  color_publisher_ =
      create_publisher<sensor_msgs::msg::Image>(config_.image_topic, qos);

  startPipeline();
}

OrbbecCameraNode::~OrbbecCameraNode()
{
  stopPipeline();
}

void OrbbecCameraNode::declareAndLoadParameters()
{
  config_.camera_name =
      declare_parameter<std::string>("camera_name", get_name());
  config_.serial_number =
      declare_parameter<std::string>("serial_number", "");
  config_.frame_id =
      declare_parameter<std::string>("frame_id", "camera_color_optical_frame");
  config_.image_topic =
      declare_parameter<std::string>("image_topic", "image_raw");
  config_.color_encoding =
      declare_parameter<std::string>("color_encoding", "rgb8");
  config_.color_width =
      static_cast<uint32_t>(declare_parameter<int64_t>("color_width", 640));
  config_.color_height =
      static_cast<uint32_t>(declare_parameter<int64_t>("color_height", 480));
  config_.color_fps =
      static_cast<uint32_t>(declare_parameter<int64_t>("color_fps", 30));

  if (config_.serial_number.empty()) {
    throw std::runtime_error("Parameter 'serial_number' must not be empty.");
  }

  if (config_.color_width == 0U || config_.color_height == 0U || config_.color_fps == 0U) {
    throw std::runtime_error("color_width, color_height and color_fps must be positive.");
  }
}

void OrbbecCameraNode::startPipeline()
{
  context_ = std::make_shared<ob::Context>();
  device_ = createDeviceBySerial(config_.serial_number);
  pipeline_ = std::make_shared<ob::Pipeline>(device_);
  auto pipeline_config = createPipelineConfig();

  pipeline_->start(
      pipeline_config,
      [this](std::shared_ptr<ob::FrameSet> frame_set) {
        handleFrameSet(frame_set);
      });

  pipeline_running_.store(true, std::memory_order_release);

  RCLCPP_INFO(
      get_logger(),
      "Started Orbbec camera '%s' serial=%s color=%ux%u@%u encoding=%s",
      config_.camera_name.c_str(),
      config_.serial_number.c_str(),
      config_.color_width,
      config_.color_height,
      config_.color_fps,
      config_.color_encoding.c_str());
}

void OrbbecCameraNode::stopPipeline()
{
  if (!pipeline_running_.exchange(false, std::memory_order_acq_rel)) {
    return;
  }

  if (pipeline_) {
    try {
      pipeline_->stop();
    } catch (const ob::Error &e) {
      RCLCPP_WARN(
          get_logger(),
          "Failed to stop pipeline cleanly: function=%s args=%s message=%s type=%d",
          e.getFunction(),
          e.getArgs(),
          e.what(),
          e.getExceptionType());
    }
  }
}

std::shared_ptr<ob::Device> OrbbecCameraNode::createDeviceBySerial(
    const std::string &serial_number)
{
  auto device_list = context_->queryDeviceList();
  if (device_list->getCount() == 0U) {
    throw std::runtime_error("No Orbbec devices detected.");
  }

  try {
    return device_list->getDeviceBySN(serial_number.c_str());
  } catch (const ob::Error &e) {
    throw std::runtime_error(
        "Failed to open Orbbec device with serial '" + serial_number + "': " +
        e.what());
  }
}

std::shared_ptr<ob::Config> OrbbecCameraNode::createPipelineConfig() const
{
  auto config = std::make_shared<ob::Config>();
  const OBFormat color_format = parseColorFormat(config_.color_encoding);

  auto profile_list = pipeline_->getStreamProfileList(OB_SENSOR_COLOR);
  auto color_profile = profile_list->getVideoStreamProfile(
      static_cast<int>(config_.color_width),
      static_cast<int>(config_.color_height),
      color_format,
      static_cast<int>(config_.color_fps));

  config->enableStream(color_profile);
  config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION);
  return config;
}

void OrbbecCameraNode::handleFrameSet(const std::shared_ptr<ob::FrameSet> &frame_set)
{
  if (!frame_set) {
    return;
  }

  if (color_publisher_->get_subscription_count() == 0U &&
      color_publisher_->get_intra_process_subscription_count() == 0U) {
    return;
  }

  auto color_frame = frame_set->getColorFrame();
  if (!color_frame) {
    return;
  }

  publishColorFrame(color_frame);
}

void OrbbecCameraNode::publishColorFrame(
    const std::shared_ptr<ob::ColorFrame> &color_frame)
{
  const OBFormat format = color_frame->getFormat();
  const std::string encoding = rosEncodingFromFormat(format);
  const size_t bpp = bytesPerPixel(format);
  const uint32_t width = color_frame->getWidth();
  const uint32_t height = color_frame->getHeight();
  const uint32_t data_size = color_frame->getDataSize();
  const size_t expected_size = static_cast<size_t>(width) * static_cast<size_t>(height) * bpp;

  if (data_size < expected_size) {
    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Dropped color frame with invalid size: got=%u expected>=%zu",
        data_size,
        expected_size);
    return;
  }

  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  msg->header.stamp = to_ros_time(color_frame, *get_clock());
  msg->header.frame_id = config_.frame_id;
  msg->height = height;
  msg->width = width;
  msg->encoding = encoding;
  msg->is_bigendian = false;
  msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(width * bpp);
  msg->data.resize(data_size);
  std::memcpy(msg->data.data(), color_frame->getData(), data_size);

  color_publisher_->publish(std::move(msg));
}

OBFormat OrbbecCameraNode::parseColorFormat(const std::string &encoding)
{
  if (encoding == "rgb8") {
    return OB_FORMAT_RGB;
  }
  if (encoding == "bgr8") {
    return OB_FORMAT_BGR;
  }
  if (encoding == "mono8") {
    return OB_FORMAT_Y8;
  }

  throw std::runtime_error(
      "Unsupported color_encoding '" + encoding +
      "'. Supported values: rgb8, bgr8, mono8.");
}

std::string OrbbecCameraNode::rosEncodingFromFormat(OBFormat format)
{
  switch (format) {
    case OB_FORMAT_RGB:
      return "rgb8";
    case OB_FORMAT_BGR:
      return "bgr8";
    case OB_FORMAT_Y8:
      return "mono8";
    default:
      throw std::runtime_error(
          "Unsupported Orbbec frame format for direct ROS publish: " +
          std::to_string(static_cast<int>(format)));
  }
}

size_t OrbbecCameraNode::bytesPerPixel(OBFormat format)
{
  switch (format) {
    case OB_FORMAT_RGB:
    case OB_FORMAT_BGR:
      return 3U;
    case OB_FORMAT_Y8:
      return 1U;
    default:
      throw std::runtime_error(
          "Unsupported Orbbec frame format for bytes_per_pixel: " +
          std::to_string(static_cast<int>(format)));
  }
}

}  // namespace camera_system
