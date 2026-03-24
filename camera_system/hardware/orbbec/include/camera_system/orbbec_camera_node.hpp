#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <libobsensor/ObSensor.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace camera_system {

struct OrbbecColorConfig {
  std::string camera_name;
  std::string serial_number;
  std::string frame_id;
  std::string image_topic;
  std::string color_encoding;
  uint32_t color_width;
  uint32_t color_height;
  uint32_t color_fps;
};

class OrbbecCameraNode : public rclcpp::Node {
public:
  explicit OrbbecCameraNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~OrbbecCameraNode() override;

private:
  void declareAndLoadParameters();
  void startPipeline();
  void stopPipeline();

  std::shared_ptr<ob::Device> createDeviceBySerial(const std::string &serial_number);
  std::shared_ptr<ob::Config> createPipelineConfig() const;

  void handleFrameSet(const std::shared_ptr<ob::FrameSet> &frame_set);
  void publishColorFrame(const std::shared_ptr<ob::ColorFrame> &color_frame);

  static OBFormat parseColorFormat(const std::string &encoding);
  static std::string rosEncodingFromFormat(OBFormat format);
  static size_t bytesPerPixel(OBFormat format);

  OrbbecColorConfig config_{};
  std::shared_ptr<ob::Context> context_;
  std::shared_ptr<ob::Device> device_;
  std::shared_ptr<ob::Pipeline> pipeline_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_publisher_;
  std::atomic<bool> pipeline_running_{false};
};

}  // namespace camera_system
