#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include "ManusSDK.h"

#include "rclcpp/rclcpp.hpp"

#include "manus_system/msg/manus_glove_raw_array.hpp"

namespace manus_system {

class ManusRawPublisherNode : public rclcpp::Node {
public:
  explicit ManusRawPublisherNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~ManusRawPublisherNode() override;

private:
  struct RawSkeletonSample {
    RawSkeletonInfo info{};
    std::vector<SkeletonNode> nodes;
  };

  void declareAndLoadParameters();
  void initializeSdk();
  void shutdownSdk();
  void registerCallbacks();
  void connectIntegrated();
  void publishCallback();
  void maybeAutoLoadCalibrations();
  bool loadCalibrationForGlove(
      uint32_t glove_id,
      const std::string &file_path,
      std::string *resolved_path,
      std::string *error_message) const;
  void setFatalCalibrationError(const std::string &message);

  static void onConnectedCallback(const ManusHost *host);
  static void onDisconnectedCallback(const ManusHost *host);
  static void onLogCallback(LogSeverity severity, const char *log, uint32_t length);
  static void onLandscapeCallback(const Landscape *landscape);
  static void onRawSkeletonStreamCallback(const SkeletonStreamInfo *stream_info);

  std::optional<Landscape> copyLandscape() const;
  std::string resolveSide(uint32_t glove_id) const;
  std::vector<NodeInfo> getNodeInfoArray(uint32_t glove_id, uint32_t node_count);
  std::string configuredCalibrationFileForSide(Side side) const;
  std::string resolveCalibrationPath(const std::string &file_path) const;

  static std::string sideToString(Side side);
  static std::string jointTypeToString(FingerJointType joint_type);
  static std::string chainTypeToString(ChainType chain_type);
  static std::string sdkReturnCodeToString(SDKReturnCode code);

  bool integrated_connected_{false};
  double publish_rate_hz_{120.0};
  std::string calibration_directory_;
  std::string user_name_;
  bool auto_load_calibrations_{false};
  std::string auto_load_left_calibration_file_;
  std::string auto_load_right_calibration_file_;
  bool suppress_sdk_statistics_logs_{true};
  bool world_space_{true};
  float unit_scale_{1.0F};
  HandMotion hand_motion_{HandMotion_None};

  mutable std::mutex raw_skeleton_mutex_;
  std::map<uint32_t, RawSkeletonSample> raw_skeleton_map_;

  mutable std::mutex node_info_mutex_;
  std::map<uint32_t, std::vector<NodeInfo>> node_info_cache_;

  mutable std::mutex landscape_mutex_;
  std::unique_ptr<Landscape> landscape_;

  mutable std::mutex auto_load_mutex_;
  std::unordered_set<std::string> auto_load_attempt_keys_;

  mutable std::mutex fatal_error_mutex_;
  bool fatal_calibration_error_{false};
  std::string fatal_calibration_error_message_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Publisher<manus_system::msg::ManusGloveRawArray>::SharedPtr gloves_raw_publisher_;

  static ManusRawPublisherNode *instance_;
};

}  // namespace manus_system
