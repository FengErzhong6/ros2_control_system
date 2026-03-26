#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
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

  static void onConnectedCallback(const ManusHost *host);
  static void onDisconnectedCallback(const ManusHost *host);
  static void onLandscapeCallback(const Landscape *landscape);
  static void onRawSkeletonStreamCallback(const SkeletonStreamInfo *stream_info);

  std::optional<Landscape> copyLandscape() const;
  std::string resolveSide(uint32_t glove_id) const;
  std::vector<NodeInfo> getNodeInfoArray(uint32_t glove_id, uint32_t node_count);

  static std::string sideToString(Side side);
  static std::string jointTypeToString(FingerJointType joint_type);
  static std::string chainTypeToString(ChainType chain_type);
  static std::string sdkReturnCodeToString(SDKReturnCode code);

  bool integrated_connected_{false};
  double publish_rate_hz_{120.0};
  bool world_space_{true};
  float unit_scale_{1.0F};
  HandMotion hand_motion_{HandMotion_None};

  mutable std::mutex raw_skeleton_mutex_;
  std::map<uint32_t, RawSkeletonSample> raw_skeleton_map_;

  mutable std::mutex node_info_mutex_;
  std::map<uint32_t, std::vector<NodeInfo>> node_info_cache_;

  mutable std::mutex landscape_mutex_;
  std::unique_ptr<Landscape> landscape_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Publisher<manus_system::msg::ManusGloveRawArray>::SharedPtr gloves_raw_publisher_;

  static ManusRawPublisherNode *instance_;
};

}  // namespace manus_system

