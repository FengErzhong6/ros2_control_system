#include "manus_system/manus_raw_publisher_node.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>

#include "ManusSDKTypeInitializers.h"

#include "geometry_msgs/msg/pose.hpp"

#include "manus_system/msg/manus_glove_raw.hpp"
#include "manus_system/msg/manus_raw_node.hpp"

namespace manus_system {

using namespace std::chrono_literals;

ManusRawPublisherNode *ManusRawPublisherNode::instance_ = nullptr;

ManusRawPublisherNode::ManusRawPublisherNode(const rclcpp::NodeOptions &options)
: Node("manus_raw_publisher_node", options)
{
  if (instance_ != nullptr) {
    throw std::runtime_error("Only one manus_raw_publisher_node instance is supported.");
  }
  instance_ = this;

  declareAndLoadParameters();
  initializeSdk();

  gloves_raw_publisher_ =
      create_publisher<manus_system::msg::ManusGloveRawArray>("~/gloves_raw", 10);

  const auto publish_period =
      std::chrono::duration<double>(1.0 / std::max(publish_rate_hz_, 1.0));
  publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
      std::bind(&ManusRawPublisherNode::publishCallback, this));

  RCLCPP_INFO(
      get_logger(),
      "MANUS raw publisher node ready. publish_rate=%.2fHz",
      publish_rate_hz_);
}

ManusRawPublisherNode::~ManusRawPublisherNode()
{
  shutdownSdk();
  instance_ = nullptr;
}

void ManusRawPublisherNode::declareAndLoadParameters()
{
  publish_rate_hz_ = declare_parameter<double>("publish_rate", 120.0);
  world_space_ = declare_parameter<bool>("world_space", true);
  unit_scale_ = static_cast<float>(declare_parameter<double>("unit_scale", 1.0));
  const auto hand_motion_name =
      declare_parameter<std::string>("hand_motion", "none");

  if (hand_motion_name == "auto") {
    hand_motion_ = HandMotion_Auto;
  } else if (hand_motion_name == "imu") {
    hand_motion_ = HandMotion_IMU;
  } else {
    hand_motion_ = HandMotion_None;
  }
}

void ManusRawPublisherNode::initializeSdk()
{
  const SDKReturnCode initialize_result = CoreSdk_InitializeIntegrated();
  if (initialize_result != SDKReturnCode_Success) {
    throw std::runtime_error(
        "CoreSdk_InitializeIntegrated failed: " + sdkReturnCodeToString(initialize_result));
  }

  registerCallbacks();

  CoordinateSystemVUH coordinate_system;
  CoordinateSystemVUH_Init(&coordinate_system);
  coordinate_system.handedness = Side_Right;
  coordinate_system.up = AxisPolarity_PositiveZ;
  coordinate_system.view = AxisView_XFromViewer;
  coordinate_system.unitScale = unit_scale_;

  const SDKReturnCode coordinate_result =
      CoreSdk_InitializeCoordinateSystemWithVUH(coordinate_system, world_space_);
  if (coordinate_result != SDKReturnCode_Success) {
    CoreSdk_ShutDown();
    throw std::runtime_error(
        "CoreSdk_InitializeCoordinateSystemWithVUH failed: " +
        sdkReturnCodeToString(coordinate_result));
  }

  connectIntegrated();

  const SDKReturnCode hand_motion_result =
      CoreSdk_SetRawSkeletonHandMotion(hand_motion_);
  if (hand_motion_result != SDKReturnCode_Success) {
    RCLCPP_WARN(
        get_logger(),
        "CoreSdk_SetRawSkeletonHandMotion failed: %s",
        sdkReturnCodeToString(hand_motion_result).c_str());
  }
}

void ManusRawPublisherNode::shutdownSdk()
{
  const SDKReturnCode shutdown_result = CoreSdk_ShutDown();
  if (shutdown_result != SDKReturnCode_Success) {
    RCLCPP_WARN(
        get_logger(),
        "CoreSdk_ShutDown returned %s",
        sdkReturnCodeToString(shutdown_result).c_str());
  }
  integrated_connected_ = false;
}

void ManusRawPublisherNode::registerCallbacks()
{
  SDKReturnCode result = CoreSdk_RegisterCallbackForOnConnect(*onConnectedCallback);
  if (result != SDKReturnCode_Success) {
    throw std::runtime_error(
        "CoreSdk_RegisterCallbackForOnConnect failed: " + sdkReturnCodeToString(result));
  }

  result = CoreSdk_RegisterCallbackForOnDisconnect(*onDisconnectedCallback);
  if (result != SDKReturnCode_Success) {
    throw std::runtime_error(
        "CoreSdk_RegisterCallbackForOnDisconnect failed: " + sdkReturnCodeToString(result));
  }

  result = CoreSdk_RegisterCallbackForLandscapeStream(*onLandscapeCallback);
  if (result != SDKReturnCode_Success) {
    throw std::runtime_error(
        "CoreSdk_RegisterCallbackForLandscapeStream failed: " + sdkReturnCodeToString(result));
  }

  result = CoreSdk_RegisterCallbackForRawSkeletonStream(*onRawSkeletonStreamCallback);
  if (result != SDKReturnCode_Success) {
    throw std::runtime_error(
        "CoreSdk_RegisterCallbackForRawSkeletonStream failed: " + sdkReturnCodeToString(result));
  }
}

void ManusRawPublisherNode::connectIntegrated()
{
  ManusHost host;
  ManusHost_Init(&host);

  const SDKReturnCode connect_result = CoreSdk_ConnectToHost(host);
  if (connect_result != SDKReturnCode_Success) {
    throw std::runtime_error(
        "CoreSdk_ConnectToHost(integrated) failed: " + sdkReturnCodeToString(connect_result));
  }

  integrated_connected_ = true;
}

void ManusRawPublisherNode::onConnectedCallback(const ManusHost * /*host*/)
{
  if (instance_ == nullptr) {
    return;
  }
  instance_->integrated_connected_ = true;
  RCLCPP_INFO(instance_->get_logger(), "Connected to MANUS SDK in integrated mode.");
}

void ManusRawPublisherNode::onDisconnectedCallback(const ManusHost * /*host*/)
{
  if (instance_ == nullptr) {
    return;
  }
  instance_->integrated_connected_ = false;
  RCLCPP_WARN(instance_->get_logger(), "Disconnected from MANUS SDK.");
}

void ManusRawPublisherNode::onLandscapeCallback(const Landscape *landscape)
{
  if (instance_ == nullptr || landscape == nullptr) {
    return;
  }

  auto latest = std::make_unique<Landscape>(*landscape);
  {
    std::lock_guard<std::mutex> lock(instance_->landscape_mutex_);
    instance_->landscape_ = std::move(latest);
  }
}

void ManusRawPublisherNode::onRawSkeletonStreamCallback(const SkeletonStreamInfo *stream_info)
{
  if (instance_ == nullptr || stream_info == nullptr) {
    return;
  }

  std::lock_guard<std::mutex> lock(instance_->raw_skeleton_mutex_);
  for (uint32_t i = 0; i < stream_info->skeletonsCount; ++i) {
    RawSkeletonSample sample;
    RawSkeletonInfo_Init(&sample.info);

    SDKReturnCode result = CoreSdk_GetRawSkeletonInfo(i, &sample.info);
    if (result != SDKReturnCode_Success) {
      continue;
    }

    sample.info.publishTime = stream_info->publishTime;
    sample.nodes.resize(sample.info.nodesCount);
    result = CoreSdk_GetRawSkeletonData(i, sample.nodes.data(), sample.info.nodesCount);
    if (result != SDKReturnCode_Success) {
      continue;
    }

    instance_->raw_skeleton_map_.insert_or_assign(sample.info.gloveId, std::move(sample));
  }
}

std::optional<Landscape> ManusRawPublisherNode::copyLandscape() const
{
  std::lock_guard<std::mutex> lock(landscape_mutex_);
  if (!landscape_) {
    return std::nullopt;
  }
  return *landscape_;
}

std::string ManusRawPublisherNode::resolveSide(uint32_t glove_id) const
{
  auto landscape_copy = copyLandscape();
  if (!landscape_copy.has_value()) {
    return "unknown";
  }

  for (uint32_t i = 0; i < landscape_copy->gloveDevices.gloveCount; ++i) {
    const auto &glove = landscape_copy->gloveDevices.gloves[i];
    if (glove.id == glove_id) {
      return sideToString(glove.side);
    }
  }

  return "unknown";
}

std::vector<NodeInfo> ManusRawPublisherNode::getNodeInfoArray(uint32_t glove_id, uint32_t node_count)
{
  {
    std::lock_guard<std::mutex> lock(node_info_mutex_);
    auto it = node_info_cache_.find(glove_id);
    if (it != node_info_cache_.end() && it->second.size() == node_count) {
      return it->second;
    }
  }

  std::vector<NodeInfo> node_infos(node_count);
  const SDKReturnCode result =
      CoreSdk_GetRawSkeletonNodeInfoArray(glove_id, node_infos.data(), node_count);
  if (result != SDKReturnCode_Success) {
    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "CoreSdk_GetRawSkeletonNodeInfoArray failed for glove %u: %s",
        glove_id,
        sdkReturnCodeToString(result).c_str());
    return {};
  }

  {
    std::lock_guard<std::mutex> lock(node_info_mutex_);
    node_info_cache_.insert_or_assign(glove_id, node_infos);
  }
  return node_infos;
}

void ManusRawPublisherNode::publishCallback()
{
  if (gloves_raw_publisher_->get_subscription_count() == 0U &&
      gloves_raw_publisher_->get_intra_process_subscription_count() == 0U) {
    return;
  }

  std::map<uint32_t, RawSkeletonSample> raw_skeleton_map_copy;
  {
    std::lock_guard<std::mutex> lock(raw_skeleton_mutex_);
    raw_skeleton_map_copy = raw_skeleton_map_;
  }

  if (raw_skeleton_map_copy.empty()) {
    return;
  }

  manus_system::msg::ManusGloveRawArray array_msg;
  array_msg.header.stamp = now();
  array_msg.header.frame_id = world_space_ ? "world" : "manus_local";
  array_msg.gloves.reserve(raw_skeleton_map_copy.size());

  for (const auto &[glove_id, sample] : raw_skeleton_map_copy) {
    if (sample.info.nodesCount == 0U || sample.nodes.empty()) {
      continue;
    }

    const auto node_infos = getNodeInfoArray(glove_id, sample.info.nodesCount);
    if (node_infos.size() != sample.info.nodesCount) {
      continue;
    }

    manus_system::msg::ManusGloveRaw glove_msg;
    glove_msg.glove_id = glove_id;
    glove_msg.side = resolveSide(glove_id);
    glove_msg.raw_node_count = sample.info.nodesCount;
    glove_msg.raw_nodes.reserve(sample.nodes.size());

    for (size_t node_index = 0; node_index < sample.nodes.size(); ++node_index) {
      const auto &node = sample.nodes[node_index];
      const auto &node_info = node_infos[node_index];

      manus_system::msg::ManusRawNode raw_node_msg;
      raw_node_msg.node_id = node.id;
      raw_node_msg.parent_node_id = node_info.parentId;
      raw_node_msg.joint_type = jointTypeToString(node_info.fingerJointType);
      raw_node_msg.chain_type = chainTypeToString(node_info.chainType);

      geometry_msgs::msg::Pose pose_msg;
      pose_msg.position.x = node.transform.position.x;
      pose_msg.position.y = node.transform.position.y;
      pose_msg.position.z = node.transform.position.z;
      pose_msg.orientation.x = node.transform.rotation.x;
      pose_msg.orientation.y = node.transform.rotation.y;
      pose_msg.orientation.z = node.transform.rotation.z;
      pose_msg.orientation.w = node.transform.rotation.w;
      raw_node_msg.pose = std::move(pose_msg);

      glove_msg.raw_nodes.push_back(std::move(raw_node_msg));
    }

    array_msg.gloves.push_back(std::move(glove_msg));
  }

  if (!array_msg.gloves.empty()) {
    gloves_raw_publisher_->publish(std::move(array_msg));
  }
}

std::string ManusRawPublisherNode::sideToString(Side side)
{
  switch (side) {
    case Side_Left:
      return "left";
    case Side_Right:
      return "right";
    default:
      return "invalid";
  }
}

std::string ManusRawPublisherNode::jointTypeToString(FingerJointType joint_type)
{
  switch (joint_type) {
    case FingerJointType_Metacarpal:
      return "MCP";
    case FingerJointType_Proximal:
      return "PIP";
    case FingerJointType_Intermediate:
      return "IP";
    case FingerJointType_Distal:
      return "DIP";
    case FingerJointType_Tip:
      return "TIP";
    default:
      return "Invalid";
  }
}

std::string ManusRawPublisherNode::chainTypeToString(ChainType chain_type)
{
  switch (chain_type) {
    case ChainType_Arm:
      return "Arm";
    case ChainType_Hand:
      return "Hand";
    case ChainType_FingerThumb:
      return "Thumb";
    case ChainType_FingerIndex:
      return "Index";
    case ChainType_FingerMiddle:
      return "Middle";
    case ChainType_FingerRing:
      return "Ring";
    case ChainType_FingerPinky:
      return "Pinky";
    default:
      return "Invalid";
  }
}

std::string ManusRawPublisherNode::sdkReturnCodeToString(SDKReturnCode code)
{
  return std::to_string(static_cast<int32_t>(code));
}

}  // namespace manus_system
