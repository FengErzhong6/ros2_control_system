#include "manus_system/manus_raw_publisher_node.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <utility>

#include "ManusSDKTypeInitializers.h"

#include "geometry_msgs/msg/pose.hpp"

#include "manus_system/msg/manus_glove_raw.hpp"
#include "manus_system/msg/manus_raw_node.hpp"

namespace manus_system {

using namespace std::chrono_literals;

namespace {

std::string toLowerCopy(std::string value)
{
  std::transform(
      value.begin(),
      value.end(),
      value.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

std::string trimCopy(const std::string &value)
{
  const auto begin = std::find_if_not(
      value.begin(), value.end(), [](unsigned char c) { return std::isspace(c) != 0; });
  const auto end = std::find_if_not(
      value.rbegin(), value.rend(), [](unsigned char c) { return std::isspace(c) != 0; })
                       .base();
  if (begin >= end) {
    return "";
  }
  return std::string(begin, end);
}

bool isValidUserName(const std::string &user_name)
{
  if (user_name.empty() || user_name == "." || user_name == "..") {
    return false;
  }

  return std::all_of(user_name.begin(), user_name.end(), [](unsigned char c) {
    return std::isalnum(c) != 0 || c == '_' || c == '-' || c == '.';
  });
}

std::string resolveUserCalibrationDirectory(
    const std::string &base_directory,
    const std::string &user_name)
{
  const std::string normalized_user_name = trimCopy(user_name);
  if (!isValidUserName(normalized_user_name)) {
    throw std::runtime_error(
        "Parameter 'user_name' must be set to a simple directory name "
        "containing letters, digits, '_', '-' or '.'.");
  }

  return (std::filesystem::path(base_directory) / normalized_user_name).string();
}

bool shouldSuppressSdkLogMessage(
    const std::string &message,
    bool suppress_statistics_logs)
{
  const std::string lower_message = toLowerCopy(message);
  if (lower_message.find("average frame duration") != std::string::npos ||
      lower_message.find("average data delay") != std::string::npos) {
    return true;
  }

  if (!suppress_statistics_logs) {
    return false;
  }

  return lower_message.find("devices datarate") != std::string::npos ||
         lower_message.find("data is discarded") != std::string::npos;
}

bool isSuccessSetCalibration(SetGloveCalibrationReturnCode code)
{
  return code == SetGloveCalibrationReturnCode_Success;
}

}  // namespace

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
      "MANUS raw publisher node ready for user '%s'. publish_rate=%.2fHz, calibration_directory=%s",
      user_name_.c_str(),
      publish_rate_hz_,
      calibration_directory_.c_str());
}

ManusRawPublisherNode::~ManusRawPublisherNode()
{
  shutdownSdk();
  instance_ = nullptr;
}

void ManusRawPublisherNode::declareAndLoadParameters()
{
  publish_rate_hz_ = declare_parameter<double>("publish_rate", 120.0);
#ifdef MANUS_SYSTEM_SOURCE_DIR
  const std::string default_calibration_dir =
      std::string(MANUS_SYSTEM_SOURCE_DIR) + "/bringup/calibrations";
#else
  const std::string default_calibration_dir = "./bringup/calibrations";
#endif

  calibration_directory_ =
      declare_parameter<std::string>("calibration_directory", default_calibration_dir);
  user_name_ = declare_parameter<std::string>("user_name", "");
  calibration_directory_ = resolveUserCalibrationDirectory(calibration_directory_, user_name_);
  auto_load_calibrations_ =
      declare_parameter<bool>("auto_load_calibrations", true);
  auto_load_left_calibration_file_ =
      declare_parameter<std::string>("auto_load_left_calibration_file", "left_glove.mcal");
  auto_load_right_calibration_file_ =
      declare_parameter<std::string>("auto_load_right_calibration_file", "right_glove.mcal");
  suppress_sdk_statistics_logs_ =
      declare_parameter<bool>("suppress_sdk_statistics_logs", true);
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

  const auto sdk_log_path =
      (std::filesystem::temp_directory_path() / "manus_raw_publisher_sdk.log").string();
  const SDKReturnCode log_location_result = CoreSdk_SetLogLocation(sdk_log_path.c_str());
  if (log_location_result != SDKReturnCode_Success) {
    RCLCPP_WARN(
        get_logger(),
        "CoreSdk_SetLogLocation failed: %s",
        sdkReturnCodeToString(log_location_result).c_str());
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

  result = CoreSdk_RegisterCallbackForOnLog(*onLogCallback);
  if (result != SDKReturnCode_Success) {
    throw std::runtime_error(
        "CoreSdk_RegisterCallbackForOnLog failed: " + sdkReturnCodeToString(result));
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

void ManusRawPublisherNode::onLogCallback(
    LogSeverity severity,
    const char *log,
    uint32_t length)
{
  if (instance_ == nullptr || log == nullptr) {
    return;
  }

  std::string message;
  if (length > 0U) {
    message.assign(log, log + length);
  } else {
    message = log;
  }

  if (shouldSuppressSdkLogMessage(message, instance_->suppress_sdk_statistics_logs_)) {
    return;
  }

  switch (severity) {
    case LogSeverity_Debug:
      RCLCPP_DEBUG(instance_->get_logger(), "MANUS SDK: %s", message.c_str());
      break;
    case LogSeverity_Info:
      RCLCPP_INFO(instance_->get_logger(), "MANUS SDK: %s", message.c_str());
      break;
    case LogSeverity_Warn:
      RCLCPP_WARN(instance_->get_logger(), "MANUS SDK: %s", message.c_str());
      break;
    case LogSeverity_Error:
    default:
      RCLCPP_ERROR(instance_->get_logger(), "MANUS SDK: %s", message.c_str());
      break;
  }
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

  instance_->maybeAutoLoadCalibrations();
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
  {
    std::lock_guard<std::mutex> lock(fatal_error_mutex_);
    if (fatal_calibration_error_) {
      RCLCPP_ERROR(get_logger(), "%s", fatal_calibration_error_message_.c_str());
      rclcpp::shutdown();
      return;
    }
  }

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

void ManusRawPublisherNode::maybeAutoLoadCalibrations()
{
  if (!auto_load_calibrations_) {
    return;
  }

  auto landscape_copy = copyLandscape();
  if (!landscape_copy.has_value()) {
    return;
  }

  for (uint32_t i = 0; i < landscape_copy->gloveDevices.gloveCount; ++i) {
    const auto &glove = landscape_copy->gloveDevices.gloves[i];
    if (glove.side != Side_Left && glove.side != Side_Right) {
      continue;
    }
    if (glove.pairedState != DevicePairedState_Paired) {
      continue;
    }

    const std::string configured_file = configuredCalibrationFileForSide(glove.side);
    if (configured_file.empty()) {
      setFatalCalibrationError(
          "Missing configured calibration file name for side '" + sideToString(glove.side) + "'.");
      return;
    }

    const std::string attempt_key =
        sideToString(glove.side) + ":" + std::to_string(glove.id) + ":" + configured_file;
    {
      std::lock_guard<std::mutex> lock(auto_load_mutex_);
      if (auto_load_attempt_keys_.count(attempt_key) != 0U) {
        continue;
      }
      auto_load_attempt_keys_.insert(attempt_key);
    }

    std::string resolved_path;
    std::string error_message;
    if (!loadCalibrationForGlove(glove.id, configured_file, &resolved_path, &error_message)) {
      setFatalCalibrationError(
          "Failed to auto-load calibration for user '" + user_name_ + "', glove " +
          std::to_string(glove.id) + " (" + sideToString(glove.side) + "): " + error_message);
      return;
    }

    RCLCPP_INFO(
        get_logger(),
        "Auto-loaded calibration for glove %u (%s) from %s",
        glove.id,
        sideToString(glove.side).c_str(),
        resolved_path.c_str());
  }
}

bool ManusRawPublisherNode::loadCalibrationForGlove(
    uint32_t glove_id,
    const std::string &file_path,
    std::string *resolved_path,
    std::string *error_message) const
{
  if (resolved_path == nullptr || error_message == nullptr) {
    return false;
  }

  *resolved_path = resolveCalibrationPath(file_path);

  std::ifstream input(*resolved_path, std::ios::binary);
  if (!input) {
    *error_message = "Calibration file does not exist: " + *resolved_path;
    return false;
  }

  input.seekg(0, std::ios::end);
  const std::streamsize file_size = input.tellg();
  input.seekg(0, std::ios::beg);
  if (file_size <= 0) {
    *error_message = "Calibration file is empty: " + *resolved_path;
    return false;
  }

  std::vector<unsigned char> bytes(static_cast<size_t>(file_size));
  input.read(reinterpret_cast<char *>(bytes.data()), file_size);
  if (!input) {
    *error_message = "Failed to read calibration file: " + *resolved_path;
    return false;
  }

  SetGloveCalibrationReturnCode set_result = SetGloveCalibrationReturnCode_Error;
  const SDKReturnCode result = CoreSdk_SetGloveCalibration(
      glove_id,
      bytes.data(),
      static_cast<uint32_t>(bytes.size()),
      &set_result);
  if (result != SDKReturnCode_Success || !isSuccessSetCalibration(set_result)) {
    *error_message =
        "Failed to load calibration into glove: sdk=" + sdkReturnCodeToString(result) +
        ", set_result=" + std::to_string(static_cast<int32_t>(set_result));
    return false;
  }

  return true;
}

void ManusRawPublisherNode::setFatalCalibrationError(const std::string &message)
{
  std::lock_guard<std::mutex> lock(fatal_error_mutex_);
  if (fatal_calibration_error_) {
    return;
  }

  fatal_calibration_error_ = true;
  fatal_calibration_error_message_ = message;
}

std::string ManusRawPublisherNode::configuredCalibrationFileForSide(Side side) const
{
  switch (side) {
    case Side_Left:
      return auto_load_left_calibration_file_;
    case Side_Right:
      return auto_load_right_calibration_file_;
    default:
      return "";
  }
}

std::string ManusRawPublisherNode::resolveCalibrationPath(const std::string &file_path) const
{
  const std::filesystem::path candidate(file_path);
  if (candidate.is_absolute()) {
    return candidate.string();
  }
  return (std::filesystem::path(calibration_directory_) / candidate).string();
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

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<manus_system::ManusRawPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
