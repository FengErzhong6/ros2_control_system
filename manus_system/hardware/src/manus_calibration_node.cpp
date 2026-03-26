#include "manus_system/manus_calibration_node.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include "ManusSDKTypeInitializers.h"

namespace manus_system {

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

bool isSuccessSetCalibration(SetGloveCalibrationReturnCode code)
{
  return code == SetGloveCalibrationReturnCode_Success;
}

}  // namespace

ManusCalibrationNode *ManusCalibrationNode::instance_ = nullptr;

ManusCalibrationNode::ManusCalibrationNode(const rclcpp::NodeOptions &options)
: Node("manus_calibration_node", options)
{
  if (instance_ != nullptr) {
    throw std::runtime_error("Only one manus_calibration_node instance is supported.");
  }
  instance_ = this;

  declareAndLoadParameters();
  initializeSdk();

  list_gloves_service_ = create_service<ListGloves>(
      "~/list_gloves",
      std::bind(&ManusCalibrationNode::handleListGloves, this, std::placeholders::_1, std::placeholders::_2));
  start_calibration_service_ = create_service<StartGloveCalibration>(
      "~/start_calibration",
      std::bind(&ManusCalibrationNode::handleStartCalibration, this, std::placeholders::_1, std::placeholders::_2));
  run_step_service_ = create_service<RunGloveCalibrationStep>(
      "~/run_calibration_step",
      std::bind(&ManusCalibrationNode::handleRunCalibrationStep, this, std::placeholders::_1, std::placeholders::_2));
  cancel_calibration_service_ = create_service<CancelGloveCalibration>(
      "~/cancel_calibration",
      std::bind(&ManusCalibrationNode::handleCancelCalibration, this, std::placeholders::_1, std::placeholders::_2));
  finish_calibration_service_ = create_service<FinishGloveCalibration>(
      "~/finish_calibration",
      std::bind(&ManusCalibrationNode::handleFinishCalibration, this, std::placeholders::_1, std::placeholders::_2));
  load_calibration_service_ = create_service<LoadGloveCalibration>(
      "~/load_calibration",
      std::bind(&ManusCalibrationNode::handleLoadCalibration, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
      get_logger(),
      "MANUS calibration node ready. Calibration files will be stored in: %s",
      calibration_directory_.c_str());
}

ManusCalibrationNode::~ManusCalibrationNode()
{
  shutdownSdk();
  instance_ = nullptr;
}

void ManusCalibrationNode::declareAndLoadParameters()
{
#ifdef MANUS_SYSTEM_SOURCE_DIR
  const std::string default_calibration_dir =
      std::string(MANUS_SYSTEM_SOURCE_DIR) + "/bringup/calibrations";
#else
  const std::string default_calibration_dir = "./bringup/calibrations";
#endif

  calibration_directory_ =
      declare_parameter<std::string>("calibration_directory", default_calibration_dir);
  default_glove_side_ =
      declare_parameter<std::string>("default_glove_side", "left");
  world_space_ =
      declare_parameter<bool>("world_space", true);
  unit_scale_ =
      static_cast<float>(declare_parameter<double>("unit_scale", 1.0));
}

void ManusCalibrationNode::initializeSdk()
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
}

void ManusCalibrationNode::shutdownSdk()
{
  if (!integrated_connected_ && instance_ == nullptr) {
    return;
  }

  const SDKReturnCode shutdown_result = CoreSdk_ShutDown();
  if (shutdown_result != SDKReturnCode_Success) {
    RCLCPP_WARN(
        get_logger(),
        "CoreSdk_ShutDown returned %s",
        sdkReturnCodeToString(shutdown_result).c_str());
  }
  integrated_connected_ = false;
}

void ManusCalibrationNode::registerCallbacks()
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
}

void ManusCalibrationNode::connectIntegrated()
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

void ManusCalibrationNode::onConnectedCallback(const ManusHost * /*host*/)
{
  if (instance_ == nullptr) {
    return;
  }
  RCLCPP_INFO(instance_->get_logger(), "Connected to MANUS SDK in integrated mode.");
}

void ManusCalibrationNode::onDisconnectedCallback(const ManusHost * /*host*/)
{
  if (instance_ == nullptr) {
    return;
  }
  instance_->integrated_connected_ = false;
  RCLCPP_WARN(instance_->get_logger(), "Disconnected from MANUS SDK.");
}

void ManusCalibrationNode::onLandscapeCallback(const Landscape *landscape)
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

std::optional<Landscape> ManusCalibrationNode::copyLandscape() const
{
  std::lock_guard<std::mutex> lock(landscape_mutex_);
  if (!landscape_) {
    return std::nullopt;
  }
  return *landscape_;
}

std::optional<GloveLandscapeData> ManusCalibrationNode::getGloveById(uint32_t glove_id) const
{
  auto landscape_copy = copyLandscape();
  if (!landscape_copy.has_value()) {
    return std::nullopt;
  }

  for (uint32_t i = 0; i < landscape_copy->gloveDevices.gloveCount; ++i) {
    if (landscape_copy->gloveDevices.gloves[i].id == glove_id) {
      return landscape_copy->gloveDevices.gloves[i];
    }
  }
  return std::nullopt;
}

std::optional<GloveLandscapeData> ManusCalibrationNode::getFirstGloveBySide(const std::string &side_hint) const
{
  auto landscape_copy = copyLandscape();
  if (!landscape_copy.has_value()) {
    return std::nullopt;
  }

  const std::string normalized_side =
      side_hint.empty() ? toLowerCopy(default_glove_side_) : toLowerCopy(side_hint);
  const Side requested_side = normalized_side == "right" ? Side_Right : Side_Left;

  for (uint32_t i = 0; i < landscape_copy->gloveDevices.gloveCount; ++i) {
    if (landscape_copy->gloveDevices.gloves[i].side == requested_side) {
      return landscape_copy->gloveDevices.gloves[i];
    }
  }
  return std::nullopt;
}

std::optional<GloveLandscapeData> ManusCalibrationNode::resolveGlove(
    uint32_t glove_id,
    const std::string &side_hint) const
{
  if (glove_id != 0U) {
    return getGloveById(glove_id);
  }
  return getFirstGloveBySide(side_hint);
}

void ManusCalibrationNode::handleListGloves(
    const std::shared_ptr<ListGloves::Request> /*request*/,
    std::shared_ptr<ListGloves::Response> response)
{
  auto landscape_copy = copyLandscape();
  if (!landscape_copy.has_value()) {
    response->success = false;
    response->message = "No MANUS landscape received yet.";
    return;
  }

  for (uint32_t i = 0; i < landscape_copy->gloveDevices.gloveCount; ++i) {
    const auto &glove = landscape_copy->gloveDevices.gloves[i];
    response->glove_ids.push_back(glove.id);
    response->sides.push_back(sideToString(glove.side));
    response->is_haptics.push_back(glove.isHaptics);
    response->paired_states.push_back(pairedStateToString(glove.pairedState));
  }

  response->success = true;
  response->message = "Listed gloves from latest MANUS landscape.";
}

void ManusCalibrationNode::handleStartCalibration(
    const std::shared_ptr<StartGloveCalibration::Request> request,
    std::shared_ptr<StartGloveCalibration::Response> response)
{
  auto glove = resolveGlove(request->glove_id, request->side);
  if (!glove.has_value()) {
    response->success = false;
    response->message = "Unable to resolve glove. Check glove_id or side and confirm landscape is available.";
    return;
  }

  std::lock_guard<std::mutex> session_lock(session_mutex_);
  if (calibration_session_.active) {
    response->success = false;
    response->message = "A calibration session is already active. Cancel or finish it first.";
    return;
  }

  GloveCalibrationArgs calibration_args;
  GloveCalibrationArgs_Init(&calibration_args);
  calibration_args.gloveId = glove->id;

  uint32_t step_count = 0;
  SDKReturnCode result =
      CoreSdk_GloveCalibrationGetNumberOfSteps(calibration_args, &step_count);
  if (result != SDKReturnCode_Success || step_count == 0U) {
    response->success = false;
    response->message =
        "Failed to query calibration steps: " + sdkReturnCodeToString(result);
    return;
  }

  bool start_ok = false;
  result = CoreSdk_GloveCalibrationStart(calibration_args, &start_ok);
  if (result != SDKReturnCode_Success || !start_ok) {
    response->success = false;
    response->message =
        "Failed to start glove calibration: " + sdkReturnCodeToString(result);
    return;
  }

  calibration_session_.active = true;
  calibration_session_.glove_id = glove->id;
  calibration_session_.steps.clear();
  calibration_session_.steps.reserve(step_count);

  for (uint32_t step_index = 0; step_index < step_count; ++step_index) {
    GloveCalibrationStepArgs step_args;
    GloveCalibrationStepArgs_Init(&step_args);
    step_args.gloveId = glove->id;
    step_args.stepIndex = step_index;

    GloveCalibrationStepData step_data;
    GloveCalibrationStepData_Init(&step_data);

    result = CoreSdk_GloveCalibrationGetStepData(step_args, &step_data);
    if (result != SDKReturnCode_Success) {
      bool stop_ok = false;
      CoreSdk_GloveCalibrationStop(calibration_args, &stop_ok);
      calibration_session_ = CalibrationSession{};
      response->success = false;
      response->message =
          "Calibration started but step metadata retrieval failed: " +
          sdkReturnCodeToString(result);
      return;
    }

    calibration_session_.steps.push_back(step_data);
    response->step_titles.emplace_back(step_data.title);
    response->step_descriptions.emplace_back(step_data.description);
    response->step_durations.push_back(step_data.time);
  }

  response->success = true;
  response->message = "Calibration started successfully.";
  response->resolved_glove_id = glove->id;
  response->step_count = step_count;

  RCLCPP_INFO(
      get_logger(),
      "Started calibration for glove %u (%s) with %u steps.",
      glove->id,
      sideToString(glove->side).c_str(),
      step_count);
}

void ManusCalibrationNode::handleRunCalibrationStep(
    const std::shared_ptr<RunGloveCalibrationStep::Request> request,
    std::shared_ptr<RunGloveCalibrationStep::Response> response)
{
  std::lock_guard<std::mutex> session_lock(session_mutex_);
  if (!calibration_session_.active) {
    response->success = false;
    response->message = "No active calibration session. Start calibration first.";
    return;
  }

  const uint32_t glove_id =
      request->glove_id == 0U ? calibration_session_.glove_id : request->glove_id;
  if (glove_id != calibration_session_.glove_id) {
    response->success = false;
    response->message = "Requested glove_id does not match active calibration session.";
    return;
  }

  if (request->step_index >= calibration_session_.steps.size()) {
    response->success = false;
    response->message = "Requested step index is out of range.";
    return;
  }

  const auto &step_data = calibration_session_.steps[request->step_index];
  response->title = step_data.title;
  response->description = step_data.description;
  response->time_hint = step_data.time;

  GloveCalibrationStepArgs step_args;
  GloveCalibrationStepArgs_Init(&step_args);
  step_args.gloveId = glove_id;
  step_args.stepIndex = request->step_index;

  bool step_ok = false;
  const SDKReturnCode result =
      CoreSdk_GloveCalibrationStartStep(step_args, &step_ok);

  if (result != SDKReturnCode_Success || !step_ok) {
    response->success = false;
    response->message =
        "Calibration step failed: " + sdkReturnCodeToString(result);
    return;
  }

  response->success = true;
  response->message = "Calibration step finished successfully.";

  RCLCPP_INFO(
      get_logger(),
      "Finished calibration step %u for glove %u: %s",
      request->step_index,
      glove_id,
      step_data.title);
}

void ManusCalibrationNode::handleCancelCalibration(
    const std::shared_ptr<CancelGloveCalibration::Request> request,
    std::shared_ptr<CancelGloveCalibration::Response> response)
{
  std::lock_guard<std::mutex> session_lock(session_mutex_);
  if (!calibration_session_.active) {
    response->success = false;
    response->message = "No active calibration session.";
    return;
  }

  const uint32_t glove_id =
      request->glove_id == 0U ? calibration_session_.glove_id : request->glove_id;
  if (glove_id != calibration_session_.glove_id) {
    response->success = false;
    response->message = "Requested glove_id does not match active calibration session.";
    return;
  }

  GloveCalibrationArgs calibration_args;
  GloveCalibrationArgs_Init(&calibration_args);
  calibration_args.gloveId = glove_id;

  bool cancel_ok = false;
  const SDKReturnCode result =
      CoreSdk_GloveCalibrationStop(calibration_args, &cancel_ok);
  if (result != SDKReturnCode_Success || !cancel_ok) {
    response->success = false;
    response->message =
        "Failed to cancel calibration: " + sdkReturnCodeToString(result);
    return;
  }

  calibration_session_ = CalibrationSession{};
  response->success = true;
  response->message = "Calibration cancelled.";
}

void ManusCalibrationNode::handleFinishCalibration(
    const std::shared_ptr<FinishGloveCalibration::Request> request,
    std::shared_ptr<FinishGloveCalibration::Response> response)
{
  uint32_t glove_id = request->glove_id;
  {
    std::lock_guard<std::mutex> session_lock(session_mutex_);
    if (!calibration_session_.active) {
      response->success = false;
      response->message = "No active calibration session.";
      return;
    }

    glove_id = glove_id == 0U ? calibration_session_.glove_id : glove_id;
    if (glove_id != calibration_session_.glove_id) {
      response->success = false;
      response->message = "Requested glove_id does not match active calibration session.";
      return;
    }
  }

  GloveCalibrationArgs calibration_args;
  GloveCalibrationArgs_Init(&calibration_args);
  calibration_args.gloveId = glove_id;

  bool finish_ok = false;
  const SDKReturnCode finish_result =
      CoreSdk_GloveCalibrationFinish(calibration_args, &finish_ok);
  if (finish_result != SDKReturnCode_Success || !finish_ok) {
    response->success = false;
    response->message =
        "Failed to finish calibration: " + sdkReturnCodeToString(finish_result);
    return;
  }

  std::string saved_path;
  if (request->save_to_file) {
    try {
      saved_path = saveCalibration(glove_id, request->file_name);
    } catch (const std::exception &e) {
      response->success = false;
      response->message = e.what();
      return;
    }
  }

  {
    std::lock_guard<std::mutex> session_lock(session_mutex_);
    calibration_session_ = CalibrationSession{};
  }

  response->success = true;
  response->message = request->save_to_file
                          ? "Calibration finished and saved."
                          : "Calibration finished.";
  response->saved_path = saved_path;
}

void ManusCalibrationNode::handleLoadCalibration(
    const std::shared_ptr<LoadGloveCalibration::Request> request,
    std::shared_ptr<LoadGloveCalibration::Response> response)
{
  auto glove = resolveGlove(request->glove_id, "");
  if (!glove.has_value()) {
    response->success = false;
    response->message = "Unable to resolve glove_id for loading calibration.";
    return;
  }

  const std::string resolved_path = resolveCalibrationPath(request->file_path);
  std::ifstream input(resolved_path, std::ios::binary);
  if (!input) {
    response->success = false;
    response->message = "Calibration file does not exist: " + resolved_path;
    return;
  }

  input.seekg(0, std::ios::end);
  const std::streamsize file_size = input.tellg();
  input.seekg(0, std::ios::beg);
  if (file_size <= 0) {
    response->success = false;
    response->message = "Calibration file is empty: " + resolved_path;
    return;
  }

  std::vector<unsigned char> bytes(static_cast<size_t>(file_size));
  input.read(reinterpret_cast<char *>(bytes.data()), file_size);
  if (!input) {
    response->success = false;
    response->message = "Failed to read calibration file: " + resolved_path;
    return;
  }

  SetGloveCalibrationReturnCode set_result = SetGloveCalibrationReturnCode_Error;
  const SDKReturnCode result = CoreSdk_SetGloveCalibration(
      glove->id,
      bytes.data(),
      static_cast<uint32_t>(bytes.size()),
      &set_result);
  if (result != SDKReturnCode_Success || !isSuccessSetCalibration(set_result)) {
    response->success = false;
    response->message =
        "Failed to load calibration into glove: sdk=" + sdkReturnCodeToString(result) +
        ", set_result=" + std::to_string(static_cast<int32_t>(set_result));
    return;
  }

  response->success = true;
  response->message = "Calibration loaded successfully.";
  response->resolved_path = resolved_path;
}

std::string ManusCalibrationNode::saveCalibration(
    uint32_t glove_id,
    const std::string &requested_file_name)
{
  uint32_t size = 0;
  SDKReturnCode result = CoreSdk_GetGloveCalibrationSize(glove_id, &size);
  if (result != SDKReturnCode_Success || size == 0U) {
    throw std::runtime_error(
        "Failed to query calibration blob size: " + sdkReturnCodeToString(result));
  }

  std::vector<unsigned char> bytes(size);
  result = CoreSdk_GetGloveCalibration(bytes.data(), size);
  if (result != SDKReturnCode_Success) {
    throw std::runtime_error(
        "Failed to fetch calibration blob: " + sdkReturnCodeToString(result));
  }

  std::filesystem::create_directories(calibration_directory_);
  std::string file_name = requested_file_name;
  if (file_name.empty()) {
    auto glove = getGloveById(glove_id);
    const std::string side = glove.has_value() ? toLowerCopy(sideToString(glove->side)) : "unknown";
    file_name =
        "glove_" + std::to_string(glove_id) + "_" + side + "_" + timestampString() + ".mcal";
  } else if (std::filesystem::path(file_name).extension() != ".mcal") {
    file_name += ".mcal";
  }

  const std::filesystem::path output_path =
      std::filesystem::path(calibration_directory_) / file_name;
  std::ofstream output(output_path, std::ios::binary);
  if (!output) {
    throw std::runtime_error("Failed to open calibration file for writing: " + output_path.string());
  }

  output.write(reinterpret_cast<const char *>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
  if (!output) {
    throw std::runtime_error("Failed to write calibration file: " + output_path.string());
  }

  RCLCPP_INFO(get_logger(), "Saved MANUS calibration to %s", output_path.c_str());
  return output_path.string();
}

std::string ManusCalibrationNode::resolveCalibrationPath(const std::string &file_path) const
{
  const std::filesystem::path candidate(file_path);
  if (candidate.is_absolute()) {
    return candidate.string();
  }
  return (std::filesystem::path(calibration_directory_) / candidate).string();
}

std::string ManusCalibrationNode::sideToString(Side side)
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

std::string ManusCalibrationNode::pairedStateToString(DevicePairedState state)
{
  switch (state) {
    case DevicePairedState_Paired:
      return "paired";
    case DevicePairedState_Unpaired:
      return "unpaired";
    case DevicePairedState_Pairing:
      return "pairing";
    default:
      return "unknown";
  }
}

std::string ManusCalibrationNode::sdkReturnCodeToString(SDKReturnCode code)
{
  return std::to_string(static_cast<int32_t>(code));
}

std::string ManusCalibrationNode::timestampString()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_tm{};
  localtime_r(&now_time, &local_tm);

  std::ostringstream stream;
  stream << std::put_time(&local_tm, "%Y%m%d_%H%M%S");
  return stream.str();
}

}  // namespace manus_system
