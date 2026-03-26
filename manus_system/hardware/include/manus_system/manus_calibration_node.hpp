#pragma once

#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "ManusSDK.h"

#include "rclcpp/rclcpp.hpp"

#include "manus_system/srv/cancel_glove_calibration.hpp"
#include "manus_system/srv/finish_glove_calibration.hpp"
#include "manus_system/srv/list_gloves.hpp"
#include "manus_system/srv/load_glove_calibration.hpp"
#include "manus_system/srv/run_glove_calibration_step.hpp"
#include "manus_system/srv/start_glove_calibration.hpp"

namespace manus_system {

class ManusCalibrationNode : public rclcpp::Node {
public:
  explicit ManusCalibrationNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~ManusCalibrationNode() override;

private:
  using ListGloves = manus_system::srv::ListGloves;
  using StartGloveCalibration = manus_system::srv::StartGloveCalibration;
  using RunGloveCalibrationStep = manus_system::srv::RunGloveCalibrationStep;
  using CancelGloveCalibration = manus_system::srv::CancelGloveCalibration;
  using FinishGloveCalibration = manus_system::srv::FinishGloveCalibration;
  using LoadGloveCalibration = manus_system::srv::LoadGloveCalibration;

  struct CalibrationSession {
    bool active{false};
    uint32_t glove_id{0};
    std::vector<GloveCalibrationStepData> steps;
  };

  void declareAndLoadParameters();
  void initializeSdk();
  void shutdownSdk();
  void registerCallbacks();
  void connectIntegrated();

  static void onConnectedCallback(const ManusHost *host);
  static void onDisconnectedCallback(const ManusHost *host);
  static void onLandscapeCallback(const Landscape *landscape);

  void handleListGloves(
      const std::shared_ptr<ListGloves::Request> request,
      std::shared_ptr<ListGloves::Response> response);
  void handleStartCalibration(
      const std::shared_ptr<StartGloveCalibration::Request> request,
      std::shared_ptr<StartGloveCalibration::Response> response);
  void handleRunCalibrationStep(
      const std::shared_ptr<RunGloveCalibrationStep::Request> request,
      std::shared_ptr<RunGloveCalibrationStep::Response> response);
  void handleCancelCalibration(
      const std::shared_ptr<CancelGloveCalibration::Request> request,
      std::shared_ptr<CancelGloveCalibration::Response> response);
  void handleFinishCalibration(
      const std::shared_ptr<FinishGloveCalibration::Request> request,
      std::shared_ptr<FinishGloveCalibration::Response> response);
  void handleLoadCalibration(
      const std::shared_ptr<LoadGloveCalibration::Request> request,
      std::shared_ptr<LoadGloveCalibration::Response> response);

  std::optional<GloveLandscapeData> resolveGlove(uint32_t glove_id, const std::string &side_hint) const;
  std::optional<GloveLandscapeData> getGloveById(uint32_t glove_id) const;
  std::optional<GloveLandscapeData> getFirstGloveBySide(const std::string &side_hint) const;
  std::optional<Landscape> copyLandscape() const;

  std::string saveCalibration(uint32_t glove_id, const std::string &requested_file_name);
  std::string resolveCalibrationPath(const std::string &file_path) const;
  static std::string sideToString(Side side);
  static std::string pairedStateToString(DevicePairedState state);
  static std::string sdkReturnCodeToString(SDKReturnCode code);
  static std::string timestampString();

  bool integrated_connected_{false};
  std::string calibration_directory_;
  std::string default_glove_side_;
  bool world_space_{true};
  float unit_scale_{1.0F};

  mutable std::mutex landscape_mutex_;
  std::unique_ptr<Landscape> landscape_;

  mutable std::mutex session_mutex_;
  CalibrationSession calibration_session_;

  rclcpp::Service<ListGloves>::SharedPtr list_gloves_service_;
  rclcpp::Service<StartGloveCalibration>::SharedPtr start_calibration_service_;
  rclcpp::Service<RunGloveCalibrationStep>::SharedPtr run_step_service_;
  rclcpp::Service<CancelGloveCalibration>::SharedPtr cancel_calibration_service_;
  rclcpp::Service<FinishGloveCalibration>::SharedPtr finish_calibration_service_;
  rclcpp::Service<LoadGloveCalibration>::SharedPtr load_calibration_service_;

  static ManusCalibrationNode *instance_;
};

}  // namespace manus_system

