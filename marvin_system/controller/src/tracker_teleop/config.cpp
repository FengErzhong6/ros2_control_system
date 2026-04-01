#include "internal.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"

namespace marvin_system {

namespace {

tf2::Transform readRigidTransform(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
    const std::string &prefix)
{
    std::vector<double> pos_vec, ori_vec;
    node->get_parameter(prefix + ".position", pos_vec);
    node->get_parameter(prefix + ".orientation", ori_vec);

    tf2::Vector3 t(0, 0, 0);
    tf2::Quaternion q(0, 0, 0, 1);

    if (pos_vec.size() >= 3) {
        t.setValue(pos_vec[0], pos_vec[1], pos_vec[2]);
    }
    if (ori_vec.size() >= 4) {
        q.setValue(ori_vec[0], ori_vec[1], ori_vec[2], ori_vec[3]);
    }
    return tf2::Transform(q, t);
}

}  // namespace

bool TrackerTeleopController::readJointNames()
{
    auto node = get_node();
    const auto logger = node->get_logger();

    std::vector<std::string> joints_param;
    if (!node->get_parameter("joints", joints_param)) {
        RCLCPP_ERROR(logger, "Missing required parameter 'joints'.");
        return false;
    }
    if (joints_param.size() != kTotalJoints) {
        RCLCPP_ERROR(
            logger, "Parameter 'joints' must have exactly %zu entries (got %zu).",
            kTotalJoints, joints_param.size());
        return false;
    }

    joint_names_ = std::move(joints_param);
    return true;
}

bool TrackerTeleopController::initializeKinematics()
{
    auto node = get_node();
    const auto logger = node->get_logger();

    if (!node->get_parameter("kine_config_path", kine_config_path_) ||
        kine_config_path_.empty()) {
        RCLCPP_ERROR(logger, "Missing or empty parameter 'kine_config_path'.");
        return false;
    }
    if (!initMarvinKinematics(kine_config_path_, kine_data_, logger)) {
        RCLCPP_ERROR(logger, "Kinematics initialization failed.");
        return false;
    }
    if (!tracking_ik::LoadGeometryFromMvKDCfg(
            kine_config_path_.c_str(), &tracking_ik_geometry_)) {
        RCLCPP_ERROR(
            logger, "tracking_ik::LoadGeometryFromMvKDCfg failed for '%s'",
            kine_config_path_.c_str());
        return false;
    }
    tracking_ik_geometry_loaded_ = true;

    kine_initialized_ = true;
    return true;
}

bool TrackerTeleopController::loadControllerParameters()
{
    auto node = get_node();

    node->get_parameter("position_scale", position_scale_);
    node->get_parameter("enable_ik_reference_logs", enable_ik_reference_logs_);
    node->get_parameter("j4_bound", j4_bound_);
    node->get_parameter("dh_d1", dh_d1_);
    node->get_parameter("smoothing_alpha", smoothing_alpha_);
    node->get_parameter("max_joint_velocity", max_joint_velocity_);
    node->get_parameter("base_x_scale", base_x_scale_);
    node->get_parameter("tracking_ik.fk_accept_tol", tracking_ik_config_.fk_accept_tol);
    node->get_parameter(
        "tracking_ik.fine_psi_range_deg",
        tracking_ik_config_.fine_psi_range_deg);
    node->get_parameter(
        "tracking_ik.fine_psi_step_deg",
        tracking_ik_config_.fine_psi_step_deg);
    node->get_parameter(
        "tracking_ik.fast_psi_range_deg",
        tracking_ik_config_.fast_psi_range_deg);
    node->get_parameter(
        "tracking_ik.fast_psi_step_deg",
        tracking_ik_config_.fast_psi_step_deg);
    node->get_parameter(
        "tracking_ik.expand_psi_range_deg",
        tracking_ik_config_.expand_psi_range_deg);
    node->get_parameter(
        "tracking_ik.expand_psi_step_deg",
        tracking_ik_config_.expand_psi_step_deg);
    node->get_parameter(
        "tracking_ik.score.desired_dir_weight",
        tracking_ik_config_.desired_dir_weight);
    node->get_parameter(
        "tracking_ik.score.continuity_dir_weight",
        tracking_ik_config_.continuity_dir_weight);
    node->get_parameter(
        "tracking_ik.score.magnitude_weight",
        tracking_ik_config_.magnitude_weight);
    node->get_parameter(
        "tracking_ik.score.psi_delta_weight",
        tracking_ik_config_.psi_delta_weight);
    node->get_parameter(
        "tracking_ik.score.branch_switch_penalty",
        tracking_ik_config_.branch_switch_penalty);

    std::vector<std::string> viz_frames;
    if (node->get_parameter("viz_base_frames", viz_frames) && viz_frames.size() >= 2) {
        viz_base_frames_[kLeft] = viz_frames[kLeft];
        viz_base_frames_[kRight] = viz_frames[kRight];
    }

    double tracker_timeout_sec = tracker_timeout_.seconds();
    node->get_parameter("tracker_timeout_sec", tracker_timeout_sec);
    tracker_timeout_ = rclcpp::Duration::from_seconds(std::max(0.0, tracker_timeout_sec));

    double home_tolerance_deg = home_tolerance_rad_ * kRad2Deg;
    node->get_parameter("home_tolerance_deg", home_tolerance_deg);
    home_tolerance_rad_ = std::max(0.0, home_tolerance_deg) * kDeg2Rad;

    smoothing_alpha_ = std::clamp(smoothing_alpha_, 0.01, 1.0);
    tracking_ik_config_.fk_accept_tol = std::max(1e-9, tracking_ik_config_.fk_accept_tol);
    tracking_ik_config_.fine_psi_range_deg = std::max(0.0, tracking_ik_config_.fine_psi_range_deg);
    tracking_ik_config_.fine_psi_step_deg = std::max(0.1, tracking_ik_config_.fine_psi_step_deg);
    tracking_ik_config_.fast_psi_range_deg =
        std::max(tracking_ik_config_.fine_psi_range_deg, tracking_ik_config_.fast_psi_range_deg);
    tracking_ik_config_.fast_psi_range_deg = std::max(0.0, tracking_ik_config_.fast_psi_range_deg);
    tracking_ik_config_.fast_psi_step_deg = std::max(0.1, tracking_ik_config_.fast_psi_step_deg);
    tracking_ik_config_.expand_psi_range_deg =
        std::max(tracking_ik_config_.fast_psi_range_deg, tracking_ik_config_.expand_psi_range_deg);
    tracking_ik_config_.expand_psi_step_deg = std::max(0.1, tracking_ik_config_.expand_psi_step_deg);
    tracking_ik_config_.desired_dir_weight = std::max(0.0, tracking_ik_config_.desired_dir_weight);
    tracking_ik_config_.continuity_dir_weight = std::max(0.0, tracking_ik_config_.continuity_dir_weight);
    tracking_ik_config_.magnitude_weight = std::max(0.0, tracking_ik_config_.magnitude_weight);
    tracking_ik_config_.psi_delta_weight = std::max(0.0, tracking_ik_config_.psi_delta_weight);
    tracking_ik_config_.branch_switch_penalty = std::max(0.0, tracking_ik_config_.branch_switch_penalty);

    loadTransformParameters();
    loadElbowCorrectionParameters();
    for (size_t arm = 0; arm < kArmCount; ++arm) {
        double startup_ref_dir_sign = startup_ref_dir_sign_[arm];
        node->get_parameter(
            std::string("startup_ref_dir_sign.") + kSideLabels[arm],
            startup_ref_dir_sign);
        startup_ref_dir_sign_[arm] = startup_ref_dir_sign >= 0.0 ? 1.0 : -1.0;
    }

    const auto home_left = node->get_parameter("home_joint_positions.left").as_double_array();
    const auto home_right = node->get_parameter("home_joint_positions.right").as_double_array();
    if (!loadHomeJointParameters(home_left, home_right, home_tolerance_deg)) {
        return false;
    }

    loadTrackerFrameParameters();
    logConfigurationSummary(home_tolerance_deg);
    return true;
}

void TrackerTeleopController::loadTransformParameters()
{
    auto node = get_node();
    const auto logger = node->get_logger();

    struct TransformGroup {
        const char *name;
        std::array<tf2::Transform, kArmCount> *storage;
    };

    const std::array<TransformGroup, 3> transform_groups{{
        {"shoulder_T_chest", &shoulder_T_chest_},
        {"wrist_T_ee", &wrist_T_ee_},
        {"arm_human_T_arm_robot", &arm_human_T_arm_robot_},
    }};

    for (const auto &group : transform_groups) {
        for (size_t arm = 0; arm < kArmCount; ++arm) {
            (*group.storage)[arm] = readRigidTransform(
                node, std::string(group.name) + "." + kSideLabels[arm]);
            const auto &transform = (*group.storage)[arm];
            const auto &t = transform.getOrigin();
            const auto &q = transform.getRotation();
            RCLCPP_INFO(
                logger,
                "%s %s: t=[%.3f, %.3f, %.3f] quat=[%.4f, %.4f, %.4f, %.4f]",
                group.name, kSideTags[arm],
                t.x(), t.y(), t.z(),
                q.x(), q.y(), q.z(), q.w());
        }
    }
}

void TrackerTeleopController::loadElbowCorrectionParameters()
{
    auto node = get_node();
    const auto logger = node->get_logger();

    for (size_t arm = 0; arm < kArmCount; ++arm) {
        std::vector<double> rpy_deg;
        node->get_parameter(
            std::string("elbow_direction_correction.") + kSideLabels[arm], rpy_deg);

        if (rpy_deg.size() >= 3 &&
            (rpy_deg[0] != 0.0 || rpy_deg[1] != 0.0 || rpy_deg[2] != 0.0)) {
            elbow_dir_correction_[arm].setRPY(
                rpy_deg[0] * kDeg2Rad, rpy_deg[1] * kDeg2Rad, rpy_deg[2] * kDeg2Rad);
            RCLCPP_INFO(
                logger, "Elbow dir correction %s: RPY=[%.1f, %.1f, %.1f] deg",
                kSideTags[arm], rpy_deg[0], rpy_deg[1], rpy_deg[2]);
            continue;
        }

        elbow_dir_correction_[arm] = tf2::Quaternion::getIdentity();
    }
}

bool TrackerTeleopController::loadHomeJointParameters(
    const std::vector<double> &home_left,
    const std::vector<double> &home_right,
    double home_tolerance_deg)
{
    const auto logger = get_node()->get_logger();

    if (home_left.empty() && home_right.empty()) {
        has_home_joints_ = false;
        RCLCPP_WARN(logger, "Go-home service disabled: home_joint_positions not configured.");
        return true;
    }

    if (home_left.size() != kJointsPerArm || home_right.size() != kJointsPerArm) {
        RCLCPP_ERROR(
            logger,
            "Invalid home_joint_positions size (left=%zu, right=%zu, expected=%zu).",
            home_left.size(), home_right.size(), kJointsPerArm);
        return false;
    }

    for (size_t joint = 0; joint < kJointsPerArm; ++joint) {
        arm_state_[kLeft].home_joints_rad[joint] = home_left[joint];
        arm_state_[kRight].home_joints_rad[joint] = home_right[joint];
    }

    has_home_joints_ = true;
    RCLCPP_INFO(logger, "Go-home pose configured (tol=%.2f deg).", home_tolerance_deg);
    return true;
}

void TrackerTeleopController::loadTrackerFrameParameters()
{
    auto node = get_node();
    node->get_parameter("tracker_frames.torso", frame_torso_);
    node->get_parameter("tracker_frames.left_hand", frame_left_hand_);
    node->get_parameter("tracker_frames.right_hand", frame_right_hand_);
    node->get_parameter("tracker_frames.left_upper_arm", frame_left_upper_arm_);
    node->get_parameter("tracker_frames.right_upper_arm", frame_right_upper_arm_);
}

void TrackerTeleopController::logConfigurationSummary(double home_tolerance_deg) const
{
    const auto logger = get_node()->get_logger();

    RCLCPP_INFO(logger, "J4 bound: %.2f deg", j4_bound_);
    RCLCPP_INFO(
        logger, "Smoothing: alpha=%.3f, max_vel=%.2f rad/s",
        smoothing_alpha_, max_joint_velocity_);
    RCLCPP_INFO(logger, "Base X scale: %.3f", base_x_scale_);
    RCLCPP_INFO(
        logger, "IK reference logs: %s",
        enable_ik_reference_logs_ ? "ON" : "OFF");
    RCLCPP_INFO(
        logger,
        "Tracking IK: fk_tol=%.3e fine[range=%.1f step=%.1f] fast[range=%.1f step=%.1f] "
        "expand[range=%.1f step=%.1f] score[desired=%.3f continuity=%.3f mag=%.3f psi=%.3f branch=%.3f]",
        tracking_ik_config_.fk_accept_tol,
        tracking_ik_config_.fine_psi_range_deg,
        tracking_ik_config_.fine_psi_step_deg,
        tracking_ik_config_.fast_psi_range_deg,
        tracking_ik_config_.fast_psi_step_deg,
        tracking_ik_config_.expand_psi_range_deg,
        tracking_ik_config_.expand_psi_step_deg,
        tracking_ik_config_.desired_dir_weight,
        tracking_ik_config_.continuity_dir_weight,
        tracking_ik_config_.magnitude_weight,
        tracking_ik_config_.psi_delta_weight,
        tracking_ik_config_.branch_switch_penalty);
    RCLCPP_INFO(logger, "Tracker timeout: %.3f s", tracker_timeout_.seconds());
    if (has_home_joints_) {
        RCLCPP_INFO(logger, "Home tolerance: %.2f deg", home_tolerance_deg);
    }
    RCLCPP_INFO(
        logger, "TF frames: torso=%s, L_hand=%s, R_hand=%s, L_arm=%s, R_arm=%s",
        frame_torso_.c_str(), frame_left_hand_.c_str(), frame_right_hand_.c_str(),
        frame_left_upper_arm_.c_str(), frame_right_upper_arm_.c_str());
    RCLCPP_INFO(
        logger, "Startup ref-dir sign: LEFT=%+.0f RIGHT=%+.0f",
        startup_ref_dir_sign_[kLeft], startup_ref_dir_sign_[kRight]);
    RCLCPP_INFO(
        logger, "Teleop gate: startup=%s, control=service + keyboard helper",
        teleopStateToString(getTeleopState()));
}

void TrackerTeleopController::resetTrackerState()
{
    for (auto &runtime : arm_state_) {
        runtime.last_hand_tf_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
        runtime.last_arm_tf_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
        runtime.last_hand_tf_valid = false;
        runtime.last_arm_tf_valid = false;
        runtime.last_hand_tf_fresh = false;
        runtime.last_arm_tf_fresh = false;
        runtime.tracker_fresh = false;
        runtime.marker_visible = false;
    }
    tf_cache_.fill(CachedTrackerData());
    tf_snapshot_.fill(CachedTrackerData());
}

void TrackerTeleopController::resetTeleopRuntime(TeleopState teleop_state)
{
    teleop_state_.store(static_cast<int>(teleop_state), std::memory_order_relaxed);
    force_tracker_reacquire_.store(false, std::memory_order_relaxed);
    go_home_requested_.store(false, std::memory_order_relaxed);
    go_home_active_.store(false, std::memory_order_relaxed);
    active_home_joint_index_.store(-1, std::memory_order_relaxed);
}

void TrackerTeleopController::resetRuntimeState()
{
    for (auto &runtime : arm_state_) {
        runtime.last_joint_deg.fill(0.0);
        runtime.last_selected_ref_dir =
            {{kDefaultShoulderVElbow[0], kDefaultShoulderVElbow[1], kDefaultShoulderVElbow[2]}};
        runtime.last_selected_branch = -1;
        runtime.smoothed_joints_rad.fill(0.0);
        runtime.target_joints_rad.fill(0.0);
        runtime.has_valid_target = false;
        runtime.last_ik_result = IKResult::kNoTarget;
        runtime.pending_diagnostics.sequence.store(0, std::memory_order_relaxed);
        runtime.pending_diagnostics.pending.store(false, std::memory_order_relaxed);
        runtime.pending_diagnostics.snapshot = ArmDiagnostics();
    }
    cmd_interfaces_.fill(nullptr);
    state_interfaces_pos_.fill(nullptr);
    resetTrackerState();
    resetTeleopRuntime(TeleopState::kDisarmed);
}

void TrackerTeleopController::createRosInterfaces()
{
    auto node = get_node();

    pub_viz_markers_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/viz_markers", rclcpp::SystemDefaultsQoS());
    pub_ik_status_[kLeft] = node->create_publisher<std_msgs::msg::String>(
        "~/ik_status_left", rclcpp::SystemDefaultsQoS());
    pub_ik_status_[kRight] = node->create_publisher<std_msgs::msg::String>(
        "~/ik_status_right", rclcpp::SystemDefaultsQoS());
    pub_teleop_state_ = node->create_publisher<std_msgs::msg::String>(
        "~/teleop_state", rclcpp::QoS(1).transient_local());

    auto fk_qos = rclcpp::QoS(1).transient_local();
    pub_current_pose_[kLeft] = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "~/current_pose_left", fk_qos);
    pub_current_pose_[kRight] = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "~/current_pose_right", fk_qos);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node, false);

    tf_poll_timer_ = node->create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&TrackerTeleopController::pollTfCallback, this));
    diagnostics_timer_ = node->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&TrackerTeleopController::diagnosticsTimerCallback, this));
    srv_set_armed_ = node->create_service<std_srvs::srv::SetBool>(
        "~/set_armed",
        std::bind(
            &TrackerTeleopController::handleSetArmed, this,
            std::placeholders::_1, std::placeholders::_2));
    srv_set_enabled_ = node->create_service<std_srvs::srv::SetBool>(
        "~/set_enabled",
        std::bind(
            &TrackerTeleopController::handleSetEnabled, this,
            std::placeholders::_1, std::placeholders::_2));
    srv_go_home_ = node->create_service<std_srvs::srv::Trigger>(
        "~/go_home",
        std::bind(
            &TrackerTeleopController::handleGoHome, this,
            std::placeholders::_1, std::placeholders::_2));
}

}  // namespace marvin_system
