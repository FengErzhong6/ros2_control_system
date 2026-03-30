#include "marvin_system/tracker_teleop_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <sstream>
#include <utility>

#include "rclcpp/logging.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using config_type = controller_interface::interface_configuration_type;

namespace marvin_system {

namespace {

constexpr std::array<const char *, kArmCount> kSideLabels{{"left", "right"}};
constexpr std::array<const char *, kArmCount> kSideTags{{"LEFT", "RIGHT"}};

template <typename LoanedInterfaceT>
LoanedInterfaceT *find_loaned_interface(
    std::vector<LoanedInterfaceT> &interfaces, const std::string &full_name)
{
    for (auto &iface : interfaces) {
        if (iface.get_name() == full_name) {
            return &iface;
        }
    }
    return nullptr;
}

bool normalizeVector(std::array<double, 3> &vector)
{
    const double norm = std::sqrt(
        vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
    if (norm < 1e-12) {
        return false;
    }
    vector[0] /= norm;
    vector[1] /= norm;
    vector[2] /= norm;
    return true;
}

double clampUnit(double value)
{
    return std::clamp(value, -1.0, 1.0);
}

double angleBetweenVectorsDeg(
    const std::array<double, 3> &lhs, const std::array<double, 3> &rhs)
{
    return std::acos(clampUnit(
        lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2])) * kRad2Deg;
}

bool extractSolvedUpperArmDir(
    FX_INT32L arm,
    const std::array<double, kJointsPerArm> &joints_rad,
    std::array<double, 3> &upper_arm_dir)
{
    FX_DOUBLE joints_deg[kJointsPerArm];
    Matrix4 fk_pose{};
    Matrix3 nsp_pose{};

    for (size_t j = 0; j < kJointsPerArm; ++j) {
        joints_deg[j] = joints_rad[j] * kRad2Deg;
    }

    if (!FX_Robot_Kine_FK_NSP(arm, joints_deg, fk_pose, nsp_pose)) {
        return false;
    }

    // Keep the diagnostics in the same definition as the tracker input:
    // the upper-arm vector rooted at shoulder is the Y axis of the solved NSP frame.
    upper_arm_dir = {{nsp_pose[0][1], nsp_pose[1][1], nsp_pose[2][1]}};
    return normalizeVector(upper_arm_dir);
}

}  // namespace

TrackerTeleopController::TrackerTeleopController()
    : controller_interface::ControllerInterface() {}

TrackerTeleopController::~TrackerTeleopController() = default;

controller_interface::CallbackReturn TrackerTeleopController::on_init()
{
    try {
        auto_declare<std::vector<std::string>>("joints", {});
        auto_declare<std::string>("kine_config_path", "");

        auto_declare<std::string>("tracker_frames.torso", "tracker_torso");
        auto_declare<std::string>("tracker_frames.left_hand", "tracker_left_hand");
        auto_declare<std::string>("tracker_frames.right_hand", "tracker_right_hand");
        auto_declare<std::string>("tracker_frames.left_upper_arm", "tracker_left_upper_arm");
        auto_declare<std::string>("tracker_frames.right_upper_arm", "tracker_right_upper_arm");

        auto_declare<double>("position_scale", 1.0);
        auto_declare<bool>("enable_orientation", true);
        auto_declare<std::string>("base_frame", "base_link");
        auto_declare<std::vector<double>>("default_elbow_direction",
                                          {0.0, 0.0, -1.0});
        auto_declare<double>("j4_bound", 0.0);
        auto_declare<double>("dh_d1", 0.0);
        auto_declare<std::vector<std::string>>("viz_base_frames", {"Base_L", "Base_R"});
        auto_declare<double>("tracking_ik.fk_accept_tol", 1e-3);
        auto_declare<double>("tracking_ik.fast_psi_range_deg", 12.0);
        auto_declare<double>("tracking_ik.fast_psi_step_deg", 1.0);
        auto_declare<double>("tracking_ik.expand_psi_range_deg", 36.0);
        auto_declare<double>("tracking_ik.expand_psi_step_deg", 3.0);
        auto_declare<double>("tracking_ik.score.desired_dir_weight", 0.03);
        auto_declare<double>("tracking_ik.score.continuity_dir_weight", 0.03);
        auto_declare<double>("tracking_ik.score.magnitude_weight", 0.05);
        auto_declare<double>("tracking_ik.score.psi_delta_weight", 0.02);
        auto_declare<double>("tracking_ik.score.branch_switch_penalty", 20.0);

        auto_declare<double>("smoothing_alpha", 0.3);
        auto_declare<double>("max_joint_velocity", 2.0);
        auto_declare<double>("base_x_scale", 1.0);
        auto_declare<double>("tracker_timeout_sec", 0.1);
        auto_declare<std::vector<double>>("home_joint_positions.left", {});
        auto_declare<std::vector<double>>("home_joint_positions.right", {});
        auto_declare<double>("home_tolerance_deg", 0.5);
        for (const auto &side : {"left", "right"}) {
            for (const auto &group :
                 {"shoulder_T_chest", "wrist_T_ee", "arm_human_T_arm_robot"}) {
                std::string prefix = std::string(group) + "." + side;
                auto_declare<std::vector<double>>(prefix + ".position", {0.0, 0.0, 0.0});
                auto_declare<std::vector<double>>(prefix + ".orientation", {0.0, 0.0, 0.0, 1.0});
            }
            auto_declare<std::vector<double>>(
                std::string("elbow_direction_correction.") + side, {0.0, 0.0, 0.0});
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
TrackerTeleopController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};
    conf.names.reserve(joint_names_.size());
    for (const auto &jn : joint_names_) {
        conf.names.push_back(jn + "/position");
    }
    return conf;
}

controller_interface::InterfaceConfiguration
TrackerTeleopController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};
    conf.names.reserve(joint_names_.size());
    for (const auto &jn : joint_names_) {
        conf.names.push_back(jn + "/position");
    }
    return conf;
}

bool TrackerTeleopController::lookupTf(
    const std::string &target_frame,
    const std::string &source_frame,
    geometry_msgs::msg::TransformStamped &out_target_T_source)
{
    try {
        out_target_T_source = tf_buffer_->lookupTransform(
            target_frame, source_frame, tf2::TimePointZero);
        return true;
    } catch (const tf2::TransformException &) {
        return false;
    }
}

void TrackerTeleopController::pollTfCallback()
{
    const std::string *frame_hand[] = {&frame_left_hand_, &frame_right_hand_};
    const std::string *frame_arm[] = {&frame_left_upper_arm_, &frame_right_upper_arm_};

    std::lock_guard<std::mutex> lk(tf_cache_mutex_);
    std::array<CachedTrackerData, kArmCount> fresh = tf_cache_;
    for (int arm = 0; arm < static_cast<int>(kArmCount); ++arm) {
        geometry_msgs::msg::TransformStamped chest_T_hand;
        if (lookupTf(frame_torso_, *frame_hand[arm], chest_T_hand)) {
            fresh[arm].chest_T_hand = std::move(chest_T_hand);
            fresh[arm].hand_valid = true;
        }

        geometry_msgs::msg::TransformStamped chest_T_arm;
        if (lookupTf(frame_torso_, *frame_arm[arm], chest_T_arm)) {
            fresh[arm].chest_T_arm = std::move(chest_T_arm);
            fresh[arm].arm_valid = true;
        }
    }
    tf_cache_ = fresh;
}

TrackerTeleopController::IKResult TrackerTeleopController::solveIK(
    size_t arm,
    const geometry_msgs::msg::PoseStamped &base_T_ee,
    const double (&shoulder_v_elbow)[3],
    double (&out_q_joints_rad)[kJointsPerArm],
    ArmDiagnostics *diag)
{
    if (diag) {
        diag->dir_result = IKResult::kNoTarget;
        diag->ref_result = IKResult::kNoTarget;
        diag->final_result = IKResult::kNoTarget;
        diag->used_near_ref = false;
        diag->clamped = false;
        diag->has_solution = false;
        diag->solution_j4_deg = 0.0;
        diag->q_joints_rad.fill(0.0);
        diag->solver_reachable = false;
        diag->used_expanded_search = false;
        diag->candidate_count = 0;
        diag->psi_eval_count = 0;
        diag->selected_branch = -1;
        diag->selected_psi_deg = 0.0;
        diag->best_fk_residual_l1 = 0.0;
        diag->best_ref_score_l1 = 0.0;
        diag->best_desired_dir_score_deg = 0.0;
        diag->best_continuity_dir_score_deg = 0.0;
        diag->solved_upper_arm_dir_valid = false;
        diag->solved_upper_arm_dir = {{0.0, 0.0, 0.0}};
        diag->solved_upper_arm_dir_angle_deg = 0.0;
    }

    if (!isQuaternionValid(base_T_ee.pose.orientation)) {
        if (diag) {
            diag->final_result = IKResult::kInvalidQuaternion;
        }
        return IKResult::kInvalidQuaternion;
    }

    tracking_ik::Request request;
    tracking_ik::Result result{};
    tracking_ik::SetDefaultRequest(&request);

    poseToMatrix4(base_T_ee, request.target_tcp);
    request.desired_upper_arm_dir[0] = shoulder_v_elbow[0];
    request.desired_upper_arm_dir[1] = shoulder_v_elbow[1];
    request.desired_upper_arm_dir[2] = shoulder_v_elbow[2];
    if (last_selected_branch_[arm] < 0) {
        request.prev_selected_ref_dir[0] = shoulder_v_elbow[0];
        request.prev_selected_ref_dir[1] = shoulder_v_elbow[1];
        request.prev_selected_ref_dir[2] = shoulder_v_elbow[2];
    } else {
        request.prev_selected_ref_dir[0] = last_selected_ref_dir_[arm][0];
        request.prev_selected_ref_dir[1] = last_selected_ref_dir_[arm][1];
        request.prev_selected_ref_dir[2] = last_selected_ref_dir_[arm][2];
    }
    request.prev_selected_branch = static_cast<FX_INT32L>(last_selected_branch_[arm]);
    for (size_t j = 0; j < kJointsPerArm; ++j) {
        request.ref_joint_deg[j] = last_joint_deg_[arm][j];
    }

    request.fk_accept_tol = tracking_ik_config_.fk_accept_tol;
    request.fast_psi_range_deg = tracking_ik_config_.fast_psi_range_deg;
    request.fast_psi_step_deg = tracking_ik_config_.fast_psi_step_deg;
    request.expand_psi_range_deg = tracking_ik_config_.expand_psi_range_deg;
    request.expand_psi_step_deg = tracking_ik_config_.expand_psi_step_deg;
    request.score_params.desired_dir_weight =
        tracking_ik_config_.desired_dir_weight;
    request.score_params.continuity_dir_weight =
        tracking_ik_config_.continuity_dir_weight;
    request.score_params.magnitude_weight =
        tracking_ik_config_.magnitude_weight;
    request.score_params.psi_delta_weight =
        tracking_ik_config_.psi_delta_weight;
    request.score_params.branch_switch_penalty =
        tracking_ik_config_.branch_switch_penalty;

    tracking_ik::Solve(&tracking_ik_geometry_, &request, &result);

    IKResult final_result = IKResult::kSolveFailed;
    if (result.success) {
        for (size_t j = 0; j < kJointsPerArm; ++j) {
            out_q_joints_rad[j] = result.best_joints_deg[j] * kDeg2Rad;
        }
        last_selected_ref_dir_[arm] = {
            {result.selected_ref_dir[0], result.selected_ref_dir[1], result.selected_ref_dir[2]}};
        last_selected_branch_[arm] = result.selected_branch;
        final_result = IKResult::kSuccess;
    } else if (!result.reachable) {
        final_result = IKResult::kOutOfRange;
    }

    if (diag) {
        diag->dir_result = final_result;
        diag->final_result = final_result;
        diag->has_solution = (final_result == IKResult::kSuccess);
        diag->solver_reachable = result.reachable;
        diag->used_expanded_search = result.used_expanded_search;
        diag->candidate_count = result.candidate_count;
        diag->psi_eval_count = result.psi_eval_count;
        diag->selected_branch = result.selected_branch;
        diag->selected_psi_deg = result.selected_psi_deg;
        diag->best_fk_residual_l1 = result.best_fk_residual_l1;
        diag->best_ref_score_l1 = result.best_ref_score_l1;
        diag->best_desired_dir_score_deg = result.best_desired_dir_score;
        diag->best_continuity_dir_score_deg = result.best_continuity_dir_score;
        if (diag->has_solution) {
            std::array<double, 3> input_upper_arm_dir{
                {shoulder_v_elbow[0], shoulder_v_elbow[1], shoulder_v_elbow[2]}};
            diag->solution_j4_deg = out_q_joints_rad[3] * kRad2Deg;
            for (size_t j = 0; j < kJointsPerArm; ++j) {
                diag->q_joints_rad[j] = out_q_joints_rad[j];
            }
            if (normalizeVector(input_upper_arm_dir) &&
                extractSolvedUpperArmDir(
                    static_cast<FX_INT32L>(arm),
                    diag->q_joints_rad,
                    diag->solved_upper_arm_dir)) {
                diag->solved_upper_arm_dir_valid = true;
                diag->solved_upper_arm_dir_angle_deg = angleBetweenVectorsDeg(
                    input_upper_arm_dir, diag->solved_upper_arm_dir);
            }
        }
    }

    return final_result;
}

const char *TrackerTeleopController::ikResultToString(IKResult result)
{
    switch (result) {
        case IKResult::kSuccess:             return "IK solved successfully";
        case IKResult::kNoTarget:            return "No target";
        case IKResult::kJointLimitClamped:   return "IK solved (clamped to joint limits)";
        case IKResult::kInvalidQuaternion:   return "Invalid quaternion";
        case IKResult::kOutOfRange:          return "Target out of reachable workspace";
        case IKResult::kJointLimitExceeded:  return "IK solution exceeds joint limits";
        case IKResult::kSingularity:         return "IK solution near singularity";
        case IKResult::kSolveFailed:         return "Tracking IK solver failed";
        default:                             return "Unknown IK error";
    }
}

void TrackerTeleopController::publishIKStatus(size_t arm, IKResult result)
{
    if (arm >= kArmCount || !pub_ik_status_[arm]) return;
    if (result == last_ik_result_[arm]) return;
    last_ik_result_[arm] = result;

    std_msgs::msg::String msg;
    const bool ok = (result == IKResult::kSuccess ||
                     result == IKResult::kJointLimitClamped);
    msg.data = std::string(ok ? "[OK] " : "[FAIL] ") + ikResultToString(result);
    pub_ik_status_[arm]->publish(msg);
}

void TrackerTeleopController::queueDiagnostics(size_t arm, const ArmDiagnostics &diag)
{
    if (arm >= kArmCount) {
        return;
    }

    auto &slot = pending_diagnostics_[arm];
    slot.sequence.fetch_add(1, std::memory_order_acq_rel);
    slot.snapshot = diag;
    slot.snapshot.pending = false;
    slot.pending.store(true, std::memory_order_release);
    slot.sequence.fetch_add(1, std::memory_order_release);
}

bool TrackerTeleopController::updateTfSnapshot()
{
    if (!tf_cache_mutex_.try_lock()) {
        return false;
    }
    tf_snapshot_ = tf_cache_;
    tf_cache_mutex_.unlock();
    return true;
}

TrackerTeleopController::TrackerInputState
TrackerTeleopController::evaluateTrackerInputState(
    size_t arm, const CachedTrackerData &snap, const rclcpp::Time &now)
{
    TrackerInputState state;
    state.hand_stamp = snap.hand_valid ?
        rclcpp::Time(snap.chest_T_hand.header.stamp) :
        rclcpp::Time(0, 0, RCL_ROS_TIME);
    state.arm_stamp = snap.arm_valid ?
        rclcpp::Time(snap.chest_T_arm.header.stamp) :
        rclcpp::Time(0, 0, RCL_ROS_TIME);

    state.hand_fresh =
        snap.hand_valid &&
        state.hand_stamp.nanoseconds() > 0 &&
        (tracker_timeout_.nanoseconds() == 0 ||
         (now >= state.hand_stamp && (now - state.hand_stamp) <= tracker_timeout_));
    state.arm_fresh =
        snap.arm_valid &&
        state.arm_stamp.nanoseconds() > 0 &&
        (tracker_timeout_.nanoseconds() == 0 ||
         (now >= state.arm_stamp && (now - state.arm_stamp) <= tracker_timeout_));

    state.hand_changed = (state.hand_stamp != last_hand_tf_stamp_[arm]) ||
                         (snap.hand_valid != last_hand_tf_valid_[arm]) ||
                         (state.hand_fresh != last_hand_tf_fresh_[arm]);
    state.arm_changed = (state.arm_stamp != last_arm_tf_stamp_[arm]) ||
                        (snap.arm_valid != last_arm_tf_valid_[arm]) ||
                        (state.arm_fresh != last_arm_tf_fresh_[arm]);
    state.tracker_input_changed = state.hand_changed || state.arm_changed;

    last_hand_tf_stamp_[arm] = state.hand_stamp;
    last_arm_tf_stamp_[arm] = state.arm_stamp;
    last_hand_tf_valid_[arm] = snap.hand_valid;
    last_arm_tf_valid_[arm] = snap.arm_valid;
    last_hand_tf_fresh_[arm] = state.hand_fresh;
    last_arm_tf_fresh_[arm] = state.arm_fresh;

    return state;
}

void TrackerTeleopController::holdCurrentPosition(size_t arm)
{
    for (size_t j = 0; j < kJointsPerArm; ++j) {
        target_joints_rad_[arm][j] = smoothed_joints_rad_[arm][j];
    }
}

void TrackerTeleopController::holdAllArms(double dt)
{
    for (size_t arm = 0; arm < kArmCount; ++arm) {
        holdCurrentPosition(arm);
        applySmoothedCommand(arm, dt);
    }
}

void TrackerTeleopController::applySmoothedCommand(size_t arm, double dt)
{
    if (!has_valid_target_[arm]) {
        return;
    }

    const size_t offset = arm * kJointsPerArm;
    const double max_delta = max_joint_velocity_ * dt;

    for (size_t j = 0; j < kJointsPerArm; ++j) {
        double filtered = smoothing_alpha_ * target_joints_rad_[arm][j] +
                          (1.0 - smoothing_alpha_) * smoothed_joints_rad_[arm][j];

        double delta = filtered - smoothed_joints_rad_[arm][j];
        if (std::abs(delta) > max_delta) {
            filtered = smoothed_joints_rad_[arm][j] +
                       std::copysign(max_delta, delta);
        }

        smoothed_joints_rad_[arm][j] = filtered;
        (void)cmd_interfaces_[offset + j]->set_value(filtered);
        last_joint_deg_[arm][j] = filtered * kRad2Deg;
    }
}

void TrackerTeleopController::fillArmTargetFromTracker(
    size_t arm, const CachedTrackerData &snap,
    geometry_msgs::msg::PoseStamped &base_T_ee,
    std::array<double, 3> &shoulder_v_elbow) const
{
    const tf2::Transform &shoulder_T_chest = shoulder_T_chest_[arm];
    const auto &tf_wrist = snap.chest_T_hand.transform;

    tf2::Transform chest_T_wrist(
        tf2::Quaternion(tf_wrist.rotation.x, tf_wrist.rotation.y,
                       tf_wrist.rotation.z, tf_wrist.rotation.w),
        tf2::Vector3(tf_wrist.translation.x * position_scale_,
                    tf_wrist.translation.y * position_scale_,
                    tf_wrist.translation.z * position_scale_));

    if (!enable_orientation_) {
        chest_T_wrist.setRotation(tf2::Quaternion::getIdentity());
    }

    tf2::Transform shoulder_T_wrist = shoulder_T_chest * chest_T_wrist;
    tf2::Transform shoulder_T_ee_tf = shoulder_T_wrist * wrist_T_ee_[arm];

    tf2::Vector3 ee_pos = shoulder_T_ee_tf.getOrigin();
    double offset_x = ee_pos.x() - shoulder_T_chest.getOrigin().x();
    ee_pos.setX(shoulder_T_chest.getOrigin().x() + offset_x * base_x_scale_);
    shoulder_T_ee_tf.setOrigin(ee_pos);

    base_T_ee.header.frame_id = base_frame_;
    base_T_ee.header.stamp = snap.chest_T_hand.header.stamp;
    base_T_ee.pose.position.x = shoulder_T_ee_tf.getOrigin().x();
    base_T_ee.pose.position.y = shoulder_T_ee_tf.getOrigin().y();
    base_T_ee.pose.position.z = shoulder_T_ee_tf.getOrigin().z() + dh_d1_;
    const tf2::Quaternion &ee_q = shoulder_T_ee_tf.getRotation();
    base_T_ee.pose.orientation.x = ee_q.x();
    base_T_ee.pose.orientation.y = ee_q.y();
    base_T_ee.pose.orientation.z = ee_q.z();
    base_T_ee.pose.orientation.w = ee_q.w();

    shoulder_v_elbow = shoulder_v_elbow_default_;
    if (!snap.arm_valid) {
        return;
    }

    const auto &tf_arm = snap.chest_T_arm.transform;
    tf2::Quaternion chest_R_arm(
        tf_arm.rotation.x, tf_arm.rotation.y,
        tf_arm.rotation.z, tf_arm.rotation.w);
    tf2::Quaternion shoulder_R_arm_robot =
        shoulder_T_chest.getRotation() * chest_R_arm *
        arm_human_T_arm_robot_[arm].getRotation();

    tf2::Matrix3x3 shoulder_M_arm_robot(shoulder_R_arm_robot);
    tf2::Vector3 corrected = tf2::quatRotate(
        elbow_dir_correction_[arm], shoulder_M_arm_robot.getColumn(1));
    shoulder_v_elbow = {{corrected.x(), corrected.y(), corrected.z()}};
}

void TrackerTeleopController::handleStaleTracker(
    size_t arm, const CachedTrackerData &snap, bool tracker_input_changed)
{
    if (tracker_fresh_[arm] || tracker_input_changed) {
        ArmDiagnostics diag;
        diag.tracker_fresh = false;
        diag.hand_valid = snap.hand_valid;
        diag.arm_valid = snap.arm_valid;
        diag.final_result = IKResult::kNoTarget;
        queueDiagnostics(arm, diag);
    }
    tracker_fresh_[arm] = false;
    holdCurrentPosition(arm);
}

void TrackerTeleopController::handleFreshTrackerUpdate(size_t arm, const CachedTrackerData &snap)
{
    tracker_fresh_[arm] = true;

    geometry_msgs::msg::PoseStamped base_T_ee;
    std::array<double, 3> shoulder_v_elbow{};
    fillArmTargetFromTracker(arm, snap, base_T_ee, shoulder_v_elbow);

    ArmDiagnostics diag;
    diag.tracker_fresh = true;
    diag.hand_valid = snap.hand_valid;
    diag.arm_valid = snap.arm_valid;
    diag.has_base_T_ee = true;
    diag.base_T_ee = base_T_ee;
    diag.shoulder_v_elbow = shoulder_v_elbow;

    const double shoulder_v_elbow_arr[3] = {
        shoulder_v_elbow[0], shoulder_v_elbow[1], shoulder_v_elbow[2]};
    double out_q_joints_rad[kJointsPerArm];
    IKResult ik_result = solveIK(
        arm, base_T_ee, shoulder_v_elbow_arr, out_q_joints_rad, &diag);
    queueDiagnostics(arm, diag);

    if (ik_result == IKResult::kSuccess ||
        ik_result == IKResult::kJointLimitClamped) {
        for (size_t j = 0; j < kJointsPerArm; ++j) {
            target_joints_rad_[arm][j] = out_q_joints_rad[j];
        }
        has_valid_target_[arm] = true;
        return;
    }

    holdCurrentPosition(arm);
}

void TrackerTeleopController::startGoHomeSequence()
{
    if (!has_home_joints_) {
        return;
    }

    for (size_t arm = 0; arm < kArmCount; ++arm) {
        for (size_t j = 0; j < kJointsPerArm; ++j) {
            target_joints_rad_[arm][j] = smoothed_joints_rad_[arm][j];
        }
        has_valid_target_[arm] = true;
        target_joints_rad_[arm][kJointsPerArm - 1] =
            home_joints_rad_[arm][kJointsPerArm - 1];
    }

    active_home_joint_index_.store(
        static_cast<int>(kJointsPerArm) - 1, std::memory_order_relaxed);
    go_home_active_.store(true, std::memory_order_relaxed);

    RCLCPP_INFO(
        get_node()->get_logger(),
        "Go-home sequence started (Joint%zu -> Joint1).",
        kJointsPerArm);
}

bool TrackerTeleopController::isHomeJointReached(size_t arm, size_t joint) const
{
    const size_t idx = arm * kJointsPerArm + joint;
    if (!state_interfaces_pos_[idx]) {
        return false;
    }

    const auto pos_opt = state_interfaces_pos_[idx]->get_optional<double>();
    const double pos_rad = pos_opt.has_value() ?
        pos_opt.value() : smoothed_joints_rad_[arm][joint];
    return std::abs(pos_rad - home_joints_rad_[arm][joint]) <= home_tolerance_rad_;
}

void TrackerTeleopController::processGoHome(double dt)
{
    if (go_home_requested_.exchange(false, std::memory_order_relaxed)) {
        startGoHomeSequence();
    }

    if (!go_home_active_.load(std::memory_order_relaxed)) {
        return;
    }

    const int joint_index = active_home_joint_index_.load(std::memory_order_relaxed);
    if (joint_index < 0) {
        go_home_active_.store(false, std::memory_order_relaxed);
        return;
    }

    const size_t jt = static_cast<size_t>(joint_index);
    for (size_t arm = 0; arm < kArmCount; ++arm) {
        target_joints_rad_[arm][jt] = home_joints_rad_[arm][jt];
        has_valid_target_[arm] = true;
        applySmoothedCommand(arm, dt);
    }

    bool reached = true;
    for (size_t arm = 0; arm < kArmCount; ++arm) {
        reached = reached && isHomeJointReached(arm, jt);
    }
    if (!reached) {
        return;
    }

    const int next_joint_index = joint_index - 1;
    active_home_joint_index_.store(next_joint_index, std::memory_order_relaxed);

    if (next_joint_index >= 0) {
        const size_t next_jt = static_cast<size_t>(next_joint_index);
        for (size_t arm = 0; arm < kArmCount; ++arm) {
            target_joints_rad_[arm][next_jt] = home_joints_rad_[arm][next_jt];
        }
        RCLCPP_INFO(get_node()->get_logger(), "Go-home reached Joint%zu, continuing to Joint%zu.",
                    jt + 1, next_jt + 1);
        return;
    }

    go_home_active_.store(false, std::memory_order_relaxed);
    RCLCPP_INFO(get_node()->get_logger(), "Go-home sequence completed.");
}

void TrackerTeleopController::processArmUpdate(
    size_t arm, const CachedTrackerData &snap, const rclcpp::Time &now, double dt,
    bool force_reacquire)
{
    const TrackerInputState input_state = evaluateTrackerInputState(arm, snap, now);
    if (!input_state.hand_fresh) {
        handleStaleTracker(arm, snap, input_state.tracker_input_changed);
        applySmoothedCommand(arm, dt);
        return;
    }

    CachedTrackerData effective_snap = snap;
    effective_snap.arm_valid = input_state.arm_fresh;

    if (force_reacquire || input_state.tracker_input_changed) {
        handleFreshTrackerUpdate(arm, effective_snap);
    }

    applySmoothedCommand(arm, dt);
}

void TrackerTeleopController::handleSetArmed(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (request->data) {
        if (getTeleopState() == TeleopState::kDisarmed) {
            setTeleopState(TeleopState::kArmed, "service arm");
        }
        response->success = true;
        response->message = "Tracker teleop armed; holding current position.";
        return;
    }

    go_home_requested_.store(false, std::memory_order_relaxed);
    go_home_active_.store(false, std::memory_order_relaxed);
    active_home_joint_index_.store(-1, std::memory_order_relaxed);
    setTeleopState(TeleopState::kDisarmed, "service disarm");
    response->success = true;
    response->message = "Tracker teleop disarmed; holding current position.";
}

void TrackerTeleopController::handleSetEnabled(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (request->data) {
        if (getTeleopState() == TeleopState::kDisarmed) {
            response->success = false;
            response->message = "Teleop is disarmed; arm before enabling.";
            return;
        }
        go_home_requested_.store(false, std::memory_order_relaxed);
        go_home_active_.store(false, std::memory_order_relaxed);
        active_home_joint_index_.store(-1, std::memory_order_relaxed);
        setTeleopState(TeleopState::kEnabled, "service enable");
        response->success = true;
        response->message = "Tracker teleop enabled.";
        return;
    }

    if (getTeleopState() == TeleopState::kEnabled) {
        setTeleopState(TeleopState::kArmed, "service disable");
    }
    response->success = true;
    response->message = "Tracker teleop disabled; holding current position.";
}

void TrackerTeleopController::handleGoHome(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;

    if (!has_home_joints_) {
        response->success = false;
        response->message = "Home joint positions are not configured.";
        return;
    }

    setTeleopState(TeleopState::kArmed, "service go_home");
    go_home_requested_.store(true, std::memory_order_relaxed);
    response->success = true;
    response->message = "Go-home accepted; moving to default pose.";
}

TrackerTeleopController::TeleopState TrackerTeleopController::getTeleopState() const
{
    return static_cast<TeleopState>(teleop_state_.load(std::memory_order_relaxed));
}

void TrackerTeleopController::setTeleopState(
    TeleopState new_state, const std::string &reason)
{
    const TeleopState old_state = getTeleopState();
    teleop_state_.store(static_cast<int>(new_state), std::memory_order_relaxed);
    if (new_state == TeleopState::kEnabled) {
        force_tracker_reacquire_.store(true, std::memory_order_relaxed);
    }

    if (old_state == new_state) {
        return;
    }

    RCLCPP_WARN(
        get_node()->get_logger(),
        "Tracker teleop state: %s -> %s (%s)",
        teleopStateToString(old_state),
        teleopStateToString(new_state),
        reason.c_str());
    publishTeleopState(reason);
}

bool TrackerTeleopController::isTeleopEnabled(const rclcpp::Time &now)
{
    (void)now;
    return getTeleopState() == TeleopState::kEnabled;
}

void TrackerTeleopController::publishTeleopState(const std::string &reason)
{
    if (!pub_teleop_state_) {
        return;
    }

    std_msgs::msg::String msg;
    msg.data = std::string(teleopStateToString(getTeleopState())) + " | " + reason;
    pub_teleop_state_->publish(msg);
}

const char *TrackerTeleopController::teleopStateToString(TeleopState state)
{
    switch (state) {
        case TeleopState::kDisarmed: return "DISARMED";
        case TeleopState::kArmed: return "ARMED";
        case TeleopState::kEnabled: return "ENABLED";
        default: return "UNKNOWN";
    }
}

std::string TrackerTeleopController::buildIkLogChain(const ArmDiagnostics &diag) const
{
    auto classify_j4 = [this](double j4_deg) -> const char * {
        return (j4_deg < j4_bound_) ? "A" : "B";
    };

    std::ostringstream chain;
    chain << "TRACKING_IK ";
    if (diag.final_result == IKResult::kSuccess) {
        chain << "OK(" << classify_j4(diag.solution_j4_deg) << ")";
    } else {
        chain << "FAIL(" << ikResultToString(diag.final_result) << ")";
    }
    chain << " reachable=" << (diag.solver_reachable ? "Y" : "N");
    chain << " expand=" << (diag.used_expanded_search ? "Y" : "N");
    chain << " cands=" << diag.candidate_count;
    chain << " psi_eval=" << diag.psi_eval_count;
    chain << " branch=" << diag.selected_branch;
    chain << " psi=" << diag.selected_psi_deg;
    if (diag.has_solution) {
        chain << " fk_l1=" << diag.best_fk_residual_l1;
        chain << " ref_l1=" << diag.best_ref_score_l1;
        chain << " desired_dir=" << diag.best_desired_dir_score_deg << "deg";
        chain << " continuity_dir=" << diag.best_continuity_dir_score_deg << "deg";
        if (diag.solved_upper_arm_dir_valid) {
            chain << " nsp_dir=" << diag.solved_upper_arm_dir_angle_deg << "deg";
        } else {
            chain << " nsp_dir=NA";
        }
    }
    return chain.str();
}

void TrackerTeleopController::logArmDiagnostics(size_t arm, const ArmDiagnostics &diag)
{
    const auto logger = get_node()->get_logger();
    const std::string chain = buildIkLogChain(diag);

    if (diag.has_solution) {
        if (diag.solved_upper_arm_dir_valid) {
            RCLCPP_INFO(
                logger,
                "Arm %zu IK | shoulder_T_ee=[%.4f, %.4f, %.4f] quat=[%.4f, %.4f, %.4f, %.4f] "
                "| elbow=[%.3f, %.3f, %.3f] | solved_elbow=[%.3f, %.3f, %.3f] | %s "
                "| q=[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f] deg",
                arm,
                diag.base_T_ee.pose.position.x, diag.base_T_ee.pose.position.y,
                diag.base_T_ee.pose.position.z - dh_d1_,
                diag.base_T_ee.pose.orientation.x, diag.base_T_ee.pose.orientation.y,
                diag.base_T_ee.pose.orientation.z, diag.base_T_ee.pose.orientation.w,
                diag.shoulder_v_elbow[0], diag.shoulder_v_elbow[1], diag.shoulder_v_elbow[2],
                diag.solved_upper_arm_dir[0], diag.solved_upper_arm_dir[1], diag.solved_upper_arm_dir[2],
                chain.c_str(),
                diag.q_joints_rad[0] * kRad2Deg, diag.q_joints_rad[1] * kRad2Deg,
                diag.q_joints_rad[2] * kRad2Deg, diag.q_joints_rad[3] * kRad2Deg,
                diag.q_joints_rad[4] * kRad2Deg, diag.q_joints_rad[5] * kRad2Deg,
                diag.q_joints_rad[6] * kRad2Deg);
            return;
        }
        RCLCPP_INFO(
            logger,
            "Arm %zu IK | shoulder_T_ee=[%.4f, %.4f, %.4f] quat=[%.4f, %.4f, %.4f, %.4f] "
            "| elbow=[%.3f, %.3f, %.3f] | %s "
            "| q=[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f] deg",
            arm,
            diag.base_T_ee.pose.position.x, diag.base_T_ee.pose.position.y,
            diag.base_T_ee.pose.position.z - dh_d1_,
            diag.base_T_ee.pose.orientation.x, diag.base_T_ee.pose.orientation.y,
            diag.base_T_ee.pose.orientation.z, diag.base_T_ee.pose.orientation.w,
            diag.shoulder_v_elbow[0], diag.shoulder_v_elbow[1], diag.shoulder_v_elbow[2],
            chain.c_str(),
            diag.q_joints_rad[0] * kRad2Deg, diag.q_joints_rad[1] * kRad2Deg,
            diag.q_joints_rad[2] * kRad2Deg, diag.q_joints_rad[3] * kRad2Deg,
            diag.q_joints_rad[4] * kRad2Deg, diag.q_joints_rad[5] * kRad2Deg,
            diag.q_joints_rad[6] * kRad2Deg);
        return;
    }

    RCLCPP_WARN(
        logger,
        "Arm %zu IK | shoulder_T_ee=[%.4f, %.4f, %.4f] quat=[%.4f, %.4f, %.4f, %.4f] "
        "| elbow=[%.3f, %.3f, %.3f] | %s",
        arm,
        diag.base_T_ee.pose.position.x, diag.base_T_ee.pose.position.y,
        diag.base_T_ee.pose.position.z - dh_d1_,
        diag.base_T_ee.pose.orientation.x, diag.base_T_ee.pose.orientation.y,
        diag.base_T_ee.pose.orientation.z, diag.base_T_ee.pose.orientation.w,
        diag.shoulder_v_elbow[0], diag.shoulder_v_elbow[1], diag.shoulder_v_elbow[2],
        chain.c_str());
}

void TrackerTeleopController::publishVizMarkers(size_t arm, const ArmDiagnostics &diag)
{
    if (!pub_viz_markers_ || !diag.has_base_T_ee) {
        return;
    }

    visualization_msgs::msg::MarkerArray viz_markers;
    auto now_stamp = get_node()->get_clock()->now();
    const std::string &viz_frame = viz_base_frames_[arm];
    const double ox = 0.0, oy = 0.0, oz = dh_d1_;

    visualization_msgs::msg::Marker ee_arrow;
    ee_arrow.header.frame_id = viz_frame;
    ee_arrow.header.stamp = now_stamp;
    ee_arrow.ns = "ee_position";
    ee_arrow.id = static_cast<int>(arm);
    ee_arrow.type = visualization_msgs::msg::Marker::ARROW;
    ee_arrow.action = visualization_msgs::msg::Marker::ADD;
    geometry_msgs::msg::Point p0, p1;
    p0.x = ox; p0.y = oy; p0.z = oz;
    p1.x = diag.base_T_ee.pose.position.x;
    p1.y = diag.base_T_ee.pose.position.y;
    p1.z = diag.base_T_ee.pose.position.z;
    ee_arrow.points.push_back(p0);
    ee_arrow.points.push_back(p1);
    ee_arrow.scale.x = 0.008;
    ee_arrow.scale.y = 0.015;
    ee_arrow.scale.z = 0.02;
    ee_arrow.color.r = (arm == 0) ? 1.0f : 0.2f;
    ee_arrow.color.g = (arm == 0) ? 0.2f : 0.5f;
    ee_arrow.color.b = (arm == 0) ? 0.2f : 1.0f;
    ee_arrow.color.a = 1.0f;
    viz_markers.markers.push_back(ee_arrow);

    auto append_delete_marker = [&](const char *ns) {
        visualization_msgs::msg::Marker marker;
        marker.header = ee_arrow.header;
        marker.ns = ns;
        marker.id = static_cast<int>(arm);
        marker.action = visualization_msgs::msg::Marker::DELETE;
        viz_markers.markers.push_back(marker);
    };

    constexpr double kElbowVizScale = 0.3;
    if (diag.arm_valid) {
        visualization_msgs::msg::Marker elbow_arrow;
        elbow_arrow.header = ee_arrow.header;
        elbow_arrow.ns = "elbow_direction";
        elbow_arrow.id = static_cast<int>(arm);
        elbow_arrow.type = visualization_msgs::msg::Marker::ARROW;
        elbow_arrow.action = visualization_msgs::msg::Marker::ADD;
        geometry_msgs::msg::Point e0, e1;
        e0.x = ox; e0.y = oy; e0.z = oz;
        e1.x = ox + diag.shoulder_v_elbow[0] * kElbowVizScale;
        e1.y = oy + diag.shoulder_v_elbow[1] * kElbowVizScale;
        e1.z = oz + diag.shoulder_v_elbow[2] * kElbowVizScale;
        elbow_arrow.points.push_back(e0);
        elbow_arrow.points.push_back(e1);
        elbow_arrow.scale = ee_arrow.scale;
        elbow_arrow.color.r = (arm == 0) ? 1.0f : 0.0f;
        elbow_arrow.color.g = (arm == 0) ? 0.8f : 0.8f;
        elbow_arrow.color.b = (arm == 0) ? 0.0f : 1.0f;
        elbow_arrow.color.a = 1.0f;
        viz_markers.markers.push_back(elbow_arrow);

        if (diag.solved_upper_arm_dir_valid) {
            visualization_msgs::msg::Marker solved_elbow_arrow;
            solved_elbow_arrow.header = ee_arrow.header;
            solved_elbow_arrow.ns = "solved_elbow_direction";
            solved_elbow_arrow.id = static_cast<int>(arm);
            solved_elbow_arrow.type = visualization_msgs::msg::Marker::ARROW;
            solved_elbow_arrow.action = visualization_msgs::msg::Marker::ADD;
            geometry_msgs::msg::Point s0, s1;
            s0.x = ox; s0.y = oy; s0.z = oz;
            s1.x = ox + diag.solved_upper_arm_dir[0] * kElbowVizScale;
            s1.y = oy + diag.solved_upper_arm_dir[1] * kElbowVizScale;
            s1.z = oz + diag.solved_upper_arm_dir[2] * kElbowVizScale;
            solved_elbow_arrow.points.push_back(s0);
            solved_elbow_arrow.points.push_back(s1);
            solved_elbow_arrow.scale = ee_arrow.scale;
            solved_elbow_arrow.color.r = 0.1f;
            solved_elbow_arrow.color.g = 1.0f;
            solved_elbow_arrow.color.b = 0.2f;
            solved_elbow_arrow.color.a = 1.0f;
            viz_markers.markers.push_back(solved_elbow_arrow);
        } else {
            append_delete_marker("solved_elbow_direction");
        }
    } else {
        append_delete_marker("elbow_direction");
        append_delete_marker("solved_elbow_direction");
    }

    pub_viz_markers_->publish(viz_markers);
    marker_visible_[arm] = true;
}

void TrackerTeleopController::clearVizMarkers(size_t arm)
{
    if (!pub_viz_markers_ || !marker_visible_[arm]) {
        return;
    }

    visualization_msgs::msg::MarkerArray clear_markers;
    for (const char *ns : {"ee_position", "elbow_direction", "solved_elbow_direction"}) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = viz_base_frames_[arm];
        marker.header.stamp = get_node()->get_clock()->now();
        marker.ns = ns;
        marker.id = static_cast<int>(arm);
        marker.action = visualization_msgs::msg::Marker::DELETE;
        clear_markers.markers.push_back(marker);
    }
    pub_viz_markers_->publish(clear_markers);
    marker_visible_[arm] = false;
}

void TrackerTeleopController::diagnosticsTimerCallback()
{
    const auto logger = get_node()->get_logger();
    std::array<ArmDiagnostics, kArmCount> diagnostics;
    std::array<bool, kArmCount> has_pending{{false, false}};

    for (size_t arm = 0; arm < kArmCount; ++arm) {
        auto &slot = pending_diagnostics_[arm];
        if (!slot.pending.exchange(false, std::memory_order_acq_rel)) {
            continue;
        }

        while (true) {
            const uint64_t seq_begin = slot.sequence.load(std::memory_order_acquire);
            if ((seq_begin & 1U) != 0U) {
                continue;
            }

            diagnostics[arm] = slot.snapshot;

            const uint64_t seq_end = slot.sequence.load(std::memory_order_acquire);
            if (seq_begin == seq_end) {
                has_pending[arm] = true;
                break;
            }
        }
    }

    for (size_t arm = 0; arm < kArmCount; ++arm) {
        if (!has_pending[arm]) {
            continue;
        }

        const auto &diag = diagnostics[arm];

        publishIKStatus(arm, diag.final_result);

        if (!diag.tracker_fresh) {
            clearVizMarkers(arm);
            RCLCPP_WARN(logger,
                        "Arm %zu tracker data unavailable or stale; holding current joint command.",
                        arm);
            continue;
        }

        if (diag.has_base_T_ee) {
            publishVizMarkers(arm, diag);
        }

        logArmDiagnostics(arm, diag);
    }
}

void TrackerTeleopController::computeAndPublishFK()
{
    const auto logger = get_node()->get_logger();
    static const char *arm_labels[] = {"LEFT", "RIGHT"};

    for (size_t arm = 0; arm < kArmCount; ++arm) {
        FX_DOUBLE joints[kJointsPerArm];
        for (size_t j = 0; j < kJointsPerArm; ++j) {
            joints[j] = last_joint_deg_[arm][j];
        }

        Matrix4 tcp_mat;
        if (!FX_Robot_Kine_FK(static_cast<FX_INT32L>(arm), joints, tcp_mat)) {
            RCLCPP_WARN(logger, "FK failed for arm %s", arm_labels[arm]);
            continue;
        }

        geometry_msgs::msg::PoseStamped base_T_tcp;
        base_T_tcp.header.frame_id = base_frame_;
        base_T_tcp.header.stamp = get_node()->get_clock()->now();
        matrix4ToPose(tcp_mat, base_T_tcp);
        pub_current_pose_[arm]->publish(base_T_tcp);

        RCLCPP_INFO(logger,
            "Arm %s FK base_T_tcp: pos=(%.4f, %.4f, %.4f) m  quat=(%.4f, %.4f, %.4f, %.4f)",
            arm_labels[arm],
            base_T_tcp.pose.position.x, base_T_tcp.pose.position.y, base_T_tcp.pose.position.z,
            base_T_tcp.pose.orientation.x, base_T_tcp.pose.orientation.y,
            base_T_tcp.pose.orientation.z, base_T_tcp.pose.orientation.w);
    }
}

static tf2::Transform readRigidTransform(
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
    node->get_parameter("enable_orientation", enable_orientation_);
    node->get_parameter("base_frame", base_frame_);
    node->get_parameter("j4_bound", j4_bound_);
    node->get_parameter("dh_d1", dh_d1_);
    node->get_parameter("smoothing_alpha", smoothing_alpha_);
    node->get_parameter("max_joint_velocity", max_joint_velocity_);
    node->get_parameter("base_x_scale", base_x_scale_);
    node->get_parameter("tracking_ik.fk_accept_tol", tracking_ik_config_.fk_accept_tol);
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

    std::vector<double> default_elbow_direction;
    if (node->get_parameter("default_elbow_direction", default_elbow_direction) &&
        default_elbow_direction.size() >= 3) {
        shoulder_v_elbow_default_ = {
            {default_elbow_direction[0], default_elbow_direction[1], default_elbow_direction[2]}};
    }

    smoothing_alpha_ = std::clamp(smoothing_alpha_, 0.01, 1.0);
    tracking_ik_config_.fk_accept_tol = std::max(1e-9, tracking_ik_config_.fk_accept_tol);
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
        home_joints_rad_[kLeft][joint] = home_left[joint];
        home_joints_rad_[kRight][joint] = home_right[joint];
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

    RCLCPP_INFO(
        logger, "Default shoulder_v_elbow: [%.2f, %.2f, %.2f]",
        shoulder_v_elbow_default_[0], shoulder_v_elbow_default_[1], shoulder_v_elbow_default_[2]);
    RCLCPP_INFO(logger, "J4 bound: %.2f deg", j4_bound_);
    RCLCPP_INFO(
        logger, "Smoothing: alpha=%.3f, max_vel=%.2f rad/s",
        smoothing_alpha_, max_joint_velocity_);
    RCLCPP_INFO(logger, "Base X scale: %.3f", base_x_scale_);
    RCLCPP_INFO(
        logger,
        "Tracking IK: fk_tol=%.3e fast[range=%.1f step=%.1f] "
        "expand[range=%.1f step=%.1f] score[desired=%.3f continuity=%.3f mag=%.3f psi=%.3f branch=%.3f]",
        tracking_ik_config_.fk_accept_tol,
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
        logger, "Teleop gate: startup=%s, control=service + keyboard helper",
        teleopStateToString(getTeleopState()));
}

void TrackerTeleopController::resetTrackerState()
{
    last_hand_tf_stamp_.fill(rclcpp::Time(0, 0, RCL_ROS_TIME));
    last_arm_tf_stamp_.fill(rclcpp::Time(0, 0, RCL_ROS_TIME));
    last_hand_tf_valid_.fill(false);
    last_arm_tf_valid_.fill(false);
    last_hand_tf_fresh_.fill(false);
    last_arm_tf_fresh_.fill(false);
    tracker_fresh_.fill(false);
    marker_visible_.fill(false);
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
    for (auto &arm : last_joint_deg_) arm.fill(0.0);
    for (auto &dir : last_selected_ref_dir_) {
        dir = {{shoulder_v_elbow_default_[0], shoulder_v_elbow_default_[1], shoulder_v_elbow_default_[2]}};
    }
    last_selected_branch_.fill(-1);
    for (auto &arm : smoothed_joints_rad_) arm.fill(0.0);
    for (auto &arm : target_joints_rad_) arm.fill(0.0);
    has_valid_target_.fill(false);
    cmd_interfaces_.fill(nullptr);
    state_interfaces_pos_.fill(nullptr);
    for (auto &slot : pending_diagnostics_) {
        slot.sequence.store(0, std::memory_order_relaxed);
        slot.pending.store(false, std::memory_order_relaxed);
        slot.snapshot = ArmDiagnostics();
    }
    last_ik_result_.fill(IKResult::kNoTarget);
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

controller_interface::CallbackReturn
TrackerTeleopController::on_configure(const rclcpp_lifecycle::State &)
{
    if (!readJointNames() || !initializeKinematics() || !loadControllerParameters()) {
        return CallbackReturn::ERROR;
    }

    resetRuntimeState();
    createRosInterfaces();

    const auto logger = get_node()->get_logger();
    RCLCPP_INFO(logger, "TrackerTeleopController configured (scale=%.2f, orientation=%s)",
                position_scale_, enable_orientation_ ? "ON" : "OFF");
    publishTeleopState("configured: teleop locked until armed");
    return CallbackReturn::SUCCESS;
}

bool TrackerTeleopController::bindJointInterfaces()
{
    const auto logger = get_node()->get_logger();

    for (size_t index = 0; index < kTotalJoints; ++index) {
        const std::string interface_name = joint_names_[index] + "/position";

        auto *cmd = find_loaned_interface(command_interfaces_, interface_name);
        if (!cmd) {
            RCLCPP_ERROR(logger, "Missing command interface '%s'.", interface_name.c_str());
            return false;
        }
        cmd_interfaces_[index] = cmd;

        auto *state = find_loaned_interface(state_interfaces_, interface_name);
        if (!state) {
            RCLCPP_ERROR(logger, "Missing state interface '%s'.", interface_name.c_str());
            return false;
        }
        state_interfaces_pos_[index] = state;
    }

    return true;
}

void TrackerTeleopController::initializeJointTargetsFromState()
{
    for (size_t index = 0; index < kTotalJoints; ++index) {
        const size_t arm = index / kJointsPerArm;
        const size_t joint = index % kJointsPerArm;
        const auto pos_opt = state_interfaces_pos_[index]->get_optional<double>();
        const double pos_rad = pos_opt.has_value() ? pos_opt.value() : 0.0;

        last_joint_deg_[arm][joint] = pos_rad * kRad2Deg;
        smoothed_joints_rad_[arm][joint] = pos_rad;
        target_joints_rad_[arm][joint] = pos_rad;
        (void)cmd_interfaces_[index]->set_value(pos_rad);
    }

    for (size_t arm = 0; arm < kArmCount; ++arm) {
        last_selected_ref_dir_[arm] = {
            {shoulder_v_elbow_default_[0], shoulder_v_elbow_default_[1], shoulder_v_elbow_default_[2]}};
        normalizeVector(last_selected_ref_dir_[arm]);
        last_selected_branch_[arm] = -1;
        (void)seedTrackingStateFromCurrentJoints(arm);
    }
    has_valid_target_.fill(true);
}

bool TrackerTeleopController::seedTrackingStateFromCurrentJoints(size_t arm)
{
    if (arm >= kArmCount) {
        return false;
    }

    const auto logger = get_node()->get_logger();
    std::array<double, 3> seeded_ref_dir = last_selected_ref_dir_[arm];
    if (!extractSolvedUpperArmDir(
            static_cast<FX_INT32L>(arm), smoothed_joints_rad_[arm], seeded_ref_dir)) {
        RCLCPP_WARN(
            logger,
            "Failed to extract current upper-arm direction for %s arm; "
            "keeping default startup ref_dir and branch.",
            kSideTags[arm]);
        return false;
    }

    last_selected_ref_dir_[arm] = seeded_ref_dir;

    FX_DOUBLE joints_deg[kJointsPerArm];
    Matrix4 tcp_mat{};
    for (size_t j = 0; j < kJointsPerArm; ++j) {
        joints_deg[j] = last_joint_deg_[arm][j];
    }

    if (!FX_Robot_Kine_FK(static_cast<FX_INT32L>(arm), joints_deg, tcp_mat)) {
        RCLCPP_WARN(
            logger,
            "Failed to compute FK for %s arm startup pose; "
            "seeded ref_dir only, branch remains unknown.",
            kSideTags[arm]);
        return false;
    }

    geometry_msgs::msg::PoseStamped base_T_ee;
    base_T_ee.header.frame_id = base_frame_;
    base_T_ee.header.stamp = get_node()->get_clock()->now();
    matrix4ToPose(tcp_mat, base_T_ee);

    tracking_ik::Request request;
    tracking_ik::Result result{};
    tracking_ik::SetDefaultRequest(&request);
    poseToMatrix4(base_T_ee, request.target_tcp);
    request.desired_upper_arm_dir[0] = seeded_ref_dir[0];
    request.desired_upper_arm_dir[1] = seeded_ref_dir[1];
    request.desired_upper_arm_dir[2] = seeded_ref_dir[2];
    request.prev_selected_ref_dir[0] = seeded_ref_dir[0];
    request.prev_selected_ref_dir[1] = seeded_ref_dir[1];
    request.prev_selected_ref_dir[2] = seeded_ref_dir[2];
    request.prev_selected_branch = -1;
    for (size_t j = 0; j < kJointsPerArm; ++j) {
        request.ref_joint_deg[j] = last_joint_deg_[arm][j];
    }

    request.fk_accept_tol = tracking_ik_config_.fk_accept_tol;
    request.fast_psi_range_deg = tracking_ik_config_.fast_psi_range_deg;
    request.fast_psi_step_deg = tracking_ik_config_.fast_psi_step_deg;
    request.expand_psi_range_deg = tracking_ik_config_.expand_psi_range_deg;
    request.expand_psi_step_deg = tracking_ik_config_.expand_psi_step_deg;
    request.score_params.desired_dir_weight =
        tracking_ik_config_.desired_dir_weight;
    request.score_params.continuity_dir_weight =
        tracking_ik_config_.continuity_dir_weight;
    request.score_params.magnitude_weight =
        tracking_ik_config_.magnitude_weight;
    request.score_params.psi_delta_weight =
        tracking_ik_config_.psi_delta_weight;
    request.score_params.branch_switch_penalty =
        tracking_ik_config_.branch_switch_penalty;

    if (!tracking_ik::Solve(&tracking_ik_geometry_, &request, &result)) {
        RCLCPP_WARN(
            logger,
            "Failed to seed startup branch for %s arm from current pose; "
            "using current upper-arm dir only.",
            kSideTags[arm]);
        return false;
    }

    if (!result.success) {
        RCLCPP_WARN(
            logger,
            "trackingIK could not classify %s arm startup pose; "
            "using current upper-arm dir only.",
            kSideTags[arm]);
        return false;
    }

    last_selected_ref_dir_[arm] = {
        {result.selected_ref_dir[0], result.selected_ref_dir[1], result.selected_ref_dir[2]}};
    normalizeVector(last_selected_ref_dir_[arm]);
    last_selected_branch_[arm] = result.selected_branch;

    RCLCPP_INFO(
        logger,
        "Seeded %s arm startup tracking state from hardware pose: "
        "branch=%ld ref_dir=[%.3f, %.3f, %.3f] psi=%.1f deg",
        kSideTags[arm],
        static_cast<long>(last_selected_branch_[arm]),
        last_selected_ref_dir_[arm][0],
        last_selected_ref_dir_[arm][1],
        last_selected_ref_dir_[arm][2],
        result.selected_psi_deg);
    return true;
}

controller_interface::CallbackReturn
TrackerTeleopController::on_activate(const rclcpp_lifecycle::State &)
{
    if (!bindJointInterfaces()) {
        return CallbackReturn::ERROR;
    }

    initializeJointTargetsFromState();
    resetTrackerState();
    resetTeleopRuntime(TeleopState::kDisarmed);
    setTeleopState(TeleopState::kDisarmed, "controller activated: teleop locked");

    computeAndPublishFK();

    const auto logger = get_node()->get_logger();
    RCLCPP_INFO(
        logger,
        "TrackerTeleopController activated (absolute mapping mode, startup state=%s).",
        teleopStateToString(getTeleopState()));
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
TrackerTeleopController::on_deactivate(const rclcpp_lifecycle::State &)
{
    resetTeleopRuntime(TeleopState::kDisarmed);
    cmd_interfaces_.fill(nullptr);
    state_interfaces_pos_.fill(nullptr);
    return CallbackReturn::SUCCESS;
}

/*
 * 位姿链（与 TF 一致，均为 ROS 系）：
 *   vr_T_tracker → corrections.yaml → ros_T_world_tracker (tracker_publisher)
 *   chest_T_wrist = (ros_T_chest)^{-1} * ros_T_wrist  [TF: torso_T_hand]
 *   shoulder_T_wrist  = shoulder_T_chest * chest_T_wrist
 *   shoulder_T_ee     = shoulder_T_wrist * wrist_T_ee
 *   shoulder_R_arm_robot = shoulder_R_chest * chest_R_arm * arm_human_R_arm_robot
 *   shoulder_v_elbow = Y 列 of shoulder_R_arm_robot → IK 零空间
 *
 * NOTE: marvin SDK 的 IK 输入为 base_T_ee（base 坐标系下的末端位姿）。
 *       shoulder_T_ee 需加 [0, 0, +d1] 偏置转换为 base_T_ee 后传入 IK。
 *       肘向量原点在 shoulder，直接传入 IK 即可。
 *
 * Runtime behavior:
 *   - TF lookups run in a non-RT timer (pollTfCallback, 200 Hz).
 *   - update() only consumes cached TF, updates IK targets, and applies smoothing.
 *   - Diagnostics, marker publication, and IK status publication run in a
 *     separate non-RT timer, outside the main control loop.
 *   - When tracker data times out, the controller holds the current joint command
 *     instead of continuing toward the last IK target.
 */
controller_interface::return_type
TrackerTeleopController::update(const rclcpp::Time &, const rclcpp::Duration &period)
{
    if (!kine_initialized_) {
        return controller_interface::return_type::OK;
    }

    const double dt = period.seconds();
    const auto now = get_node()->get_clock()->now();
    (void)updateTfSnapshot();
    if (go_home_requested_.load(std::memory_order_relaxed) ||
        go_home_active_.load(std::memory_order_relaxed)) {
        processGoHome(dt);
        return controller_interface::return_type::OK;
    }

    const bool teleop_enabled = isTeleopEnabled(now);
    const bool force_reacquire = teleop_enabled &&
        force_tracker_reacquire_.exchange(false, std::memory_order_relaxed);

    if (!teleop_enabled) {
        holdAllArms(dt);
        return controller_interface::return_type::OK;
    }

    for (size_t arm = 0; arm < kArmCount; ++arm) {
        processArmUpdate(arm, tf_snapshot_[arm], now, dt, force_reacquire);
    }

    return controller_interface::return_type::OK;
}

}  // namespace marvin_system

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(marvin_system::TrackerTeleopController,
                       controller_interface::ControllerInterface)
