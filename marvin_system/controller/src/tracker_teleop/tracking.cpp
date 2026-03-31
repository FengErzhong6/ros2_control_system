#include "internal.hpp"

#include <cmath>
#include <utility>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/exceptions.h"

namespace marvin_system {

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
    auto &runtime = arm_state_[arm];
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
        diag->best_desired_dir_score = 0.0;
        diag->best_continuity_dir_score = 0.0;
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
    if (runtime.last_selected_branch < 0) {
        request.prev_selected_ref_dir[0] = shoulder_v_elbow[0];
        request.prev_selected_ref_dir[1] = shoulder_v_elbow[1];
        request.prev_selected_ref_dir[2] = shoulder_v_elbow[2];
    } else {
        request.prev_selected_ref_dir[0] = runtime.last_selected_ref_dir[0];
        request.prev_selected_ref_dir[1] = runtime.last_selected_ref_dir[1];
        request.prev_selected_ref_dir[2] = runtime.last_selected_ref_dir[2];
    }
    request.prev_selected_branch = static_cast<FX_INT32L>(runtime.last_selected_branch);
    for (size_t j = 0; j < kJointsPerArm; ++j) {
        request.ref_joint_deg[j] = runtime.last_joint_deg[j];
    }

    request.fk_accept_tol = tracking_ik_config_.fk_accept_tol;
    request.fine_psi_range_deg = tracking_ik_config_.fine_psi_range_deg;
    request.fine_psi_step_deg = tracking_ik_config_.fine_psi_step_deg;
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
        runtime.last_selected_ref_dir = {
            {result.selected_ref_dir[0], result.selected_ref_dir[1], result.selected_ref_dir[2]}};
        runtime.last_selected_branch = result.selected_branch;
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
        diag->best_desired_dir_score = result.best_desired_dir_score;
        diag->best_continuity_dir_score = result.best_continuity_dir_score;
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
    auto &runtime = arm_state_[arm];
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

    state.hand_changed = (state.hand_stamp != runtime.last_hand_tf_stamp) ||
                         (snap.hand_valid != runtime.last_hand_tf_valid) ||
                         (state.hand_fresh != runtime.last_hand_tf_fresh);
    state.arm_changed = (state.arm_stamp != runtime.last_arm_tf_stamp) ||
                        (snap.arm_valid != runtime.last_arm_tf_valid) ||
                        (state.arm_fresh != runtime.last_arm_tf_fresh);
    state.tracker_input_changed = state.hand_changed || state.arm_changed;

    runtime.last_hand_tf_stamp = state.hand_stamp;
    runtime.last_arm_tf_stamp = state.arm_stamp;
    runtime.last_hand_tf_valid = snap.hand_valid;
    runtime.last_arm_tf_valid = snap.arm_valid;
    runtime.last_hand_tf_fresh = state.hand_fresh;
    runtime.last_arm_tf_fresh = state.arm_fresh;

    return state;
}

void TrackerTeleopController::holdCurrentPosition(size_t arm)
{
    auto &runtime = arm_state_[arm];
    for (size_t j = 0; j < kJointsPerArm; ++j) {
        runtime.target_joints_rad[j] = runtime.smoothed_joints_rad[j];
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
    auto &runtime = arm_state_[arm];
    if (!runtime.has_valid_target) {
        return;
    }

    const size_t offset = arm * kJointsPerArm;
    const double max_delta = max_joint_velocity_ * dt;

    for (size_t j = 0; j < kJointsPerArm; ++j) {
        double filtered = smoothing_alpha_ * runtime.target_joints_rad[j] +
                          (1.0 - smoothing_alpha_) * runtime.smoothed_joints_rad[j];

        double delta = filtered - runtime.smoothed_joints_rad[j];
        if (std::abs(delta) > max_delta) {
            filtered = runtime.smoothed_joints_rad[j] +
                       std::copysign(max_delta, delta);
        }

        runtime.smoothed_joints_rad[j] = filtered;
        (void)cmd_interfaces_[offset + j]->set_value(filtered);
        runtime.last_joint_deg[j] = filtered * kRad2Deg;
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

void TrackerTeleopController::filterShoulderElbowDirection(
    size_t arm, bool arm_valid, std::array<double, 3> &shoulder_v_elbow)
{
    auto &runtime = arm_state_[arm];

    std::array<double, 3> fallback_dir = shoulder_v_elbow_default_;
    if (!normalizeVector(fallback_dir)) {
        fallback_dir = {{0.0, 0.0, -1.0}};
    }

    std::array<double, 3> measured_dir = shoulder_v_elbow;
    const bool measured_valid = arm_valid && normalizeVector(measured_dir);

    if (!runtime.filtered_elbow_dir_valid) {
        runtime.filtered_elbow_dir = measured_valid ? measured_dir : fallback_dir;
        runtime.filtered_elbow_dir_valid = true;
        shoulder_v_elbow = runtime.filtered_elbow_dir;
        return;
    }

    if (!measured_valid) {
        if (!elbow_dir_filter_config_.hold_last_on_invalid) {
            runtime.filtered_elbow_dir = fallback_dir;
        }
        shoulder_v_elbow = runtime.filtered_elbow_dir;
        return;
    }

    if (angleBetweenVectorsDeg(runtime.filtered_elbow_dir, measured_dir) <=
        elbow_dir_filter_config_.deadband_deg) {
        shoulder_v_elbow = runtime.filtered_elbow_dir;
        return;
    }

    const double alpha = elbow_dir_filter_config_.alpha;
    std::array<double, 3> filtered_dir{{
        alpha * measured_dir[0] + (1.0 - alpha) * runtime.filtered_elbow_dir[0],
        alpha * measured_dir[1] + (1.0 - alpha) * runtime.filtered_elbow_dir[1],
        alpha * measured_dir[2] + (1.0 - alpha) * runtime.filtered_elbow_dir[2],
    }};
    if (!normalizeVector(filtered_dir)) {
        filtered_dir = measured_dir;
    }

    runtime.filtered_elbow_dir = filtered_dir;
    shoulder_v_elbow = filtered_dir;
}

bool TrackerTeleopController::applyTrackerTargetHysteresis(
    size_t arm,
    geometry_msgs::msg::PoseStamped &base_T_ee,
    std::array<double, 3> &shoulder_v_elbow)
{
    auto &runtime = arm_state_[arm];

    std::array<double, 3> normalized_elbow = shoulder_v_elbow;
    if (!normalizeVector(normalized_elbow)) {
        normalized_elbow = runtime.filtered_elbow_dir_valid ?
            runtime.filtered_elbow_dir :
            shoulder_v_elbow_default_;
        if (!normalizeVector(normalized_elbow)) {
            normalized_elbow = {{0.0, 0.0, -1.0}};
        }
    }
    shoulder_v_elbow = normalized_elbow;

    auto accept_current_target = [&]() {
        runtime.accepted_base_T_ee = base_T_ee;
        runtime.accepted_shoulder_v_elbow = shoulder_v_elbow;
        runtime.accepted_tracker_target_valid = true;
        runtime.tracker_deadband_active = false;
    };

    if (!runtime.accepted_tracker_target_valid) {
        accept_current_target();
        return false;
    }

    const double position_delta_m = pointDistanceMeters(
        base_T_ee.pose.position, runtime.accepted_base_T_ee.pose.position);
    const double orientation_delta_deg = quaternionAngularDistanceDeg(
        base_T_ee.pose.orientation, runtime.accepted_base_T_ee.pose.orientation);
    const double elbow_delta_deg = angleBetweenVectorsDeg(
        shoulder_v_elbow, runtime.accepted_shoulder_v_elbow);

    if (!tracker_deadband_config_.enabled) {
        accept_current_target();
        return false;
    }

    auto within_enter = [](double delta, double enter, double exit) {
        return exit <= 0.0 || delta <= enter;
    };
    auto beyond_exit = [](double delta, double exit) {
        return exit > 0.0 && delta >= exit;
    };

    const bool should_hold =
        within_enter(
            position_delta_m,
            tracker_deadband_config_.position_enter_m,
            tracker_deadband_config_.position_exit_m) &&
        within_enter(
            orientation_delta_deg,
            tracker_deadband_config_.orientation_enter_deg,
            tracker_deadband_config_.orientation_exit_deg) &&
        within_enter(
            elbow_delta_deg,
            tracker_deadband_config_.elbow_enter_deg,
            tracker_deadband_config_.elbow_exit_deg);
    const bool should_release =
        beyond_exit(position_delta_m, tracker_deadband_config_.position_exit_m) ||
        beyond_exit(orientation_delta_deg, tracker_deadband_config_.orientation_exit_deg) ||
        beyond_exit(elbow_delta_deg, tracker_deadband_config_.elbow_exit_deg);

    if (runtime.tracker_deadband_active) {
        if (!should_release) {
            base_T_ee = runtime.accepted_base_T_ee;
            shoulder_v_elbow = runtime.accepted_shoulder_v_elbow;
            return true;
        }
        runtime.tracker_deadband_active = false;
    }

    if (should_hold) {
        runtime.tracker_deadband_active = true;
        base_T_ee = runtime.accepted_base_T_ee;
        shoulder_v_elbow = runtime.accepted_shoulder_v_elbow;
        return true;
    }

    accept_current_target();
    return false;
}

void TrackerTeleopController::handleStaleTracker(
    size_t arm, const CachedTrackerData &snap, bool tracker_input_changed)
{
    auto &runtime = arm_state_[arm];
    if (runtime.tracker_fresh || tracker_input_changed) {
        ArmDiagnostics diag;
        diag.tracker_fresh = false;
        diag.hand_valid = snap.hand_valid;
        diag.arm_valid = snap.arm_valid;
        diag.final_result = IKResult::kNoTarget;
        queueDiagnostics(arm, diag);
    }
    runtime.tracker_fresh = false;
    holdCurrentPosition(arm);
}

void TrackerTeleopController::handleFreshTrackerUpdate(size_t arm, const CachedTrackerData &snap)
{
    auto &runtime = arm_state_[arm];
    runtime.tracker_fresh = true;

    geometry_msgs::msg::PoseStamped base_T_ee;
    std::array<double, 3> shoulder_v_elbow{};
    fillArmTargetFromTracker(arm, snap, base_T_ee, shoulder_v_elbow);
    filterShoulderElbowDirection(arm, snap.arm_valid, shoulder_v_elbow);
    if (applyTrackerTargetHysteresis(arm, base_T_ee, shoulder_v_elbow) &&
        runtime.last_tracker_ik_succeeded) {
        return;
    }

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
            runtime.target_joints_rad[j] = out_q_joints_rad[j];
        }
        runtime.has_valid_target = true;
        runtime.last_tracker_ik_succeeded = true;
        return;
    }

    runtime.last_tracker_ik_succeeded = false;
    holdCurrentPosition(arm);
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
        if (force_reacquire) {
            auto &runtime = arm_state_[arm];
            runtime.accepted_tracker_target_valid = false;
            runtime.tracker_deadband_active = false;
            runtime.last_tracker_ik_succeeded = false;
        }
        handleFreshTrackerUpdate(arm, effective_snap);
    }

    applySmoothedCommand(arm, dt);
}

}  // namespace marvin_system
