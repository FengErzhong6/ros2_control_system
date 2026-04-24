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
        diag->request_prev_selected_ref_dir = {{0.0, 0.0, -1.0}};
        diag->request_prev_selected_branch = -1;
        diag->selected_ref_dir = {{0.0, 0.0, -1.0}};
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

    if (diag) {
        diag->request_prev_selected_ref_dir = {{
            request.prev_selected_ref_dir[0],
            request.prev_selected_ref_dir[1],
            request.prev_selected_ref_dir[2]}};
        diag->request_prev_selected_branch = request.prev_selected_branch;
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
        diag->selected_ref_dir = {{
            result.selected_ref_dir[0],
            result.selected_ref_dir[1],
            result.selected_ref_dir[2]}};
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

void TrackerTeleopController::enqueueAnalysisSample(
    size_t arm,
    const rclcpp::Time &sample_time,
    bool tracker_input_changed)
{
    if (!enable_analysis_recording_ ||
        !analysis_record_active_.load(std::memory_order_relaxed) ||
        arm >= kArmCount) {
        return;
    }

    const auto &runtime = arm_state_[arm];
    auto &queue = analysis_record_queue_;
    const size_t write_index = queue.write_index.load(std::memory_order_relaxed);
    const size_t next_index = (write_index + 1) % kAnalysisRecordQueueCapacity;
    const size_t read_index = queue.read_index.load(std::memory_order_acquire);
    if (next_index == read_index) {
        queue.dropped_count.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    auto &sample = queue.samples[write_index];
    sample.sample_stamp_ns = sample_time.nanoseconds();
    sample.target_stamp_ns =
        runtime.analysis_has_base_T_ee ?
            rclcpp::Time(runtime.analysis_base_T_ee.header.stamp).nanoseconds() :
            0;
    sample.arm = static_cast<uint8_t>(arm);
    sample.tracker_fresh = runtime.tracker_fresh;
    sample.tracker_input_changed = tracker_input_changed;
    sample.has_base_T_ee = runtime.analysis_has_base_T_ee;
    sample.ik_result = static_cast<int8_t>(runtime.analysis_ik_result);
    sample.has_ik_solution = runtime.analysis_has_ik_solution;
    sample.interp_active = runtime.interp_active;
    sample.interp_step = runtime.interp_step;
    sample.interp_total_steps = runtime.interp_total_steps;

    if (runtime.analysis_has_base_T_ee) {
        sample.base_t_ee_pos_x = runtime.analysis_base_T_ee.pose.position.x;
        sample.base_t_ee_pos_y = runtime.analysis_base_T_ee.pose.position.y;
        sample.base_t_ee_pos_z = runtime.analysis_base_T_ee.pose.position.z;
        sample.base_t_ee_quat_x = runtime.analysis_base_T_ee.pose.orientation.x;
        sample.base_t_ee_quat_y = runtime.analysis_base_T_ee.pose.orientation.y;
        sample.base_t_ee_quat_z = runtime.analysis_base_T_ee.pose.orientation.z;
        sample.base_t_ee_quat_w = runtime.analysis_base_T_ee.pose.orientation.w;
    } else {
        sample.base_t_ee_pos_x = 0.0;
        sample.base_t_ee_pos_y = 0.0;
        sample.base_t_ee_pos_z = 0.0;
        sample.base_t_ee_quat_x = 0.0;
        sample.base_t_ee_quat_y = 0.0;
        sample.base_t_ee_quat_z = 0.0;
        sample.base_t_ee_quat_w = 1.0;
    }

    std::array<double, kJointsPerArm> state_joint_rad{};
    sample.has_state_joint = readCurrentJointPositions(arm, state_joint_rad);
    const SdkObservationState sdk_observation = readSdkObservationState(arm);
    sample.has_control_profile = sdk_observation.has_control_profile;
    sample.active_control_profile = sdk_observation.active_control_profile;
    sample.requested_control_profile = sdk_observation.requested_control_profile;
    sample.has_sdk_diag = sdk_observation.has_sdk_diag;
    sample.sdk_cur_state = sdk_observation.sdk_cur_state;
    sample.sdk_cmd_state = sdk_observation.sdk_cmd_state;
    sample.sdk_err_code = sdk_observation.sdk_err_code;
    sample.sdk_in_frame_serial = sdk_observation.sdk_in_frame_serial;
    sample.sdk_out_frame_serial = sdk_observation.sdk_out_frame_serial;
    sample.has_sdk_command_joint = sdk_observation.has_sdk_command_joint;

    for (size_t joint = 0; joint < kJointsPerArm; ++joint) {
        sample.ik_solution_joint_deg[joint] =
            runtime.analysis_ik_solution_rad[joint] * kRad2Deg;
        sample.command_joint_deg[joint] =
            runtime.smoothed_joints_rad[joint] * kRad2Deg;
        sample.sdk_command_joint_deg[joint] =
            sample.has_sdk_command_joint ?
                sdk_observation.sdk_command_joints_rad[joint] * kRad2Deg :
                0.0;
        sample.state_joint_deg[joint] =
            sample.has_state_joint ? state_joint_rad[joint] * kRad2Deg : 0.0;
    }

    queue.write_index.store(next_index, std::memory_order_release);
}

void TrackerTeleopController::holdCurrentPosition(size_t arm)
{
    auto &runtime = arm_state_[arm];
    cancelTrackerInterpolation(arm);
    for (size_t j = 0; j < kJointsPerArm; ++j) {
        runtime.target_joints_rad[j] = runtime.smoothed_joints_rad[j];
    }
    runtime.has_valid_target = true;
}

bool TrackerTeleopController::syncCommandStateToMeasuredPose(size_t arm)
{
    if (arm >= kArmCount) {
        return false;
    }

    std::array<double, kJointsPerArm> joints_rad{};
    if (!readCurrentJointPositions(arm, joints_rad)) {
        return false;
    }

    auto &runtime = arm_state_[arm];
    runtime.smoothed_joints_rad = joints_rad;
    runtime.target_joints_rad = joints_rad;
    runtime.interp_start_joints_rad = joints_rad;
    runtime.interp_goal_joints_rad = joints_rad;
    runtime.interp_step = 0;
    runtime.interp_total_steps = 0;
    runtime.interp_active = false;
    runtime.has_valid_target = true;
    for (size_t j = 0; j < kJointsPerArm; ++j) {
        runtime.last_joint_deg[j] = joints_rad[j] * kRad2Deg;
        if (cmd_interfaces_[arm * kJointsPerArm + j]) {
            (void)cmd_interfaces_[arm * kJointsPerArm + j]->set_value(joints_rad[j]);
        }
    }
    return true;
}

void TrackerTeleopController::cancelTrackerInterpolation(size_t arm)
{
    auto &runtime = arm_state_[arm];
    runtime.interp_active = false;
    runtime.interp_step = 0;
    runtime.interp_total_steps = 0;
    runtime.interp_start_joints_rad = runtime.smoothed_joints_rad;
    runtime.interp_goal_joints_rad = runtime.smoothed_joints_rad;
}

void TrackerTeleopController::startTrackerInterpolation(
    size_t arm, const std::array<double, kJointsPerArm> &goal_joints_rad)
{
    auto &runtime = arm_state_[arm];
    runtime.interp_start_joints_rad = runtime.smoothed_joints_rad;
    runtime.interp_goal_joints_rad = goal_joints_rad;
    runtime.interp_step = 0;
    runtime.interp_total_steps = std::max(1, tracker_interp_cycles_);
    runtime.interp_active = true;
}

std::array<double, kJointsPerArm> TrackerTeleopController::lowPassFilterIkTarget(
    size_t arm, const std::array<double, kJointsPerArm> &raw_joints_rad) const
{
    if (arm >= kArmCount || smoothing_alpha_ >= 1.0) {
        return raw_joints_rad;
    }

    const auto &runtime = arm_state_[arm];
    if (!runtime.has_valid_target) {
        return raw_joints_rad;
    }

    std::array<double, kJointsPerArm> filtered_joints_rad{};
    for (size_t joint = 0; joint < kJointsPerArm; ++joint) {
        const double previous_target = runtime.target_joints_rad[joint];
        filtered_joints_rad[joint] =
            previous_target + (raw_joints_rad[joint] - previous_target) * smoothing_alpha_;
    }
    return filtered_joints_rad;
}

void TrackerTeleopController::publishJointCommand(const rclcpp::Time &stamp)
{
    if (!rt_pub_joint_command_ || !rt_pub_joint_command_->trylock()) {
        return;
    }

    auto &msg = rt_pub_joint_command_->msg_;
    msg.header.stamp = stamp;
    if (msg.name.size() != joint_names_.size()) {
        msg.name = joint_names_;
    }
    msg.position.resize(kTotalJoints);
    size_t index = 0;
    for (size_t arm = 0; arm < kArmCount; ++arm) {
        const auto &runtime = arm_state_[arm];
        for (double joint : runtime.smoothed_joints_rad) {
            msg.position[index++] = joint;
        }
    }
    msg.velocity.clear();
    msg.effort.clear();
    rt_pub_joint_command_->unlockAndPublish();
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
    std::array<double, kJointsPerArm> desired_joints_rad = runtime.target_joints_rad;

    if (runtime.interp_active) {
        const double t = static_cast<double>(runtime.interp_step + 1) /
                         static_cast<double>(std::max(1, runtime.interp_total_steps));
        for (size_t j = 0; j < kJointsPerArm; ++j) {
            desired_joints_rad[j] =
                runtime.interp_start_joints_rad[j] +
                (runtime.interp_goal_joints_rad[j] - runtime.interp_start_joints_rad[j]) * t;
        }

        runtime.interp_step++;
        if (runtime.interp_step >= runtime.interp_total_steps) {
            runtime.interp_active = false;
            runtime.interp_step = 0;
            runtime.interp_total_steps = 0;
            runtime.interp_start_joints_rad = runtime.interp_goal_joints_rad;
        }
    }

    for (size_t j = 0; j < kJointsPerArm; ++j) {
        double commanded = desired_joints_rad[j];
        double delta = commanded - runtime.smoothed_joints_rad[j];
        if (std::abs(delta) > max_delta) {
            commanded = runtime.smoothed_joints_rad[j] + std::copysign(max_delta, delta);
        }

        runtime.smoothed_joints_rad[j] = commanded;
        (void)cmd_interfaces_[offset + j]->set_value(commanded);
        runtime.last_joint_deg[j] = commanded * kRad2Deg;
    }
}

void TrackerTeleopController::fillArmTargetFromTracker(
    size_t arm, const CachedTrackerData &snap,
    geometry_msgs::msg::PoseStamped &base_T_ee,
    std::array<double, 3> &shoulder_v_elbow,
    std::array<double, 3> *tracker_y_axis) const
{
    const tf2::Transform &shoulder_T_chest = shoulder_T_chest_[arm];
    const auto &tf_wrist = snap.chest_T_hand.transform;

    tf2::Transform chest_T_wrist(
        tf2::Quaternion(tf_wrist.rotation.x, tf_wrist.rotation.y,
                       tf_wrist.rotation.z, tf_wrist.rotation.w),
        tf2::Vector3(tf_wrist.translation.x * position_scale_,
                    tf_wrist.translation.y * position_scale_,
                    tf_wrist.translation.z * position_scale_));

    tf2::Transform shoulder_T_wrist = shoulder_T_chest * chest_T_wrist;
    tf2::Transform shoulder_T_ee_tf = shoulder_T_wrist * wrist_T_ee_[arm];

    tf2::Vector3 ee_pos = shoulder_T_ee_tf.getOrigin();
    double offset_x = ee_pos.x() - shoulder_T_chest.getOrigin().x();
    ee_pos.setX(shoulder_T_chest.getOrigin().x() + offset_x * base_x_scale_);
    shoulder_T_ee_tf.setOrigin(ee_pos);

    base_T_ee.header.stamp = snap.chest_T_hand.header.stamp;
    base_T_ee.pose.position.x = shoulder_T_ee_tf.getOrigin().x();
    base_T_ee.pose.position.y = shoulder_T_ee_tf.getOrigin().y();
    base_T_ee.pose.position.z = shoulder_T_ee_tf.getOrigin().z() + dh_d1_;
    const tf2::Quaternion &ee_q = shoulder_T_ee_tf.getRotation();
    base_T_ee.pose.orientation.x = ee_q.x();
    base_T_ee.pose.orientation.y = ee_q.y();
    base_T_ee.pose.orientation.z = ee_q.z();
    base_T_ee.pose.orientation.w = ee_q.w();

    shoulder_v_elbow = kDefaultShoulderVElbow;
    if (tracker_y_axis) {
        *tracker_y_axis = kDefaultShoulderVElbow;
    }
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
    const tf2::Vector3 raw_tracker_y = shoulder_M_arm_robot.getColumn(1);
    if (tracker_y_axis) {
        *tracker_y_axis = {{raw_tracker_y.x(), raw_tracker_y.y(), raw_tracker_y.z()}};
    }
    tf2::Vector3 corrected = tf2::quatRotate(
        elbow_dir_correction_[arm], raw_tracker_y);
    shoulder_v_elbow = {{corrected.x(), corrected.y(), corrected.z()}};
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
    runtime.analysis_has_base_T_ee = false;
    runtime.analysis_ik_result = IKResult::kNoTarget;
    runtime.analysis_has_ik_solution = false;
    runtime.analysis_ik_solution_rad.fill(0.0);
    holdCurrentPosition(arm);
}

void TrackerTeleopController::handleFreshTrackerUpdate(size_t arm, const CachedTrackerData &snap)
{
    auto &runtime = arm_state_[arm];
    runtime.tracker_fresh = true;

    geometry_msgs::msg::PoseStamped base_T_ee;
    std::array<double, 3> shoulder_v_elbow{};
    std::array<double, 3> tracker_y_axis{};
    fillArmTargetFromTracker(arm, snap, base_T_ee, shoulder_v_elbow, &tracker_y_axis);
    if (!normalizeVector(shoulder_v_elbow)) {
        shoulder_v_elbow = kDefaultShoulderVElbow;
    }

    ArmDiagnostics diag;
    diag.tracker_fresh = true;
    diag.hand_valid = snap.hand_valid;
    diag.arm_valid = snap.arm_valid;
    diag.has_base_T_ee = true;
    diag.base_T_ee = base_T_ee;
    diag.tracker_y_axis_valid = snap.arm_valid;
    diag.tracker_y_axis = tracker_y_axis;
    diag.shoulder_v_elbow = shoulder_v_elbow;

    const double shoulder_v_elbow_arr[3] = {
        shoulder_v_elbow[0], shoulder_v_elbow[1], shoulder_v_elbow[2]};
    double out_q_joints_rad[kJointsPerArm];
    IKResult ik_result = solveIK(
        arm, base_T_ee, shoulder_v_elbow_arr, out_q_joints_rad, &diag);
    queueDiagnostics(arm, diag);

    runtime.analysis_has_base_T_ee = true;
    runtime.analysis_base_T_ee = base_T_ee;
    runtime.analysis_ik_result = ik_result;
    runtime.analysis_has_ik_solution =
        ik_result == IKResult::kSuccess || ik_result == IKResult::kJointLimitClamped;
    runtime.analysis_ik_solution_rad.fill(0.0);
    if (runtime.analysis_has_ik_solution) {
        for (size_t j = 0; j < kJointsPerArm; ++j) {
            runtime.analysis_ik_solution_rad[j] = out_q_joints_rad[j];
        }
    }

    if (ik_result == IKResult::kSuccess ||
        ik_result == IKResult::kJointLimitClamped) {
        std::array<double, kJointsPerArm> raw_goal_joints_rad{};
        for (size_t j = 0; j < kJointsPerArm; ++j) {
            raw_goal_joints_rad[j] = out_q_joints_rad[j];
        }

        const auto goal_joints_rad = lowPassFilterIkTarget(arm, raw_goal_joints_rad);
        for (size_t j = 0; j < kJointsPerArm; ++j) {
            runtime.target_joints_rad[j] = goal_joints_rad[j];
        }
        runtime.has_valid_target = true;
        startTrackerInterpolation(arm, goal_joints_rad);
        return;
    }

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
        enqueueAnalysisSample(arm, now, input_state.tracker_input_changed);
        return;
    }

    CachedTrackerData effective_snap = snap;
    effective_snap.arm_valid = input_state.arm_fresh;

    if (force_reacquire || input_state.tracker_input_changed) {
        handleFreshTrackerUpdate(arm, effective_snap);
    }

    applySmoothedCommand(arm, dt);
    enqueueAnalysisSample(arm, now, input_state.tracker_input_changed);
}

}  // namespace marvin_system
