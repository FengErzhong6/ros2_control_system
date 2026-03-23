#include "ros2_control_marvin/tracker_teleop_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>

#include "rclcpp/logging.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using config_type = controller_interface::interface_configuration_type;

namespace ros2_control_marvin {

namespace {

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
        auto_declare<double>("zsp_angle", 0.0);
        auto_declare<double>("j4_bound", 0.0);
        auto_declare<double>("dh_d1", 0.0);
        auto_declare<std::vector<std::string>>("viz_base_frames", {"Base_L", "Base_R"});

        auto_declare<double>("smoothing_alpha", 0.3);
        auto_declare<double>("max_joint_velocity", 2.0);
        auto_declare<double>("base_x_scale", 1.0);
        auto_declare<bool>("ik_fallback_near_ref", true);
        auto_declare<bool>("ik_clamp_joint_limits", true);
        auto_declare<double>("tracker_timeout_sec", 0.1);
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

    std::array<CachedTrackerData, kArmCount> fresh;
    for (int arm = 0; arm < static_cast<int>(kArmCount); ++arm) {
        fresh[arm].hand_valid = lookupTf(
            frame_torso_, *frame_hand[arm], fresh[arm].chest_T_hand);
        fresh[arm].arm_valid = lookupTf(
            frame_torso_, *frame_arm[arm], fresh[arm].chest_T_arm);
    }

    std::lock_guard<std::mutex> lk(tf_cache_mutex_);
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
    }

    if (!isQuaternionValid(base_T_ee.pose.orientation)) {
        if (diag) {
            diag->final_result = IKResult::kInvalidQuaternion;
        }
        return IKResult::kInvalidQuaternion;
    }

    auto &ik = ik_params_[arm];
    FX_INT32L serial = static_cast<FX_INT32L>(arm);

    auto setup_common = [&]() {
        std::memset(&ik, 0, sizeof(FX_InvKineSolvePara));
        poseToMatrix4(base_T_ee, ik.m_Input_IK_TargetTCP);
        for (size_t j = 0; j < kJointsPerArm; ++j) {
            ik.m_Input_IK_RefJoint[j] = last_joint_deg_[arm][j];
        }
    };

    auto evaluate_ik = [&]() -> IKResult {
        if (!FX_Robot_Kine_IK(serial, &ik)) {
            return IKResult::kOutOfRange;
        }
        if (ik.m_Output_IsJntExd) {
            return IKResult::kJointLimitExceeded;
        }
        for (size_t j = 0; j < kJointsPerArm; ++j) {
            if (ik.m_Output_IsDeg[j]) {
                return IKResult::kSingularity;
            }
        }
        for (size_t j = 0; j < kJointsPerArm; ++j) {
            out_q_joints_rad[j] = ik.m_Output_RetJoint[j] * kDeg2Rad;
        }
        return IKResult::kSuccess;
    };

    // --- Attempt 1: NEAR_DIR ---
    setup_common();
    ik.m_Input_IK_ZSPType = FX_PILOT_NSP_TYPES_NEAR_DIR;
    ik.m_Input_IK_ZSPPara[0] = shoulder_v_elbow[0];
    ik.m_Input_IK_ZSPPara[1] = shoulder_v_elbow[1];
    ik.m_Input_IK_ZSPPara[2] = shoulder_v_elbow[2];
    ik.m_Input_ZSP_Angle = zsp_angle_;

    IKResult dir_result = evaluate_ik();
    if (diag) {
        diag->dir_result = dir_result;
    }

    // --- Attempt 2: NEAR_REF (optional fallback) ---
    IKResult ref_result = IKResult::kNoTarget;
    if (dir_result != IKResult::kSuccess && ik_fallback_near_ref_) {
        setup_common();
        ik.m_Input_IK_ZSPType = FX_PILOT_NSP_TYPES_NEAR_REF;
        ref_result = evaluate_ik();
        if (diag) {
            diag->used_near_ref = true;
            diag->ref_result = ref_result;
        }
    }

    // --- Attempt 3: Clamp (optional) ---
    bool clamped = false;
    if (dir_result != IKResult::kSuccess && ref_result != IKResult::kSuccess) {
        IKResult clamp_src = ik_fallback_near_ref_ ? ref_result : dir_result;
        if (ik_clamp_joint_limits_ &&
            clamp_src == IKResult::kJointLimitExceeded) {
            for (size_t j = 0; j < kJointsPerArm; ++j) {
                out_q_joints_rad[j] = std::clamp(ik.m_Output_RetJoint[j],
                                                  ik.m_Output_RunLmtN[j],
                                                  ik.m_Output_RunLmtP[j]) * kDeg2Rad;
            }
            clamped = true;
        }
    }

    // --- Determine final result ---
    IKResult final_result;
    if (dir_result == IKResult::kSuccess) {
        final_result = IKResult::kSuccess;
    } else if (ref_result == IKResult::kSuccess) {
        final_result = IKResult::kSuccess;
    } else if (clamped) {
        final_result = IKResult::kJointLimitClamped;
    } else {
        final_result = dir_result;
    }
    if (diag) {
        diag->clamped = clamped;
        diag->final_result = final_result;
        diag->has_solution = (final_result == IKResult::kSuccess ||
                              final_result == IKResult::kJointLimitClamped);
        if (diag->has_solution) {
            diag->solution_j4_deg = out_q_joints_rad[3] * kRad2Deg;
            for (size_t j = 0; j < kJointsPerArm; ++j) {
                diag->q_joints_rad[j] = out_q_joints_rad[j];
            }
        } else if (dir_result == IKResult::kSuccess || ref_result == IKResult::kSuccess) {
            diag->solution_j4_deg = ik.m_Output_RetJoint[3];
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
    // Latest-snapshot semantics: keep the newest diagnostics for each arm and
    // let the non-RT timer publish/log that coherent snapshot.
    if (diagnostics_mutex_.try_lock()) {
        pending_diagnostics_[arm] = diag;
        pending_diagnostics_[arm].pending = true;
        diagnostics_mutex_.unlock();
    }
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

    state.hand_changed = (state.hand_stamp != last_hand_tf_stamp_[arm]) ||
                         (snap.hand_valid != last_hand_tf_valid_[arm]);
    state.arm_changed = (state.arm_stamp != last_arm_tf_stamp_[arm]) ||
                        (snap.arm_valid != last_arm_tf_valid_[arm]);
    state.tracker_input_changed = state.hand_changed || state.arm_changed;

    last_hand_tf_stamp_[arm] = state.hand_stamp;
    last_arm_tf_stamp_[arm] = state.arm_stamp;
    last_hand_tf_valid_[arm] = snap.hand_valid;
    last_arm_tf_valid_[arm] = snap.arm_valid;

    state.hand_fresh =
        snap.hand_valid &&
        state.hand_stamp.nanoseconds() > 0 &&
        (tracker_timeout_.nanoseconds() == 0 ||
         (now >= state.hand_stamp && (now - state.hand_stamp) <= tracker_timeout_));
    return state;
}

void TrackerTeleopController::holdCurrentPosition(size_t arm)
{
    for (size_t j = 0; j < kJointsPerArm; ++j) {
        target_joints_rad_[arm][j] = smoothed_joints_rad_[arm][j];
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

    if (force_reacquire || input_state.tracker_input_changed) {
        handleFreshTrackerUpdate(arm, snap);
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

    std::string chain = "NEAR_DIR ";
    if (diag.dir_result == IKResult::kSuccess) {
        chain += std::string("OK(") + classify_j4(diag.solution_j4_deg) + ")";
        return chain;
    }

    chain += std::string("FAIL(") + ikResultToString(diag.dir_result) + ")";
    if (diag.used_near_ref) {
        chain += " -> NEAR_REF ";
        if (diag.ref_result == IKResult::kSuccess) {
            chain += std::string("OK(") + classify_j4(diag.solution_j4_deg) + ")";
        } else {
            chain += std::string("FAIL(") + ikResultToString(diag.ref_result) + ")";
        }
    }
    if (diag.clamped) {
        chain += " -> CLAMPED";
    }
    return chain;
}

void TrackerTeleopController::logArmDiagnostics(size_t arm, const ArmDiagnostics &diag)
{
    const auto logger = get_node()->get_logger();
    const std::string chain = buildIkLogChain(diag);

    if (diag.has_solution) {
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

    constexpr double kElbowVizScale = 0.3;
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

    pub_viz_markers_->publish(viz_markers);
    marker_visible_[arm] = true;
}

void TrackerTeleopController::clearVizMarkers(size_t arm)
{
    if (!pub_viz_markers_ || !marker_visible_[arm]) {
        return;
    }

    visualization_msgs::msg::MarkerArray clear_markers;
    for (const char *ns : {"ee_position", "elbow_direction"}) {
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

    {
        std::lock_guard<std::mutex> lk(diagnostics_mutex_);
        for (size_t arm = 0; arm < kArmCount; ++arm) {
            if (!pending_diagnostics_[arm].pending) {
                continue;
            }
            diagnostics[arm] = pending_diagnostics_[arm];
            pending_diagnostics_[arm].pending = false;
            has_pending[arm] = true;
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

controller_interface::CallbackReturn
TrackerTeleopController::on_configure(const rclcpp_lifecycle::State &)
{
    const auto logger = get_node()->get_logger();

    std::vector<std::string> joints_param;
    if (!get_node()->get_parameter("joints", joints_param)) {
        RCLCPP_ERROR(logger, "Missing required parameter 'joints'.");
        return CallbackReturn::ERROR;
    }
    if (joints_param.size() != kTotalJoints) {
        RCLCPP_ERROR(logger, "Parameter 'joints' must have exactly %zu entries (got %zu).",
                     kTotalJoints, joints_param.size());
        return CallbackReturn::ERROR;
    }
    joint_names_ = joints_param;

    if (!get_node()->get_parameter("kine_config_path", kine_config_path_) ||
        kine_config_path_.empty()) {
        RCLCPP_ERROR(logger, "Missing or empty parameter 'kine_config_path'.");
        return CallbackReturn::ERROR;
    }
    if (!initMarvinKinematics(kine_config_path_, kine_data_, logger)) {
        RCLCPP_ERROR(logger, "Kinematics initialization failed.");
        return CallbackReturn::ERROR;
    }
    kine_initialized_ = true;

    get_node()->get_parameter("position_scale", position_scale_);
    get_node()->get_parameter("enable_orientation", enable_orientation_);
    get_node()->get_parameter("base_frame", base_frame_);
    get_node()->get_parameter("zsp_angle", zsp_angle_);
    get_node()->get_parameter("j4_bound", j4_bound_);
    get_node()->get_parameter("dh_d1", dh_d1_);

    std::vector<std::string> viz_frames;
    if (get_node()->get_parameter("viz_base_frames", viz_frames) && viz_frames.size() >= 2) {
        viz_base_frames_[0] = viz_frames[0];
        viz_base_frames_[1] = viz_frames[1];
    }
    get_node()->get_parameter("smoothing_alpha", smoothing_alpha_);
    get_node()->get_parameter("max_joint_velocity", max_joint_velocity_);
    get_node()->get_parameter("base_x_scale", base_x_scale_);
    get_node()->get_parameter("ik_fallback_near_ref", ik_fallback_near_ref_);
    get_node()->get_parameter("ik_clamp_joint_limits", ik_clamp_joint_limits_);
    double tracker_timeout_sec = tracker_timeout_.seconds();
    get_node()->get_parameter("tracker_timeout_sec", tracker_timeout_sec);

    smoothing_alpha_ = std::clamp(smoothing_alpha_, 0.01, 1.0);
    tracker_timeout_ = rclcpp::Duration::from_seconds(std::max(0.0, tracker_timeout_sec));

    std::vector<double> elbow_dir_param;
    if (get_node()->get_parameter("default_elbow_direction", elbow_dir_param) &&
        elbow_dir_param.size() >= 3) {
        shoulder_v_elbow_default_ = {{elbow_dir_param[0], elbow_dir_param[1], elbow_dir_param[2]}};
    }

    static const char *side_labels[] = {"left", "right"};
    static const char *side_tags[]   = {"LEFT", "RIGHT"};
    struct {
        const char *name;
        std::array<tf2::Transform, kArmCount> &arr;
    } param_groups[] = {
        {"shoulder_T_chest", shoulder_T_chest_},
        {"wrist_T_ee", wrist_T_ee_},
        {"arm_human_T_arm_robot", arm_human_T_arm_robot_},
    };
    for (auto &[name, arr] : param_groups) {
        for (int i = 0; i < static_cast<int>(kArmCount); ++i) {
            arr[i] = readRigidTransform(get_node(),
                                        std::string(name) + "." + side_labels[i]);
            const auto &t = arr[i].getOrigin();
            const auto &q = arr[i].getRotation();
            RCLCPP_INFO(logger,
                        "%s %s: t=[%.3f, %.3f, %.3f] quat=[%.4f, %.4f, %.4f, %.4f]",
                        name, side_tags[i],
                        t.x(), t.y(), t.z(),
                        q.x(), q.y(), q.z(), q.w());
        }
    }

    static const char *side_labels_elbow[] = {"left", "right"};
    for (int i = 0; i < static_cast<int>(kArmCount); ++i) {
        std::vector<double> rpy_deg;
        get_node()->get_parameter(
            std::string("elbow_direction_correction.") + side_labels_elbow[i], rpy_deg);
        if (rpy_deg.size() >= 3 &&
            (rpy_deg[0] != 0.0 || rpy_deg[1] != 0.0 || rpy_deg[2] != 0.0)) {
            elbow_dir_correction_[i].setRPY(
                rpy_deg[0] * kDeg2Rad, rpy_deg[1] * kDeg2Rad, rpy_deg[2] * kDeg2Rad);
            RCLCPP_INFO(logger, "Elbow dir correction %s: RPY=[%.1f, %.1f, %.1f] deg",
                        side_tags[i], rpy_deg[0], rpy_deg[1], rpy_deg[2]);
        } else {
            elbow_dir_correction_[i] = tf2::Quaternion::getIdentity();
        }
    }

    RCLCPP_INFO(logger, "Default shoulder_v_elbow: [%.2f, %.2f, %.2f]",
                shoulder_v_elbow_default_[0], shoulder_v_elbow_default_[1], shoulder_v_elbow_default_[2]);
    RCLCPP_INFO(logger, "ZSP angle: %.2f deg, J4 bound: %.2f deg", zsp_angle_, j4_bound_);
    RCLCPP_INFO(logger, "Smoothing: alpha=%.3f, max_vel=%.2f rad/s",
                smoothing_alpha_, max_joint_velocity_);
    RCLCPP_INFO(logger, "Base X scale: %.3f", base_x_scale_);
    RCLCPP_INFO(logger, "IK fallback NEAR_REF: %s, clamp joint limits: %s",
                ik_fallback_near_ref_ ? "ON" : "OFF",
                ik_clamp_joint_limits_ ? "ON" : "OFF");
    RCLCPP_INFO(logger, "Tracker timeout: %.3f s", tracker_timeout_.seconds());
    RCLCPP_INFO(
        logger,
        "Teleop gate: startup=%s, control=service + keyboard helper",
        teleopStateToString(getTeleopState()));

    for (auto &arm : last_joint_deg_) arm.fill(0.0);
    for (auto &arm : smoothed_joints_rad_) arm.fill(0.0);
    for (auto &arm : target_joints_rad_) arm.fill(0.0);
    cmd_interfaces_.fill(nullptr);
    state_interfaces_pos_.fill(nullptr);
    pending_diagnostics_.fill(ArmDiagnostics());
    marker_visible_.fill(false);
    tracker_fresh_.fill(false);
    last_ik_result_.fill(IKResult::kNoTarget);
    teleop_state_.store(static_cast<int>(TeleopState::kDisarmed), std::memory_order_relaxed);
    force_tracker_reacquire_.store(false, std::memory_order_relaxed);

    pub_viz_markers_ = get_node()->create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/viz_markers", rclcpp::SystemDefaultsQoS());

    pub_ik_status_[0] = get_node()->create_publisher<std_msgs::msg::String>(
        "~/ik_status_left", rclcpp::SystemDefaultsQoS());
    pub_ik_status_[1] = get_node()->create_publisher<std_msgs::msg::String>(
        "~/ik_status_right", rclcpp::SystemDefaultsQoS());
    pub_teleop_state_ = get_node()->create_publisher<std_msgs::msg::String>(
        "~/teleop_state", rclcpp::QoS(1).transient_local());

    auto fk_qos = rclcpp::QoS(1).transient_local();
    pub_current_pose_[0] = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
        "~/current_pose_left", fk_qos);
    pub_current_pose_[1] = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
        "~/current_pose_right", fk_qos);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_node()->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
        *tf_buffer_, get_node(), false);

    tf_poll_timer_ = get_node()->create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&TrackerTeleopController::pollTfCallback, this));
    diagnostics_timer_ = get_node()->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&TrackerTeleopController::diagnosticsTimerCallback, this));
    srv_set_armed_ = get_node()->create_service<std_srvs::srv::SetBool>(
        "~/set_armed",
        std::bind(
            &TrackerTeleopController::handleSetArmed, this,
            std::placeholders::_1, std::placeholders::_2));
    srv_set_enabled_ = get_node()->create_service<std_srvs::srv::SetBool>(
        "~/set_enabled",
        std::bind(
            &TrackerTeleopController::handleSetEnabled, this,
            std::placeholders::_1, std::placeholders::_2));

    get_node()->get_parameter("tracker_frames.torso", frame_torso_);
    get_node()->get_parameter("tracker_frames.left_hand", frame_left_hand_);
    get_node()->get_parameter("tracker_frames.right_hand", frame_right_hand_);
    get_node()->get_parameter("tracker_frames.left_upper_arm", frame_left_upper_arm_);
    get_node()->get_parameter("tracker_frames.right_upper_arm", frame_right_upper_arm_);

    RCLCPP_INFO(logger, "TF frames: torso=%s, L_hand=%s, R_hand=%s, L_arm=%s, R_arm=%s",
                frame_torso_.c_str(), frame_left_hand_.c_str(), frame_right_hand_.c_str(),
                frame_left_upper_arm_.c_str(), frame_right_upper_arm_.c_str());

    RCLCPP_INFO(logger, "TrackerTeleopController configured (scale=%.2f, orientation=%s)",
                position_scale_, enable_orientation_ ? "ON" : "OFF");
    publishTeleopState("configured: teleop locked until armed");
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
TrackerTeleopController::on_activate(const rclcpp_lifecycle::State &)
{
    const auto logger = get_node()->get_logger();

    for (size_t i = 0; i < kTotalJoints; ++i) {
        const std::string name = joint_names_[i] + "/position";

        auto *cmd = find_loaned_interface(command_interfaces_, name);
        if (!cmd) {
            RCLCPP_ERROR(logger, "Missing command interface '%s'.", name.c_str());
            return CallbackReturn::ERROR;
        }
        cmd_interfaces_[i] = cmd;

        auto *state = find_loaned_interface(state_interfaces_, name);
        if (!state) {
            RCLCPP_ERROR(logger, "Missing state interface '%s'.", name.c_str());
            return CallbackReturn::ERROR;
        }
        state_interfaces_pos_[i] = state;
    }

    for (size_t i = 0; i < kTotalJoints; ++i) {
        const size_t arm = i / kJointsPerArm;
        const size_t j = i % kJointsPerArm;
        const auto pos_opt = state_interfaces_pos_[i]->get_optional<double>();
        const double pos_rad = pos_opt.has_value() ? pos_opt.value() : 0.0;
        last_joint_deg_[arm][j] = pos_rad * kRad2Deg;
        smoothed_joints_rad_[arm][j] = pos_rad;
        target_joints_rad_[arm][j] = pos_rad;
        (void)cmd_interfaces_[i]->set_value(pos_rad);
    }

    has_valid_target_.fill(true);
    last_hand_tf_stamp_.fill(rclcpp::Time(0, 0, RCL_ROS_TIME));
    last_arm_tf_stamp_.fill(rclcpp::Time(0, 0, RCL_ROS_TIME));
    last_hand_tf_valid_.fill(false);
    last_arm_tf_valid_.fill(false);
    tracker_fresh_.fill(false);
    setTeleopState(TeleopState::kDisarmed, "controller activated: teleop locked");

    computeAndPublishFK();

    RCLCPP_INFO(
        logger,
        "TrackerTeleopController activated (absolute mapping mode, startup state=%s).",
        teleopStateToString(getTeleopState()));
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
TrackerTeleopController::on_deactivate(const rclcpp_lifecycle::State &)
{
    teleop_state_.store(static_cast<int>(TeleopState::kDisarmed), std::memory_order_relaxed);
    force_tracker_reacquire_.store(false, std::memory_order_relaxed);
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
    const bool teleop_enabled = isTeleopEnabled(now);
    const bool force_reacquire = teleop_enabled &&
        force_tracker_reacquire_.exchange(false, std::memory_order_relaxed);

    for (int arm = 0; arm < static_cast<int>(kArmCount); ++arm) {
        if (!teleop_enabled) {
            holdCurrentPosition(static_cast<size_t>(arm));
            applySmoothedCommand(static_cast<size_t>(arm), dt);
            continue;
        }
        processArmUpdate(static_cast<size_t>(arm), tf_snapshot_[arm], now, dt, force_reacquire);
    }

    return controller_interface::return_type::OK;
}

}  // namespace ros2_control_marvin

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ros2_control_marvin::TrackerTeleopController,
                       controller_interface::ControllerInterface)
