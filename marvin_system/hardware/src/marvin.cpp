#include "marvin_system/marvin.hpp"
#include "marvin_system/MarvinSDK.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <sstream>
#include <string>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace {

using Clock = std::chrono::steady_clock;
constexpr double kDeg2Rad = M_PI / 180.0;
constexpr double kRad2Deg = 180.0 / M_PI;

struct Ip4 {
    FX_UCHAR a, b, c, d;
};

bool parse_ip4(const std::string &s, Ip4 &out)
{
    int ia, ib, ic, id;
    if (std::sscanf(s.c_str(), "%d.%d.%d.%d", &ia, &ib, &ic, &id) != 4) {
        return false;
    }
    auto ok = [](int v) { return v >= 0 && v <= 255; };
    if (!ok(ia) || !ok(ib) || !ok(ic) || !ok(id)) {
        return false;
    }
    out = {static_cast<FX_UCHAR>(ia), static_cast<FX_UCHAR>(ib),
           static_cast<FX_UCHAR>(ic), static_cast<FX_UCHAR>(id)};
    return true;
}

int param_int(const std::unordered_map<std::string, std::string> &m,
              const std::string &key, int def, int lo, int hi)
{
    const auto it = m.find(key);
    if (it == m.end()) return def;
    return std::clamp(std::atoi(it->second.c_str()), lo, hi);
}

double param_double(const std::unordered_map<std::string, std::string> &m,
                    const std::string &key, double def, double lo, double hi)
{
    const auto it = m.find(key);
    if (it == m.end()) return def;
    try {
        return std::clamp(std::stod(it->second), lo, hi);
    } catch (...) {
        return def;
    }
}

bool param_bool(const std::unordered_map<std::string, std::string> &m,
                const std::string &key, bool def)
{
    const auto it = m.find(key);
    if (it == m.end()) return def;

    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    if (value == "true" || value == "1" || value == "yes" || value == "on") {
        return true;
    }
    if (value == "false" || value == "0" || value == "no" || value == "off") {
        return false;
    }
    return def;
}

double clamp_unit(double value)
{
    return std::clamp(value, 0.0, 1.0);
}

std::string param_str(const std::unordered_map<std::string, std::string> &m,
                      const std::string &key, const std::string &def = {})
{
    const auto it = m.find(key);
    return (it != m.end()) ? it->second : def;
}

template <size_t N>
bool parse_fixed_double_array(
    const std::string &text,
    std::array<double, N> &out)
{
    std::istringstream stream(text);
    for (size_t i = 0; i < N; ++i) {
        if (!(stream >> out[i])) {
            return false;
        }
    }
    double extra = 0.0;
    if (stream >> extra) {
        return false;
    }
    return true;
}

const char *target_arm_state_name(marvin_system::ControlProfile profile)
{
    switch (profile) {
    case marvin_system::ControlProfile::kPositionFollow:
        return "POSITION";
    case marvin_system::ControlProfile::kJointImpedance:
        return "TORQ";
    case marvin_system::ControlProfile::kUnknown:
    default:
        return "UNKNOWN";
    }
}

int target_arm_state_code(marvin_system::ControlProfile profile)
{
    switch (profile) {
    case marvin_system::ControlProfile::kPositionFollow:
        return ARM_STATE_POSITION;
    case marvin_system::ControlProfile::kJointImpedance:
        return ARM_STATE_TORQ;
    case marvin_system::ControlProfile::kUnknown:
    default:
        return -1;
    }
}

int target_arm_transition_state_code(marvin_system::ControlProfile profile)
{
    switch (profile) {
    case marvin_system::ControlProfile::kPositionFollow:
        return ARM_STATE_TRANS_TO_POSITION;
    case marvin_system::ControlProfile::kJointImpedance:
        return ARM_STATE_TRANS_TO_TORQ;
    case marvin_system::ControlProfile::kUnknown:
    default:
        return -1;
    }
}

inline std::string pos_if(const std::string &jn)
{
    return jn + "/" + hardware_interface::HW_IF_POSITION;
}

inline std::string vel_if(const std::string &jn)
{
    return jn + "/" + hardware_interface::HW_IF_VELOCITY;
}

inline std::string eff_if(const std::string &jn)
{
    return jn + "/" + hardware_interface::HW_IF_EFFORT;
}

inline std::string sdk_cmd_if(const std::string &jn)
{
    return marvin_system::joint_state_if(
        jn, marvin_system::kSdkCommandPositionStateInterface);
}

const char *arm_state_name(ArmState state)
{
    switch (state) {
    case ARM_STATE_IDLE:
        return "IDLE";
    case ARM_STATE_POSITION:
        return "POSITION";
    case ARM_STATE_PVT:
        return "PVT";
    case ARM_STATE_TORQ:
        return "TORQ";
    case ARM_STATE_RELEASE:
        return "RELEASE";
    case ARM_STATE_ERROR:
        return "ERROR";
    case ARM_STATE_TRANS_TO_POSITION:
        return "TRANS_TO_POSITION";
    case ARM_STATE_TRANS_TO_PVT:
        return "TRANS_TO_PVT";
    case ARM_STATE_TRANS_TO_TORQ:
        return "TRANS_TO_TORQ";
    case ARM_STATE_TRANS_TO_RELEASE:
        return "TRANS_TO_RELEASE";
    case ARM_STATE_TRANS_TO_IDLE:
        return "TRANS_TO_IDLE";
    default:
        return "UNKNOWN";
    }
}

}  // namespace

// ===========================================================================

namespace marvin_system {

void MarvinHardware::update_sdk_observation_states(const DCSS &dcss)
{
    if (info_.joints.size() < kTotalJoints) {
        return;
    }

    const auto &left_anchor_joint = info_.joints[0].name;

    set_state(
        joint_state_if(left_anchor_joint, kActiveControlProfileStateInterface),
        static_cast<double>(static_cast<int>(
            active_control_profile_.load(std::memory_order_relaxed))));
    set_state(
        joint_state_if(left_anchor_joint, kRequestedControlProfileStateInterface),
        static_cast<double>(static_cast<int>(
            requested_control_profile_.load(std::memory_order_relaxed))));

    for (size_t arm = 0; arm < kArmCount; ++arm) {
        const auto &anchor_joint =
            info_.joints[arm == 0 ? 0 : kJointsPerArm].name;
        set_state(
            joint_state_if(anchor_joint, kSdkCurrentStateStateInterface),
            static_cast<double>(dcss.m_State[arm].m_CurState));
        set_state(
            joint_state_if(anchor_joint, kSdkCommandStateStateInterface),
            static_cast<double>(dcss.m_State[arm].m_CmdState));
        set_state(
            joint_state_if(anchor_joint, kSdkErrorCodeStateInterface),
            static_cast<double>(dcss.m_State[arm].m_ERRCode));
        set_state(
            joint_state_if(anchor_joint, kSdkInFrameSerialStateInterface),
            static_cast<double>(dcss.m_In[arm].m_InFrameSerial));
        set_state(
            joint_state_if(anchor_joint, kSdkOutFrameSerialStateInterface),
            static_cast<double>(dcss.m_Out[arm].m_OutFrameSerial));

        const auto &out = dcss.m_Out[arm];
        for (size_t joint = 0; joint < kJointsPerArm; ++joint) {
            const size_t index = arm * kJointsPerArm + joint;
            set_state(
                sdk_cmd_if(info_.joints[index].name),
                static_cast<double>(out.m_FB_Joint_Cmd[joint]) * kDeg2Rad);
        }
    }
}

bool MarvinHardware::send_position_hold_command(
    const std::array<std::array<double, kJointsPerArm>, kArmCount> &hold_deg,
    bool request_position_state)
{
    double hold_a[kJointsPerArm];
    double hold_b[kJointsPerArm];
    for (size_t joint = 0; joint < kJointsPerArm; ++joint) {
        hold_a[joint] = hold_deg[0][joint];
        hold_b[joint] = hold_deg[1][joint];
    }

    OnClearSet();
    bool ok = true;
    if (request_position_state) {
        ok = ok && OnSetTargetState_A(1);
        ok = ok && OnSetJointLmt_A(joint_vel_ratio_, joint_acc_ratio_);
        ok = ok && OnSetTargetState_B(1);
        ok = ok && OnSetJointLmt_B(joint_vel_ratio_, joint_acc_ratio_);
    }
    ok = ok && OnSetJointCmdPos_A(hold_a);
    ok = ok && OnSetJointCmdPos_B(hold_b);
    return ok && OnSetSend();
}

bool MarvinHardware::send_joint_impedance_hold_command(
    const std::array<std::array<double, kJointsPerArm>, kArmCount> &hold_deg,
    bool request_torque_state)
{
    double hold_a[kJointsPerArm];
    double hold_b[kJointsPerArm];
    double tool_kine_a[6];
    double tool_kine_b[6];
    double tool_dyn_a[10];
    double tool_dyn_b[10];
    double joint_k_a[kJointsPerArm];
    double joint_k_b[kJointsPerArm];
    double joint_d_a[kJointsPerArm];
    double joint_d_b[kJointsPerArm];

    for (size_t joint = 0; joint < kJointsPerArm; ++joint) {
        hold_a[joint] = hold_deg[0][joint];
        hold_b[joint] = hold_deg[1][joint];
        joint_k_a[joint] = joint_impedance_k_[0][joint];
        joint_k_b[joint] = joint_impedance_k_[1][joint];
        joint_d_a[joint] = joint_impedance_d_[0][joint];
        joint_d_b[joint] = joint_impedance_d_[1][joint];
    }
    for (size_t i = 0; i < 6; ++i) {
        tool_kine_a[i] = tool_kine_[0][i];
        tool_kine_b[i] = tool_kine_[1][i];
    }
    for (size_t i = 0; i < 10; ++i) {
        tool_dyn_a[i] = tool_dyn_[0][i];
        tool_dyn_b[i] = tool_dyn_[1][i];
    }

    OnClearSet();
    bool ok = true;
    if (request_torque_state) {
        ok = ok && OnSetTool_A(tool_kine_a, tool_dyn_a);
        ok = ok && OnSetTool_B(tool_kine_b, tool_dyn_b);
        ok = ok && OnSetJointKD_A(joint_k_a, joint_d_a);
        ok = ok && OnSetJointKD_B(joint_k_b, joint_d_b);
        ok = ok && OnSetJointLmt_A(joint_vel_ratio_, joint_acc_ratio_);
        ok = ok && OnSetJointLmt_B(joint_vel_ratio_, joint_acc_ratio_);
        ok = ok && OnSetImpType_A(1);
        ok = ok && OnSetImpType_B(1);
        ok = ok && OnSetTargetState_A(3);
        ok = ok && OnSetTargetState_B(3);
    }
    ok = ok && OnSetJointCmdPos_A(hold_a);
    ok = ok && OnSetJointCmdPos_B(hold_b);
    return ok && OnSetSend();
}

bool MarvinHardware::process_control_profile_transition()
{
    std::unique_lock<std::mutex> lock(control_profile_mutex_);
    if (!control_profile_request_pending_) {
        return false;
    }

    auto build_arm_status_summary = [this]() {
        const auto now = Clock::now();
        std::ostringstream stream;
        stream << "A[state="
               << arm_state_code_to_string(current_sdk_cur_state_[0].load(std::memory_order_relaxed))
               << '(' << current_sdk_cur_state_[0].load(std::memory_order_relaxed) << ')'
               << " cmd=" << current_sdk_cmd_state_[0].load(std::memory_order_relaxed)
               << " err=" << current_sdk_err_code_[0].load(std::memory_order_relaxed)
               << " in=" << current_sdk_in_frame_serial_[0].load(std::memory_order_relaxed)
               << " out=" << current_sdk_out_frame_serial_[0].load(std::memory_order_relaxed)
               << " frame_age_ms="
               << std::chrono::duration_cast<std::chrono::milliseconds>(
                      now - last_frame_time_[0])
                      .count()
               << "] B[state="
               << arm_state_code_to_string(current_sdk_cur_state_[1].load(std::memory_order_relaxed))
               << '(' << current_sdk_cur_state_[1].load(std::memory_order_relaxed) << ')'
               << " cmd=" << current_sdk_cmd_state_[1].load(std::memory_order_relaxed)
               << " err=" << current_sdk_err_code_[1].load(std::memory_order_relaxed)
               << " in=" << current_sdk_in_frame_serial_[1].load(std::memory_order_relaxed)
               << " out=" << current_sdk_out_frame_serial_[1].load(std::memory_order_relaxed)
               << " frame_age_ms="
               << std::chrono::duration_cast<std::chrono::milliseconds>(
                      now - last_frame_time_[1])
                      .count()
               << ']';
        return stream.str();
    };

    auto build_hold_target_summary = [this]() {
        std::ostringstream stream;
        stream << "hold_deg_L=[";
        for (size_t joint = 0; joint < kJointsPerArm; ++joint) {
            if (joint > 0) {
                stream << ' ';
            }
            stream << control_profile_hold_deg_[0][joint];
        }
        stream << "] hold_deg_R=[";
        for (size_t joint = 0; joint < kJointsPerArm; ++joint) {
            if (joint > 0) {
                stream << ' ';
            }
            stream << control_profile_hold_deg_[1][joint];
        }
        stream << ']';
        return stream.str();
    };

    auto diagnostic_transition_active = [this]() {
        return control_profile_source_profile_ == ControlProfile::kJointImpedance &&
               control_profile_pending_target_ == ControlProfile::kPositionFollow;
    };

    auto maybe_log_diagnostic_progress_locked =
        [this, &build_arm_status_summary, &diagnostic_transition_active](
            const char *phase,
            bool force) {
        if (!diagnostic_transition_active()) {
            return;
        }

        const auto now = Clock::now();
        bool changed = !control_profile_diag_snapshot_valid_;
        for (size_t arm = 0; arm < kArmCount && !changed; ++arm) {
            changed =
                control_profile_diag_last_cur_state_[arm] !=
                    current_sdk_cur_state_[arm].load(std::memory_order_relaxed) ||
                control_profile_diag_last_cmd_state_[arm] !=
                    current_sdk_cmd_state_[arm].load(std::memory_order_relaxed) ||
                control_profile_diag_last_err_code_[arm] !=
                    current_sdk_err_code_[arm].load(std::memory_order_relaxed) ||
                control_profile_diag_last_in_frame_serial_[arm] !=
                    current_sdk_in_frame_serial_[arm].load(std::memory_order_relaxed) ||
                control_profile_diag_last_out_frame_serial_[arm] !=
                    current_sdk_out_frame_serial_[arm].load(std::memory_order_relaxed);
        }
        if (!force &&
            !changed &&
            control_profile_last_progress_log_at_ != Clock::time_point{} &&
            (now - control_profile_last_progress_log_at_) < std::chrono::milliseconds(250)) {
            return;
        }

        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    now - control_profile_transition_started_at_)
                                    .count();
        RCLCPP_WARN(
            get_logger(),
            "Diagnostic JOINT_IMPEDANCE->POSITION_FOLLOW progress: phase=%s elapsed=%lldms "
            "resend=%llu %s",
            phase,
            static_cast<long long>(elapsed_ms),
            static_cast<unsigned long long>(control_profile_resend_count_),
            build_arm_status_summary().c_str());

        for (size_t arm = 0; arm < kArmCount; ++arm) {
            control_profile_diag_last_cur_state_[arm] =
                current_sdk_cur_state_[arm].load(std::memory_order_relaxed);
            control_profile_diag_last_cmd_state_[arm] =
                current_sdk_cmd_state_[arm].load(std::memory_order_relaxed);
            control_profile_diag_last_err_code_[arm] =
                current_sdk_err_code_[arm].load(std::memory_order_relaxed);
            control_profile_diag_last_in_frame_serial_[arm] =
                current_sdk_in_frame_serial_[arm].load(std::memory_order_relaxed);
            control_profile_diag_last_out_frame_serial_[arm] =
                current_sdk_out_frame_serial_[arm].load(std::memory_order_relaxed);
        }
        control_profile_diag_snapshot_valid_ = true;
        control_profile_last_progress_log_at_ = now;
    };

    auto complete_locked =
        [this, &lock, &build_arm_status_summary, &diagnostic_transition_active](
            bool success,
            const std::string &message) {
        const auto completed_at = Clock::now();
        const bool diagnostic = diagnostic_transition_active();
        const auto elapsed_ms =
            control_profile_transition_started_at_ == Clock::time_point{} ?
                0LL :
                static_cast<long long>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                           completed_at - control_profile_transition_started_at_)
                                           .count());
        if (diagnostic || !success) {
            RCLCPP_WARN(
                get_logger(),
                "Control profile transition finished: success=%s source=%s target=%s "
                "elapsed=%lldms resend=%llu message=%s %s",
                success ? "true" : "false",
                control_profile_to_string(control_profile_source_profile_),
                control_profile_to_string(control_profile_pending_target_),
                elapsed_ms,
                static_cast<unsigned long long>(control_profile_resend_count_),
                message.c_str(),
                build_arm_status_summary().c_str());
        }
        if (success &&
            control_profile_source_profile_ == ControlProfile::kJointImpedance &&
            control_profile_pending_target_ == ControlProfile::kPositionFollow) {
            control_profile_post_monitor_active_ = true;
            control_profile_post_monitor_source_ = control_profile_source_profile_;
            control_profile_post_monitor_target_ = control_profile_pending_target_;
            control_profile_post_monitor_started_at_ = completed_at;
            control_profile_post_monitor_deadline_ =
                completed_at + std::chrono::milliseconds(profile_switch_post_monitor_ms_);
            control_profile_post_monitor_last_log_at_ = Clock::time_point{};
            control_profile_post_monitor_snapshot_valid_ = false;
        } else if (!success) {
            control_profile_post_monitor_active_ = false;
            control_profile_post_monitor_source_ = ControlProfile::kUnknown;
            control_profile_post_monitor_target_ = ControlProfile::kUnknown;
            control_profile_post_monitor_started_at_ = Clock::time_point{};
            control_profile_post_monitor_deadline_ = Clock::time_point{};
            control_profile_post_monitor_last_log_at_ = Clock::time_point{};
            control_profile_post_monitor_snapshot_valid_ = false;
        }
        control_profile_transition_phase_ =
            success ? ControlProfileTransitionPhase::kCompleted :
                      ControlProfileTransitionPhase::kFailed;
        control_profile_transition_active_ = false;
        control_profile_request_pending_ = false;
        if (!success) {
            requested_control_profile_.store(
                active_control_profile_.load(std::memory_order_relaxed),
                std::memory_order_relaxed);
        }
        control_profile_last_message_ = message;
        control_profile_completed_sequence_ = control_profile_inflight_sequence_;
        lock.unlock();
        control_profile_cv_.notify_all();
    };

    auto fail_locked = [&](const std::string &message) {
        complete_locked(false, message);
    };

    switch (control_profile_transition_phase_) {
    case ControlProfileTransitionPhase::kRequested:
        control_profile_transition_phase_ = ControlProfileTransitionPhase::kCaptureHold;
        control_profile_last_message_ =
            std::string("Accepted control profile request for ") +
            control_profile_to_string(control_profile_pending_target_) +
            "; capturing current feedback hold target.";
        return true;
    case ControlProfileTransitionPhase::kCaptureHold:
        if (control_profile_pending_target_ == ControlProfile::kUnknown) {
            fail_locked("Control profile transition target is UNKNOWN.");
            return true;
        }
        if (control_profile_pending_target_ == ControlProfile::kJointImpedance &&
            !joint_impedance_profile_enabled_) {
            fail_locked("Joint impedance profile is disabled by configuration.");
            return true;
        }
        if (!current_feedback_valid_.load(std::memory_order_relaxed)) {
            fail_locked(
                "Current joint feedback is unavailable; cannot capture a no-jump hold target.");
            return true;
        }
        for (size_t arm = 0; arm < kArmCount; ++arm) {
            for (size_t joint = 0; joint < kJointsPerArm; ++joint) {
                const size_t index = arm * kJointsPerArm + joint;
                control_profile_hold_deg_[arm][joint] =
                    current_feedback_deg_[index].load(std::memory_order_relaxed);
            }
        }
        control_profile_transition_started_at_ = Clock::now();
        control_profile_last_packet_sent_at_ = Clock::time_point{};
        control_profile_last_progress_log_at_ = Clock::time_point{};
        control_profile_stabilize_cycles_remaining_ =
            std::max(1, profile_switch_stabilize_cycles_);
        control_profile_transition_phase_ = ControlProfileTransitionPhase::kSendSwitchPacket;
        control_profile_last_message_ =
            std::string("Captured current feedback hold target; sending ") +
            control_profile_to_string(control_profile_pending_target_) +
            " switch packet.";
        if (diagnostic_transition_active()) {
            RCLCPP_WARN(
                get_logger(),
                "Diagnostic JOINT_IMPEDANCE->POSITION_FOLLOW captured hold target: %s %s",
                build_hold_target_summary().c_str(),
                build_arm_status_summary().c_str());
        }
        return true;
    case ControlProfileTransitionPhase::kSendSwitchPacket: {
        const ControlProfile target_profile = control_profile_pending_target_;
        const auto hold_deg = control_profile_hold_deg_;
        lock.unlock();
        const bool ok =
            target_profile == ControlProfile::kJointImpedance ?
                send_joint_impedance_hold_command(hold_deg, true) :
                send_position_hold_command(hold_deg, true);
        lock.lock();
        if (!control_profile_request_pending_) {
            return true;
        }
        if (!ok) {
            fail_locked(
                std::string("Failed to send switch packet while starting the ") +
                control_profile_to_string(control_profile_pending_target_) +
                " transition. " + build_arm_status_summary());
            return true;
        }
        RCLCPP_INFO(
            get_logger(),
            "Control profile switch packet sent: target=%s %s",
            control_profile_to_string(target_profile),
            build_arm_status_summary().c_str());
        control_profile_last_packet_sent_at_ = Clock::now();
        maybe_log_diagnostic_progress_locked("send_switch_packet", true);
        control_profile_transition_phase_ = ControlProfileTransitionPhase::kWaitArmState;
        control_profile_last_message_ =
            std::string("Switch packet sent; waiting for both arms to report ") +
            target_arm_state_name(control_profile_pending_target_) +
            ".";
        return true;
    }
    case ControlProfileTransitionPhase::kWaitArmState: {
        const ControlProfile target_profile = control_profile_pending_target_;
        if ((Clock::now() - control_profile_transition_started_at_) >
            std::chrono::milliseconds(profile_switch_timeout_ms_)) {
            fail_locked(
                std::string("Timed out waiting for both arms to reach ") +
                target_arm_state_name(target_profile) +
                " during control profile transition. " + build_arm_status_summary());
            return true;
        }

        for (size_t arm = 0; arm < kArmCount; ++arm) {
            if (current_sdk_err_code_[arm].load(std::memory_order_relaxed) != 0) {
                fail_locked(
                    "SDK reported a non-zero error code during control profile transition. " +
                    build_arm_status_summary());
                return true;
            }
        }

        const int target_state_code = target_arm_state_code(target_profile);
        bool all_position = true;
        for (size_t arm = 0; arm < kArmCount; ++arm) {
            all_position =
                all_position &&
                current_sdk_cur_state_[arm].load(std::memory_order_relaxed) == target_state_code;
        }
        maybe_log_diagnostic_progress_locked("wait_arm_state", false);

        if (!all_position) {
            const auto hold_deg = control_profile_hold_deg_;
            const auto now = Clock::now();
            const bool paced_position_restore =
                control_profile_source_profile_ == ControlProfile::kJointImpedance &&
                target_profile == ControlProfile::kPositionFollow;
            bool any_arm_in_transition_state = false;
            bool resend_switch_packet = false;
            if (paced_position_restore) {
                const auto elapsed_since_start =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - control_profile_transition_started_at_);
                const int transition_state_code = target_arm_transition_state_code(target_profile);
                for (size_t arm = 0; arm < kArmCount; ++arm) {
                    any_arm_in_transition_state =
                        any_arm_in_transition_state ||
                        current_sdk_cur_state_[arm].load(std::memory_order_relaxed) ==
                            transition_state_code;
                }
                const bool initial_delay_elapsed =
                    elapsed_since_start >=
                    std::chrono::milliseconds(profile_switch_initial_resend_delay_ms_);
                const bool resend_interval_elapsed =
                    control_profile_last_packet_sent_at_ == Clock::time_point{} ||
                    (now - control_profile_last_packet_sent_at_) >=
                        std::chrono::milliseconds(profile_switch_resend_interval_ms_);
                resend_switch_packet =
                    initial_delay_elapsed &&
                    resend_interval_elapsed &&
                    !any_arm_in_transition_state;
            } else {
                resend_switch_packet =
                    control_profile_last_packet_sent_at_ == Clock::time_point{} ||
                    (now - control_profile_last_packet_sent_at_) >=
                        std::chrono::milliseconds(profile_switch_resend_interval_ms_);
            }
            lock.unlock();
            const bool ok =
                target_profile == ControlProfile::kJointImpedance ?
                    send_joint_impedance_hold_command(hold_deg, resend_switch_packet) :
                    send_position_hold_command(hold_deg, resend_switch_packet);
            lock.lock();
            if (!control_profile_request_pending_) {
                return true;
            }
            if (!ok) {
                fail_locked(
                    std::string("Failed to maintain hold command while waiting for ") +
                    target_arm_state_name(target_profile) +
                    " during control profile transition. " + build_arm_status_summary());
                return true;
            }
            if (resend_switch_packet) {
                control_profile_resend_count_++;
                control_profile_last_packet_sent_at_ = now;
                maybe_log_diagnostic_progress_locked("wait_arm_state_resend", true);
            } else if (diagnostic_transition_active() && any_arm_in_transition_state) {
                maybe_log_diagnostic_progress_locked(
                    "wait_arm_state_hold_only_transition_seen", false);
            }
            return true;
        }

        control_profile_transition_phase_ = ControlProfileTransitionPhase::kStabilize;
        control_profile_last_message_ =
            std::string("Both arms reached ") +
            target_arm_state_name(target_profile) +
            "; entering stabilization hold.";
        maybe_log_diagnostic_progress_locked("enter_stabilize", true);
        return true;
    }
    case ControlProfileTransitionPhase::kStabilize: {
        const ControlProfile target_profile = control_profile_pending_target_;
        if ((Clock::now() - control_profile_transition_started_at_) >
            std::chrono::milliseconds(profile_switch_timeout_ms_)) {
            fail_locked(
                "Timed out while stabilizing the hold target during control profile transition. " +
                build_arm_status_summary());
            return true;
        }

        const int target_state_code = target_arm_state_code(target_profile);
        for (size_t arm = 0; arm < kArmCount; ++arm) {
            if (current_sdk_err_code_[arm].load(std::memory_order_relaxed) != 0) {
                fail_locked(
                    "SDK reported a non-zero error code during stabilization. " +
                    build_arm_status_summary());
                return true;
            }
            if (current_sdk_cur_state_[arm].load(std::memory_order_relaxed) != target_state_code) {
                fail_locked(
                    std::string("An arm left ") +
                    target_arm_state_name(target_profile) +
                    " during stabilization. " + build_arm_status_summary());
                return true;
            }
        }
        maybe_log_diagnostic_progress_locked("stabilize", false);

        const auto hold_deg = control_profile_hold_deg_;
        lock.unlock();
        const bool ok =
            target_profile == ControlProfile::kJointImpedance ?
                send_joint_impedance_hold_command(hold_deg, false) :
                send_position_hold_command(hold_deg, false);
        lock.lock();
        if (!control_profile_request_pending_) {
            return true;
        }
        if (!ok) {
            fail_locked(
                std::string("Failed to maintain hold command during ") +
                control_profile_to_string(target_profile) +
                " stabilization. " + build_arm_status_summary());
            return true;
        }

        control_profile_stabilize_cycles_remaining_--;
        if (control_profile_stabilize_cycles_remaining_ > 0) {
            return true;
        }

        requested_control_profile_.store(
            target_profile, std::memory_order_relaxed);
        active_control_profile_.store(
            target_profile, std::memory_order_relaxed);
        complete_locked(
            true,
            std::string("Phase 3 transition completed with no-jump hold stabilization: ") +
            control_profile_to_string(target_profile) + ".");
        return true;
    }
    case ControlProfileTransitionPhase::kCompleted:
    case ControlProfileTransitionPhase::kFailed:
        control_profile_transition_active_ = false;
        control_profile_request_pending_ = false;
        control_profile_completed_sequence_ = control_profile_inflight_sequence_;
        lock.unlock();
        control_profile_cv_.notify_all();
        return true;
    case ControlProfileTransitionPhase::kIdle:
    default:
        control_profile_transition_active_ = false;
        control_profile_request_pending_ = false;
        control_profile_last_message_ =
            "Control profile transition state became idle unexpectedly.";
        control_profile_completed_sequence_ = control_profile_inflight_sequence_;
        lock.unlock();
        control_profile_cv_.notify_all();
        return true;
    }
}

MarvinHardware::~MarvinHardware()
{
    stop_collision_guard_worker();
    stop_gripper_worker();
    for (size_t g = 0; g < gripper_count_; ++g) {
        if (grippers_[g].device) {
            if (grippers_[g].device->is_connected())
                grippers_[g].device->disconnect();
            omnipicker_destroy(grippers_[g].device);
            grippers_[g].device = nullptr;
        }
    }
    if (connected_) {
        OnRelease();
        connected_ = false;
    }
}

void MarvinHardware::start_collision_guard_worker()
{
    stop_collision_guard_worker();
    if (!collision_guard_.ready()) {
        return;
    }

    collision_guard_stop_.store(false, std::memory_order_relaxed);
    collision_guard_thread_ = std::thread([this]() { collision_guard_worker_loop(); });
    RCLCPP_INFO(
        get_logger(),
        "Started collision guard worker thread (rate=%.1f Hz).",
        collision_guard_.check_rate_hz());
}

void MarvinHardware::stop_collision_guard_worker()
{
    collision_guard_stop_.store(true, std::memory_order_relaxed);
    if (collision_guard_thread_.joinable()) {
        collision_guard_thread_.join();
        RCLCPP_INFO(get_logger(), "Stopped collision guard worker thread.");
    }
}

void MarvinHardware::collision_guard_worker_loop()
{
    using JointArray = std::array<std::array<double, kJointsPerArm>, kArmCount>;
    const auto interval = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / std::max(1.0, collision_guard_.check_rate_hz())));
    auto next_wake = Clock::now();
    int block_streak = 0;

    auto load_snapshot = [this](const auto &source) {
        JointArray snapshot{};
        for (size_t arm = 0; arm < kArmCount; ++arm) {
            for (size_t joint = 0; joint < kJointsPerArm; ++joint) {
                const size_t index = arm * kJointsPerArm + joint;
                snapshot[arm][joint] = source[index].load(std::memory_order_relaxed);
            }
        }
        return snapshot;
    };

    auto store_snapshot = [this](const JointArray &snapshot, auto &target) {
        for (size_t arm = 0; arm < kArmCount; ++arm) {
            for (size_t joint = 0; joint < kJointsPerArm; ++joint) {
                const size_t index = arm * kJointsPerArm + joint;
                target[index].store(snapshot[arm][joint], std::memory_order_relaxed);
            }
        }
    };

    while (!collision_guard_stop_.load(std::memory_order_relaxed)) {
        if (!activated_ ||
            !collision_guard_.active() ||
            !collision_guard_runtime_enabled_.load(std::memory_order_relaxed) ||
            !current_feedback_valid_.load(std::memory_order_relaxed)) {
            next_wake += interval;
            const auto current_time = Clock::now();
            if (next_wake <= current_time) {
                next_wake = current_time + interval;
            }
            std::this_thread::sleep_until(next_wake);
            continue;
        }

        const JointArray current_deg = load_snapshot(current_feedback_deg_);
        const JointArray desired_deg = load_snapshot(collision_guard_target_deg_);
        const bool approved_snapshot_valid =
            collision_guard_approved_valid_.load(std::memory_order_relaxed);
        const JointArray approved_base_deg =
            approved_snapshot_valid ? load_snapshot(collision_guard_approved_deg_) : current_deg;
        JointArray approved_deg = desired_deg;

        std::array<double, CollisionGuard::kGripperCount> gripper_percent{{0.0, 0.0}};
        std::array<bool, CollisionGuard::kGripperCount> gripper_valid{{false, false}};
        for (size_t g = 0; g < gripper_count_ && g < CollisionGuard::kGripperCount; ++g) {
            gripper_percent[g] = grippers_[g].state_percent.load(std::memory_order_relaxed);
            gripper_valid[g] = grippers_[g].state_valid.load(std::memory_order_relaxed);
        }

        const auto evaluation =
            collision_guard_.evaluate_motion(current_deg, desired_deg, gripper_percent, gripper_valid, get_logger());

        if (!evaluation.ready) {
            next_wake += interval;
            const auto current_time = Clock::now();
            if (next_wake <= current_time) {
                next_wake = current_time + interval;
            }
            std::this_thread::sleep_until(next_wake);
            continue;
        }

        auto apply_alpha = [](const JointArray &from,
                              const JointArray &to,
                              double left_alpha,
                              double right_alpha) {
            JointArray result = from;
            for (size_t joint = 0; joint < kJointsPerArm; ++joint) {
                result[0][joint] =
                    from[0][joint] + (to[0][joint] - from[0][joint]) * left_alpha;
                result[1][joint] =
                    from[1][joint] + (to[1][joint] - from[1][joint]) * right_alpha;
            }
            return result;
        };

        if (!evaluation.safe) {
            const double combined_alpha = std::clamp(evaluation.max_safe_alpha, 0.0, 1.0);
            double left_alpha = combined_alpha;
            double right_alpha = combined_alpha;
            const bool single_arm_environment_case =
                evaluation.environment_pair &&
                (evaluation.affect_left != evaluation.affect_right);
            if (single_arm_environment_case) {
                left_alpha = evaluation.affect_left ? combined_alpha : 1.0;
                right_alpha = evaluation.affect_right ? combined_alpha : 1.0;
            }
            approved_deg = apply_alpha(approved_base_deg, desired_deg, left_alpha, right_alpha);

            ++block_streak;
            if (block_streak == 1 || block_streak % 100 == 0) {
                RCLCPP_WARN(
                    get_logger(),
                    "Collision guard blocked command path at sample %d; applying combined_alpha=%.4f "
                    "(left_alpha=%.4f, right_alpha=%.4f, current_distance=%.4f m, "
                    "best_distance=%.4f m, pair=%s, mode=%s, count=%d).",
                    evaluation.blocking_sample,
                    combined_alpha,
                    left_alpha,
                    right_alpha,
                    evaluation.current_distance_m,
                    evaluation.best_distance_m,
                    evaluation.current_pair.empty() ? "unknown" : evaluation.current_pair.c_str(),
                    single_arm_environment_case ? "single_arm_environment" : "combined",
                    block_streak);
            }
        } else if (block_streak > 0) {
            RCLCPP_INFO(
                get_logger(),
                "Collision guard path is clear again after %d blocked cycles.",
                block_streak);
            block_streak = 0;
        }

        store_snapshot(approved_deg, collision_guard_approved_deg_);
        collision_guard_approved_valid_.store(true, std::memory_order_relaxed);

        next_wake += interval;
        const auto current_time = Clock::now();
        if (next_wake <= current_time) {
            next_wake = current_time + interval;
        }
        std::this_thread::sleep_until(next_wake);
    }
}

void MarvinHardware::start_gripper_worker()
{
    stop_gripper_worker();
    if (gripper_count_ == 0) {
        return;
    }

    bool has_real_gripper = false;
    for (size_t g = 0; g < gripper_count_; ++g) {
        if (grippers_[g].device != nullptr) {
            has_real_gripper = true;
            break;
        }
    }
    if (!has_real_gripper) {
        RCLCPP_INFO(
            get_logger(),
            "Gripper hardware is mocked for %zu gripper(s); worker thread not started.",
            gripper_count_);
        return;
    }

    gripper_worker_stop_.store(false, std::memory_order_relaxed);
    gripper_worker_thread_ = std::thread([this]() { gripper_worker_loop(); });
    RCLCPP_INFO(
        get_logger(),
        "Started gripper worker thread (%zu gripper(s), max_send_rate=%.1f Hz).",
        gripper_count_,
        gripper_command_rate_hz_);
}

void MarvinHardware::stop_gripper_worker()
{
    gripper_worker_stop_.store(true, std::memory_order_relaxed);
    if (gripper_worker_thread_.joinable()) {
        gripper_worker_thread_.join();
        RCLCPP_INFO(get_logger(), "Stopped gripper worker thread.");
    }
}

void MarvinHardware::gripper_worker_loop()
{
    const auto send_interval = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / std::max(1.0, gripper_command_rate_hz_)));
    auto next_wake = Clock::now();

    while (!gripper_worker_stop_.load(std::memory_order_relaxed)) {
        const auto now = Clock::now();
        for (size_t g = 0; g < gripper_count_; ++g) {
            auto &slot = grippers_[g];
            const double desired =
                clamp_unit(slot.command_target_percent.load(std::memory_order_relaxed));
            if (slot.mock) {
                slot.state_percent.store(desired, std::memory_order_relaxed);
                slot.state_valid.store(true, std::memory_order_relaxed);
                slot.has_sent_command = true;
                slot.last_command_percent = desired;
                slot.last_command_time = now;
                slot.consecutive_send_failures = 0;
                continue;
            }
            if (!slot.device) {
                continue;
            }

            const double observed =
                clamp_unit(static_cast<double>(slot.device->get_position_percent()));
            slot.state_percent.store(observed, std::memory_order_relaxed);
            slot.state_valid.store(true, std::memory_order_relaxed);

            const bool command_changed =
                !slot.has_sent_command ||
                std::abs(desired - slot.last_command_percent) >= gripper_command_epsilon_;
            const bool state_needs_progress =
                std::abs(desired - observed) >= gripper_command_epsilon_;
            const bool interval_elapsed =
                !slot.has_sent_command ||
                (now - slot.last_command_time) >= send_interval;
            if (!interval_elapsed || (!command_changed && !state_needs_progress)) {
                continue;
            }

            const auto err = slot.device->set_position_percent(static_cast<float>(desired));
            const double refreshed =
                clamp_unit(static_cast<double>(slot.device->get_position_percent()));
            slot.state_percent.store(refreshed, std::memory_order_relaxed);
            slot.state_valid.store(true, std::memory_order_relaxed);
            if (err != omnipicker::ErrorCode::kOK) {
                ++slot.consecutive_send_failures;
                if (slot.consecutive_send_failures == 1 ||
                    slot.consecutive_send_failures % 50 == 0) {
                    const auto &jn = info_.joints[slot.joint_index].name;
                    RCLCPP_WARN(
                        get_logger(),
                        "Gripper '%s' worker send failed (%d consecutive): %s",
                        jn.c_str(),
                        slot.consecutive_send_failures,
                        omnipicker::error_to_string(err));
                }
                continue;
            }

            slot.has_sent_command = true;
            slot.last_command_percent = desired;
            slot.last_command_time = now;
            slot.consecutive_send_failures = 0;
        }

        next_wake += send_interval;
        const auto current_time = Clock::now();
        if (next_wake <= current_time) {
            next_wake = current_time + send_interval;
        }
        std::this_thread::sleep_until(next_wake);
    }
}

// ---------------------------------------------------------------------------
// on_init – validate URDF interfaces, parse joint limits
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn MarvinHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams &params)
{
    if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    for (size_t i = 0; i < kTotalJoints; ++i) {
        current_feedback_deg_[i].store(0.0, std::memory_order_relaxed);
        collision_guard_target_deg_[i].store(0.0, std::memory_order_relaxed);
        collision_guard_approved_deg_[i].store(0.0, std::memory_order_relaxed);
    }
    for (size_t arm = 0; arm < kArmCount; ++arm) {
        current_sdk_cur_state_[arm].store(ARM_STATE_IDLE, std::memory_order_relaxed);
        current_sdk_cmd_state_[arm].store(0, std::memory_order_relaxed);
        current_sdk_err_code_[arm].store(0, std::memory_order_relaxed);
        current_sdk_in_frame_serial_[arm].store(0, std::memory_order_relaxed);
        current_sdk_out_frame_serial_[arm].store(0, std::memory_order_relaxed);
    }
    current_feedback_valid_.store(false, std::memory_order_relaxed);
    collision_guard_approved_valid_.store(false, std::memory_order_relaxed);

    const size_t n_joints = info_.joints.size();
    if (n_joints < kTotalJoints || n_joints > kTotalJoints + kMaxGrippers) {
        RCLCPP_FATAL(get_logger(), "Expected %zu~%zu joints, got %zu.",
                     kTotalJoints, kTotalJoints + kMaxGrippers, n_joints);
        return hardware_interface::CallbackReturn::ERROR;
    }

    bool any_vel = false, all_vel = true;
    bool any_eff = false, all_eff = true;

    for (size_t i = 0; i < kTotalJoints; ++i) {
        const auto &j = info_.joints[i];

        bool has_pos_cmd = false;
        for (const auto &ci : j.command_interfaces) {
            if (ci.name != hardware_interface::HW_IF_POSITION) continue;
            has_pos_cmd = true;
            auto it_min = ci.parameters.find("min");
            auto it_max = ci.parameters.find("max");
            joint_min_[i] = (it_min != ci.parameters.end())
                                ? std::stod(it_min->second)
                                : -std::numeric_limits<double>::infinity();
            joint_max_[i] = (it_max != ci.parameters.end())
                                ? std::stod(it_max->second)
                                : std::numeric_limits<double>::infinity();
        }
        if (!has_pos_cmd) {
            RCLCPP_FATAL(get_logger(), "Joint '%s' missing position command interface.",
                         j.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        bool has_pos = false, has_vel = false, has_eff = false;
        for (const auto &si : j.state_interfaces) {
            if      (si.name == hardware_interface::HW_IF_POSITION) has_pos = true;
            else if (si.name == hardware_interface::HW_IF_VELOCITY) has_vel = true;
            else if (si.name == hardware_interface::HW_IF_EFFORT)   has_eff = true;
        }
        if (!has_pos) {
            RCLCPP_FATAL(get_logger(), "Joint '%s' missing position state interface.",
                         j.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        any_vel |= has_vel;  all_vel &= has_vel;
        any_eff |= has_eff;  all_eff &= has_eff;
    }

    has_velocity_state_ = all_vel;
    has_effort_state_   = all_eff;

    if (any_vel && !all_vel)
        RCLCPP_WARN(get_logger(), "Velocity state declared on some joints only; disabled.");
    if (any_eff && !all_eff)
        RCLCPP_WARN(get_logger(), "Effort state declared on some joints only; disabled.");

    if (const auto node = get_node()) {
        control_profile_service_ = node->create_service<marvin_system::srv::SetControlProfile>(
            "~/set_control_profile",
            [this](
                const std::shared_ptr<marvin_system::srv::SetControlProfile::Request> request,
                std::shared_ptr<marvin_system::srv::SetControlProfile::Response> response) {
                const ControlProfile target_profile =
                    control_profile_from_string(request->profile);
                if (target_profile == ControlProfile::kUnknown) {
                    response->success = false;
                    response->message = "Unsupported control profile: " + request->profile;
                    response->active_profile = control_profile_to_string(
                        active_control_profile_.load(std::memory_order_relaxed));
                    response->requested_profile =
                        control_profile_to_string(
                            requested_control_profile_.load(std::memory_order_relaxed));
                    response->transition_active = false;
                    return;
                }

                if (target_profile == ControlProfile::kJointImpedance &&
                    !joint_impedance_profile_enabled_) {
                    response->success = false;
                    response->message =
                        "Joint impedance profile is disabled by configuration.";
                    response->active_profile = control_profile_to_string(
                        active_control_profile_.load(std::memory_order_relaxed));
                    response->requested_profile =
                        control_profile_to_string(
                            requested_control_profile_.load(std::memory_order_relaxed));
                    response->transition_active = false;
                    return;
                }

                if (!activated_) {
                    response->success = false;
                    response->message =
                        "Control profile request rejected because hardware is not activated.";
                    response->active_profile = control_profile_to_string(
                        active_control_profile_.load(std::memory_order_relaxed));
                    response->requested_profile =
                        control_profile_to_string(
                            requested_control_profile_.load(std::memory_order_relaxed));
                    response->transition_active = false;
                    return;
                }

                uint64_t request_sequence = 0;
                {
                    std::lock_guard<std::mutex> lock(control_profile_mutex_);
                    if (control_profile_transition_active_ || control_profile_request_pending_) {
                        response->success = false;
                        response->message =
                            "Another control profile transition is already in progress.";
                        response->active_profile = control_profile_to_string(
                            active_control_profile_.load(std::memory_order_relaxed));
                        response->requested_profile =
                            control_profile_to_string(
                                requested_control_profile_.load(std::memory_order_relaxed));
                        response->transition_active = true;
                        return;
                    }

                    requested_control_profile_.store(target_profile, std::memory_order_relaxed);
                    control_profile_pending_target_ = target_profile;
                    control_profile_source_profile_ =
                        active_control_profile_.load(std::memory_order_relaxed);
                    control_profile_transition_active_ = true;
                    control_profile_request_pending_ = true;
                    control_profile_transition_phase_ = ControlProfileTransitionPhase::kRequested;
                    control_profile_inflight_sequence_ = ++control_profile_request_sequence_;
                    control_profile_resend_count_ = 0;
                    control_profile_last_progress_log_at_ = Clock::time_point{};
                    control_profile_diag_snapshot_valid_ = false;
                    request_sequence = control_profile_inflight_sequence_;
                    control_profile_last_message_ =
                        std::string("Accepted control profile request: ") +
                        control_profile_to_string(target_profile);
                    RCLCPP_INFO(
                        get_logger(),
                        "Control profile request accepted: source=%s target=%s timeout=%dms",
                        control_profile_to_string(control_profile_source_profile_),
                        control_profile_to_string(target_profile),
                        profile_switch_timeout_ms_);
                }

                std::unique_lock<std::mutex> lock(control_profile_mutex_);
                const bool completed = control_profile_cv_.wait_for(
                    lock,
                    std::chrono::milliseconds(profile_switch_timeout_ms_),
                    [this, request_sequence]() {
                        return control_profile_completed_sequence_ >= request_sequence;
                    });

                response->active_profile = control_profile_to_string(
                    active_control_profile_.load(std::memory_order_relaxed));
                response->requested_profile =
                    control_profile_to_string(
                        requested_control_profile_.load(std::memory_order_relaxed));
                response->transition_active = control_profile_transition_active_;

                if (!completed) {
                    if (control_profile_completed_sequence_ < request_sequence &&
                        control_profile_inflight_sequence_ == request_sequence) {
                        control_profile_transition_phase_ = ControlProfileTransitionPhase::kFailed;
                        control_profile_transition_active_ = false;
                        control_profile_request_pending_ = false;
                        requested_control_profile_.store(
                            active_control_profile_.load(std::memory_order_relaxed),
                            std::memory_order_relaxed);
                        control_profile_last_message_ =
                            "Timed out waiting for control profile transition to finish.";
                        control_profile_completed_sequence_ = request_sequence;
                        control_profile_cv_.notify_all();
                    }
                    response->success = false;
                    response->message = control_profile_last_message_;
                    response->active_profile = control_profile_to_string(
                        active_control_profile_.load(std::memory_order_relaxed));
                    response->requested_profile = control_profile_to_string(
                        requested_control_profile_.load(std::memory_order_relaxed));
                    response->transition_active = false;
                    return;
                }

                response->success =
                    active_control_profile_.load(std::memory_order_relaxed) == target_profile &&
                    control_profile_transition_phase_ != ControlProfileTransitionPhase::kFailed;
                response->message = control_profile_last_message_;
            });

        workspace_guard_service_ = node->create_service<std_srvs::srv::SetBool>(
            "~/set_workspace_guard_enabled",
            [this](
                const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
                workspace_guard_runtime_enabled_.store(
                    request->data, std::memory_order_relaxed);
                response->success = true;
                if (!workspace_guard_.enabled()) {
                    response->message =
                        "Workspace guard is not configured on this hardware instance.";
                    RCLCPP_INFO(
                        get_logger(),
                        "Workspace guard runtime request ignored because the guard is not configured.");
                    return;
                }

                response->message = request->data
                                        ? "Workspace guard runtime enforcement enabled."
                                        : "Workspace guard runtime enforcement disabled.";
                RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
            });

        collision_guard_service_ = node->create_service<std_srvs::srv::SetBool>(
            "~/set_collision_guard_enabled",
            [this](
                const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
                collision_guard_runtime_enabled_.store(request->data, std::memory_order_relaxed);
                response->success = true;

                if (!collision_guard_.enabled()) {
                    response->message =
                        "Collision guard is not configured on this hardware instance.";
                    RCLCPP_INFO(
                        get_logger(),
                        "Collision guard runtime request ignored because the guard is not configured.");
                    return;
                }

                if (!collision_guard_.ready()) {
                    response->message = "Collision guard is configured but not ready: " +
                                        collision_guard_.status_message();
                    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
                    return;
                }

                if (request->data && current_feedback_valid_.load(std::memory_order_relaxed)) {
                    for (size_t i = 0; i < kTotalJoints; ++i) {
                        const double feedback = current_feedback_deg_[i].load(std::memory_order_relaxed);
                        collision_guard_target_deg_[i].store(feedback, std::memory_order_relaxed);
                        collision_guard_approved_deg_[i].store(feedback, std::memory_order_relaxed);
                    }
                    collision_guard_approved_valid_.store(true, std::memory_order_relaxed);
                }

                response->message = request->data
                                        ? "Collision guard runtime enforcement enabled."
                                        : "Collision guard runtime enforcement disabled.";
                RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
            });
    }

    // Detect optional OmniPicker gripper joints (indices kTotalJoints+)
    gripper_count_ = 0;
    for (size_t i = kTotalJoints; i < n_joints; ++i) {
        const auto &j = info_.joints[i];
        const auto &jp = j.parameters;

        auto it_type = jp.find("type");
        if (it_type == jp.end() || it_type->second != "omnipicker") {
            RCLCPP_FATAL(get_logger(),
                "Extra joint '%s' (index %zu) must have param type=omnipicker.",
                j.name.c_str(), i);
            return hardware_interface::CallbackReturn::ERROR;
        }

        bool has_pos_cmd = false, has_pos_state = false;
        for (const auto &ci : j.command_interfaces)
            if (ci.name == hardware_interface::HW_IF_POSITION) has_pos_cmd = true;
        for (const auto &si : j.state_interfaces)
            if (si.name == hardware_interface::HW_IF_POSITION) has_pos_state = true;
        if (!has_pos_cmd || !has_pos_state) {
            RCLCPP_FATAL(get_logger(),
                "Gripper joint '%s' requires position command & state interfaces.",
                j.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        auto &slot = grippers_[gripper_count_];
        slot.joint_index = i;

        auto it_arm = jp.find("arm_side");
        slot.arm_side = (it_arm != jp.end() &&
                         (it_arm->second == "A" || it_arm->second == "a"))
                        ? omnipicker::ArmSide::kA
                        : omnipicker::ArmSide::kB;

        auto it_can = jp.find("can_node_id");
        slot.can_node_id = (it_can != jp.end())
                           ? static_cast<uint32_t>(std::atoi(it_can->second.c_str()))
                           : 1u;

        RCLCPP_INFO(get_logger(), "Gripper joint '%s': arm=%s, CAN node=%u.",
                     j.name.c_str(),
                     slot.arm_side == omnipicker::ArmSide::kA ? "A" : "B",
                     slot.can_node_id);
        ++gripper_count_;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_configure – parse params, connect SDK, clear errors
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn MarvinHardware::on_configure(
    const rclcpp_lifecycle::State &)
{
    stop_collision_guard_worker();
    stop_gripper_worker();
    activated_ = false;
    workspace_guard_.disarm();
    collision_guard_.disarm();
    current_feedback_valid_.store(false, std::memory_order_relaxed);
    collision_guard_approved_valid_.store(false, std::memory_order_relaxed);
    active_control_profile_.store(ControlProfile::kUnknown, std::memory_order_relaxed);
    requested_control_profile_.store(ControlProfile::kUnknown, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lock(control_profile_mutex_);
        control_profile_transition_phase_ = ControlProfileTransitionPhase::kIdle;
        control_profile_transition_active_ = false;
        control_profile_request_pending_ = false;
        control_profile_request_sequence_ = 0;
        control_profile_completed_sequence_ = 0;
        control_profile_inflight_sequence_ = 0;
        control_profile_resend_count_ = 0;
        control_profile_pending_target_ = ControlProfile::kUnknown;
        control_profile_source_profile_ = ControlProfile::kUnknown;
        control_profile_stabilize_cycles_remaining_ = 0;
        control_profile_last_packet_sent_at_ = Clock::time_point{};
        control_profile_last_progress_log_at_ = Clock::time_point{};
        control_profile_diag_snapshot_valid_ = false;
        control_profile_post_monitor_active_ = false;
        control_profile_post_monitor_source_ = ControlProfile::kUnknown;
        control_profile_post_monitor_target_ = ControlProfile::kUnknown;
        control_profile_post_monitor_started_at_ = Clock::time_point{};
        control_profile_post_monitor_deadline_ = Clock::time_point{};
        control_profile_post_monitor_last_log_at_ = Clock::time_point{};
        control_profile_post_monitor_snapshot_valid_ = false;
        control_profile_last_message_ =
            "Control profile transition framework reset during configure.";
    }
    mock_grippers_ = false;
    for (auto &slot : grippers_) {
        slot.mock = false;
        slot.command_target_percent.store(0.0, std::memory_order_relaxed);
        slot.state_percent.store(0.0, std::memory_order_relaxed);
        slot.state_valid.store(false, std::memory_order_relaxed);
        slot.has_sent_command = false;
        slot.last_command_percent = 0.0;
        slot.consecutive_send_failures = 0;
    }
    const auto &p = info_.hardware_parameters;

    joint_vel_ratio_     = param_int(p, "joint_vel_ratio",     30,   1, 100);
    joint_acc_ratio_     = param_int(p, "joint_acc_ratio",     30,   1, 100);
    joint_impedance_profile_enabled_ =
        param_bool(p, "joint_impedance_enabled", true);
    gripper_velocity_    = param_int(p, "gripper_velocity",    255,  0, 255);
    gripper_acceleration_= param_int(p, "gripper_acceleration",255,  0, 255);
    gripper_command_rate_hz_ = param_double(
        p, "gripper_command_rate_hz", 50.0, 1.0, 1000.0);
    gripper_command_epsilon_ = param_double(
        p, "gripper_command_epsilon", 1.0e-3, 0.0, 1.0);
    mock_grippers_ = param_bool(p, "mock_grippers", false);
    connect_timeout_ms_ = param_int(p, "connect_timeout_ms", 1500, 100, 30000);
    state_timeout_ms_ = param_int(p, "state_timeout_ms", 8000, 100, 30000);
    profile_switch_timeout_ms_ = param_int(
        p, "profile_switch_timeout_ms", 5000, 100, 10000);
    profile_switch_stabilize_cycles_ = param_int(
        p, "profile_switch_stabilize_cycles", 1, 1, 1000);
    profile_switch_post_monitor_ms_ = param_int(
        p, "profile_switch_post_monitor_ms", 500, 0, 5000);
    profile_switch_initial_resend_delay_ms_ = param_int(
        p, "profile_switch_initial_resend_delay_ms", 80, 0, 5000);
    profile_switch_resend_interval_ms_ = param_int(
        p, "profile_switch_resend_interval_ms", 20, 1, 1000);
    activation_retry_settle_ms_ = param_int(
        p, "activation_retry_settle_ms", 1500, 0, 10000);
    activation_max_attempts_ = param_int(
        p, "activation_max_attempts", 2, 1, 5);
    no_frame_timeout_ms_ = param_int(p, "no_frame_timeout_ms", 800, 50, 10000);

    if (!parse_fixed_double_array(
            param_str(p, "joint_impedance_k_left", "2 2 2 2 2 1 1"),
            joint_impedance_k_[0]) ||
        !parse_fixed_double_array(
            param_str(p, "joint_impedance_d_left", "0.4 0.4 0.4 0.4 0.4 0.4 0.4"),
            joint_impedance_d_[0]) ||
        !parse_fixed_double_array(
            param_str(p, "joint_impedance_k_right", "2 2 2 2 2 1 1"),
            joint_impedance_k_[1]) ||
        !parse_fixed_double_array(
            param_str(p, "joint_impedance_d_right", "0.4 0.4 0.4 0.4 0.4 0.4 0.4"),
            joint_impedance_d_[1])) {
        RCLCPP_ERROR(
            get_logger(),
            "Failed to parse joint impedance K/D parameters. "
            "Expected 7 whitespace-separated values per arm.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (!parse_fixed_double_array(
            param_str(p, "tool_kine_left", "0 0 0 0 0 0"),
            tool_kine_[0]) ||
        !parse_fixed_double_array(
            param_str(p, "tool_dyn_left", "0 0 0 0 0 0 0 0 0 0"),
            tool_dyn_[0]) ||
        !parse_fixed_double_array(
            param_str(p, "tool_kine_right", "0 0 0 0 0 0"),
            tool_kine_[1]) ||
        !parse_fixed_double_array(
            param_str(p, "tool_dyn_right", "0 0 0 0 0 0 0 0 0 0"),
            tool_dyn_[1])) {
        RCLCPP_ERROR(
            get_logger(),
            "Failed to parse tool kinematics/dynamics parameters. "
            "Expected 6 kine values and 10 dyn values per arm.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    Ip4 ip{};
    const auto ip_str = param_str(p, "ip", "192.168.1.190");
    if (!parse_ip4(ip_str, ip)) {
        RCLCPP_ERROR(get_logger(), "Invalid IP '%s'.", ip_str.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    const auto deadline = Clock::now() + std::chrono::milliseconds(connect_timeout_ms_);
    while (!OnLinkTo(ip.a, ip.b, ip.c, ip.d)) {
        if (Clock::now() > deadline) {
            RCLCPP_ERROR(get_logger(), "OnLinkTo(%s) timeout.", ip_str.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    connected_ = true;

    constexpr auto kPostLinkSettle = std::chrono::milliseconds(100);
    constexpr auto kPreClearSetDelay = std::chrono::milliseconds(10);
    constexpr auto kClearErrCommandDelay = std::chrono::milliseconds(200);
    constexpr auto kPostClearSendSettle = std::chrono::milliseconds(300);

    auto clear_arm_error = [this, kPreClearSetDelay, kClearErrCommandDelay, kPostClearSendSettle](
                               const char *arm_name,
                               void (*clear_fn)()) -> hardware_interface::CallbackReturn {
        std::this_thread::sleep_for(kPreClearSetDelay);
        if (!OnClearSet()) {
            RCLCPP_ERROR(get_logger(), "OnClearSet failed before clearing %s error.", arm_name);
            return hardware_interface::CallbackReturn::ERROR;
        }
        std::this_thread::sleep_for(kPreClearSetDelay);
        clear_fn();
        std::this_thread::sleep_for(kClearErrCommandDelay);
        if (!OnSetSend()) {
            RCLCPP_ERROR(get_logger(), "OnSetSend failed while clearing %s error.", arm_name);
            return hardware_interface::CallbackReturn::ERROR;
        }
        std::this_thread::sleep_for(kPostClearSendSettle);
        return hardware_interface::CallbackReturn::SUCCESS;
    };

    std::this_thread::sleep_for(kPostLinkSettle);
    RCLCPP_INFO(get_logger(), "Clearing Arm A errors with conservative timing.");
    if (clear_arm_error("Arm A", OnClearErr_A) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_logger(), "Clearing Arm B errors with conservative timing.");
    if (clear_arm_error("Arm B", OnClearErr_B) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (param_int(p, "sdk_log_enabled", 0, 0, 1) == 0) {
        OnLogOff();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        OnLocalLogOff();
    }

    RCLCPP_INFO(get_logger(), "SDK connected to %s. Settling for 1s ...", ip_str.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Verify that both arms are error-free after clearing.
    DCSS check{};
    if (OnGetBuf(&check)) {
        for (size_t arm = 0; arm < kArmCount; ++arm) {
            if (static_cast<ArmState>(check.m_State[arm].m_CurState) == ARM_STATE_ERROR) {
                RCLCPP_ERROR(get_logger(),
                    "Arm %zu still in ERROR state (err=%d) after clearing.",
                    arm, check.m_State[arm].m_ERRCode);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
    }

    // Connect OmniPicker grippers (Marvin link already established, manage_link=false)
    for (size_t g = 0; g < gripper_count_; ++g) {
        auto &slot = grippers_[g];
        const auto &jn = info_.joints[slot.joint_index].name;
        slot.mock = mock_grippers_;

        if (slot.mock) {
            slot.command_target_percent.store(0.0, std::memory_order_relaxed);
            slot.state_percent.store(0.0, std::memory_order_relaxed);
            slot.state_valid.store(true, std::memory_order_relaxed);
            slot.has_sent_command = false;
            slot.last_command_percent = 0.0;
            slot.consecutive_send_failures = 0;
            RCLCPP_WARN(
                get_logger(),
                "Gripper '%s' hardware is mocked; skipping physical connection.",
                jn.c_str());
            continue;
        }

        slot.device = omnipicker_create();
        auto err = slot.device->connect(slot.arm_side, slot.can_node_id,
                                        ip_str.c_str(), false);
        if (err != omnipicker::ErrorCode::kOK) {
            RCLCPP_ERROR(get_logger(), "Gripper '%s' connect failed: %s.",
                         jn.c_str(), omnipicker::error_to_string(err));
            for (size_t k = 0; k <= g; ++k) {
                if (grippers_[k].device) {
                    omnipicker_destroy(grippers_[k].device);
                    grippers_[k].device = nullptr;
                }
            }
            return hardware_interface::CallbackReturn::ERROR;
        }
        slot.device->set_default_velocity(static_cast<uint8_t>(gripper_velocity_));
        slot.device->set_default_acceleration(static_cast<uint8_t>(gripper_acceleration_));
        slot.device->set_default_deceleration(static_cast<uint8_t>(gripper_acceleration_));
        RCLCPP_INFO(get_logger(), "Gripper '%s' connected (arm=%s, CAN=%u).",
                     jn.c_str(),
                     slot.arm_side == omnipicker::ArmSide::kA ? "A" : "B",
                     slot.can_node_id);
        RCLCPP_INFO(get_logger(),
                    "Gripper '%s' defaults: velocity=%d, acceleration=%d, deceleration=%d.",
                    jn.c_str(),
                    gripper_velocity_,
                    gripper_acceleration_,
                    gripper_acceleration_);
    }

    // Parse home position (radians, space-separated, 7 values per arm)
    auto home_l_str = param_str(p, "home_position_L", "");
    auto home_r_str = param_str(p, "home_position_R", "");

    if (!home_l_str.empty() && !home_r_str.empty()) {
        double vals[kJointsPerArm];
        if (std::sscanf(home_l_str.c_str(), "%lf %lf %lf %lf %lf %lf %lf",
                        &vals[0], &vals[1], &vals[2], &vals[3],
                        &vals[4], &vals[5], &vals[6]) == 7) {
            for (size_t j = 0; j < kJointsPerArm; ++j)
                home_position_deg_[0][j] = vals[j] * kRad2Deg;
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to parse 'home_position_L' (expected 7 values).");
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (std::sscanf(home_r_str.c_str(), "%lf %lf %lf %lf %lf %lf %lf",
                        &vals[0], &vals[1], &vals[2], &vals[3],
                        &vals[4], &vals[5], &vals[6]) == 7) {
            for (size_t j = 0; j < kJointsPerArm; ++j)
                home_position_deg_[1][j] = vals[j] * kRad2Deg;
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to parse 'home_position_R' (expected 7 values).");
            return hardware_interface::CallbackReturn::ERROR;
        }

        has_home_position_ = true;
        home_timeout_ms_ = param_int(p, "home_timeout_ms", 30000, 1000, 120000);
        RCLCPP_INFO(get_logger(),
            "Home position configured (timeout=%d ms).", home_timeout_ms_);
    }

    workspace_guard_.configure(p, get_logger());
    collision_guard_.configure(p, get_node(), get_logger());
    if (collision_guard_.enabled()) {
        if (collision_guard_.ready()) {
            RCLCPP_INFO(get_logger(), "%s", collision_guard_.status_message().c_str());
        } else {
            RCLCPP_WARN(get_logger(), "%s", collision_guard_.status_message().c_str());
        }
    }

    RCLCPP_INFO(get_logger(),
                "Configured dual-arm system (joint_vel=%d%%, joint_acc=%d%%, "
                "joint_impedance=%s, profile_switch_timeout_ms=%d, stabilize_cycles=%d, "
                "post_monitor_ms=%d, initial_resend_delay_ms=%d, resend_interval_ms=%d, "
                "gripper_velocity=%d, gripper_acceleration=%d, grippers=%zu, "
                "collision_guard=%s, mock_grippers=%s).",
                joint_vel_ratio_,
                joint_acc_ratio_,
                joint_impedance_profile_enabled_ ? "enabled" : "disabled",
                profile_switch_timeout_ms_,
                profile_switch_stabilize_cycles_,
                profile_switch_post_monitor_ms_,
                profile_switch_initial_resend_delay_ms_,
                profile_switch_resend_interval_ms_,
                gripper_velocity_,
                gripper_acceleration_,
                gripper_count_,
                collision_guard_.active() ? "active" : (collision_guard_.enabled() ? "configured" : "off"),
                mock_grippers_ ? "on" : "off");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_activate – enter position-follow mode on BOTH arms, seed state
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn MarvinHardware::on_activate(
    const rclcpp_lifecycle::State &)
{
    stop_collision_guard_worker();
    constexpr auto kPreClearSetDelay = std::chrono::milliseconds(10);
    constexpr auto kClearErrCommandDelay = std::chrono::milliseconds(200);
    constexpr auto kPostClearSendSettle = std::chrono::milliseconds(300);
    constexpr auto kPostActivationStability = std::chrono::milliseconds(300);

    stop_gripper_worker();
    DCSS dcss{};
    if (!OnGetBuf(&dcss)) {
        RCLCPP_ERROR(get_logger(), "OnGetBuf failed before activation.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    double hold_a[kJointsPerArm], hold_b[kJointsPerArm];

    auto log_mode_switch_status =
        [this](size_t arm, const StateCtr &state, const char *phase) {
            const auto cur_state = static_cast<ArmState>(state.m_CurState);
            RCLCPP_INFO(
                get_logger(),
                "Arm %s %s: cur_state=%s(%d), cmd_state=%d, err=%d.",
                arm == 0 ? "A" : "B",
                phase,
                arm_state_name(cur_state),
                state.m_CurState,
                state.m_CmdState,
                state.m_ERRCode);
        };

    auto refresh_hold_positions = [&]() {
        for (size_t j = 0; j < kJointsPerArm; ++j) {
            hold_a[j] = static_cast<double>(dcss.m_Out[0].m_FB_Joint_PosE[j]);
            hold_b[j] = static_cast<double>(dcss.m_Out[1].m_FB_Joint_PosE[j]);
        }
    };

    auto request_position_mode = [&](bool arm_a_pending, bool arm_b_pending) {
        OnClearSet();
        bool ok = true;
        if (arm_a_pending) {
            ok = ok && OnSetTargetState_A(1);
            ok = ok && OnSetJointLmt_A(joint_vel_ratio_, joint_acc_ratio_);
        }
        if (arm_b_pending) {
            ok = ok && OnSetTargetState_B(1);
            ok = ok && OnSetJointLmt_B(joint_vel_ratio_, joint_acc_ratio_);
        }
        ok = ok && OnSetJointCmdPos_A(hold_a);
        ok = ok && OnSetJointCmdPos_B(hold_b);
        return ok && OnSetSend();
    };

    auto request_idle_mode = [&]() {
        OnClearSet();
        const bool ok = OnSetTargetState_A(0)
                     && OnSetTargetState_B(0)
                     && OnSetSend();
        if (!ok) {
            RCLCPP_WARN(get_logger(),
                        "Failed to request idle mode during activation recovery.");
        }
    };

    auto clear_arm_error =
        [this, kPreClearSetDelay, kClearErrCommandDelay, kPostClearSendSettle](
            const char *arm_name, void (*clear_fn)()) {
        std::this_thread::sleep_for(kPreClearSetDelay);
        if (!OnClearSet()) {
            RCLCPP_ERROR(get_logger(),
                         "OnClearSet failed before clearing %s during activation recovery.",
                         arm_name);
            return false;
        }
        std::this_thread::sleep_for(kPreClearSetDelay);
        clear_fn();
        std::this_thread::sleep_for(kClearErrCommandDelay);
        if (!OnSetSend()) {
            RCLCPP_ERROR(get_logger(),
                         "OnSetSend failed while clearing %s during activation recovery.",
                         arm_name);
            return false;
        }
        std::this_thread::sleep_for(kPostClearSendSettle);
        return true;
    };

    auto clear_errors_for_retry = [&]() {
        RCLCPP_INFO(get_logger(),
                    "Clearing Arm A/B errors before activation retry.");
        return clear_arm_error("Arm A", OnClearErr_A) &&
               clear_arm_error("Arm B", OnClearErr_B);
    };

    refresh_hold_positions();

    bool mode_switch_succeeded = false;
    std::array<bool, kArmCount> timeout_has_status{{false, false}};
    std::array<int, kArmCount> timeout_cur_state{};
    std::array<int, kArmCount> timeout_cmd_state{};
    std::array<int, kArmCount> timeout_err_code{};
    bool timeout_arm_ready_a = false;
    bool timeout_arm_ready_b = false;

    for (int attempt = 1; attempt <= activation_max_attempts_; ++attempt) {
        bool attempt_faulted = false;
        size_t fault_arm = 0;
        int fault_cur_state = 0;
        int fault_cmd_state = 0;
        int fault_err_code = 0;
        const char *fault_phase = "";

        auto record_fault = [&](size_t arm, const StateCtr &state, const char *phase) {
            attempt_faulted = true;
            fault_arm = arm;
            fault_cur_state = state.m_CurState;
            fault_cmd_state = state.m_CmdState;
            fault_err_code = state.m_ERRCode;
            fault_phase = phase;
        };

        const char *pre_activate_phase =
            attempt == 1 ? "pre-activate state" : "pre-activate retry state";
        for (size_t arm = 0; arm < kArmCount; ++arm) {
            log_mode_switch_status(arm, dcss.m_State[arm], pre_activate_phase);
        }

        if (!request_position_mode(true, true)) {
            if (attempt == activation_max_attempts_) {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to request position mode on dual-arm (attempt %d/%d).",
                    attempt,
                    activation_max_attempts_);
                return hardware_interface::CallbackReturn::ERROR;
            }

            RCLCPP_WARN(
                get_logger(),
                "Failed to request position mode on dual-arm (attempt %d/%d). "
                "Waiting %d ms and retrying for cold-start recovery.",
                attempt,
                activation_max_attempts_,
                activation_retry_settle_ms_);
            request_idle_mode();
            if (activation_retry_settle_ms_ > 0) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(activation_retry_settle_ms_));
            }
            if (!OnGetBuf(&dcss)) {
                RCLCPP_ERROR(get_logger(), "OnGetBuf failed before activation retry.");
                return hardware_interface::CallbackReturn::ERROR;
            }
            refresh_hold_positions();
            continue;
        }

        bool arm_a_ready = false;
        bool arm_b_ready = false;
        const auto deadline = Clock::now() + std::chrono::milliseconds(state_timeout_ms_);
        std::array<bool, kArmCount> has_status_snapshot{{false, false}};
        std::array<int, kArmCount> last_cur_state{};
        std::array<int, kArmCount> last_cmd_state{};
        std::array<int, kArmCount> last_err_code{};

        while (!arm_a_ready || !arm_b_ready) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            if (OnGetBuf(&dcss)) {
                // Do not refresh hold_a/hold_b here: if the arm is sagging while
                // switching into position mode, following the latest feedback would
                // turn that sag into the new commanded hold target.
                for (size_t arm = 0; arm < kArmCount; ++arm) {
                    const auto &state = dcss.m_State[arm];
                    if (!has_status_snapshot[arm] ||
                        state.m_CurState != last_cur_state[arm] ||
                        state.m_CmdState != last_cmd_state[arm] ||
                        state.m_ERRCode != last_err_code[arm]) {
                        log_mode_switch_status(
                            arm, state, has_status_snapshot[arm]
                            ? "mode-switch update"
                            : "mode-switch feedback");
                        has_status_snapshot[arm] = true;
                        last_cur_state[arm] = state.m_CurState;
                        last_cmd_state[arm] = state.m_CmdState;
                        last_err_code[arm] = state.m_ERRCode;
                    }
                }

                if (!arm_a_ready) {
                    const auto &state = dcss.m_State[0];
                    const auto st = static_cast<ArmState>(state.m_CurState);
                    if (st == ARM_STATE_ERROR) {
                        record_fault(0, state, "mode switch");
                        break;
                    }
                    if (state.m_ERRCode != 0) {
                        record_fault(0, state, "mode switch");
                        break;
                    }
                    if (st == ARM_STATE_POSITION) {
                        arm_a_ready = true;
                        RCLCPP_INFO(get_logger(), "Arm A (left) entered position mode.");
                    }
                }
                if (!arm_b_ready) {
                    const auto &state = dcss.m_State[1];
                    const auto st = static_cast<ArmState>(state.m_CurState);
                    if (st == ARM_STATE_ERROR) {
                        record_fault(1, state, "mode switch");
                        break;
                    }
                    if (state.m_ERRCode != 0) {
                        record_fault(1, state, "mode switch");
                        break;
                    }
                    if (st == ARM_STATE_POSITION) {
                        arm_b_ready = true;
                        RCLCPP_INFO(get_logger(), "Arm B (right) entered position mode.");
                    }
                }
            }

            if (attempt_faulted) {
                break;
            }

            if (arm_a_ready && arm_b_ready) {
                mode_switch_succeeded = true;
                break;
            }

            if (Clock::now() > deadline) {
                timeout_has_status = has_status_snapshot;
                timeout_cur_state = last_cur_state;
                timeout_cmd_state = last_cmd_state;
                timeout_err_code = last_err_code;
                timeout_arm_ready_a = arm_a_ready;
                timeout_arm_ready_b = arm_b_ready;
                break;
            }

            // Keep feeding both arms; re-request state for arms still transitioning.
            request_position_mode(!arm_a_ready, !arm_b_ready);
        }

        if (mode_switch_succeeded) {
            const auto stability_deadline = Clock::now() + kPostActivationStability;
            while (Clock::now() < stability_deadline) {
                if (!request_position_mode(false, false)) {
                    RCLCPP_ERROR(
                        get_logger(),
                        "Failed to maintain hold-position command during activation stability check.");
                    return hardware_interface::CallbackReturn::ERROR;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                if (!OnGetBuf(&dcss)) {
                    RCLCPP_ERROR(get_logger(), "OnGetBuf failed during activation stability check.");
                    return hardware_interface::CallbackReturn::ERROR;
                }

                bool stable = true;
                for (size_t arm = 0; arm < kArmCount; ++arm) {
                    const auto &state = dcss.m_State[arm];
                    const auto arm_state = static_cast<ArmState>(state.m_CurState);
                    if (arm_state != ARM_STATE_POSITION || state.m_ERRCode != 0) {
                        record_fault(arm, state, "post-activation stability");
                        stable = false;
                        break;
                    }
                }

                if (!stable) {
                    mode_switch_succeeded = false;
                    break;
                }
            }
        }

        if (mode_switch_succeeded) {
            const int activation_frame_timeout_ms =
                std::max(no_frame_timeout_ms_, activation_retry_settle_ms_);
            std::array<int, kArmCount> initial_frame_serial{};
            std::array<bool, kArmCount> frame_advanced{{false, false}};
            for (size_t arm = 0; arm < kArmCount; ++arm) {
                initial_frame_serial[arm] = dcss.m_Out[arm].m_OutFrameSerial;
            }

            const auto frame_deadline =
                Clock::now() + std::chrono::milliseconds(activation_frame_timeout_ms);
            while (!(frame_advanced[0] && frame_advanced[1])) {
                if (!request_position_mode(false, false)) {
                    RCLCPP_ERROR(
                        get_logger(),
                        "Failed to maintain hold-position command during activation frame-progress check.");
                    return hardware_interface::CallbackReturn::ERROR;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                if (!OnGetBuf(&dcss)) {
                    RCLCPP_ERROR(
                        get_logger(),
                        "OnGetBuf failed during activation frame-progress check.");
                    return hardware_interface::CallbackReturn::ERROR;
                }

                bool stable = true;
                for (size_t arm = 0; arm < kArmCount; ++arm) {
                    const auto &state = dcss.m_State[arm];
                    const auto arm_state = static_cast<ArmState>(state.m_CurState);
                    if (arm_state != ARM_STATE_POSITION || state.m_ERRCode != 0) {
                        record_fault(arm, state, "post-activation frame check");
                        stable = false;
                        break;
                    }
                    if (dcss.m_Out[arm].m_OutFrameSerial != initial_frame_serial[arm]) {
                        frame_advanced[arm] = true;
                    }
                }

                if (!stable) {
                    mode_switch_succeeded = false;
                    break;
                }

                if (Clock::now() > frame_deadline) {
                    break;
                }
            }

            if (mode_switch_succeeded && !(frame_advanced[0] && frame_advanced[1])) {
                if (attempt == activation_max_attempts_) {
                    RCLCPP_ERROR(
                        get_logger(),
                        "Activation succeeded logically but feedback frames did not advance "
                        "within %d ms on attempt %d/%d (A: start=%d, current=%d; "
                        "B: start=%d, current=%d).",
                        activation_frame_timeout_ms,
                        attempt,
                        activation_max_attempts_,
                        initial_frame_serial[0],
                        dcss.m_Out[0].m_OutFrameSerial,
                        initial_frame_serial[1],
                        dcss.m_Out[1].m_OutFrameSerial);
                    return hardware_interface::CallbackReturn::ERROR;
                }

                RCLCPP_WARN(
                    get_logger(),
                    "Activation attempt %d/%d reached POSITION mode but feedback frames did "
                    "not advance within %d ms (A: start=%d, current=%d; "
                    "B: start=%d, current=%d). Waiting %d ms and retrying.",
                    attempt,
                    activation_max_attempts_,
                    activation_frame_timeout_ms,
                    initial_frame_serial[0],
                    dcss.m_Out[0].m_OutFrameSerial,
                    initial_frame_serial[1],
                    dcss.m_Out[1].m_OutFrameSerial,
                    activation_retry_settle_ms_);
                mode_switch_succeeded = false;
                request_idle_mode();
                if (activation_retry_settle_ms_ > 0) {
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(activation_retry_settle_ms_));
                }
                if (!OnGetBuf(&dcss)) {
                    RCLCPP_ERROR(get_logger(), "OnGetBuf failed before activation retry.");
                    return hardware_interface::CallbackReturn::ERROR;
                }
                continue;
            }
        }

        if (mode_switch_succeeded) {
            break;
        }

        if (attempt_faulted) {
            const auto state = static_cast<ArmState>(fault_cur_state);
            if (attempt == activation_max_attempts_) {
                RCLCPP_ERROR(
                    get_logger(),
                    "Activation attempt %d/%d failed during %s on arm %s "
                    "(cur_state=%s(%d), cmd_state=%d, err=%d).",
                    attempt,
                    activation_max_attempts_,
                    fault_phase,
                    fault_arm == 0 ? "A" : "B",
                    arm_state_name(state),
                    fault_cur_state,
                    fault_cmd_state,
                    fault_err_code);
                return hardware_interface::CallbackReturn::ERROR;
            }

            RCLCPP_WARN(
                get_logger(),
                "Activation attempt %d/%d failed during %s on arm %s "
                "(cur_state=%s(%d), cmd_state=%d, err=%d). "
                "Clearing errors, waiting %d ms, and retrying for cold-start recovery.",
                attempt,
                activation_max_attempts_,
                fault_phase,
                fault_arm == 0 ? "A" : "B",
                arm_state_name(state),
                fault_cur_state,
                fault_cmd_state,
                fault_err_code,
                activation_retry_settle_ms_);
            request_idle_mode();
            if (!clear_errors_for_retry()) {
                return hardware_interface::CallbackReturn::ERROR;
            }
            if (activation_retry_settle_ms_ > 0) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(activation_retry_settle_ms_));
            }
            if (!OnGetBuf(&dcss)) {
                RCLCPP_ERROR(get_logger(), "OnGetBuf failed before activation retry.");
                return hardware_interface::CallbackReturn::ERROR;
            }
            refresh_hold_positions();
            continue;
        }

        const auto a_state = timeout_has_status[0]
                           ? static_cast<ArmState>(timeout_cur_state[0])
                           : static_cast<ArmState>(dcss.m_State[0].m_CurState);
        const auto b_state = timeout_has_status[1]
                           ? static_cast<ArmState>(timeout_cur_state[1])
                           : static_cast<ArmState>(dcss.m_State[1].m_CurState);

        if (attempt == activation_max_attempts_) {
            RCLCPP_ERROR(
                get_logger(),
                "Timeout waiting for position mode after %d ms on attempt %d/%d "
                "(A: ready=%s, cur_state=%s(%d), cmd_state=%d, err=%d; "
                "B: ready=%s, cur_state=%s(%d), cmd_state=%d, err=%d).",
                state_timeout_ms_,
                attempt,
                activation_max_attempts_,
                timeout_arm_ready_a ? "OK" : "PENDING",
                arm_state_name(a_state),
                timeout_has_status[0] ? timeout_cur_state[0] : dcss.m_State[0].m_CurState,
                timeout_has_status[0] ? timeout_cmd_state[0] : dcss.m_State[0].m_CmdState,
                timeout_has_status[0] ? timeout_err_code[0] : dcss.m_State[0].m_ERRCode,
                timeout_arm_ready_b ? "OK" : "PENDING",
                arm_state_name(b_state),
                timeout_has_status[1] ? timeout_cur_state[1] : dcss.m_State[1].m_CurState,
                timeout_has_status[1] ? timeout_cmd_state[1] : dcss.m_State[1].m_CmdState,
                timeout_has_status[1] ? timeout_err_code[1] : dcss.m_State[1].m_ERRCode);
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_WARN(
            get_logger(),
            "Position-mode activation attempt %d/%d timed out after %d ms "
            "(A: ready=%s, cur_state=%s(%d), cmd_state=%d, err=%d; "
            "B: ready=%s, cur_state=%s(%d), cmd_state=%d, err=%d). "
            "Waiting %d ms and retrying for cold-start recovery.",
            attempt,
            activation_max_attempts_,
            state_timeout_ms_,
            timeout_arm_ready_a ? "OK" : "PENDING",
            arm_state_name(a_state),
            timeout_has_status[0] ? timeout_cur_state[0] : dcss.m_State[0].m_CurState,
            timeout_has_status[0] ? timeout_cmd_state[0] : dcss.m_State[0].m_CmdState,
            timeout_has_status[0] ? timeout_err_code[0] : dcss.m_State[0].m_ERRCode,
            timeout_arm_ready_b ? "OK" : "PENDING",
            arm_state_name(b_state),
            timeout_has_status[1] ? timeout_cur_state[1] : dcss.m_State[1].m_CurState,
            timeout_has_status[1] ? timeout_cmd_state[1] : dcss.m_State[1].m_CmdState,
            timeout_has_status[1] ? timeout_err_code[1] : dcss.m_State[1].m_ERRCode,
            activation_retry_settle_ms_);
        request_idle_mode();
        if (!clear_errors_for_retry()) {
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (activation_retry_settle_ms_ > 0) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(activation_retry_settle_ms_));
        }
        if (!OnGetBuf(&dcss)) {
            RCLCPP_ERROR(get_logger(), "OnGetBuf failed before activation retry.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        refresh_hold_positions();
    }

    if (!mode_switch_succeeded) {
        RCLCPP_ERROR(get_logger(), "Failed to activate dual-arm position mode.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (has_home_position_) {
        RCLCPP_WARN(
            get_logger(),
            "Home position is configured in hardware parameters but startup auto-home is disabled. "
            "Use the motion layer to execute a planned go-home sequence.");
    }

    // Seed ros2_control state & command interfaces from current feedback.
    OnGetBuf(&dcss);
    for (size_t arm = 0; arm < kArmCount; ++arm) {
        current_sdk_cur_state_[arm].store(
            dcss.m_State[arm].m_CurState, std::memory_order_relaxed);
        current_sdk_cmd_state_[arm].store(
            dcss.m_State[arm].m_CmdState, std::memory_order_relaxed);
        current_sdk_err_code_[arm].store(
            dcss.m_State[arm].m_ERRCode, std::memory_order_relaxed);
        current_sdk_in_frame_serial_[arm].store(
            dcss.m_In[arm].m_InFrameSerial, std::memory_order_relaxed);
        current_sdk_out_frame_serial_[arm].store(
            dcss.m_Out[arm].m_OutFrameSerial, std::memory_order_relaxed);
        const auto &out = dcss.m_Out[arm];
        for (size_t j = 0; j < kJointsPerArm; ++j) {
            const size_t idx = arm * kJointsPerArm + j;
            const auto &jn = info_.joints[idx].name;
            const double pos_rad = static_cast<double>(out.m_FB_Joint_PosE[j]) * kDeg2Rad;
            const double pos_deg = static_cast<double>(out.m_FB_Joint_PosE[j]);
            set_state(pos_if(jn), pos_rad);
            set_command(pos_if(jn), pos_rad);
            current_feedback_deg_[idx].store(pos_deg, std::memory_order_relaxed);
            collision_guard_target_deg_[idx].store(pos_deg, std::memory_order_relaxed);
            collision_guard_approved_deg_[idx].store(pos_deg, std::memory_order_relaxed);
            if (has_velocity_state_) {
                set_state(vel_if(jn), static_cast<double>(out.m_FB_Joint_Vel[j]) * kDeg2Rad);
            }
            if (has_effort_state_) {
                set_state(eff_if(jn), static_cast<double>(out.m_FB_Joint_SToq[j]));
            }
        }
        last_frame_serial_[arm] = out.m_OutFrameSerial;
        last_frame_time_[arm] = Clock::now();
    }
    current_feedback_valid_.store(true, std::memory_order_relaxed);
    collision_guard_approved_valid_.store(true, std::memory_order_relaxed);
    active_control_profile_.store(ControlProfile::kPositionFollow, std::memory_order_relaxed);
    requested_control_profile_.store(ControlProfile::kPositionFollow, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lock(control_profile_mutex_);
        control_profile_transition_phase_ = ControlProfileTransitionPhase::kIdle;
        control_profile_transition_active_ = false;
        control_profile_request_pending_ = false;
        control_profile_resend_count_ = 0;
        control_profile_pending_target_ = ControlProfile::kUnknown;
        control_profile_source_profile_ = ControlProfile::kUnknown;
        control_profile_stabilize_cycles_remaining_ = 0;
        control_profile_last_packet_sent_at_ = Clock::time_point{};
        control_profile_last_progress_log_at_ = Clock::time_point{};
        control_profile_diag_snapshot_valid_ = false;
        control_profile_post_monitor_active_ = false;
        control_profile_post_monitor_source_ = ControlProfile::kUnknown;
        control_profile_post_monitor_target_ = ControlProfile::kUnknown;
        control_profile_post_monitor_started_at_ = Clock::time_point{};
        control_profile_post_monitor_deadline_ = Clock::time_point{};
        control_profile_post_monitor_last_log_at_ = Clock::time_point{};
        control_profile_post_monitor_snapshot_valid_ = false;
        control_profile_last_message_ =
            "Hardware activated in POSITION_FOLLOW profile.";
    }
    update_sdk_observation_states(dcss);

    // Seed gripper state & command (query_states blocks briefly, OK during activation)
    for (size_t g = 0; g < gripper_count_; ++g) {
        auto &slot = grippers_[g];
        const auto &jn = info_.joints[slot.joint_index].name;

        double init_pos = 0.0;
        if (slot.mock || !slot.device) {
            init_pos = clamp_unit(slot.command_target_percent.load(std::memory_order_relaxed));
            RCLCPP_INFO(get_logger(), "Gripper '%s' ready in mock mode (pos=%.1f%%).",
                        jn.c_str(), init_pos * 100.0);
        } else {
            omnipicker::GripperStatus gst;
            auto err = slot.device->query_states(gst);
            if (err == omnipicker::ErrorCode::kOK && gst.valid) {
                init_pos = static_cast<double>(gst.position) / 255.0;
            } else {
                RCLCPP_WARN(get_logger(),
                    "Gripper '%s' initial query returned %s; defaulting to 0.0.",
                    jn.c_str(), omnipicker::error_to_string(err));
            }
            RCLCPP_INFO(get_logger(), "Gripper '%s' ready (pos=%.1f%%).",
                        jn.c_str(), init_pos * 100.0);
        }
        set_state(pos_if(jn), init_pos);
        set_command(pos_if(jn), init_pos);
        slot.command_target_percent.store(init_pos, std::memory_order_relaxed);
        slot.state_percent.store(init_pos, std::memory_order_relaxed);
        slot.state_valid.store(true, std::memory_order_relaxed);
        slot.has_sent_command = false;
        slot.last_command_percent = init_pos;
        slot.last_command_time = Clock::now();
        slot.consecutive_send_failures = 0;
    }

    auto evaluate_activation_collision_clearance =
        [this, &dcss]() -> std::optional<CollisionGuard::Evaluation> {
        if (!collision_guard_.enabled() || !collision_guard_.ready()) {
            return std::nullopt;
        }

        std::array<std::array<double, kJointsPerArm>, kArmCount> current_deg{};
        for (size_t arm = 0; arm < kArmCount; ++arm) {
            for (size_t joint = 0; joint < kJointsPerArm; ++joint) {
                current_deg[arm][joint] =
                    static_cast<double>(dcss.m_Out[arm].m_FB_Joint_PosE[joint]);
            }
        }

        std::array<double, CollisionGuard::kGripperCount> gripper_percent{{0.0, 0.0}};
        std::array<bool, CollisionGuard::kGripperCount> gripper_valid{{false, false}};
        for (size_t g = 0; g < gripper_count_ && g < CollisionGuard::kGripperCount; ++g) {
            gripper_percent[g] = grippers_[g].state_percent.load(std::memory_order_relaxed);
            gripper_valid[g] = grippers_[g].state_valid.load(std::memory_order_relaxed);
        }

        const bool was_armed = collision_guard_.armed();
        if (!was_armed) {
            collision_guard_.arm();
        }
        const auto evaluation = collision_guard_.evaluate_motion(
            current_deg, current_deg, gripper_percent, gripper_valid, get_logger());
        if (!was_armed) {
            collision_guard_.disarm();
        }
        return evaluation;
    };

    if (const auto evaluation = evaluate_activation_collision_clearance()) {
        if (evaluation->current_distance_m <= collision_guard_.hard_collision_distance_m()) {
            constexpr auto kActivationCollisionRecoveryTimeout = std::chrono::milliseconds(1500);
            constexpr auto kActivationCollisionRecoveryStep = std::chrono::milliseconds(20);
            RCLCPP_ERROR(
                get_logger(),
                "Activation starts inside hard collision band (distance=%.4f m <= %.4f m). "
                "Holding current joint targets before allowing controllers to start.",
                evaluation->current_distance_m,
                collision_guard_.hard_collision_distance_m());

            const auto recovery_deadline = Clock::now() + kActivationCollisionRecoveryTimeout;
            auto latest_evaluation = *evaluation;
            while (Clock::now() < recovery_deadline) {
                if (!request_position_mode(false, false)) {
                    RCLCPP_ERROR(
                        get_logger(),
                        "Failed to resend hold-position command while recovering from startup collision.");
                    return hardware_interface::CallbackReturn::ERROR;
                }
                std::this_thread::sleep_for(kActivationCollisionRecoveryStep);
                if (!OnGetBuf(&dcss)) {
                    RCLCPP_ERROR(
                        get_logger(),
                        "OnGetBuf failed while verifying startup collision recovery.");
                    return hardware_interface::CallbackReturn::ERROR;
                }
                if (const auto refreshed = evaluate_activation_collision_clearance()) {
                    latest_evaluation = *refreshed;
                    if (latest_evaluation.current_distance_m >
                        collision_guard_.hard_collision_distance_m() +
                            collision_guard_.escape_min_distance_improvement_m()) {
                        break;
                    }
                }
            }

            if (latest_evaluation.current_distance_m <= collision_guard_.hard_collision_distance_m()) {
                RCLCPP_ERROR(
                    get_logger(),
                    "Refusing to finish activation because startup clearance is still inside "
                    "the hard collision band (distance=%.4f m, hard=%.4f m).",
                    latest_evaluation.current_distance_m,
                    collision_guard_.hard_collision_distance_m());
                return hardware_interface::CallbackReturn::ERROR;
            }

            RCLCPP_WARN(
                get_logger(),
                "Startup collision clearance recovered to %.4f m before controller activation.",
                latest_evaluation.current_distance_m);
        } else if (evaluation->current_distance_m < collision_guard_.near_distance_m()) {
            RCLCPP_WARN(
                get_logger(),
                "Activation starts inside the near-collision band (distance=%.4f m, near=%.4f m).",
                evaluation->current_distance_m,
                collision_guard_.near_distance_m());
        }
    }

    consecutive_write_failures_ = 0;
    total_write_failures_ = 0;

    if (workspace_guard_.enabled()) {
        for (size_t arm = 0; arm < kArmCount; ++arm) {
            double fb[kJointsPerArm];
            for (size_t j = 0; j < kJointsPerArm; ++j)
                fb[j] = static_cast<double>(dcss.m_Out[arm].m_FB_Joint_PosE[j]);
            workspace_guard_.seed(arm, fb);
        }
        if (workspace_guard_runtime_enabled_.load(std::memory_order_relaxed)) {
            workspace_guard_.arm();
            RCLCPP_INFO(get_logger(),
                        "Workspace z-floor check armed for runtime commands.");
        } else {
            RCLCPP_INFO(
                get_logger(),
                "Workspace z-floor check is configured but runtime enforcement is disabled.");
        }
    }

    if (collision_guard_.enabled()) {
        if (collision_guard_.ready()) {
            if (collision_guard_runtime_enabled_.load(std::memory_order_relaxed)) {
                collision_guard_.arm();
                start_collision_guard_worker();
                RCLCPP_INFO(get_logger(), "Async collision guard armed for runtime commands.");
            } else {
                RCLCPP_INFO(
                    get_logger(),
                    "Collision guard is configured but runtime enforcement is disabled.");
            }
        } else {
            RCLCPP_WARN(
                get_logger(),
                "Collision guard requested but not ready: %s",
                collision_guard_.status_message().c_str());
        }
    }

    start_gripper_worker();
    activated_ = true;

    RCLCPP_INFO(
        get_logger(),
        "System activated (position mode, grippers=%zu, gripper_command_rate=%.1f Hz, "
        "gripper_command_epsilon=%.4f).",
        gripper_count_,
        gripper_command_rate_hz_,
        gripper_command_epsilon_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_deactivate
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn MarvinHardware::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    activated_ = false;
    stop_collision_guard_worker();
    stop_gripper_worker();
    workspace_guard_.disarm();
    collision_guard_.disarm();
    current_feedback_valid_.store(false, std::memory_order_relaxed);

    bool holding_position = false;
    if (connected_) {
        DCSS dcss{};
        if (OnGetBuf(&dcss)) {
            double hold_a[kJointsPerArm], hold_b[kJointsPerArm];
            for (size_t joint = 0; joint < kJointsPerArm; ++joint) {
                hold_a[joint] = static_cast<double>(dcss.m_Out[0].m_FB_Joint_PosE[joint]);
                hold_b[joint] = static_cast<double>(dcss.m_Out[1].m_FB_Joint_PosE[joint]);
            }

            OnClearSet();
            holding_position =
                OnSetTargetState_A(1) &&
                OnSetJointLmt_A(joint_vel_ratio_, joint_acc_ratio_) &&
                OnSetTargetState_B(1) &&
                OnSetJointLmt_B(joint_vel_ratio_, joint_acc_ratio_) &&
                OnSetJointCmdPos_A(hold_a) &&
                OnSetJointCmdPos_B(hold_b) &&
                OnSetSend();
            if (!holding_position) {
                RCLCPP_WARN(
                    get_logger(),
                    "Failed to request hold-position mode during deactivation; leaving current target state unchanged.");
            }
        } else {
            RCLCPP_WARN(
                get_logger(),
                "OnGetBuf failed during deactivation; skipping any transition to idle for safety.");
        }
    }

    requested_control_profile_.store(
        ControlProfile::kPositionFollow, std::memory_order_relaxed);
    active_control_profile_.store(
        holding_position ? ControlProfile::kPositionFollow : ControlProfile::kUnknown,
        std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lock(control_profile_mutex_);
        control_profile_transition_phase_ = ControlProfileTransitionPhase::kIdle;
        control_profile_transition_active_ = false;
        control_profile_request_pending_ = false;
        control_profile_resend_count_ = 0;
        control_profile_pending_target_ = ControlProfile::kUnknown;
        control_profile_source_profile_ = ControlProfile::kUnknown;
        control_profile_stabilize_cycles_remaining_ = 0;
        control_profile_last_packet_sent_at_ = Clock::time_point{};
        control_profile_last_progress_log_at_ = Clock::time_point{};
        control_profile_diag_snapshot_valid_ = false;
        control_profile_post_monitor_active_ = false;
        control_profile_post_monitor_source_ = ControlProfile::kUnknown;
        control_profile_post_monitor_target_ = ControlProfile::kUnknown;
        control_profile_post_monitor_started_at_ = Clock::time_point{};
        control_profile_post_monitor_deadline_ = Clock::time_point{};
        control_profile_post_monitor_last_log_at_ = Clock::time_point{};
        control_profile_post_monitor_snapshot_valid_ = false;
        control_profile_last_message_ = holding_position ?
            "Hardware deactivated while keeping POSITION_FOLLOW hold." :
            "Hardware deactivated before any control profile transition completed.";
        control_profile_completed_sequence_ = control_profile_inflight_sequence_;
    }
    control_profile_cv_.notify_all();

    if (gripper_count_ > 0)
        RCLCPP_INFO(get_logger(), "%zu gripper(s) holding last commanded position.",
                     gripper_count_);
    if (holding_position) {
        RCLCPP_INFO(
            get_logger(),
            "Dual-arm system deactivated while holding the last measured joint position.");
    } else {
        RCLCPP_INFO(
            get_logger(),
            "Dual-arm system deactivated without requesting idle mode.");
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_cleanup – release SDK
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn MarvinHardware::on_cleanup(
    const rclcpp_lifecycle::State &)
{
    activated_ = false;
    stop_collision_guard_worker();
    stop_gripper_worker();
    workspace_guard_.disarm();
    collision_guard_.disarm();
    current_feedback_valid_.store(false, std::memory_order_relaxed);
    active_control_profile_.store(ControlProfile::kUnknown, std::memory_order_relaxed);
    requested_control_profile_.store(ControlProfile::kUnknown, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lock(control_profile_mutex_);
        control_profile_transition_phase_ = ControlProfileTransitionPhase::kIdle;
        control_profile_transition_active_ = false;
        control_profile_request_pending_ = false;
        control_profile_resend_count_ = 0;
        control_profile_pending_target_ = ControlProfile::kUnknown;
        control_profile_source_profile_ = ControlProfile::kUnknown;
        control_profile_stabilize_cycles_remaining_ = 0;
        control_profile_last_packet_sent_at_ = Clock::time_point{};
        control_profile_last_progress_log_at_ = Clock::time_point{};
        control_profile_diag_snapshot_valid_ = false;
        control_profile_post_monitor_active_ = false;
        control_profile_post_monitor_source_ = ControlProfile::kUnknown;
        control_profile_post_monitor_target_ = ControlProfile::kUnknown;
        control_profile_post_monitor_started_at_ = Clock::time_point{};
        control_profile_post_monitor_deadline_ = Clock::time_point{};
        control_profile_post_monitor_last_log_at_ = Clock::time_point{};
        control_profile_post_monitor_snapshot_valid_ = false;
        control_profile_last_message_ =
            "Hardware cleanup reset the control profile transition skeleton.";
        control_profile_completed_sequence_ = control_profile_inflight_sequence_;
    }
    control_profile_cv_.notify_all();

    // Disconnect grippers BEFORE releasing Marvin link (they need it to send close cmd)
    for (size_t g = 0; g < gripper_count_; ++g) {
        if (grippers_[g].device) {
            const auto &jn = info_.joints[grippers_[g].joint_index].name;
            if (grippers_[g].device->is_connected())
                grippers_[g].device->disconnect();
            omnipicker_destroy(grippers_[g].device);
            grippers_[g].device = nullptr;
            RCLCPP_INFO(get_logger(), "Gripper '%s' released.", jn.c_str());
        }
    }

    if (connected_) {
        OnRelease();
        connected_ = false;
        RCLCPP_INFO(get_logger(), "SDK connection released.");
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// read – pull latest state from SDK for BOTH arms in one call
// ---------------------------------------------------------------------------
hardware_interface::return_type MarvinHardware::read(
    const rclcpp::Time &, const rclcpp::Duration &)
{
    if (!activated_) return hardware_interface::return_type::OK;

    DCSS dcss{};
    if (!OnGetBuf(&dcss)) {
        RCLCPP_ERROR(get_logger(), "OnGetBuf failed.");
        return hardware_interface::return_type::ERROR;
    }

    bool transition_active_snapshot = false;
    {
        std::lock_guard<std::mutex> lock(control_profile_mutex_);
        transition_active_snapshot =
            control_profile_transition_active_ || control_profile_request_pending_;
    }
    const int effective_no_frame_timeout_ms =
        transition_active_snapshot ?
            std::max(no_frame_timeout_ms_, profile_switch_timeout_ms_) :
            no_frame_timeout_ms_;

    auto build_post_monitor_summary = [&dcss]() {
        std::ostringstream stream;
        for (size_t arm = 0; arm < kArmCount; ++arm) {
            if (arm > 0) {
                stream << ' ';
            }
            stream << (arm == 0 ? "A" : "B")
                   << "[state="
                   << arm_state_name(static_cast<ArmState>(dcss.m_State[arm].m_CurState))
                   << '(' << dcss.m_State[arm].m_CurState << ')'
                   << " cmd=" << dcss.m_State[arm].m_CmdState
                   << " err=" << dcss.m_State[arm].m_ERRCode
                   << " in=" << dcss.m_In[arm].m_InFrameSerial
                   << " out=" << dcss.m_Out[arm].m_OutFrameSerial
                   << ']';
        }
        return stream.str();
    };

    for (size_t arm = 0; arm < kArmCount; ++arm) {
        current_sdk_cur_state_[arm].store(
            dcss.m_State[arm].m_CurState, std::memory_order_relaxed);
        current_sdk_cmd_state_[arm].store(
            dcss.m_State[arm].m_CmdState, std::memory_order_relaxed);
        current_sdk_err_code_[arm].store(
            dcss.m_State[arm].m_ERRCode, std::memory_order_relaxed);
        current_sdk_in_frame_serial_[arm].store(
            dcss.m_In[arm].m_InFrameSerial, std::memory_order_relaxed);
        current_sdk_out_frame_serial_[arm].store(
            dcss.m_Out[arm].m_OutFrameSerial, std::memory_order_relaxed);
        const auto state = static_cast<ArmState>(dcss.m_State[arm].m_CurState);
        if (state == ARM_STATE_ERROR) {
            if (control_profile_post_monitor_active_) {
                const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                            Clock::now() - control_profile_post_monitor_started_at_)
                                            .count();
                RCLCPP_ERROR(
                    get_logger(),
                    "Post-transition diagnostic caught ERROR after successful profile switch: "
                    "source=%s target=%s elapsed=%lldms %s",
                    control_profile_to_string(control_profile_post_monitor_source_),
                    control_profile_to_string(control_profile_post_monitor_target_),
                    static_cast<long long>(elapsed_ms),
                    build_post_monitor_summary().c_str());
            }
            RCLCPP_ERROR(get_logger(), "Arm %zu in ERROR state (err=%d).",
                         arm, dcss.m_State[arm].m_ERRCode);
            return hardware_interface::return_type::ERROR;
        }

        const int frame = dcss.m_Out[arm].m_OutFrameSerial;
        if (frame != last_frame_serial_[arm]) {
            last_frame_serial_[arm] = frame;
            last_frame_time_[arm] = Clock::now();
        } else {
            const auto stale = std::chrono::duration_cast<std::chrono::milliseconds>(
                Clock::now() - last_frame_time_[arm]);
            if (stale.count() > effective_no_frame_timeout_ms) {
                RCLCPP_ERROR(get_logger(), "No frame update for %dms on arm %zu.",
                             static_cast<int>(stale.count()), arm);
                return hardware_interface::return_type::ERROR;
            }
        }

        const auto &out = dcss.m_Out[arm];
        for (size_t j = 0; j < kJointsPerArm; ++j) {
            const size_t idx = arm * kJointsPerArm + j;
            const auto &jn = info_.joints[idx].name;
            const double pos_deg = static_cast<double>(out.m_FB_Joint_PosE[j]);
            set_state(pos_if(jn), pos_deg * kDeg2Rad);
            current_feedback_deg_[idx].store(pos_deg, std::memory_order_relaxed);
            if (has_velocity_state_) {
                set_state(vel_if(jn), static_cast<double>(out.m_FB_Joint_Vel[j]) * kDeg2Rad);
            }
            if (has_effort_state_) {
                set_state(eff_if(jn), static_cast<double>(out.m_FB_Joint_SToq[j]));
            }
        }
    }
    current_feedback_valid_.store(true, std::memory_order_relaxed);
    update_sdk_observation_states(dcss);

    if (control_profile_post_monitor_active_) {
        const auto now = Clock::now();
        bool changed = !control_profile_post_monitor_snapshot_valid_;
        for (size_t arm = 0; arm < kArmCount && !changed; ++arm) {
            changed =
                control_profile_post_monitor_last_cur_state_[arm] != dcss.m_State[arm].m_CurState ||
                control_profile_post_monitor_last_cmd_state_[arm] != dcss.m_State[arm].m_CmdState ||
                control_profile_post_monitor_last_err_code_[arm] != dcss.m_State[arm].m_ERRCode ||
                control_profile_post_monitor_last_in_frame_serial_[arm] !=
                    dcss.m_In[arm].m_InFrameSerial ||
                control_profile_post_monitor_last_out_frame_serial_[arm] !=
                    dcss.m_Out[arm].m_OutFrameSerial;
        }
        if (changed ||
            control_profile_post_monitor_last_log_at_ == Clock::time_point{} ||
            (now - control_profile_post_monitor_last_log_at_) >= std::chrono::milliseconds(100)) {
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                        now - control_profile_post_monitor_started_at_)
                                        .count();
            RCLCPP_WARN(
                get_logger(),
                "Post-transition diagnostic: source=%s target=%s elapsed=%lldms %s",
                control_profile_to_string(control_profile_post_monitor_source_),
                control_profile_to_string(control_profile_post_monitor_target_),
                static_cast<long long>(elapsed_ms),
                build_post_monitor_summary().c_str());
            for (size_t arm = 0; arm < kArmCount; ++arm) {
                control_profile_post_monitor_last_cur_state_[arm] = dcss.m_State[arm].m_CurState;
                control_profile_post_monitor_last_cmd_state_[arm] = dcss.m_State[arm].m_CmdState;
                control_profile_post_monitor_last_err_code_[arm] = dcss.m_State[arm].m_ERRCode;
                control_profile_post_monitor_last_in_frame_serial_[arm] =
                    dcss.m_In[arm].m_InFrameSerial;
                control_profile_post_monitor_last_out_frame_serial_[arm] =
                    dcss.m_Out[arm].m_OutFrameSerial;
            }
            control_profile_post_monitor_snapshot_valid_ = true;
            control_profile_post_monitor_last_log_at_ = now;
        }
        if (now >= control_profile_post_monitor_deadline_) {
            control_profile_post_monitor_active_ = false;
            control_profile_post_monitor_source_ = ControlProfile::kUnknown;
            control_profile_post_monitor_target_ = ControlProfile::kUnknown;
            control_profile_post_monitor_started_at_ = Clock::time_point{};
            control_profile_post_monitor_deadline_ = Clock::time_point{};
            control_profile_post_monitor_last_log_at_ = Clock::time_point{};
            control_profile_post_monitor_snapshot_valid_ = false;
        }
    }

    // Read gripper state cache maintained by the background worker.
    for (size_t g = 0; g < gripper_count_; ++g) {
        const auto &slot = grippers_[g];
        const auto &jn = info_.joints[slot.joint_index].name;
        const double pos = slot.state_percent.load(std::memory_order_relaxed);
        set_state(pos_if(jn), pos);
    }

    return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// write – push joint position commands to SDK for BOTH arms in one packet
// ---------------------------------------------------------------------------
hardware_interface::return_type MarvinHardware::write(
    const rclcpp::Time &, const rclcpp::Duration &)
{
    if (!activated_) return hardware_interface::return_type::OK;

    if (process_control_profile_transition()) {
        return hardware_interface::return_type::OK;
    }

    double desired_a[kJointsPerArm];
    double desired_b[kJointsPerArm];
    double cmd_a[kJointsPerArm];
    double cmd_b[kJointsPerArm];

    for (size_t j = 0; j < kJointsPerArm; ++j) {
        // Arm A (left): joints 0..6
        const double va = get_command<double>(pos_if(info_.joints[j].name));
        if (!std::isfinite(va)) {
            RCLCPP_ERROR(get_logger(), "Non-finite command on joint %zu (arm A).", j);
            return hardware_interface::return_type::ERROR;
        }
        desired_a[j] = std::clamp(va, joint_min_[j], joint_max_[j]) * kRad2Deg;

        // Arm B (right): joints 7..13
        const size_t idx_b = kJointsPerArm + j;
        const double vb = get_command<double>(pos_if(info_.joints[idx_b].name));
        if (!std::isfinite(vb)) {
            RCLCPP_ERROR(get_logger(), "Non-finite command on joint %zu (arm B).", j);
            return hardware_interface::return_type::ERROR;
        }
        desired_b[j] = std::clamp(vb, joint_min_[idx_b], joint_max_[idx_b]) * kRad2Deg;
        collision_guard_target_deg_[j].store(desired_a[j], std::memory_order_relaxed);
        collision_guard_target_deg_[idx_b].store(desired_b[j], std::memory_order_relaxed);
    }

    bool using_collision_guard = false;
    if (collision_guard_.active() &&
        collision_guard_runtime_enabled_.load(std::memory_order_relaxed) &&
        collision_guard_approved_valid_.load(std::memory_order_relaxed)) {
        using_collision_guard = true;
        for (size_t j = 0; j < kJointsPerArm; ++j) {
            const size_t idx_b = kJointsPerArm + j;
            cmd_a[j] = collision_guard_approved_deg_[j].load(std::memory_order_relaxed);
            cmd_b[j] = collision_guard_approved_deg_[idx_b].load(std::memory_order_relaxed);
        }
    } else {
        std::copy(desired_a, desired_a + kJointsPerArm, cmd_a);
        std::copy(desired_b, desired_b + kJointsPerArm, cmd_b);
    }

    if (workspace_guard_.enabled() &&
        workspace_guard_runtime_enabled_.load(std::memory_order_relaxed) &&
        !using_collision_guard) {
        workspace_guard_.filter(0, cmd_a, get_logger());
        workspace_guard_.filter(1, cmd_b, get_logger());
    }

    // Hand gripper targets to the non-realtime worker to keep SDK I/O out of the control loop.
    for (size_t g = 0; g < gripper_count_; ++g) {
        auto &slot = grippers_[g];
        const auto &jn = info_.joints[slot.joint_index].name;
        const double cmd = get_command<double>(pos_if(jn));
        if (!std::isfinite(cmd)) {
            RCLCPP_ERROR(get_logger(), "Non-finite command on gripper '%s'.", jn.c_str());
            return hardware_interface::return_type::ERROR;
        }

        const double clamped = std::clamp(cmd, 0.0, 1.0);
        slot.command_target_percent.store(clamped, std::memory_order_relaxed);
        if (slot.mock) {
            slot.state_percent.store(clamped, std::memory_order_relaxed);
            slot.state_valid.store(true, std::memory_order_relaxed);
        }
    }

    // Single atomic send: ClearSet → set A → set B → Send
    OnClearSet();
    const bool ok = OnSetJointCmdPos_A(cmd_a)
                 && OnSetJointCmdPos_B(cmd_b)
                 && OnSetSend();
    if (!ok) {
        consecutive_write_failures_++;
        total_write_failures_++;
        if (consecutive_write_failures_ >= kMaxWriteFailures) {
            RCLCPP_ERROR(get_logger(),
                "SDK send failed %d times consecutively (total %d), aborting.",
                consecutive_write_failures_, total_write_failures_);
            return hardware_interface::return_type::ERROR;
        }
        if (consecutive_write_failures_ >= 3) {
            RCLCPP_WARN(get_logger(), "SDK send failure streak %d/%d (total %d).",
                        consecutive_write_failures_, kMaxWriteFailures,
                        total_write_failures_);
        }
        return hardware_interface::return_type::OK;
    }
    consecutive_write_failures_ = 0;

    if (!using_collision_guard && collision_guard_.active() &&
        collision_guard_runtime_enabled_.load(std::memory_order_relaxed)) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Collision guard is active but no approved command is available yet; "
            "falling back to direct command output for this cycle.");
    }

    return hardware_interface::return_type::OK;
}

}  // namespace marvin_system

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(marvin_system::MarvinHardware, hardware_interface::SystemInterface)
