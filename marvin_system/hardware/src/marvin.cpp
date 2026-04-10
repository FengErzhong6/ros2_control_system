#include "marvin_system/marvin.hpp"
#include "marvin_system/MarvinSDK.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
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

MarvinHardware::~MarvinHardware()
{
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

void MarvinHardware::start_gripper_worker()
{
    stop_gripper_worker();
    if (gripper_count_ == 0) {
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
            if (!slot.device) {
                continue;
            }

            const double desired =
                clamp_unit(slot.command_target_percent.load(std::memory_order_relaxed));
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
    stop_gripper_worker();
    activated_ = false;
    workspace_guard_.disarm();
    const auto &p = info_.hardware_parameters;

    joint_vel_ratio_     = param_int(p, "joint_vel_ratio",     30,   1, 100);
    joint_acc_ratio_     = param_int(p, "joint_acc_ratio",     30,   1, 100);
    gripper_velocity_    = param_int(p, "gripper_velocity",    255,  0, 255);
    gripper_acceleration_= param_int(p, "gripper_acceleration",255,  0, 255);
    gripper_command_rate_hz_ = param_double(
        p, "gripper_command_rate_hz", 50.0, 1.0, 1000.0);
    gripper_command_epsilon_ = param_double(
        p, "gripper_command_epsilon", 1.0e-3, 0.0, 1.0);
    connect_timeout_ms_ = param_int(p, "connect_timeout_ms", 1500, 100, 30000);
    state_timeout_ms_ = param_int(p, "state_timeout_ms", 8000, 100, 30000);
    activation_retry_settle_ms_ = param_int(
        p, "activation_retry_settle_ms", 1500, 0, 10000);
    activation_max_attempts_ = param_int(
        p, "activation_max_attempts", 2, 1, 5);
    no_frame_timeout_ms_ = param_int(p, "no_frame_timeout_ms", 800, 50, 10000);

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

    RCLCPP_INFO(get_logger(),
                "Configured dual-arm system (joint_vel=%d%%, joint_acc=%d%%, "
                "gripper_velocity=%d, gripper_acceleration=%d, grippers=%zu).",
                joint_vel_ratio_,
                joint_acc_ratio_,
                gripper_velocity_,
                gripper_acceleration_,
                gripper_count_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_activate – enter position-follow mode on BOTH arms, seed state
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn MarvinHardware::on_activate(
    const rclcpp_lifecycle::State &)
{
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

    refresh_hold_positions();

    bool mode_switch_succeeded = false;
    std::array<bool, kArmCount> timeout_has_status{{false, false}};
    std::array<int, kArmCount> timeout_cur_state{};
    std::array<int, kArmCount> timeout_cmd_state{};
    std::array<int, kArmCount> timeout_err_code{};
    bool timeout_arm_ready_a = false;
    bool timeout_arm_ready_b = false;

    for (int attempt = 1; attempt <= activation_max_attempts_; ++attempt) {
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

                refresh_hold_positions();

                if (!arm_a_ready) {
                    const auto st = static_cast<ArmState>(dcss.m_State[0].m_CurState);
                    if (st == ARM_STATE_ERROR) {
                        RCLCPP_ERROR(get_logger(), "Arm A error during mode switch (err=%d).",
                                     dcss.m_State[0].m_ERRCode);
                        return hardware_interface::CallbackReturn::ERROR;
                    }
                    if (st == ARM_STATE_POSITION) {
                        arm_a_ready = true;
                        RCLCPP_INFO(get_logger(), "Arm A (left) entered position mode.");
                    }
                }
                if (!arm_b_ready) {
                    const auto st = static_cast<ArmState>(dcss.m_State[1].m_CurState);
                    if (st == ARM_STATE_ERROR) {
                        RCLCPP_ERROR(get_logger(), "Arm B error during mode switch (err=%d).",
                                     dcss.m_State[1].m_ERRCode);
                        return hardware_interface::CallbackReturn::ERROR;
                    }
                    if (st == ARM_STATE_POSITION) {
                        arm_b_ready = true;
                        RCLCPP_INFO(get_logger(), "Arm B (right) entered position mode.");
                    }
                }
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
            break;
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
        const auto &out = dcss.m_Out[arm];
        for (size_t j = 0; j < kJointsPerArm; ++j) {
            const size_t idx = arm * kJointsPerArm + j;
            const auto &jn = info_.joints[idx].name;
            const double pos_rad = static_cast<double>(out.m_FB_Joint_PosE[j]) * kDeg2Rad;
            set_state(pos_if(jn), pos_rad);
            set_command(pos_if(jn), pos_rad);
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

    // Seed gripper state & command (query_states blocks briefly, OK during activation)
    for (size_t g = 0; g < gripper_count_; ++g) {
        auto &slot = grippers_[g];
        const auto &jn = info_.joints[slot.joint_index].name;

        omnipicker::GripperStatus gst;
        auto err = slot.device->query_states(gst);
        double init_pos = 0.0;
        if (err == omnipicker::ErrorCode::kOK && gst.valid) {
            init_pos = static_cast<double>(gst.position) / 255.0;
        } else {
            RCLCPP_WARN(get_logger(),
                "Gripper '%s' initial query returned %s; defaulting to 0.0.",
                jn.c_str(), omnipicker::error_to_string(err));
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
        RCLCPP_INFO(get_logger(), "Gripper '%s' ready (pos=%.1f%%).",
                     jn.c_str(), init_pos * 100.0);
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
    stop_gripper_worker();
    workspace_guard_.disarm();

    if (connected_) {
        OnClearSet();
        OnSetTargetState_A(0);
        OnSetTargetState_B(0);
        OnSetSend();
    }

    if (gripper_count_ > 0)
        RCLCPP_INFO(get_logger(), "%zu gripper(s) holding last commanded position.",
                     gripper_count_);
    RCLCPP_INFO(get_logger(), "Dual-arm system deactivated (arms set to idle).");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_cleanup – release SDK
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn MarvinHardware::on_cleanup(
    const rclcpp_lifecycle::State &)
{
    activated_ = false;
    stop_gripper_worker();
    workspace_guard_.disarm();

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

    for (size_t arm = 0; arm < kArmCount; ++arm) {
        const auto state = static_cast<ArmState>(dcss.m_State[arm].m_CurState);
        if (state == ARM_STATE_ERROR) {
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
            if (stale.count() > no_frame_timeout_ms_) {
                RCLCPP_ERROR(get_logger(), "No frame update for %dms on arm %zu.",
                             static_cast<int>(stale.count()), arm);
                return hardware_interface::return_type::ERROR;
            }
        }

        const auto &out = dcss.m_Out[arm];
        for (size_t j = 0; j < kJointsPerArm; ++j) {
            const size_t idx = arm * kJointsPerArm + j;
            const auto &jn = info_.joints[idx].name;
            set_state(pos_if(jn), static_cast<double>(out.m_FB_Joint_PosE[j]) * kDeg2Rad);
            if (has_velocity_state_) {
                set_state(vel_if(jn), static_cast<double>(out.m_FB_Joint_Vel[j]) * kDeg2Rad);
            }
            if (has_effort_state_) {
                set_state(eff_if(jn), static_cast<double>(out.m_FB_Joint_SToq[j]));
            }
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

    double cmd_a[kJointsPerArm];
    double cmd_b[kJointsPerArm];

    for (size_t j = 0; j < kJointsPerArm; ++j) {
        // Arm A (left): joints 0..6
        const double va = get_command<double>(pos_if(info_.joints[j].name));
        if (!std::isfinite(va)) {
            RCLCPP_ERROR(get_logger(), "Non-finite command on joint %zu (arm A).", j);
            return hardware_interface::return_type::ERROR;
        }
        cmd_a[j] = std::clamp(va, joint_min_[j], joint_max_[j]) * kRad2Deg;

        // Arm B (right): joints 7..13
        const size_t idx_b = kJointsPerArm + j;
        const double vb = get_command<double>(pos_if(info_.joints[idx_b].name));
        if (!std::isfinite(vb)) {
            RCLCPP_ERROR(get_logger(), "Non-finite command on joint %zu (arm B).", j);
            return hardware_interface::return_type::ERROR;
        }
        cmd_b[j] = std::clamp(vb, joint_min_[idx_b], joint_max_[idx_b]) * kRad2Deg;
    }

    if (workspace_guard_.enabled() &&
        workspace_guard_runtime_enabled_.load(std::memory_order_relaxed)) {
        workspace_guard_.filter(0, cmd_a, get_logger());
        workspace_guard_.filter(1, cmd_b, get_logger());
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

    // Hand gripper targets to the non-realtime worker to keep SDK I/O out of the control loop.
    for (size_t g = 0; g < gripper_count_; ++g) {
        auto &slot = grippers_[g];
        const auto &jn = info_.joints[slot.joint_index].name;
        const double cmd = get_command<double>(pos_if(jn));
        if (!std::isfinite(cmd)) {
            RCLCPP_ERROR(get_logger(), "Non-finite command on gripper '%s'.", jn.c_str());
            return hardware_interface::return_type::ERROR;
        }

        slot.command_target_percent.store(
            std::clamp(cmd, 0.0, 1.0), std::memory_order_relaxed);
    }

    return hardware_interface::return_type::OK;
}

}  // namespace marvin_system

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(marvin_system::MarvinHardware, hardware_interface::SystemInterface)
