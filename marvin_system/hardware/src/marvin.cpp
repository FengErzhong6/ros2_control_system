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

}  // namespace

// ===========================================================================

namespace marvin_system {

MarvinHardware::~MarvinHardware()
{
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
    activated_ = false;
    const auto &p = info_.hardware_parameters;

    joint_vel_ratio_     = param_int(p, "joint_vel_ratio",     30,   1, 100);
    joint_acc_ratio_     = param_int(p, "joint_acc_ratio",     30,   1, 100);
    gripper_velocity_    = param_int(p, "gripper_velocity",    255,  0, 255);
    gripper_acceleration_= param_int(p, "gripper_acceleration",255,  0, 255);
    connect_timeout_ms_  = param_int(p, "connect_timeout_ms",  1500, 100, 30000);
    state_timeout_ms_    = param_int(p, "state_timeout_ms",    5000, 100, 30000);
    no_frame_timeout_ms_ = param_int(p, "no_frame_timeout_ms", 800,  50, 10000);

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
    // Snapshot current joint positions (degrees) for hold commands.
    DCSS dcss{};
    if (!OnGetBuf(&dcss)) {
        RCLCPP_ERROR(get_logger(), "OnGetBuf failed before activation.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    double hold_a[kJointsPerArm], hold_b[kJointsPerArm];
    for (size_t j = 0; j < kJointsPerArm; ++j) {
        hold_a[j] = static_cast<double>(dcss.m_Out[0].m_FB_Joint_PosE[j]);
        hold_b[j] = static_cast<double>(dcss.m_Out[1].m_FB_Joint_PosE[j]);
    }

    // Request position mode for both arms, including initial hold positions
    // so the controller already has valid commands when it enters position mode.
    OnClearSet();
    bool ok = OnSetTargetState_A(1)
           && OnSetJointLmt_A(joint_vel_ratio_, joint_acc_ratio_)
           && OnSetTargetState_B(1)
           && OnSetJointLmt_B(joint_vel_ratio_, joint_acc_ratio_)
           && OnSetJointCmdPos_A(hold_a)
           && OnSetJointCmdPos_B(hold_b);
    ok = ok && OnSetSend();
    if (!ok) {
        RCLCPP_ERROR(get_logger(), "Failed to request position mode on dual-arm.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Poll until BOTH arms enter position mode.
    // Continuously send hold-position commands (~250 Hz) to prevent starvation:
    // the arms may enter position mode at different times, so the first one
    // would starve if we only polled without sending.
    bool arm_a_ready = false, arm_b_ready = false;
    const auto deadline = Clock::now() + std::chrono::milliseconds(state_timeout_ms_);

    while (!arm_a_ready || !arm_b_ready) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if (OnGetBuf(&dcss)) {
            for (size_t j = 0; j < kJointsPerArm; ++j) {
                hold_a[j] = static_cast<double>(dcss.m_Out[0].m_FB_Joint_PosE[j]);
                hold_b[j] = static_cast<double>(dcss.m_Out[1].m_FB_Joint_PosE[j]);
            }

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

        if (Clock::now() > deadline) {
            RCLCPP_ERROR(get_logger(), "Timeout waiting for position mode (A=%s, B=%s).",
                         arm_a_ready ? "OK" : "PENDING", arm_b_ready ? "OK" : "PENDING");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Keep feeding both arms; re-request state for arms still transitioning.
        OnClearSet();
        if (!arm_a_ready) {
            OnSetTargetState_A(1);
            OnSetJointLmt_A(joint_vel_ratio_, joint_acc_ratio_);
        }
        if (!arm_b_ready) {
            OnSetTargetState_B(1);
            OnSetJointLmt_B(joint_vel_ratio_, joint_acc_ratio_);
        }
        OnSetJointCmdPos_A(hold_a);
        OnSetJointCmdPos_B(hold_b);
        OnSetSend();
    }

    // Move to home position if configured (joints 7→1 for safety)
    if (has_home_position_) {
        RCLCPP_INFO(get_logger(), "Moving to home position (Joint7 → Joint1) ...");
        constexpr double kHomeTolDeg = 0.5;

        double cmd_a[kJointsPerArm], cmd_b[kJointsPerArm];
        for (size_t j = 0; j < kJointsPerArm; ++j) {
            cmd_a[j] = static_cast<double>(dcss.m_Out[0].m_FB_Joint_PosE[j]);
            cmd_b[j] = static_cast<double>(dcss.m_Out[1].m_FB_Joint_PosE[j]);
        }

        const auto home_deadline =
            Clock::now() + std::chrono::milliseconds(home_timeout_ms_);
        bool timed_out = false;

        for (int jt = static_cast<int>(kJointsPerArm) - 1;
             jt >= 0 && !timed_out; --jt) {
            cmd_a[jt] = home_position_deg_[0][jt];
            cmd_b[jt] = home_position_deg_[1][jt];

            if (workspace_guard_.enabled()) {
                double guarded_a[kJointsPerArm];
                double guarded_b[kJointsPerArm];
                std::copy(cmd_a, cmd_a + kJointsPerArm, guarded_a);
                std::copy(cmd_b, cmd_b + kJointsPerArm, guarded_b);
                const bool arm_a_safe = workspace_guard_.filter(0, guarded_a, get_logger());
                const bool arm_b_safe = workspace_guard_.filter(1, guarded_b, get_logger());
                if (!arm_a_safe || !arm_b_safe) {
                    RCLCPP_ERROR(
                        get_logger(),
                        "Home position for Joint%d violates workspace z-floor safety check.",
                        jt + 1);
                    return hardware_interface::CallbackReturn::ERROR;
                }
            }

            RCLCPP_INFO(get_logger(), "Homing Joint%d ...", jt + 1);

            while (true) {
                OnClearSet();
                OnSetJointCmdPos_A(cmd_a);
                OnSetJointCmdPos_B(cmd_b);
                OnSetSend();

                std::this_thread::sleep_for(std::chrono::milliseconds(10));

                if (OnGetBuf(&dcss)) {
                    for (size_t arm = 0; arm < kArmCount; ++arm) {
                        if (static_cast<ArmState>(dcss.m_State[arm].m_CurState)
                            == ARM_STATE_ERROR) {
                            RCLCPP_ERROR(get_logger(),
                                "Arm %zu error during homing at Joint%d (err=%d).",
                                arm, jt + 1, dcss.m_State[arm].m_ERRCode);
                            return hardware_interface::CallbackReturn::ERROR;
                        }
                    }

                    double fb_a = static_cast<double>(
                        dcss.m_Out[0].m_FB_Joint_PosE[jt]);
                    double fb_b = static_cast<double>(
                        dcss.m_Out[1].m_FB_Joint_PosE[jt]);

                    if (std::abs(fb_a - home_position_deg_[0][jt]) <= kHomeTolDeg &&
                        std::abs(fb_b - home_position_deg_[1][jt]) <= kHomeTolDeg) {
                        break;
                    }
                }

                if (Clock::now() > home_deadline) {
                    RCLCPP_WARN(get_logger(),
                        "Home position timeout (%d ms) at Joint%d, continuing.",
                        home_timeout_ms_, jt + 1);
                    timed_out = true;
                    break;
                }
            }
        }
        RCLCPP_INFO(get_logger(), "Home position sequence complete.");
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
    }

    activated_ = true;

    RCLCPP_INFO(get_logger(), "System activated (position mode, grippers=%zu).",
                gripper_count_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_deactivate
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn MarvinHardware::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    activated_ = false;

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

    // Read gripper cached states (updated asynchronously via write→move→try_read_response)
    for (size_t g = 0; g < gripper_count_; ++g) {
        const auto &slot = grippers_[g];
        const auto &jn = info_.joints[slot.joint_index].name;
        set_state(pos_if(jn),
                  static_cast<double>(slot.device->get_position_percent()));
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

    if (workspace_guard_.enabled()) {
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

    // Send gripper commands (independent ChData channel, no conflict with arm commands)
    for (size_t g = 0; g < gripper_count_; ++g) {
        const auto &slot = grippers_[g];
        const auto &jn = info_.joints[slot.joint_index].name;
        const double cmd = get_command<double>(pos_if(jn));
        if (!std::isfinite(cmd)) {
            RCLCPP_ERROR(get_logger(), "Non-finite command on gripper '%s'.", jn.c_str());
            return hardware_interface::return_type::ERROR;
        }
        slot.device->set_position_percent(
            static_cast<float>(std::clamp(cmd, 0.0, 1.0)));
    }

    return hardware_interface::return_type::OK;
}

}  // namespace marvin_system

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(marvin_system::MarvinHardware, hardware_interface::SystemInterface)
