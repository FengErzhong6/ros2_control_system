#ifndef ROS2_CONTROL_MARVIN__HARDWARE__MARVIN_HPP_
#define ROS2_CONTROL_MARVIN__HARDWARE__MARVIN_HPP_

#include <atomic>
#include <array>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "marvin_system/srv/set_control_profile.hpp"
#include "marvin_system/collision_guard.hpp"
#include "marvin_system/teleop_diagnostics.hpp"
#include "marvin_system/omnipicker.hpp"
#include "marvin_system/workspace_guard.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace marvin_system {

class MarvinHardware : public hardware_interface::SystemInterface {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MarvinHardware)

    ~MarvinHardware() override;

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams &params) override;
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    void update_sdk_observation_states(const DCSS &dcss);
    void start_gripper_worker();
    void stop_gripper_worker();
    void gripper_worker_loop();
    void start_collision_guard_worker();
    void stop_collision_guard_worker();
    void collision_guard_worker_loop();

    static constexpr size_t kJointsPerArm = 7;
    static constexpr size_t kArmCount = 2;
    static constexpr size_t kTotalJoints = kJointsPerArm * kArmCount;

    bool process_control_profile_transition();
    bool send_position_hold_command(
        const std::array<std::array<double, kJointsPerArm>, kArmCount> &hold_deg,
        bool request_position_state);
    bool send_joint_impedance_hold_command(
        const std::array<std::array<double, kJointsPerArm>, kArmCount> &hold_deg,
        bool request_torque_state);

    bool activated_{false};
    bool connected_{false};

    int joint_vel_ratio_{30};
    int joint_acc_ratio_{30};
    int gripper_velocity_{255};
    int gripper_acceleration_{255};
    double gripper_command_rate_hz_{50.0};
    double gripper_command_epsilon_{1.0e-3};
    int connect_timeout_ms_{1500};
    int state_timeout_ms_{8000};
    int activation_retry_settle_ms_{1500};
    int activation_max_attempts_{2};
    int no_frame_timeout_ms_{800};
    int home_timeout_ms_{30000};
    int profile_switch_timeout_ms_{5000};
    int profile_switch_stabilize_cycles_{1};
    int profile_switch_post_monitor_ms_{500};
    int profile_switch_initial_resend_delay_ms_{80};
    int profile_switch_resend_interval_ms_{20};
    bool joint_impedance_profile_enabled_{true};
    bool mock_grippers_{false};
    std::array<std::array<double, 6>, kArmCount> tool_kine_{};
    std::array<std::array<double, 10>, kArmCount> tool_dyn_{};
    std::array<std::array<double, kJointsPerArm>, kArmCount> joint_impedance_k_{};
    std::array<std::array<double, kJointsPerArm>, kArmCount> joint_impedance_d_{};
    std::atomic<ControlProfile> active_control_profile_{ControlProfile::kUnknown};
    std::atomic<ControlProfile> requested_control_profile_{ControlProfile::kUnknown};
    enum class ControlProfileTransitionPhase : int8_t {
        kIdle = 0,
        kRequested = 1,
        kCaptureHold = 2,
        kSendSwitchPacket = 3,
        kWaitArmState = 4,
        kStabilize = 5,
        kCompleted = 6,
        kFailed = 7,
    };
    mutable std::mutex control_profile_mutex_;
    std::condition_variable control_profile_cv_;
    ControlProfileTransitionPhase control_profile_transition_phase_{
        ControlProfileTransitionPhase::kIdle};
    bool control_profile_transition_active_{false};
    bool control_profile_request_pending_{false};
    uint64_t control_profile_request_sequence_{0};
    uint64_t control_profile_completed_sequence_{0};
    uint64_t control_profile_inflight_sequence_{0};
    uint64_t control_profile_resend_count_{0};
    ControlProfile control_profile_pending_target_{ControlProfile::kUnknown};
    ControlProfile control_profile_source_profile_{ControlProfile::kUnknown};
    std::array<std::array<double, kJointsPerArm>, kArmCount> control_profile_hold_deg_{};
    std::chrono::steady_clock::time_point control_profile_transition_started_at_{};
    std::chrono::steady_clock::time_point control_profile_last_packet_sent_at_{};
    std::chrono::steady_clock::time_point control_profile_last_progress_log_at_{};
    bool control_profile_diag_snapshot_valid_{false};
    std::array<int, kArmCount> control_profile_diag_last_cur_state_{};
    std::array<int, kArmCount> control_profile_diag_last_cmd_state_{};
    std::array<int, kArmCount> control_profile_diag_last_err_code_{};
    std::array<int, kArmCount> control_profile_diag_last_in_frame_serial_{};
    std::array<int, kArmCount> control_profile_diag_last_out_frame_serial_{};
    int control_profile_stabilize_cycles_remaining_{0};
    bool control_profile_post_monitor_active_{false};
    ControlProfile control_profile_post_monitor_source_{ControlProfile::kUnknown};
    ControlProfile control_profile_post_monitor_target_{ControlProfile::kUnknown};
    std::chrono::steady_clock::time_point control_profile_post_monitor_started_at_{};
    std::chrono::steady_clock::time_point control_profile_post_monitor_deadline_{};
    std::chrono::steady_clock::time_point control_profile_post_monitor_last_log_at_{};
    bool control_profile_post_monitor_snapshot_valid_{false};
    std::array<int, kArmCount> control_profile_post_monitor_last_cur_state_{};
    std::array<int, kArmCount> control_profile_post_monitor_last_cmd_state_{};
    std::array<int, kArmCount> control_profile_post_monitor_last_err_code_{};
    std::array<int, kArmCount> control_profile_post_monitor_last_in_frame_serial_{};
    std::array<int, kArmCount> control_profile_post_monitor_last_out_frame_serial_{};
    std::string control_profile_last_message_{"No control profile request has been processed."};
    rclcpp::Service<marvin_system::srv::SetControlProfile>::SharedPtr control_profile_service_;

    bool has_home_position_{false};
    std::array<std::array<double, kJointsPerArm>, kArmCount> home_position_deg_{};

    std::array<int, kArmCount> last_frame_serial_{};
    std::array<std::chrono::steady_clock::time_point, kArmCount> last_frame_time_{};

    std::array<double, kTotalJoints> joint_min_{};
    std::array<double, kTotalJoints> joint_max_{};

    bool has_velocity_state_{false};
    bool has_effort_state_{false};

    int consecutive_write_failures_{0};
    int total_write_failures_{0};
    static constexpr int kMaxWriteFailures = 10;

    // ---- OmniPicker gripper support (optional) ----
    static constexpr size_t kMaxGrippers = 2;
    struct GripperSlot {
        omnipicker::IOmniPicker* device = nullptr;
        omnipicker::ArmSide arm_side = omnipicker::ArmSide::kB;
        uint32_t can_node_id = 0x01;
        size_t joint_index = 0;
        bool mock = false;
        std::atomic<double> command_target_percent{0.0};
        std::atomic<double> state_percent{0.0};
        std::atomic<bool> state_valid{false};
        bool has_sent_command = false;
        double last_command_percent = 0.0;
        std::chrono::steady_clock::time_point last_command_time{};
        int consecutive_send_failures = 0;
    };
    size_t gripper_count_{0};
    std::array<GripperSlot, kMaxGrippers> grippers_{};
    std::thread gripper_worker_thread_{};
    std::atomic<bool> gripper_worker_stop_{false};

    // ---- Workspace z-floor safety check (optional) ----
    WorkspaceGuard workspace_guard_;
    std::atomic<bool> workspace_guard_runtime_enabled_{true};
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr workspace_guard_service_;

    // ---- Async collision guard (optional) ----
    CollisionGuard collision_guard_;
    std::atomic<bool> collision_guard_runtime_enabled_{true};
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr collision_guard_service_;
    std::thread collision_guard_thread_{};
    std::atomic<bool> collision_guard_stop_{false};
    std::array<std::atomic<double>, kTotalJoints> current_feedback_deg_{};
    std::array<std::atomic<int>, kArmCount> current_sdk_cur_state_{};
    std::array<std::atomic<int>, kArmCount> current_sdk_cmd_state_{};
    std::array<std::atomic<int>, kArmCount> current_sdk_err_code_{};
    std::array<std::atomic<int>, kArmCount> current_sdk_in_frame_serial_{};
    std::array<std::atomic<int>, kArmCount> current_sdk_out_frame_serial_{};
    std::array<std::atomic<double>, kTotalJoints> collision_guard_target_deg_{};
    std::array<std::atomic<double>, kTotalJoints> collision_guard_approved_deg_{};
    std::atomic<bool> current_feedback_valid_{false};
    std::atomic<bool> collision_guard_approved_valid_{false};

    // ---- Control loop frequency monitoring ----
    int loop_stats_interval_cycles_{5000};
    int loop_cycle_count_{0};
    std::chrono::steady_clock::time_point loop_stats_start_{};
    double loop_period_min_us_{1e9};
    double loop_period_max_us_{0.0};
    std::chrono::steady_clock::time_point loop_last_write_time_{};
};

}  // namespace marvin_system

#endif  // ROS2_CONTROL_MARVIN__HARDWARE__MARVIN_HPP_
