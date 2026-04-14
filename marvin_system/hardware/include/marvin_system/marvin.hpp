#ifndef ROS2_CONTROL_MARVIN__HARDWARE__MARVIN_HPP_
#define ROS2_CONTROL_MARVIN__HARDWARE__MARVIN_HPP_

#include <atomic>
#include <array>
#include <chrono>
#include <thread>

#include "marvin_system/collision_guard.hpp"
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
    void start_gripper_worker();
    void stop_gripper_worker();
    void gripper_worker_loop();
    void start_collision_guard_worker();
    void stop_collision_guard_worker();
    void collision_guard_worker_loop();

    static constexpr size_t kJointsPerArm = 7;
    static constexpr size_t kArmCount = 2;
    static constexpr size_t kTotalJoints = kJointsPerArm * kArmCount;

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
    bool mock_grippers_{false};

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
