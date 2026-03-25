#ifndef ROS2_CONTROL_MARVIN__HARDWARE__MARVIN_HPP_
#define ROS2_CONTROL_MARVIN__HARDWARE__MARVIN_HPP_

#include <array>
#include <chrono>

#include "marvin_system/omnipicker.hpp"
#include "marvin_system/workspace_guard.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
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
    static constexpr size_t kJointsPerArm = 7;
    static constexpr size_t kArmCount = 2;
    static constexpr size_t kTotalJoints = kJointsPerArm * kArmCount;

    bool activated_{false};
    bool connected_{false};

    int joint_vel_ratio_{30};
    int joint_acc_ratio_{30};
    int connect_timeout_ms_{1500};
    int state_timeout_ms_{5000};
    int no_frame_timeout_ms_{800};
    int home_timeout_ms_{30000};

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
    };
    size_t gripper_count_{0};
    std::array<GripperSlot, kMaxGrippers> grippers_{};

    // ---- Workspace z-floor safety check (optional) ----
    WorkspaceGuard workspace_guard_;

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
